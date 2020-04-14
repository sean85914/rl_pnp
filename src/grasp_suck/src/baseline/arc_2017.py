#!/usr/bin/env python

import sys
import os
import time
import argparse
import numpy as np
import cv2
import rospy
import copy
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
from torchvision import models
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from visualization.srv import viz_marker, viz_markerRequest
from grasp_suck.srv import recorder, recorderRequest, recorderResponse
import utils
from utils import Net
from actionWrapper import ActionWrapper

# TODO: add memory for action

class Process(object):
	def __init__(self, args):
		self.file_path = os.path.dirname(os.path.abspath(__file__)) # current file path
		self.use_cuda = args.use_cuda
		self.scale = args.scale
		self.grasp_angle = args.grasp_angle
		self.voxel_size = args.voxel_size
		self.color_topic = args.color_topic
		self.depth_topic = args.depth_topic
		self.episode = args.episode
		self.run = args.run
		self.save_root = self.file_path + "/exp_2/ep{}_run{}".format(self.episode, self.run)
		self._create_directories()
		self.suck_weight = 1.0
		self.grasp_weight = 0.25
		self.count = 0
		self.bridge = CvBridge()
		self.background_color = self.file_path + "/" + args.color_bg
		self.background_depth = self.file_path + "/" + args.depth_bg
		self.action_wrapper = ActionWrapper()
		# Service
		self.service = rospy.Service("~start", Empty, self.callback) # Start process, until workspace is empty
		self.save_background = rospy.Service("~save_bg", Empty, self.save_cb) # Save bin background color and depth image
		self.reset_service = rospy.Service("~reset", Empty, self.reset_cb) # Reset `self.episode`, `self.run` and create new save root
		# Service client
		self.record_bag_client = rospy.ServiceProxy("/autonomous_recording_node/start_recording", recorder)
		self.stop_record_client = rospy.ServiceProxy("/autonomous_recording_node/stop_recording", Empty)
		try:
			self.camera_info = rospy.wait_for_message(self.color_topic.replace("image_raw", "camera_info"), CameraInfo, timeout=5.0)
		except rospy.ROSException:
			rospy.logerr("Can't get camera intrinsic after 5 seconds, terminate node...")
			rospy.signal_shutdown("No intrinsic")
			sys.exit(0)
		load_ts = time.time()
		rospy.loginfo("Loading model...")
		self.suck_net = Net(args.n_classes)
		self.grasp_net = Net(args.n_classes)
		self.suck_net.load_state_dict(torch.load(self.file_path+"/"+args.suck_model))
		self.grasp_net.load_state_dict(torch.load(self.file_path+"/"+args.grasp_model))
		if self.use_cuda:
			self.suck_net = self.suck_net.cuda()
			self.grasp_net = self.grasp_net.cuda()
		rospy.loginfo("Load complete, time elasped: {}".format(time.time()-load_ts))
		rospy.loginfo("current episode: \t{}".format(self.episode))
		rospy.loginfo("current run: \t{}".format(self.run))
		rospy.loginfo("Service ready")
	# encode recording index
	def encode_index(self, episode, run):
		res = 30000 + episode * 10 + run
		return res
	# Create directories for saving data
	def _create_directories(self):
		self.save_paths = [self.save_root+"/color/", # color image from camera
						   self.save_root+"/depth/", # depth image from camera
						   self.save_root+"/color_heightmap/", # converted color heightmap
						   self.save_root+"/depth_heightmap/", # converted depth heightmap
						   self.save_root+"/mixed_img/", # color image and prediction heatmap
						   self.save_root+"/pc/", # pointcloud pcd files
						   self.save_root+"/viz/"] # corresponding action and symbols
		for path in self.save_paths:
			if not os.path.exists(path):
				os.makedirs(path)
	# Save bin background color and depth image
	def save_cb(self, req):
		color_topic = rospy.wait_for_message(self.color_topic, Image)
		depth_topic = rospy.wait_for_message(self.depth_topic, Image)
		color_img = self.bridge.imgmsg_to_cv2(color_topic, desired_encoding="passthrough")
		color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
		depth_img = self.bridge.imgmsg_to_cv2(depth_topic, desired_encoding="passthrough")
		cv2.imwrite(self.file_path+"/color_background.jpg", color_img)
		cv2.imwrite(self.file_path+"/depth_background.png", depth_img)
		rospy.loginfo("Background images saved")
		return EmptyResponse()
	# Forward pass and get suck prediction
	def _suck(self, color, depth):
		fg_mask = utils.background_subtraction(color, depth, self.background_color, self.background_depth)
		color_tensor, depth_tensor = utils.preprocessing(color, depth)
		if self.use_cuda:
			color_tensor = color_tensor.cuda()
			depth_tensor = depth_tensor.cuda()
		predict = self.suck_net.forward(color_tensor, depth_tensor)
		suctionable = predict.detach().cpu().numpy()[0, 1]
		suctionable = cv2.resize(suctionable, dsize=(suctionable.shape[1]*self.scale, suctionable.shape[0]*self.scale))
		suctionable[fg_mask==0] = 0.0 # Background
		return suctionable
	# Forward pass and get grasp prediction (with number `self.grasp_angle`)
	def _grasp(self, color, depth):
		color_heightmap, depth_heightmap = utils.generate_heightmap(color, depth, self.camera_info, self.background_color, self.background_depth, self.voxel_size)
		graspable = np.zeros((self.grasp_angle, depth_heightmap.shape[1], depth_heightmap.shape[0]))
		for i in range(self.grasp_angle):
			angle = -np.degrees(np.pi/self.grasp_angle * i)
			rotated_color_heightmap, rotated_depth_heightmap = utils.rotate_heightmap(color_heightmap, depth_heightmap, angle)
			color_tensor, depth_tensor = utils.preprocessing(rotated_color_heightmap, rotated_depth_heightmap)
			if self.use_cuda:
				color_tensor = color_tensor.cuda()
				depth_tensor = depth_tensor.cuda()
			predict = self.grasp_net.forward(color_tensor, depth_tensor)
			grasp = predict.detach().cpu().numpy()[0, 1]
			affordance = cv2.resize(grasp, dsize=(grasp.shape[1]*self.scale, grasp.shape[0]*self.scale))
			affordance[depth_heightmap==0] = 0.0 # Background
			graspable[i, :, :] = affordance
		return color_heightmap, depth_heightmap, graspable
	# Start process, until workspace is empty
	def callback(self, req):
		rospy.loginfo("Receive start command")
		self.action_wrapper.reset()
		empty = False
		iter_count = 0
		# Start recording
		self.record_bag_client(recorderRequest(self.encode_index(self.episode, self.run)))
		while empty is not True:
			rospy.loginfo("Baseline method, iter: {}".format(iter_count))
			# Get color and depth images
			color_topic = rospy.wait_for_message(self.color_topic, Image)
			depth_topic = rospy.wait_for_message(self.depth_topic, Image)
			color_img = self.bridge.imgmsg_to_cv2(color_topic, desired_encoding="passthrough")
			color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
			depth_img = self.bridge.imgmsg_to_cv2(depth_topic, desired_encoding="passthrough")
			suckable = self._suck(color_img, depth_img)
			color_heightmap, depth_heightmap, graspable = self._grasp(color_img, depth_img)
			cv2.imwrite(self.save_paths[0]+"{:06}.png".format(iter_count), color_img) # color
			cv2.imwrite(self.save_paths[1]+"{:06}.png".format(iter_count), depth_img) # depth
			cv2.imwrite(self.save_paths[2]+"{:06}.png".format(iter_count), color_heightmap) # color heightmap
			cv2.imwrite(self.save_paths[3]+"{:06}.png".format(iter_count), depth_heightmap) # depth heightmap
			suck_hm = utils.viz_affordance(suckable)
			suck_combined = cv2.addWeighted(color_img, 0.8, suck_hm, 0.8, 0)
			# Multiply by primitive weight
			suckable *= self.suck_weight
			graspable *= self.grasp_weight
			cv2.imwrite(self.save_paths[4]+"suck_{:06}.png".format(iter_count), suck_combined) # mixed
			grasps_combined = []
			for i in range(self.grasp_angle):
				angle = -np.degrees(np.pi/self.grasp_angle*i)
				rotated_color_heightmap, _ = utils.rotate_heightmap(color_heightmap, depth_heightmap, angle)
				grasp_hm = utils.viz_affordance(graspable[i])
				grasp_combined = cv2.addWeighted(rotated_color_heightmap, 0.8, grasp_hm, 0.8, 0)
				cv2.imwrite(self.save_paths[4]+"/grasp_{:06}_{}.png".format(iter_count, i), grasp_combined) # mixed
				grasps_combined.append(grasp_combined)
			# Get position
			best_action = [np.max(suckable), np.max(graspable[0]), np.max(graspable[1]), np.max(graspable[2]), np.max(graspable[3])]
			print "Action value: ", best_action
			best_action_idx = np.where(best_action==np.max(best_action))[0]
			gripper_angle = 0
			targetPt = np.zeros((3, 1))
			static = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
			tool_id = 3
			will_collide = False
			if best_action_idx==0: # suck
				rospy.loginfo("Use \033[1;31msuction\033[0m")
				suck_pixel = np.where(suckable==np.max(suckable))
				suck_pixel = [suck_pixel[1][0], suck_pixel[0][0]] # x, y
				cam_z = (depth_img[suck_pixel[1], suck_pixel[0]]).astype(np.float32)/1000
				cam_x = (suck_pixel[0]-self.camera_info.K[2])*cam_z/self.camera_info.K[0]
				cam_y = (suck_pixel[1]-self.camera_info.K[5])*cam_z/self.camera_info.K[4]
				camPt = np.array([[cam_x], [cam_y], [cam_z], [1.0]])
				camera_pose = np.loadtxt(self.file_path+"/camera_pose.txt")
				cam2arm = np.matmul(static.T, camera_pose[:3])
				targetPt = np.matmul(cam2arm, camPt).reshape(3)
				action = utils.draw_action(suck_combined, suck_pixel)
				cv2.imwrite(self.save_paths[6]+"action_{:06}.png".format(iter_count), action) # viz
			else: # gripper
				tool_id = 1
				best_angle_idx = best_action_idx-1
				angle = np.degrees(np.pi/self.grasp_angle*best_angle_idx)
				gripper_angle = -angle
				rospy.loginfo("Use \033[1;31mparallel-jaw gripper with angle: {}\033[0m".format(gripper_angle))
				binMiddleBottom = np.loadtxt(self.file_path+"/bin_pose.txt")
				grasp_pixel = np.where(graspable[best_angle_idx]==np.max(graspable[best_angle_idx]))
				grasp_pixel = [grasp_pixel[1][0], grasp_pixel[0][0]] # x, y
				u = grasp_pixel[0] - graspable[best_angle_idx].shape[0]/2
				v = grasp_pixel[1] - graspable[best_angle_idx].shape[1]/2 
				tempPt = np.zeros((3, 1))
				# Position in fake link
				tempPt[0] = binMiddleBottom[0] + u * self.voxel_size # X
				tempPt[1] = binMiddleBottom[1] + v * self.voxel_size # Y
				tempPt[2] = depth_heightmap[grasp_pixel[1], grasp_pixel[0]].astype(np.float32)/1000 # Z
				targetPt = np.matmul(static, tempPt).reshape(3)
				targetPt[2] = -targetPt[2] - binMiddleBottom[2]
				action = utils.draw_action(grasps_combined[best_angle_idx], grasp_pixel, "grasp")
				if angle!=0:
					action_rotate = utils.rotate_img(action, angle)
				else: action_rotate = action
				cv2.imwrite(self.save_paths[6]+"action_{:06}.png".format(iter_count), action_rotate)
				self.action_wrapper.get_pc(self.grasp_paths[5]+"{:06}_before.png".format(iter_count))
				will_collide = self.action_wrapper.check_if_empty(self.grasp_paths[5]+"{:06}_before.pcd".format(iter_count))
			print "Target position: [{}, {}, {}]".format(targetPt[0], targetPt[1], targetPt[2])
			if will_collide:
				# TODO: add memory
				rospy.logwarn("Will collide, abort action")
				continue
			self.action_wrapper.take_action(tool_id, targetPt, np.radians(gripper_angle))
			action_success = self.action_wrapper.check_if_success(tool_id, self.save_paths[5]+"{:06}_check.pcd".format(iter_count))
			self.action_wrapper.publish_data(iter_count, best_action_idx, action_success)
			if not action_success:
				pass
				# TODO: add memory, reset
				rospy.loginfo("Action fail")
				self.action_wrapper.reset()
			else:
				rospy.loginfo("Action success")
				self.action_wrapper.place()
			self.action_wrapper.get_pc(self.save_paths[5]+"{:06}_after.pcd".format(iter_count))
			empty = self.action_wrapper.check_if_empty(self.save_paths[5]+"{:06}_after.pcd".format(iter_count))
			iter_count += 1
		self.stop_record_client()
		rospy.loginfo("Complete")
		
		return EmptyResponse()
	# Reset `self.episode`, `self.run` and create new save root
	def reset_cb(self, req):
		rospy.loginfo("current episode: \t{}".format(self.episode))
		rospy.loginfo("current run: \t{}".format(self.run))
		new_episode = int(raw_input("Input new episode: "))
		new_run = int(raw_input("Input new run: "))
		if new_episode == self.episode and new_run == self.run:
			rospy.loginfo("same value, abort request")
			return EmptyResponse()
		rospy.loginfo("episode set from {} to {}".format(self.episode, new_episode))
		rospy.loginfo("run set from {} to {}".format(self.run, new_run))
		self.episode = new_episode
		self.run = new_run
		self.save_root = self.file_path + "/exp_2/ep{}_run{}".format(self.episode, self.run)
		self._create_directories()
		
		return EmptyResponse()

def main():
	parser = argparse.ArgumentParser(prog="baseline", description="method from arc2017 MIT-Princeton method")
	parser.add_argument("episode", type=int, help="Which placing method is this test")
	parser.add_argument("run", type=int, help="Which run is this test")
	parser.add_argument("--n_classes", type=int, default=3, help="number of class for classification")
	parser.add_argument("--scale", type=int, default=8, help="scale after prediction")
	parser.add_argument("--grasp_angle", type=int, default=4, help="how many grasping angle needed")
	parser.add_argument("--voxel_size", type=float, default=0.002, help="voxel size in meter for height map")
	parser.add_argument("--use_cuda", type=bool, default=True, help="if use GPU")
	parser.add_argument("--suck_model", type=str, default="sucknet.pth", help="suck net model path")
	parser.add_argument("--grasp_model", type=str, default="graspnet.pth", help="grasp net model path")
	parser.add_argument("--color_bg", type=str, default="color_background.jpg", help="color background path")
	parser.add_argument("--depth_bg", type=str, default="depth_background.png", help="depth background path")
	parser.add_argument("--color_topic", type=str, default="/camera1/color/image_raw")
	parser.add_argument("--depth_topic", type=str, default="/camera1/aligned_depth_to_color/image_raw")
	args = parser.parse_args()
	d = vars(args)
	for key in d:
		print "{}: \t {}".format(key, d[key])
	rospy.init_node("baseline")
	p = Process(args)
	rospy.spin()
	
if __name__ == "__main__":
	main()