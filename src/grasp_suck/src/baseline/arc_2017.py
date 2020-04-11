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
import utils
from utils import Net

# TODO: add memory for action

class Process(object):
	def __init__(self, args):
		self.file_path = os.path.dirname(os.path.abspath(__file__))
		self.use_cuda = args.use_cuda
		self.scale = args.scale
		self.grasp_angle = args.grasp_angle
		self.voxel_size = args.voxel_size
		self.suck_weight = 1.0
		self.grasp_weight = 0.25
		self.count = 0
		self.bridge = CvBridge()
		self.background_color = self.file_path + "/" + args.color_bg
		self.background_depth = self.file_path + "/" + args.depth_bg
		self.service = rospy.Service("~get_action", Empty, self.callback)
		self.save_background = rospy.Service("~save_bg", Empty, self.save_cb)
		self.color_topic = args.color_topic
		self.depth_topic = args.depth_topic
		self.camera_info = rospy.wait_for_message(self.color_topic.replace("image_raw", "camera_info"), CameraInfo)
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
		rospy.loginfo("Service ready")
	# Save bin background
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
			predict = self.grasp_net.forward(depth_tensor, depth_tensor)
			grasp = predict.detach().cpu().numpy()[0, 1]
			affordance = cv2.resize(grasp, dsize=(grasp.shape[1]*self.scale, grasp.shape[0]*self.scale))
			affordance[depth_heightmap==0] = 0.0 # Background
			graspable[i, :, :] = affordance
		return color_heightmap, depth_heightmap, graspable
		
	def callback(self, req):
		rospy.loginfo("Receive")
		# Get color and depth images
		color_topic = rospy.wait_for_message(self.color_topic, Image)
		depth_topic = rospy.wait_for_message(self.depth_topic, Image)
		color_img = self.bridge.imgmsg_to_cv2(color_topic, desired_encoding="passthrough")
		color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
		depth_img = self.bridge.imgmsg_to_cv2(depth_topic, desired_encoding="passthrough")
		suckable = self._suck(color_img, depth_img)
		color_heightmap, depth_heightmap, graspable = self._grasp(color_img, depth_img)
		save_root = self.file_path + "/predict_{:06}".format(self.count)
		if not os.path.exists(save_root):
			os.makedirs(save_root)
		cv2.imwrite(save_root+"/color.png", color_img)
		cv2.imwrite(save_root+"/depth.png", depth_img)
		cv2.imwrite(save_root+"/color_heightmap.png", color_heightmap)
		cv2.imwrite(save_root+"/depth_heightmap.png", depth_heightmap)
		suck_hm = utils.viz_affordance(suckable)
		suck_combined = cv2.addWeighted(color_img, 0.8, suck_hm, 0.8, 0)
		# Multiply by primitive weight
		suckable *= self.suck_weight
		graspable *= self.grasp_weight
		cv2.imwrite(save_root+"/suck.png", suck_combined)
		grasps_combined = []
		for i in range(self.grasp_angle):
			angle = -np.degrees(np.pi/self.grasp_angle*i)
			rotated_color_heightmap, _ = utils.rotate_heightmap(color_heightmap, depth_heightmap, angle)
			grasp_hm = utils.viz_affordance(graspable[i])
			grasp_combined = cv2.addWeighted(rotated_color_heightmap, 0.8, grasp_hm, 0.8, 0)
			cv2.imwrite(save_root+"/grasp_{}.png".format(i), grasp_combined)
			grasps_combined.append(grasp_combined)
		# Get position
		best_action = [np.max(suckable), np.max(graspable[0]), np.max(graspable[1]), np.max(graspable[2]), np.max(graspable[3])]
		print "Action value: ", best_action
		best_action_idx = np.where(best_action==np.max(best_action))[0]
		gripper_angle = 0
		targetPt = np.zeros((3, 1))
		static = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
		if best_action_idx==0: # suck
			rospy.loginfo("Use \033[1;31msuction\033[0m")
			suck_pixel = np.where(suckable==np.max(suckable))
			suck_pixel = [suck_pixel[1][0], suck_pixel[0][0]] # x, y
			cam_z = (depth_img[suck_pixel[0], suck_pixel[1]]).astype(np.float32)/1000
			cam_x = (suck_pixel[0]-self.camera_info.K[2])*cam_z/self.camera_info.K[0]
			cam_y = (suck_pixel[1]-self.camera_info.K[5])*cam_z/self.camera_info.K[4]
			camPt = np.array([[cam_x], [cam_y], [cam_z], [1.0]])
			camera_pose = np.loadtxt(self.file_path+"/camera_pose.txt")
			
			cam2arm = np.matmul(static.T, camera_pose[:3])
			targetPt = np.matmul(cam2arm, camPt)
			action = utils.draw_action(suck_combined, suck_pixel)
			cv2.imwrite(save_root+"/action.png", action)
		else:
			best_angle_idx = best_action_idx-1
			angle = np.degrees(np.pi/self.grasp_angle*best_angle_idx)
			gripper_angle = -angle
			rospy.loginfo("Use \033[1;31mparallel-jaw gripper with angle: {}\033[0m".format(gripper_angle))
			binMiddleBottom = np.loadtxt(self.file_path+"/bin_pose.txt")
			grasp_pixel = np.where(graspable[best_angle_idx]==np.max(graspable[best_angle_idx]))
			grasp_pixel = [grasp_pixel[1][0], grasp_pixel[0][0]]
			u = graspable[best_angle_idx].shape[1]/2 - grasp_pixel[1]
			v = graspable[best_angle_idx].shape[0]/2 - grasp_pixel[0]
			tempPt = np.zeros((3, 1))
			# Position in fake link
			tempPt[0] = binMiddleBottom[0] + u * self.voxel_size # X
			tempPt[1] = binMiddleBottom[1] + v * self.voxel_size # Y
			tempPt[2] = depth_heightmap[grasp_pixel[1], grasp_pixel[0]].astype(np.float32)/1000 # Z
			targetPt = np.matmul(static, tempPt)
			targetPt[2] = -targetPt[2] - binMiddleBottom[2]
			action = utils.draw_action(grasps_combined[best_angle_idx], grasp_pixel, "grasp")
			if angle!=0:
				action_rotate = utils.rotate_img(action, angle)
			else: action_rotate = action
			cv2.imwrite(save_root+"/action.png", action_rotate)
			
		print "Target position: [{}, {}, {}]".format(targetPt[0][0], targetPt[1][0], targetPt[2][0])
		viz_client = rospy.ServiceProxy("/viz_marker_node/viz_marker", viz_marker)
		viz_request = viz_markerRequest()
		viz_request.point.x = targetPt[0]
		viz_request.point.y = targetPt[1]
		viz_request.point.z = targetPt[2]
		viz_request.angle = np.radians(gripper_angle)
		viz_client(viz_request)
		rospy.loginfo("Complete")
		self.count+=1
		return EmptyResponse()

def main():
	parser = argparse.ArgumentParser(prog="baseline", description="method from arc2017 MIT-Princeton method")
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
