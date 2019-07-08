import os
import sys
import time
#from threading import Thread, currentThread
import numpy as np
import cv2
import argparse
import torch
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from trainer import Trainer
# SRV
from std_srvs.srv import Empty, SetBool, SetBoolRequest, SetBoolResponse, \
                         Trigger, TriggerRequest, TriggerResponse
from grasp_suck.srv import get_pose, get_poseRequest, get_poseResponse
from visual_system.srv import get_image, get_imageRequest, get_imageResponse, \
                              get_xyz, get_xyzRequest, get_xyzResponse
from vacuum_conveyor_control.srv import vacuum_control, vacuum_controlRequest

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="using color and depth images to do pick and place")
parser.add_argument("--is_testing", action="store_true", default=False)
parser.add_argument("--force_cpu",  action="store_true", default=False)
parser.add_argument("--last_model", type=str, default="", help="If provided, continue training with provided model file")

args = parser.parse_args()

# Parameter
testing      = args.is_testing
use_cpu      = args.force_cpu
episode      = 0
iteration    = 0
suck_reward  = 1.0
grasp_reward = 1.0
discount     = 0.5
Z_THRES      = 0.645
num_of_items = 10
save_every   = 5


# Logger path
r = rospkg.RosPack()
package_path = r.get_path('grasp_suck')
csv_path   = package_path + '/logger/'
image_path = package_path + '/logger/images/'
feat_path  = package_path + '/logger/feat/'
model_path = package_path + '/logger/models/'

if not os.path.exists(image_path):
	os.makedirs(image_path)
if not os.path.exists(feat_path):
	os.makedirs(feat_path)
if not os.path.exists(model_path):
	os.makedirs(model_path)

# cv bridge
br = CvBridge()

# trainer
trainer = Trainer(suck_reward, grasp_reward, discount, testing, use_cpu)

# Load model if provided last model
if args.last_model != "":
	print "Loading provided model..."
	trainer.model.load_state_dict(torch.load(args.last_model))

# Service client
# Gripper
open_gripper      = rospy.ServiceProxy('/robotiq_finger_control_node/open_gripper', Empty)
grasp_state       = rospy.ServiceProxy('/robotiq_finger_control_node/get_grasp_state', Trigger)
pheumatic         = rospy.ServiceProxy('/arduino_control/pheumatic_control', SetBool)
vacuum            = rospy.ServiceProxy('/arduino_control/vacuum_control', vacuum_control)

# Get reward
set_prior_img     = rospy.ServiceProxy('/get_reward/set_prior', Empty)
set_posterior_img = rospy.ServiceProxy('/get_reward/set_posterior', Empty)
get_result        = rospy.ServiceProxy('/get_reward/get_result', SetBool)

# Helper
goto_target = rospy.ServiceProxy('/helper_services_node/goto_target', get_pose)
go_home     = rospy.ServiceProxy('/helper_services_node/robot_go_home', Empty)
go_place    = rospy.ServiceProxy('/helper_services_node/robot_goto_place', Empty)

# pixel_to_xyz
pixel_to_xyz = rospy.ServiceProxy('/pixel_to_xyz/pixel_to_xyz', get_xyz)
get_image    = rospy.ServiceProxy('/pixel_to_xyz/get_image', get_image)

def get_imgs_from_msg(response):
	color = br.imgmsg_to_cv2(response.crop_color_img)
	depth = br.imgmsg_to_cv2(response.crop_depth_img)
	return color, depth

print "Initializing..."
go_home()
open_gripper()
pheumatic(SetBoolRequest(False))
vacuum(vacuum_controlRequest(0))

total_time = 0.0

try:
	while num_of_items != 0:
		print "\033[0;31;46mIteration: {}\033[0m".format(iteration)
		epsilon = max(0.5 * np.power(0.9998, iteration), 0.1)
		iter_ts = time.time()	
		# Get color and depth images
		images = get_image()
		# Convert to cv2
		color, depth = get_imgs_from_msg(images)
		# Save images
		color_name = image_path + "color_{:06}.jpg".format(iteration)
		depth_name = image_path + "depth_{:06}.png".format(iteration)
		cv2.imwrite(color_name, color)
		cv2.imwrite(depth_name, depth)
		ts = time.time()
		print "Forward pass...", 
		sys.stdout.write('')
		suck_predictions, grasp_predictions, state_feat = \
                          trainer.forward(color, depth, is_volatile=True)
		print "  {} seconds".format(time.time() - ts)
		# Convert feature to heatmap and save
		tmp = ((suck_predictions-np.min(suck_predictions))/np.max(suck_predictions)* \
                                                255).astype(np.uint8).transpose(1, 2, 0)
		suck_heatmap = cv2.applyColorMap(tmp, cv2.COLORMAP_JET)
		suck_name = feat_path + "suck_{:06}.jpg".format(iteration)
		cv2.imwrite(suck_name, suck_heatmap)
		for rotate_idx in range(len(grasp_predictions)):
			tmp = ((grasp_predictions[rotate_idx]-np.min(grasp_predictions[rotate_idx]))/ \
                              np.max(grasp_predictions[rotate_idx])*255).astype(np.uint8)
			tmp.shape = (tmp.shape[0], tmp.shape[1], 1)
			grasp_heatmap = cv2.applyColorMap(tmp, cv2.COLORMAP_JET)
			grasp_name = feat_path + "grasp_{:06}_idx_{}.jpg".format(iteration, rotate_idx)
			cv2.imwrite(grasp_name, grasp_heatmap)
		action = 0 # GRASP
		action_str = 'grasp'
		angle = 0
		pixel_index = [] # rotate_idx, x, y
		print "suck max: \033[0;34m{}\033[0m| grasp max: \033[0;35m{}\033[0m".format(np.max(suck_predictions), \
                                                   np.max(grasp_predictions))
		explore = np.random.uniform() < epsilon
		if np.max(suck_predictions) > np.max(grasp_predictions):
			if not explore:
				action = 1 # SUCK
				action_str = 'suck'
				tmp = np.where(suck_predictions == np.max(suck_predictions))
				pixel_index = [tmp[0][0], tmp[2][0], tmp[1][0]]
		
			else:
				tmp = np.where(grasp_predictions == np.max(grasp_predictions))
				pixel_index = [tmp[0][0], tmp[2][0], tmp[1][0]]
				angle = np.radians(-90+45*pixel_index[0])
		else:
			if not explore:
				tmp = np.where(grasp_predictions == np.max(grasp_predictions))
				pixel_index = [tmp[0][0], tmp[2][0], tmp[1][0]]
				angle = np.radians(-90+45*pixel_index[0])
			else:
				action = 1 # SUCK
				action_str = 'suck'
				tmp = np.where(suck_predictions == np.max(suck_predictions))
				pixel_index = [tmp[0][0], tmp[2][0], tmp[1][0]]
		del suck_predictions, grasp_predictions, state_feat
		print "Take action: \033[0;31m %s\033[0m at \
\033[0;32m(%d, %d)\033[0m with theta \033[0;33m%f \033[0m" %(action_str, pixel_index[1], \
                                                             pixel_index[2], angle)
		set_prior_img()
		getXYZ = get_xyzRequest()
		getXYZ.point[0] = pixel_index[1] # X
		getXYZ.point[1] = pixel_index[2] # Y
		getXYZResult = pixel_to_xyz(getXYZ)
		
		is_valid = True
		# Invalid conditions:
		# 1. NaN point
		# 2. Point with z too far, which presents the plane of convayor
		# TODO 3. Gripper collision with object
		if getXYZResult.result.x == 0.0 and \
		   getXYZResult.result.y == 0.0 and \
		   getXYZResult.result.z == 0.0:
			print "\033[0;31;44mInvalid pointcloud. Ignore...\033[0m"
			is_valid = False
			
		if getXYZResult.result.z >= Z_THRES:
			print "\033[0;31;44mTarget on converyor! Ignore...\033[0m"
			is_valid = False
		
		if is_valid:
			gotoTarget = get_poseRequest()
			gotoTarget.point_in_cam = getXYZResult.result
			gotoTarget.yaw = angle
			gotoTarget.primmitive = action
		
			goto_target(gotoTarget)
			time.sleep(0.5)

		go_home() # TODO: service maybe block
		#set_posterior_img()
		if is_valid:
			if not action: # GRASP
				action_success = grasp_state().success
			else: # SUCK
				action_success = get_result(SetBoolRequest()).success
		else:
			action_success = False
		
		if action_success:
			go_place()
			num_of_items -= 1
			print "%d items remain..." % num_of_items
		else:
			if not action: # GRASP
				open_gripper()
			else:
				pheumatic(SetBoolRequest(False))
				vacuum(vacuum_controlRequest(0))
	
		# Get images after action
		images = get_image()
		next_color, next_depth = get_imgs_from_msg(images)
		# Save images
		color_name = image_path + "next_color_{:06}.jpg".format(iteration)
		depth_name = image_path + "next_depth_{:06}.png".format(iteration)
		cv2.imwrite(color_name, next_color)
		cv2.imwrite(depth_name, next_depth)
		ts = time.time()
		label_value, prev_reward_value = trainer.get_label_value(action_str, action_success, \
		                                                         next_color, next_depth)
		print "Get label: {} seconds".format(time.time()-ts)
		ts = time.time()
		trainer.backprop(color, depth, action_str, pixel_index, label_value)
		print "Backpropagation {} seconds".format(time.time() - ts)
		iteration += 1
		if iteration % save_every == 0:
			model_name = model_path + "iter_{:06}.pth".format(iteration)
			torch.save(trainer.model.state_dict(), model_name)
			print "Model: %s saved" %model_name
		iter_time = time.time() - iter_ts
		total_time += iter_time
		print "[Total time] \033[0;31m{}\033[0m s| [Iter time] \033[0;32m{}\033[0m s".format \
                                                                     (total_time, iter_time)
		time.sleep(0.5) # Sleep 0.5 s for next iteration
except KeyboardInterrupt:
	print "Force terminate"
	model_name = model_path + "force_terminate_{}.pth".format(iteration)
	print "Model: %s saved" % model_name
	torch.save(trainer.model.state_dict(), model_name)
