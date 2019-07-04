import os
import time
from threading import Thread, currentThread
import numpy as np
import cv2
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
# Parameter
testing      = False
use_cpu      = True
episode      = 0
iteration    = 0
suck_reward  = 1.0
grasp_reward = 1.0
discount     = 0.5
epsilon      = 0.3
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
		suck_predictions, grasp_predictions, state_feat = \
                          trainer.forward(color, depth, is_volatile=True)
		print "Forward pass: ", (time.time() - ts)
		# Convert to heatmap
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
		pixel_index = []
		print "suck max: {}| grasp max: {}".format(np.max(suck_predictions), \
                                                   np.max(grasp_predictions)/3) # XXX: /3? 
		# TODO: explore
		if np.max(suck_predictions) > np.max(grasp_predictions)/3:
			action = 1 # SUCK
			action_str = 'suck'
			tmp = np.where(suck_predictions == np.max(suck_predictions))
			pixel_index = [tmp[0][0], tmp[2][0], tmp[1][0]]
		
		else:
			tmp = np.where(grasp_predictions == np.max(grasp_predictions))
			pixel_index = [tmp[0][0], tmp[2][0], tmp[1][0]]
			theta = np.radians(float(pixel_index[0])/8 * 360)
		del suck_predictions, grasp_predictions, state_feat
		set_prior_img()
		getXYZ = get_xyzRequest()
		getXYZ.point[0] = pixel_index[1] # X
		getXYZ.point[1] = pixel_index[2] # Y
		getXYZResult = pixel_to_xyz(getXYZ)
	
		if getXYZResult.result.x == 0.0 and \
		   getXYZResult.result.y == 0.0 and \
		   getXYZResult.result.z == 0.0:
			print "Invalid pointcloud, ignore..."
			continue
		
		gotoTarget = get_poseRequest()
		gotoTarget.point_in_cam = getXYZResult.result
		gotoTarget.yaw = angle
		gotoTarget.primmitive = action
		
		goto_target(gotoTarget)
		time.sleep(0.5)
		go_home() # TODO: service maybe block
		set_posterior_img()
		action_success = get_result(SetBoolRequest()).success
		if not action: # GRASP
			action_success &= grasp_state().success
		
		if action_success:
			go_place()
			num_of_items -= 1
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
		print "Get label: ", (time.time()-ts)
		ts = time.time()
		trainer.backprop(color, depth, action_str, pixel_index, label_value)
		print "backpropagation time: ", (time.time() - ts)
		iteration += 1
		if iteration % save_every == 0:
			model_name = model_path + "iter_{:06}.pth".format(iteration)
			torch.save(trainer.model.state_dict(), model_name)
		iter_time = time.time() - iter_ts
		total_time += iter_time
		print "[Total time] {} s| [Iter time] {} s".format(total_time, iter_time)
		time.sleep(0.5) # Sleep 0.5 s for next iteration
except KeyboardInterrupt:
	print "Force terminate"
