import os
import sys
import time
import numpy as np
import argparse
import rospy
import rospkg
import torch
import utils
from trainer import Trainer
from std_srvs.srv import Empty, SetBool, SetBoolRequest, SetBoolResponse, Trigger
from grasp_suck.srv import get_pose, get_poseRequest, get_poseResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse

parser = argparse.ArgumentParser(prog="ict_demo", description="For ICT demo")
parser.add_argument("--model", type=str, help="Network model", required=True)
parser.add_argument("--update_target", type=int, default=5, help="Freuqency to update target network")
args = parser.parse_args()

# Constant
run          = 0
suck_reward  = 2.0
grasp_reward = 2.0
discount     = 0.5
Z_THRES      = 0.016 # z value less than this value will be considered as invalid
update_fre = args.update_target

# Directory
r = rospkg.RosPack()
package_path = r.get_path('grasp_suck')
logger_dir = package_path + "/ict_demo/"
image_dir = logger_dir + "images/"
pc_path    = logger_dir + "pc/"

if not os.path.exists(logger_dir):
	os.makedirs(logger_dir)
if not os.path.exists(image_dir):
	os.makedirs(image_dir)
if not os.path.exists(pc_path):
	os.makedirs(pc_path)

# trainer
trainer = Trainer(suck_reward, grasp_reward, discount, True, False)
# Still using small learning rate to backpropagate when testing
for param_group in trainer.optimizer.param_groups:
	param_group['lr'] = 1e-5
	
print "[%f]: Loading provided model..." %(time.time())
trainer.model.load_state_dict(torch.load(args.model))

# Service client
# Gripper
open_gripper      = rospy.ServiceProxy('/robotiq_finger_control_node/open_gripper', Empty)
grasp_state       = rospy.ServiceProxy('/robotiq_finger_control_node/get_grasp_state', Trigger)
initial_gripper   = rospy.ServiceProxy('/robotiq_finger_control_node/initial_gripper', Empty)
suck_state        = rospy.ServiceProxy('/arduino_control/check_suck_success', SetBool)

# Helper
goto_target = rospy.ServiceProxy('/helper_services_node/goto_target', get_pose)
go_home     = rospy.ServiceProxy('/helper_services_node/robot_go_home', Empty)
go_place    = rospy.ServiceProxy('/helper_services_node/robot_goto_place', Empty)

# Visual system
get_pc_client = rospy.ServiceProxy('/pc_transform/get_pc', get_pc)
empty_checker = rospy.ServiceProxy('/pc_transform/empty_state', pc_is_empty)

# Visualization
viz = rospy.ServiceProxy('/viz_marker_node/viz_marker', viz_marker)

print "[%f] Initializing..." %(time.time())
go_home()
initial_gripper()
open_gripper()

while 1:
	command = raw_input("\n\nPress 's' to start, 'e' to exit:\n\n")
	if command == 'e':
		break
	elif command == 's':
		print "Run: %d" %(run)
		image_path = image_dir + "run_{}/".format(run)
		if not os.path.exists(image_path):
			os.makedirs(image_path)
		is_empty = False
		total_time = 0.0
		iteration = 0
		# Processing
		while not is_empty:
			iter_ts = time.time()
			get_pc_req = get_pcRequest()
			get_pc_req.file_name = pc_path + "/{}_{}_before.pcd".format(run, iteration)
			pc_response = get_pc_client(get_pc_req)
			color, depth, points, depth_img = utils.get_heightmap(pc_response.pc, image_path, iteration)
			ts = time.time()
			print "[%f]: Forward pass..." %(time.time()), 
			sys.stdout.write('')
			suck_predictions, grasp_predictions, state_feat = \
                              trainer.forward(color, depth, is_volatile=True)
			print " {} seconds".format(time.time() - ts)
			# Standardization
			suck_predictions, grasp_predictions = utils.standarization(suck_predictions, grasp_predictions)
			# Choose action, using grasp-only policy
			action = 0
			action_str = 'grasp'
			pixel_index, angle = utils.grasp_only_policy(grasp_predictions)
			del suck_predictions, grasp_predictions, state_feat
			print "[%f]: Take action: \033[0;31m %s\033[0m at \
\033[0;32m(%d, %d)\033[0m with theta \033[0;33m%f \033[0m" %(time.time(), action_str, pixel_index[1], \
                                                             pixel_index[2], angle)
			# Check if invalid
			is_valid = True
			# Invalid conditions:
			# 1. NaN point
			# 2. Point with z too far, which presents the plane of convayor
			print "###### [%f, %f, %f] ######" %(points[pixel_index[1], pixel_index[2], 0], points[pixel_index[1], pixel_index[2], 1], points[pixel_index[1], pixel_index[2], 2])
			if np.isnan(points[pixel_index[1], pixel_index[2], 0]) or \
			   np.isnan(points[pixel_index[1], pixel_index[2], 1]) or \
			   np.isnan(points[pixel_index[1], pixel_index[2], 2]) or \
			   points[pixel_index[1], pixel_index[2], 0] == 0.0 or \
			   points[pixel_index[1], pixel_index[2], 1] == 0.0:
				print "\033[0;31;44mInvalid pointcloud. Ignore...\033[0m"
				is_valid = False
			elif points[pixel_index[1], pixel_index[2], 2] <= Z_THRES:
				print "\033[0;31;44mTarget on converyor! Ignore...\033[0m"
				is_valid = False
			# Visualize marker
			viz_req = viz_markerRequest()
			viz_req.point.x = points[pixel_index[1], pixel_index[2], 0]
			viz_req.point.y = points[pixel_index[1], pixel_index[2], 1]
			viz_req.point.z = points[pixel_index[1], pixel_index[2], 2]
			viz_req.primitive = action
			viz_req.angle = angle
			viz_req.valid = is_valid
			viz(viz_req)
			# Take action
			if is_valid:
				gotoTarget = get_poseRequest()
				gotoTarget.point_in_hand.x = points[pixel_index[1], pixel_index[2], 0]
				gotoTarget.point_in_hand.y = points[pixel_index[1], pixel_index[2], 1]
				gotoTarget.point_in_hand.z = points[pixel_index[1], pixel_index[2], 2]
				gotoTarget.yaw = angle
				gotoTarget.primmitive = action
				goto_target(gotoTarget)
				time.sleep(0.5)
				go_home()
			# Get action result
			get_pc_req.file_name = pc_path + "/{}_after.pcd".format(iteration)
			next_pc = get_pc_client(get_pc_req).pc
			next_color, next_depth, next_points, next_depth_img = utils.get_heightmap(next_pc, image_path + "next_", iteration)
			if is_valid: action_success = grasp_state().success
			else: action_success = False
			if action_success: go_place()
			else: open_gripper()
			ts = time.time()
			empty_checker_req = pc_is_emptyRequest()
			empty_checker_req.input_pc = next_pc
			is_empty = empty_checker(empty_checker_req).is_empty.data
			if is_valid:
				label_value, prev_reward_value = trainer.get_label_value(action_str, action_success, \
		                                                             next_color, next_depth, is_empty)
			else: # invalid
				label_value, prev_reward_value = trainer.get_label_value("invalid", False, next_color, next_depth, is_empty)
			print "Get label: {} seconds".format(time.time()-ts)
			ts = time.time()
			loss_value = trainer.backprop(color, depth, action_str, pixel_index, label_value)
			print "Backpropagation {} seconds".format(time.time() - ts)
			iter_time = time.time() - iter_ts
			total_time += iter_time
			print "[Total time] \033[0;31m{}\033[0m s| [Iter time] \033[0;32m{}\033[0m s".format \
	                                                                        (total_time, iter_time)
			iteration += 1
			# Update target net
			if iteration % update_fre == 0:
				print "Update target network"
				trainer.target = trainer.model
			time.sleep(0.5) # Sleep 0.5 s for next iteration
			get_pc_req.file_name = str()
			empty_checker_req.input_pc = get_pc_client(get_pc_req).pc
			is_empty = empty_checker(empty_checker_req).is_empty.data
		run += 1
