import os
import sys
import time
import numpy as np
import cv2
import argparse
import torch
import rospy
import rospkg
import utils
from threading import Thread
from cv_bridge import CvBridge, CvBridgeError
# Network
from trainer import Trainer
# SRV
from std_srvs.srv import Empty, SetBool, SetBoolRequest, SetBoolResponse, \
                         Trigger, TriggerRequest, TriggerResponse
from grasp_suck.srv import get_pose, get_poseRequest, get_poseResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              get_xyz, get_xyzRequest, get_xyzResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse
from vacuum_conveyor_control.srv import vacuum_control, vacuum_controlRequest
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="pick and place by trial and error using DQN")
parser.add_argument("--is_testing", action="store_true",  default=False, help="True if testing, default is false")
parser.add_argument("--force_cpu",  action="store_true",  default=False, help="True if using CPU, default is false")
parser.add_argument("--grasp_only", action="store_true",  default=False, help="True if using grasp-only policy, default is false")
parser.add_argument("--model",         type=str,          default="",    help="If provided, continue training the model or using this model for testing, default is empty string")
parser.add_argument("--episode",       type=int,          default=0,     help="Which episode is this run, default is 0")
parser.add_argument("--num_of_items",  type=int,          default=10,     help="Number of items in the workspace, default is 10")
parser.add_argument("--epsilon",       type=float,        default=0.7,   help="Probability to do random action, default is 0.7")
parser.add_argument("--update_target", type=int,          default=3,    help="After how much iterations should update target network")
args = parser.parse_args()

utils.show_args(args)

# Parameter
testing      = args.is_testing
grasp_only   = args.grasp_only
use_cpu      = args.force_cpu
epsilon      = None
iteration    = 0
episode      = args.episode
# Reward
suck_reward  = 2.0
grasp_reward = 2.0
discount     = 0.5
Z_THRES      = 0.015 # z value less than this value will be considered as invalid
num_of_items = args.num_of_items  # Number of items when start
init_num     = num_of_items
cnt_invalid  = 0 # Consecutive invalid action counter
update_fre = args.update_target

if testing:
	print "########TESTING MODE########"
	
else:
	epsilon = args.epsilon

if args.model == "" and testing:# TEST SHOULD PROVIDE MODEL
	print "\033[0;31mNo model provided, exist!\033[0m"
	os._exit(0)

# Logger path
r = rospkg.RosPack()
package_path = r.get_path('grasp_suck')
if not testing: logger_dir = '/training/logger_{}/'.format(episode)
else: logger_dir = '/test/logger_{}/'.format(episode)
csv_path   =  package_path + logger_dir
image_path =  package_path + logger_dir + 'images/'
mixed_path =  package_path + logger_dir + 'mixed_img/'
feat_path  =  package_path + logger_dir + 'feat/'
pc_path    =  package_path + logger_dir + 'pc/'
grasp_path = [feat_path + 'rotate_idx_0/',
              feat_path + 'rotate_idx_1/',
              feat_path + 'rotate_idx_2/',
              feat_path + 'rotate_idx_3/',
              feat_path + 'rotate_idx_4/']
model_path =  package_path + logger_dir + 'models/'
vis_path   =  package_path + logger_dir + 'vis/'

if not os.path.exists(image_path):
	os.makedirs(image_path)
if not os.path.exists(feat_path):
	os.makedirs(feat_path)
if not os.path.exists(model_path) and not testing: # Test will not save models
	os.makedirs(model_path)
if not os.path.exists(mixed_path):
	os.makedirs(mixed_path)
if not os.path.exists(vis_path):
	os.makedirs(vis_path)
for i in range(5):
	if not os.path.exists(grasp_path[i]):
		os.makedirs(grasp_path[i])
if not os.path.exists(pc_path):
	os.makedirs(pc_path)

# trainer
trainer = Trainer(suck_reward, grasp_reward, discount, testing, use_cpu)
# Still using small learning rate to backpropagate when testing
if testing:
	for param_group in trainer.optimizer.param_groups:
		param_group['lr'] = 1e-5

# Load model if provided last model
if args.model != "":
	print "[%f]: Loading provided model..." %(time.time())
	trainer.model.load_state_dict(torch.load(args.model))


# Service client
# Gripper
open_gripper      = rospy.ServiceProxy('/robotiq_finger_control_node/open_gripper', Empty)
grasp_state       = rospy.ServiceProxy('/robotiq_finger_control_node/get_grasp_state', Trigger)
initial_gripper   = rospy.ServiceProxy('/robotiq_finger_control_node/initial_gripper', Empty)
pheumatic         = rospy.ServiceProxy('/arduino_control/pheumatic_control', SetBool)
vacuum            = rospy.ServiceProxy('/arduino_control/vacuum_control', vacuum_control)
suck_state        = rospy.ServiceProxy('/arduino_control/check_suck_success', SetBool)

# Get reward
#get_result         = rospy.ServiceProxy('/get_reward/get_result', get_result)

# Helper
goto_target = rospy.ServiceProxy('/helper_services_node/goto_target', get_pose)
go_home     = rospy.ServiceProxy('/helper_services_node/robot_go_home', Empty)
go_place    = rospy.ServiceProxy('/helper_services_node/robot_goto_place', Empty)

# pixel_to_xyz
get_pc_client = rospy.ServiceProxy('/pc_transform/get_pc', get_pc)
empty_checker = rospy.ServiceProxy('/pc_transform/empty_state', pc_is_empty)

# Visualization
viz = rospy.ServiceProxy('/viz_marker_node/viz_marker', viz_marker)

# Result list
action_list  = []
target_list  = []
result_list  = []
loss_list    = []
explore_list = []
return_      = 0.0
# Result file name
action_file  = csv_path + "action_primitive.csv"
target_file  = csv_path + "action_target.csv"
result_file  = csv_path + "action_result.csv"
loss_file    = csv_path + "loss.csv"
explore_file = csv_path + "explore.csv"
curve_file   = csv_path + "curve.txt"

f = open(curve_file, "w")

print "Initializing..."
go_home()
initial_gripper()
open_gripper()
pheumatic(SetBoolRequest(False))
vacuum(vacuum_controlRequest(0))

is_empty = False
total_time = 0.0

try:
	while is_empty is not True:
	#while num_of_items is not 0:
		print "\033[0;31;46mIteration: {}\033[0m".format(iteration)
		if not testing: epsilon_ = max(epsilon * np.power(0.9998, iteration), 0.2)
		iter_ts = time.time()
		get_pc_req = get_pcRequest()
		get_pc_req.file_name = pc_path + "/{}_before.pcd".format(iteration)
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
		# Convert feature to heatmap and save
		suck_heatmap = utils.vis_affordance(suck_predictions[0])
		suck_name = feat_path + "suck_{:06}.jpg".format(iteration)
		cv2.imwrite(suck_name, suck_heatmap)
		suck_mixed = cv2.addWeighted(color, 1.0, suck_heatmap, 0.4, 0)
		suck_name = mixed_path + "suck_{:06}.jpg".format(iteration)
		cv2.imwrite(suck_name, suck_mixed)
		grasp_mixed = []
		for rotate_idx in range(len(grasp_predictions)):
			grasp_heatmap = utils.vis_affordance(grasp_predictions[rotate_idx])
			grasp_name = grasp_path[rotate_idx] + "grasp_{:06}.jpg".format(iteration, rotate_idx)
			cv2.imwrite(grasp_name, grasp_heatmap)
			grasp_mixed_idx = cv2.addWeighted(color, 1.0, grasp_heatmap, 0.4, 0)
			grasp_mixed.append(grasp_mixed_idx)
			grasp_name = mixed_path + "grasp_{:06}_idx_{}.jpg".format(iteration, rotate_idx)
			cv2.imwrite(grasp_name, grasp_mixed_idx)
		print "[{:.6f}]: suck max: \033[0;34m{}\033[0m| grasp max: \033[0;35m{}\033[0m".format(time.time(), \
                                                   np.max(suck_predictions), np.max(grasp_predictions))
		explore = -1 # None
		# Policy decider
		if not testing: # Train
			explore, action, action_str, pixel_index, angle = \
				utils.epsilon_greedy_policy(epsilon_, suck_predictions, grasp_predictions)
		if testing: # Test
			if not grasp_only:
				action, action_str, pixel_index, angle = utils.greedy_policy(suck_predictions, grasp_predictions)
			else: # Grasp-only
				action = 0
				action_str = 'grasp'
				pixel_index, angle = utils.grasp_only_policy(grasp_predictions)
		explore_list.append(explore)
		if explore == 1: print "Use exploring..."
		del suck_predictions, grasp_predictions, state_feat
		print "[%f]: Take action: \033[0;31m %s\033[0m at \
\033[0;32m(%d, %d)\033[0m with theta \033[0;33m%f \033[0m" %(time.time(), action_str, pixel_index[1], \
                                                             pixel_index[2], angle)
		# Draw color + heatmap + motion
		visual_img = None
		if action: # SUCK
			visual_img = utils.draw_image(suck_mixed, action, pixel_index)
		else: # GRASP
			visual_img = utils.draw_image(grasp_mixed[pixel_index[0]], action, pixel_index)
		vis_name = vis_path + "vis_{:06}.jpg".format(iteration)
		cv2.imwrite(vis_name, visual_img)

		is_valid = True
		# Invalid conditions:
		# 1. NaN point
		# 2. Point with z too far, which presents the plane of convayor
		# TODO 3. Gripper collision with object
		print "###### [%f, %f, %f] ######" %(points[pixel_index[1], pixel_index[2], 0], points[pixel_index[1], pixel_index[2], 1], points[pixel_index[1], pixel_index[2], 2])
		if np.isnan(points[pixel_index[1], pixel_index[2], 0]) or \
		   np.isnan(points[pixel_index[1], pixel_index[2], 1]) or \
		   np.isnan(points[pixel_index[1], pixel_index[2], 2]) or \
		   points[pixel_index[1], pixel_index[2], 0] == 0.0 or \
		   points[pixel_index[1], pixel_index[2], 1] == 0.0:
			print "\033[0;31;44mInvalid pointcloud. Ignore...\033[0m"
			is_valid = False
			if testing:
				cnt_invalid += 1
			
		#if getXYZResult.result.z >= Z_THRES:
		elif points[pixel_index[1], pixel_index[2], 2] <= Z_THRES:
			print "\033[0;31;44mTarget on converyor! Ignore...\033[0m"
			is_valid = False
			if testing:
				cnt_invalid += 1
		# Visualize markers
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
			cnt_invalid = 0 # Reset cnt
			action_list.append(action)
			gotoTarget = get_poseRequest()
			gotoTarget.point_in_hand.x = points[pixel_index[1], pixel_index[2], 0]
			gotoTarget.point_in_hand.y = points[pixel_index[1], pixel_index[2], 1]
			gotoTarget.point_in_hand.z = points[pixel_index[1], pixel_index[2], 2]
			gotoTarget.yaw = angle
			gotoTarget.primmitive = action
		
			goto_target(gotoTarget)
			time.sleep(0.5)

			go_home() # TODO: service maybe block
		else: # INVALID
			action_list.append(-1)
		target_list.append(pixel_index)
		# Get action result
		get_pc_req.file_name = pc_path + "/{}_after.pcd".format(iteration)
		next_color, next_depth, next_points, next_depth_img = utils.get_heightmap(get_pc_client(get_pc_req).pc, image_path + "next_", iteration)
		#action_result_req = get_resultRequest(depth_img, next_depth_img)
		if is_valid:
			if not action: # GRASP
				action_success = grasp_state().success
			else: # SUCK
				action_success = suck_state().success
		else:
			action_success = False

		result_list.append(action_success)
		
		if action_success:
			go_place()
			num_of_items -= 1
			print "%d items remain..." % num_of_items
		else:
			if not action: # GRASP
				open_gripper()
			else: # SUCK
				for i in range(2):
					vacuum(vacuum_controlRequest(0))
					time.sleep(0.3)
					pheumatic(SetBoolRequest(False))
					time.sleep(0.3)
		ts = time.time()
		is_empty = empty_checker(empty_checker_req).is_empty.data
		if is_valid:
			label_value, prev_reward_value = trainer.get_label_value(action_str, action_success, \
		                                                         next_color, next_depth, is_empty)
		else: # invalid
			label_value, prev_reward_value = trainer.get_label_value("invalid", False, next_color, next_depth, is_empty)
		print "Get label: {} seconds".format(time.time()-ts)
		return_ += prev_reward_value * np.power(discount, iteration)
		ts = time.time()
		loss_value = trainer.backprop(color, depth, action_str, pixel_index, label_value)
		loss_list.append(loss_value)
		print "Backpropagation {} seconds".format(time.time() - ts)
		iter_time = time.time() - iter_ts
		total_time += iter_time
		print "[Total time] \033[0;31m{}\033[0m s| [Iter time] \033[0;32m{}\033[0m s".format \
	                                                                    (total_time, iter_time)
		iteration += 1
		# Update target net
		if iteration % update_fre == 0 or num_of_items == 0:
			print "Update target network"
			trainer.target = trainer.model
		# Pass test checker
		if testing:
			if iteration >= 3*init_num:
				print "Fail to pass test since too much iterations!"
				break
			if cnt_invalid >= init_num:
				print "Fail to pass test since too much invalid target"
				break
		time.sleep(0.5) # Sleep 0.5 s for next iteration
		empty_checker_req = pc_is_emptyRequest()
		get_pc_req.file_name = str()
		empty_checker_req.input_pc = get_pc_client(get_pc_req).pc
		#is_empty = (num_of_items == 0)
		is_empty = empty_checker(empty_checker_req).is_empty.data
		
	# Num of item = 0
	if not testing:
		model_name = model_path + "final.pth"
		torch.save(trainer.model.state_dict(), model_name)
		print "Model: %s saved" % model_name
	# Save files
	num_of_invalid = np.where(np.array(action_list) == -1)[0].size
	grasp_idx      = np.where(np.array(action_list) == 0)
	suck_idx       = np.where(np.array(action_list) == 1)
	num_of_grasp   = grasp_idx[0].size
	num_of_suck    = suck_idx[0].size
	grasp_success  = np.count_nonzero(np.array(result_list)[grasp_idx])
	suck_success   = np.count_nonzero(np.array(result_list)[suck_idx])
	efficiency = float(init_num-num_of_items)/(num_of_grasp+num_of_suck)
	valid_action_ratio = 1-float(num_of_invalid)/iteration
	if num_of_grasp is not 0: grasp_rate = float(grasp_success)/num_of_grasp
	else: grasp_rate = float('nan')
	if num_of_suck is not 0: suck_rate  = float(suck_success)/num_of_suck
	else: suck_rate = float('nan')
	result_str = str("Time: %f\nItem remain: %d\nIteration: %d\nReturn: %f\nMean loss: %f\nNum of Invalid: %d\nValid action ratio: %f\nNum of Grasp: %d\nGrasp success rate: %f\nNum of suck: %d\nSuck success rate: %f\nEfficiency: %f\n"
					%(total_time, num_of_items, iteration, return_, np.mean(loss_list), num_of_invalid, valid_action_ratio, num_of_grasp, grasp_rate, num_of_suck, suck_rate, efficiency))
	f.write(result_str)
	f.close()
	np.savetxt(loss_file   , loss_list   , delimiter=",")
	np.savetxt(action_file , action_list , delimiter=",")
	np.savetxt(target_file , target_list , delimiter=",")
	np.savetxt(result_file , result_list , delimiter=",")
	np.savetxt(explore_file, explore_list, delimiter=",")
	print "Regular shutdown"
	

except KeyboardInterrupt:
	if not testing:
		model_name = model_path + "force_terminate.pth"
		torch.save(trainer.model.state_dict(), model_name)
		print "Model: %s saved" % model_name
	num_of_invalid = np.where(np.array(action_list) == -1)[0].size
	grasp_idx      = np.where(np.array(action_list) == 0)
	suck_idx       = np.where(np.array(action_list) == 1)
	num_of_grasp   = grasp_idx[0].size
	num_of_suck    = suck_idx[0].size
	grasp_success  = np.count_nonzero(np.array(result_list)[grasp_idx])
	suck_success   = np.count_nonzero(np.array(result_list)[suck_idx])
	efficiency = float(init_num-num_of_items)/(num_of_grasp+num_of_suck)
	valid_action_ratio = 1-float(num_of_invalid)/iteration
	if num_of_grasp is not 0: grasp_rate = float(grasp_success)/num_of_grasp
	else: grasp_rate = float('nan')
	if num_of_suck is not 0: suck_rate  = float(suck_success)/num_of_suck
	else: suck_rate = float('nan')
	result_str = str("Time: %f\nItem remain: %d\nIteration: %d\nReturn: %f\nMean loss: %f\nNum of Invalid: %d\nValid action ratio: %f\nNum of Grasp: %d\nGrasp success rate: %f\nNum of suck: %d\nSuck success rate: %f\nEfficiency: %f\n"
					%(total_time, num_of_items, iteration, return_, np.mean(loss_list), num_of_invalid, valid_action_ratio, num_of_grasp, grasp_rate, num_of_suck, suck_rate, efficiency))
	f.write(result_str)
	f.close()
	np.savetxt(loss_file   , loss_list   , delimiter=",")
	np.savetxt(action_file , action_list , delimiter=",")
	np.savetxt(target_file , target_list , delimiter=",")
	np.savetxt(result_file , result_list , delimiter=",")
	np.savetxt(explore_file, explore_list, delimiter=",")
	print "Force terminate"
