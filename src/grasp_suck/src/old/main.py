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
from cv_bridge import CvBridge, CvBridgeError
# Network
from trainer import Trainer
# SRV
from std_srvs.srv import Empty, SetBool, SetBoolRequest, SetBoolResponse, \
                         Trigger, TriggerRequest, TriggerResponse
from grasp_suck.srv import get_pose, get_poseRequest, get_poseResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              get_surface_feature, get_surface_featureRequest, get_surface_featureResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse, \
                              check_valid, check_validRequest, check_validResponse
from vacuum_conveyor_control.srv import vacuum_control, vacuum_controlRequest
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="learning pick and place by trial and error using DQN")
parser.add_argument("--is_testing", action="store_true",  default=False, help="True if testing, default is false")
parser.add_argument("--force_cpu",  action="store_true",  default=False, help="True if using CPU, default is false")
parser.add_argument("--grasp_only", action="store_true",  default=False, help="True if using grasp-only policy, default is false")
parser.add_argument("--model",         type=str,          default="",    help="If provided, continue training the model or using this model for testing, default is empty string")
parser.add_argument("--episode",       type=int,          default=0,     help="Which episode is this run, default is 0")
parser.add_argument("--epsilon",       type=float,        default=0.7,   help="Probability to do random action, default is 0.7")
parser.add_argument("--update_target", type=int,          default=3,    help="After how much iterations should update target network")
parser.add_argument("--learn_every", type=int, default=6, help="Frequency for updating network")
args = parser.parse_args()

utils.show_args(args)

# Parameter
testing      = args.is_testing
grasp_only   = args.grasp_only
use_cpu      = args.force_cpu
epsilon      = None
max_len      = 1000
iteration    = 0
episode      = args.episode
learn_every  = args.learn_every
use_normal   = False
abnormal     = False
grasp_radius = 0.015
suck_radius  = 0.015
# Reward
suck_reward  = 2.0
grasp_reward = 2.0
discount     = 0.5
item_counter = 0
cnt_invalid  = 0 # Consecutive invalid action counter
update_fre = args.update_target

grasp_only_training = not testing and grasp_only

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
              feat_path + 'rotate_idx_3/']
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
for i in range(4):
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
	trainer.copyNetwork()


# Service client
# Robot state
get_robot_state   = rospy.ServiceProxy('/ur5_control_server/ur_control/get_robot_state', Trigger)
# Gripper
open_gripper      = rospy.ServiceProxy('/robotiq_finger_control_node/open_gripper', Empty)
grasp_state       = rospy.ServiceProxy('/robotiq_finger_control_node/get_grasp_state', Trigger)
initial_gripper   = rospy.ServiceProxy('/robotiq_finger_control_node/initial_gripper', Empty)
pheumatic         = rospy.ServiceProxy('/arduino_control/pheumatic_control', SetBool)
vacuum            = rospy.ServiceProxy('/arduino_control/vacuum_control', vacuum_control)
suck_state        = rospy.ServiceProxy('/arduino_control/check_suck_success', SetBool)

# Helper
goto_target = rospy.ServiceProxy('/helper_services_node/goto_target', get_pose)
go_home     = rospy.ServiceProxy('/helper_services_node/robot_go_home', Empty)
go_place    = rospy.ServiceProxy('/helper_services_node/robot_goto_place', Empty)

# pc_transform
get_pc_client       = rospy.ServiceProxy('/pc_transform/get_pc', get_pc)
surface_feat_client = rospy.ServiceProxy('/pc_transform/get_surface_feature', get_surface_feature)
empty_checker       = rospy.ServiceProxy('/pc_transform/empty_state', pc_is_empty)
valid_checker       = rospy.ServiceProxy('/pc_transform/check_valid', check_valid)

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
if not grasp_only_training:
	pheumatic(SetBoolRequest(False))
	vacuum(vacuum_controlRequest(0))

is_empty = False
total_time = 0.0

try:
	while is_empty is not True:
		iter_ts = time.time()
		print "\033[0;31;46mIteration: {}\033[0m".format(iteration)
		if not testing: epsilon_ = max(epsilon * np.power(0.9998, iteration), 0.2)
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
			if not grasp_only:
				explore, action, action_str, pixel_index, angle = \
					utils.epsilon_greedy_policy(epsilon_, suck_predictions, grasp_predictions)
			else:
				explore, action, action_str, pixel_index, angle = \
					utils.grasp_epsilon_greedy_policy(epsilon_, grasp_predictions)
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

		# Invalid conditions:
		# 1. NaN or origin point
		# 2. Point belongs to the plane 
		# TODO 3. Gripper collision with object
		print "###### [%f, %f, %f] ######" %(points[pixel_index[1], pixel_index[2], 0], points[pixel_index[1], pixel_index[2], 1], points[pixel_index[1], pixel_index[2], 2])
		#check_valid_req = check_validRequest()
		#check_valid_req.pc = pc_response.pc
		#check_valid_req.p.x = points[pixel_index[1], pixel_index[2], 0]
		#check_valid_req.p.y = points[pixel_index[1], pixel_index[2], 1]
		#check_valid_req.p.z = points[pixel_index[1], pixel_index[2], 2]
		#is_valid = valid_checker(check_valid_req).is_valid
		is_valid = True
		if points[pixel_index[1], pixel_index[2], 0] == 0.0 or \
		   points[pixel_index[1], pixel_index[2], 1] == 0.0 or \
		   points[pixel_index[1], pixel_index[2], 2] == 0.0:
			is_valid = False
		# Use surface feature
		surface_feat_req = get_surface_featureRequest()
		surface_feat_req.pc = pc_response.pc
		surface_feat_req.p.x = points[pixel_index[1], pixel_index[2], 0]
		surface_feat_req.p.y = points[pixel_index[1], pixel_index[2], 1]
		surface_feat_req.p.z = points[pixel_index[1], pixel_index[2], 2]
		surface_feat_req.type = 0
		if action: # SUCK
			surface_feat_req.radius = suck_radius
			if use_normal:
				surface_feat_req.type = 1 # Also compute surface normal
		else: # GRASP
			surface_feat_req.radius = grasp_radius
		surface_res = surface_feat_client(surface_feat_req)
		# TODO: Check if normal valid
		# Visualize markers
		viz_req = viz_markerRequest()
		viz_req.point.x = points[pixel_index[1], pixel_index[2], 0]
		viz_req.point.y = points[pixel_index[1], pixel_index[2], 1]
		viz_req.point.z = points[pixel_index[1], pixel_index[2], 2]
		viz_req.primitive = action
		viz_req.angle = angle
		viz_req.valid = is_valid
		viz(viz_req)
		target_list.append(pixel_index)
		# Take action
		if is_valid:
			cnt_invalid = 0 # Reset cnt
			action_list.append(action)
			gotoTarget = get_poseRequest()
			gotoTarget.point_in_hand.x = surface_res.c_p.x
			gotoTarget.point_in_hand.y = surface_res.c_p.y
			gotoTarget.point_in_hand.z = surface_res.c_p.z
			gotoTarget.x_axis.x = 0.0
			gotoTarget.x_axis.y = 0.0
			gotoTarget.x_axis.z = -1.0
			if action and use_normal: # SUCK and use surface normal
				gotoTarget.x_axis = surface_res.normal
			gotoTarget.yaw = angle
			gotoTarget.primitive = action
		
			goto_target(gotoTarget)
			time.sleep(0.5)
			# Robot emergency/protective stop
			if get_robot_state().success == False:
				abnormal = True
				print "\033[1;31m[%f] Robot protective, aborting...\033[0m" %(time.time())
				action_success = False
				result_list.append(action_success)
				go_home()
				pheumatic(SetBoolRequest(False)); rospy.sleep(0.5)
				vacuum(vacuum_controlRequest(0))
				break

			go_home() # TODO: service maybe block
		else: # INVALID
			action_list.append(-1)
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
			item_counter+=1
			print "Action successful"
		else:
			if not action: # GRASP
				open_gripper()
			else: # SUCK
				for i in range(2):
					vacuum(vacuum_controlRequest(0))
					time.sleep(0.3)
					pheumatic(SetBoolRequest(False))
					time.sleep(0.3)
		rospy.sleep(1.0)
		# Get next images
		get_pc_req.file_name = pc_path + "/{}_after.pcd".format(iteration)
		next_pc = get_pc_client(get_pc_req).pc
		next_color, next_depth, next_points, next_depth_img = utils.get_heightmap(next_pc, image_path + "next_", iteration)
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
		return_ += prev_reward_value * np.power(discount, iteration)
		if (iteration+1) % learn_every == 0:
			ts = time.time()
			loss_value = trainer.backprop(color, depth, action_str, pixel_index, label_value)
			loss_list.append(loss_value)
			print "Backpropagation: {} seconds".format(time.time() - ts)
		iter_time = time.time() - iter_ts
		total_time += iter_time
		print "[Total time] \033[0;31m{}\033[0m s| [Iter time] \033[0;32m{}\033[0m s".format \
	                                                                    (total_time, iter_time)
		iteration += 1
		# Update target net
		if iteration % update_fre == 0 or is_empty:
			print "Update target network"
			trainer.target = trainer.model
		# Pass test checker
		if testing:
			if cnt_invalid >= 10:
				print "Fail to pass test since too much invalid target"
				break
		time.sleep(0.5) # Sleep 0.5 s for next iteration

	# Workspace is empty
	if not testing:
		if not abnormal:
			model_name = model_path + "final.pth"
			torch.save(trainer.model.state_dict(), model_name)
			print "Model: %s saved" % model_name
		else:
			model_name = model_path + "force_terminate.pth"
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
	efficiency = float(item_counter)/(num_of_grasp+num_of_suck)
	valid_action_ratio = 1-float(num_of_invalid)/iteration
	if num_of_grasp is not 0: grasp_rate = float(grasp_success)/num_of_grasp
	else: grasp_rate = float('nan')
	if num_of_suck is not 0: suck_rate  = float(suck_success)/num_of_suck
	else: suck_rate = float('nan')
	result_str = str("Time: %f\nIteration: %d\nReturn: %f\nMean loss: %f\nNum of Invalid: %d\nValid action ratio: %f\nNum of Grasp: %d\nGrasp success rate: %f\nNum of suck: %d\nSuck success rate: %f\nEfficiency: %f\n"
					%(total_time, iteration, return_, np.mean(loss_list), num_of_invalid, valid_action_ratio, num_of_grasp, grasp_rate, num_of_suck, suck_rate, efficiency))
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
	efficiency = float(item_counter)/(num_of_grasp+num_of_suck)
	valid_action_ratio = 1-float(num_of_invalid)/iteration
	if num_of_grasp is not 0: grasp_rate = float(grasp_success)/num_of_grasp
	else: grasp_rate = float('nan')
	if num_of_suck is not 0: suck_rate  = float(suck_success)/num_of_suck
	else: suck_rate = float('nan')
	result_str = str("Time: %f\nIteration: %d\nReturn: %f\nMean loss: %f\nNum of Invalid: %d\nValid action ratio: %f\nNum of Grasp: %d\nGrasp success rate: %f\nNum of suck: %d\nSuck success rate: %f\nEfficiency: %f\n"
					%(total_time, iteration, return_, np.mean(loss_list), num_of_invalid, valid_action_ratio, num_of_grasp, grasp_rate, num_of_suck, suck_rate, efficiency))
	f.write(result_str)
	f.close()
	np.savetxt(loss_file   , loss_list   , delimiter=",")
	np.savetxt(action_file , action_list , delimiter=",")
	np.savetxt(target_file , target_list , delimiter=",")
	np.savetxt(result_file , result_list , delimiter=",")
	np.savetxt(explore_file, explore_list, delimiter=",")
	print "Force terminate"
