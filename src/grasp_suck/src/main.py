import os
import sys
import cv2
import time
import numpy as np
import torch
import rospy
import rospkg
import utils
import pickle
import serial
from trainer import Trainer
from prioritized_memory import Memory
from utils import Transition
# srv
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, \
                         Empty, EmptyRequest, EmptyResponse
from arm_operation.srv import agent_abb_action, agent_abb_actionRequest, agent_abb_actionResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse, \
                              check_grasp_success, check_grasp_successRequest, check_grasp_successResponse
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse

parser = utils.create_argparser()
args = parser.parse_args()
utils.show_args(args)
testing, episode, use_cpu, model_str, buffer_str, epsilon, port, buffer_size, learning_freq, updating_freq, mini_batch_size, save_every, learning_rate = utils.parse_input(args)

# Constant
reward = 1.0
discount_factor = 0.5
iteration = 0
z_thres = -0.017 # TODO is this required?
program_ts = time.time()
memory = Memory(buffer_size)
arduino = serial.Serial(port, 115200)

if model_str == "" and testing: # TEST SHOULD PROVIDE MODEL
	print "\033[0;31mNo model provided, exit!\033[0m"
	os._exit(0)
	
if buffer_str != "": memory.load_memory(buffer_file)

# trainer
trainer = Trainer(reward, discount_factor, use_cpu, learning_rate)

# Load model if provided
if model_str != "":
	print "[%f]: Loading provided model..." %(time.time())
	trainer.behavior_net.load_state_dict(torch.load(model_str))
	trainer.target_net.load_state_dict(trainer.behavior_net.state_dict())
	
# Get logger path
r = rospkg.RosPack()
package_path = r.get_path("grasp_suck")
csv_path, image_path, depth_path, mixed_paths, feat_paths, pc_path, model_path, vis_path, diff_path = utils.getLoggerPath(testing, package_path, episode)
# Service clients
vacuum_pump_control      = rospy.ServiceProxy("/vacuum_pump_control_node/vacuum_control", SetBool)
check_suck_success       = rospy.ServiceProxy("/vacuum_pump_control_node/check_suck_success", SetBool)
agent_take_action_client = rospy.ServiceProxy("/agent_server_node/agent_take_action", agent_abb_action)
calibrate_gripper_client = rospy.ServiceProxy("/change_tool_service/calibrate_gripper", Empty)
get_pc_client            = rospy.ServiceProxy("/combine_pc_node/get_pc", get_pc)
empty_checker            = rospy.ServiceProxy("/combine_pc_node/empty_state", pc_is_empty)
check_grasp_success      = rospy.ServiceProxy("/combine_pc_node/grasp_state", check_grasp_success)
go_home                  = rospy.ServiceProxy("/agent_server_node/go_home", Empty)
go_place                 = rospy.ServiceProxy("/agent_server_node/go_place", Empty)
fixed_home               = rospy.ServiceProxy("/agent_server_node/go_home_fix_orientation", Empty)
viz                      = rospy.ServiceProxy("/viz_marker_node/viz_marker", viz_marker)
# Result list
action_list   = [] # If action valid?
target_list   = [] # Takes action at which `pixel`
position_list = [] # Takes at which `3D position`
result_list   = [] # If action success?
loss_list     = [] # Training loss
explore_list  = [] # If using explore?
return_list   = [] # Episode return
episode_list  = [] # Episode step length

# Code for calling service
def _get_pc(iteration, is_before):
	get_pc_req = get_pcRequest()
	if is_before:
		get_pc_req.file_name = pc_path + "{:06}_before.pcd".format(iteration)
	else:
		get_pc_req.file_name = pc_path + "{:06}_after.pcd".format(iteration)
	return get_pc_client(get_pc_req)
def _viz(point, action, angle, valid):
	viz_req = viz_markerRequest()
	viz_req.point = utils.point_np_to_ros(point)
	viz_req.primitive = action
	viz_req.angle = angle
	viz_req.valid = valid
	viz(viz_req)
def _take_action(tool_id, point, angle):
	agent_action = agent_abb_actionRequest()
	agent_action.tool_id = tool_id
	agent_action.position = utils.point_np_to_ros(point)
	agent_action.angle = angle
	_ = agent_take_action_client(agent_action)
def _check_if_empty(pc):
	empty_checker_req = pc_is_emptyRequest()
	empty_checker_req.input_pc = pc
	return empty_checker(empty_checker_req).is_empty.data
def _check_grasp_success():
	go_home(); rospy.sleep(0.5)
	calibrate_gripper_client()
	check_grasp_success_request = check_grasp_successRequest()
	# Get temporary pc and save file
	_ = _get_pc(iteration, False)
	check_grasp_success_request.pcd_str = pc_path + "{:06}_after.pcd".format(iteration)
	return check_grasp_success(check_grasp_success_request).is_success


# Initialize
go_home()
vacuum_pump_control(SetBoolRequest(False))

try:
	while True:
		if iteration is not 0: arduino.write("gb 1000") # Green + buzzer for alarming resetting
		cmd = raw_input("\033[1;34m[%f] Reset environment, if ready, press 's' to start. 'e' to exit: \033[0m" %(time.time()-program_ts))
		if cmd == 'E' or cmd == 'e':
			utils.shutdown_process(action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, csv_path, memory, True)
		elif cmd == 'S' or cmd == 's':
			if iteration is not 0: return_list.append(return_); episode_list.append(t)
			t = 0; return_ = 0.0; is_empty = False
			while is_empty is not True:
				print "\033[0;32m[%f] Iteration: %d\033[0m" %(time.time()-program_ts, iteration)
				if not testing: epsilon_ = max(epsilon * np.power(0.998, t), 0.1) # half after 350 steps
				pc_response = _get_pc(iteration, True)
				color, depth, points = utils.get_heightmap(pc_response.pc, image_path, depth_path, iteration)
				ts = time.time()
				suck_1_prediction, suck_2_prediction, grasp_prediction = trainer.forward(color, depth, is_volatile=True)
				print "Forward past: {} seconds".format(time.time()-ts)
				heatmaps, mixed_imgs = utils.save_heatmap_and_mixed(suck_1_prediction, suck_2_prediction, grasp_prediction, feat_paths, mixed_paths, color, iteration)
				# Standarize predictions to avoid bias between them
				suck_1_prediction = utils.standarization(suck_1_prediction)
				suck_2_prediction = utils.standarization(suck_2_prediction)
				grasp_prediction = utils.standarization(grasp_prediction)
				if not testing: # Train
					explore, action, action_str, pixel_index, angle = utils.epsilon_greedy_policy(epsilon_, suck_1_prediction, suck_2_prediction, grasp_prediction, depth, diff_path, iteration)
				else: # Testing
					action, action_str, pixel_index, angle = utils.greedy_policy(suck_1_prediction, suck_2_prediction, grasp_prediction); explore = False
				explore_list.append(explore)
				target_list.append(pixel_index)
				position_list.append(points[pixel_index[1], pixel_index[2]])
				del suck_1_prediction, suck_2_prediction, grasp_prediction
				utils.print_action(action_str, pixel_index, points[pixel_index[1], pixel_index[2]])
				# Save (color heightmap + prediction heatmap + motion primitive and corresponding position), then show it
				visual_img = utils.draw_image(mixed_imgs[pixel_index[0]], explore, pixel_index)
				img_name = vis_path + "vis_{:06}.jpg".format(iteration)
				cv2.imwrite(img_name, visual_img)
				cv2.imshow("prediction", visual_img);cv2.waitKey(33)
				# Check if action valid (is NAN?)
				is_valid = utils.check_if_valid(points[pixel_index[1], pixel_index[2]])
				# Visualize in RViz
				_viz(points[pixel_index[1], pixel_index[2]], action, angle, is_valid)
				if is_valid: # Only take action if valid
					# suck_1(0) -> 3, suck_2(1) -> 2, other(2~5) (grasp) -> 1
					tool_id = (3-pixel_index[0]) if pixel_index[0] < 2 else 1
					_take_action(tool_id, points[pixel_index[1], pixel_index[2]], angle)
				else: # invalid
					action_list.append(-1); arduino.write("r 1000") # Red
					action_success = False
				if is_valid:
					if action < 2: # suction cup
						action_success = check_suck_success().success
					else: # parallel-jaw gripper TODO
						action_success = _check_grasp_success()
				result_list.append(action_success)
				if action_success: go_place(); fixed_home(); arduino.write("g 1000") # Green
				else: 
					fixed_home()
					vacuum_pump_control(SetBoolRequest(False))
					if is_valid: arduino.write("o 1000") # Orange
				time.sleep(0.5)
				# Get next images, and check if bin is empty
				next_pc = _get_pc(iteration, False)
				next_color, next_depth, next_points = utils.get_heightmap(next_pc.pc, image_path + "next_", depth_path + "next_", iteration)
				is_empty = _check_if_empty(next_pc.pc)
				# TODO: form a function
				# Reward shaping
				# Success -> +R
				# Fail and z>=z_thres -> -R
				# Fail and z< z_thres -> -2R
				# Invalid -> -5R
				if not is_valid: # NaN point
					current_reward = -5*reward
				elif action_success: # Good
					current_reward = reward
				elif is_valid and not action_success and points[pixel_index[1], pixel_index[2], 2] >= z_thres: # Bad suction point
					current_reward = -reward
				elif not action_success and points[pixel_index[1], pixel_index[2], 2] < z_thres: # Suck on box
					current_reward = -2*reward
				return_ += current_reward * np.power(discount_factor, t) 
				print "\033[1;33mCurrent reward: {} \t Return: {}\033[0m".format(current_reward, return_)
				# Store transition to experience buffer
				color_name, depth_name, next_color_name, next_depth_name = utils.wrap_strings(image_path, depth_path, iteration)
				transition = Transition(color_name, depth_name, pixel_index, current_reward, next_color_name, next_depth_name, is_empty)
				memory.add(transition)
				iteration += 1; t += 1
				if iteration % learning_freq == 0: # Training stage, update parameters
					back_ts = time.time(); arduino.write("b 1000")
					mini_batch, idxs, is_weight = memory.sample(mini_batch_size)
					sampled_iter = []
					for i in range(mini_batch_size):
						iter_str = mini_batch[i].color[mini_batch[i].color.find("color_")+6:-4]
						sampled_iter.append(int(iter_str))
						color = cv2.imread(mini_batch[i].color)
						depth = np.loadtxt(mini_batch[i].depth)
						pixel_index = mini_batch[i].pixel_idx
						next_color = cv2.imread(mini_batch[i].next_color)
						next_depth = np.loadtxt(mini_batch[i].next_depth)
						td_target = trainer.get_label_value(mini_batch[i].reward, next_color, next_depth, mini_batch[i].is_empty)
						trainer.backprop(color, depth, pixel_index, td_target, is_weight[i])
					print "Sampled at iteration: ", sampled_iter
					# After parameter updated, update prioirites tree
					for i in range(mini_batch_size):
						color = cv2.imread(mini_batch[i].color)
						depth = np.loadtxt(mini_batch[i].depth)
						pixel_index = mini_batch[i].pixel_idx
						next_color = cv2.imread(mini_batch[i].next_color)
						next_depth = np.loadtxt(mini_batch[i].next_depth)
						td_target = trainer.get_label_value(mini_batch[i].reward, next_color, next_depth, mini_batch[i].is_empty)
						action_str, rotate_idx = utils.get_action_info(pixel_index)
						old_value = trainer.forward(color, depth, action_str, False, rotate_idx, clear_grad=True)[0, pixel_index[1], pixel_index[2]]
						memory.update(idxs[i], (td_target-old_value)**2)
					back_t = time.time()-back_ts
					arduino.write("b 1000"); print "Backpropagation& Updating: {} seconds \t|\t Avg. {} seconds".format(back_t, back_t/mini_batch_size)
				if iteration % updating_freq == 0:
					print "[%f] Replace target network to behavior network" %(time.time()-program_ts)
					trainer.target_net.load_state_dict(trainer.behavior_net.state_dict)
				if iteration % save_every == 0 and not testing:
					model_name = model_path + "{}.pth".format(iteration)
					torch.save(trainer.behavior_net.state_dict(), model_name)
					print "[%f] Model: %s saved" %(time.time()-program_ts, model_name)
except KeyboardInterrupt:
	utils.shutdown_process(action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, csv_path, memory, False)
