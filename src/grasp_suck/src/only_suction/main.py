import os
import sys
import cv2
import time
import numpy as np
import argparse
import torch
import rospy
import rospkg
import utils
from cv_bridge import CvBridge, CvBridgeError
from trainer import Trainer
from collections import namedtuple, deque
# srv
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, \
                         Empty, EmptyRequest, EmptyResponse
from arm_operation.srv import agent_abb_action, agent_abb_actionRequest, agent_abb_actionResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse

# Define transition tuple
Transition = namedtuple('Transition', ['color', 'depth', 'pixel_idx', 'is_success', 'is_valid', 'next_color', 'next_depth', 'is_empty'])

# Parse argument
parser = argparse.ArgumentParser(prog="reinforcement_grasping", description="Reinforcement learning for robot arm grasping")
parser.add_argument("--is_testing", action="store_true", default=False, help="True if testing, default is false")
parser.add_argument("--force_cpu", action="store_true", default=False, help="True if using CPU, default is false")
parser.add_argument("--model", type=str, default="", help="If provided, continue training the model, or using this model for testing, default is empty srting")
parser.add_argument("--epsilon", type=float, default=0.5, help="Probability to choose random action")
parser.add_argument("--buffer_size", type=int, default=100, help="Experience buffer size, default is 100") # N
parser.add_argument("--learning_freq", type=int, default=10, help="Frequency for updating behavior network, default is 10") # M
parser.add_argument("--updating_freq", type=int, default=20, help="Frequency for updating target network, default is 20") # C
parser.add_argument("--mini_batch_size", type=int, default=2, help="How many transitions should used for learning, default is 2") # K
parser.add_argument("--save_every", type=int, default=10, help="Every how many steps should save the model, default is 10")
args = parser.parse_args()
utils.show_args(args)

testing         = args.is_testing
use_cpu         = args.force_cpu
model_str       = args.model
epsilon         = args.epsilon if not testing else 1
buffer_size     = args.buffer_size
learning_freq   = args.learning_freq
updating_freq   = args.updating_freq
mini_batch_size = args.mini_batch_size
save_every      = args.save_every
# Constant
reward = 1.0
discount_factor = 0.5
iteration = 0
return_ = 0.0
program_ts = time.time()

if model_str == "" and testing: # TEST SHOULD PROVIDE MODEL
	print "\033[0;31mNo model provided, exist!\033[0m"
	os._exit(0)

experience_buffer = deque(maxlen=buffer_size)

# trainer
trainer = Trainer(reward, discount_factor, use_cpu)
# Still using small learning rate to backpropagate when testing
if testing:
	for param_group in trainer.optimizer.param_groups:
		param_group['lr'] = 1e-5

# Load model if provided
if model_str != "":
	print "[%f]: Loading provided model..." %(time.time())
	trainer.behavior_net.load_state_dict(torch.load(args.model))
	trainer.copyNetwork()
	
# Get logger path
r = rospkg.RosPack()
package_path = r.get_path("grasp_suck")
csv_path, image_path, depth_path, mixed_path, feat_path, pc_path, model_path, vis_path = utils.getLoggerPath(testing, package_path)
# Service clients
vacuum_pump_control        = rospy.ServiceProxy("/vacuum_pump_control_node/vacuum_control", SetBool)
check_suck_success         = rospy.ServiceProxy("/vacuum_pump_control_node/check_suck_success", SetBool)
agent_take_action_client   = rospy.ServiceProxy("/agent_server_node/agent_take_action", agent_abb_action)
get_pc_client              = rospy.ServiceProxy("/combine_pc_node/get_pc", get_pc)
empty_checker              = rospy.ServiceProxy("/combine_pc_node/empty_state", pc_is_empty)
go_home                    = rospy.ServiceProxy("/agent_server_node/go_home", Empty)
go_place                   = rospy.ServiceProxy("/agent_server_node/go_place", Empty)
viz                        = rospy.ServiceProxy("/viz_marker_node/viz_marker", viz_marker)
# Result list
action_list  = []
target_list  = []
result_list  = []
loss_list    = []
explore_list = []
return_list  = []
episode_list = []

# Initialize
go_home()
vacuum_pump_control(SetBoolRequest(False))

try:
	while True:
		cmd = raw_input("[%f] Reset environment, if ready, press 's' to start. 'e' to exit: " %(time.time()-program_ts))
		if cmd == 'E' or cmd == 'e':
			utils.saveFiles(action_list, target_list, result_list, loss_list, explore_list, return_list, csv_path)
			print "Regular shutdown"
			sys.exit(0)
		elif cmd == 'S' or cmd == 's':
			episode_list.append(t)
			t = 0
			return_list.append(return_)
			return_ = 0.0
			is_empty = False
			while is_empty is not True:
				print "[%f] Iteration: %d" %(time.time()-program_ts, iteration)
				if not testing: epsilon_ = max(epsilon * np.power(0.9998, t), 0.2)
				get_pc_req = get_pcRequest()
				get_pc_req.file_name = pc_path + "/{:06}_before.pcd".format(iteration)
				ts = time.time()
				pc_response = get_pc_client(get_pc_req)
				color, depth, points, depth_img = utils.get_heightmap(pc_response.pc, image_path, depth_path, iteration)
				ts = time.time()
				prediction, state_feat = trainer.forward(color, depth, is_volatile=True)
				print "Forward past: {} seconds".format(time.time()-ts)
				prediction = utils.standarization(prediction)
				heatmap = utils.vis_affordance(prediction[0])
				img_name = feat_path + "{:06}.jpg".format(iteration)
				cv2.imwrite(img_name, heatmap)
				mixed = cv2.addWeighted(color, 1.0, heatmap, 0.4, 0)
				img_name = mixed_path + "{:06}.jpg".format(iteration)
				cv2.imwrite(img_name, mixed)
				explore = -1
				if not testing: # Train
					explore, action, action_str, pixel_index, angle = utils.epsilon_greedy_policy(epsilon_, prediction)
				else: # Testing
						explore, action, action_str, pixel_index, angle = utils.greedy_polocy(prediction)
				explore_list.append(explore)
				del prediction, state_feat
				if explore: print "|Explore|" else: print "|Exploit|"
				print "Take action at (%d, %d) -> (%f, %f, %f)" %(pixel_index[1], pixel_index[2], \
				                                                  points[pixel_index[1], pixel_index[2], 0],
				                                                  points[pixel_index[1], pixel_index[2], 1],
				                                                  points[pixel_index[1], pixel_index[2], 2])
				visual_img = utils.draw_image(mixed, pixel_index)
				img_name = vis_path + "vis_{:06}.jpg".format(iteration)
				cv2.imwrite(img_name, visual_img)
				is_valid = True
				if points[pixel_index[1], pixel_index[2], 0] == 0.0 or \
				   points[pixel_index[1], pixel_index[2], 1] == 0.0 or \
				   points[pixel_index[1], pixel_index[2], 2] == 0.0: # TODO: add a function to determine if given point in workspace
					is_valid = False
				viz_req = viz_markerRequest()
				viz_req.point.x = points[pixel_index[1], pixel_index[2], 0]
				viz_req.point.y = points[pixel_index[1], pixel_index[2], 1]
				viz_req.point.z = points[pixel_index[1], pixel_index[2], 2]
				viz_req.primitive = action
				viz_req.angle = angle
				viz_req.valid = is_valid
				viz(viz_req)
				target_list.append(pixel_index)
				if is_valid:
					action_list.append(action)
					agent_action = agent_abb_actionRequest()
					agent_action.tool_id = 3
					agent_action.position.x = viz_req.point.x
					agent_action.position.y = viz_req.point.y
					agent_action.position.z = viz_req.point.z
					agent_action.angle = 0.0
					agent_response = agent_take_action_client(agent_action)
					go_home()
				else: # invalid
					action_list.append(-1)
				if is_valid:
					action_success = check_suck_success().success
				else:
					action_success = False
				result_list.append(action_success)
				if action_success: go_place()
				else: vacuum_pump_control(SetBoolRequest(False))
				time.sleep(0.5)
				# Get next images
				get_pc_req.file_name = pc_path + "/{:06}_after.pcd".format(iteration)
				next_pc = get_pc_client(get_pc_req).pc
				next_color, next_depth, next_points, next_depth_img = utils.get_heightmap(next_pc, image_path + "next_", depth_path + "next_", iteration)
				ts = time.time()
				empty_checker_req = pc_is_emptyRequest()
				empty_checker_req.input_pc = next_pc
				is_empty = empty_checker(empty_checker_req).is_empty.data
				if action_success:
					current_reward = reward
				elif not action_success:
					current_reward = 0.0
				elif not is_valid:
					current_reward = -reward
				return_ += current_reward * np.power(discount_factor, t)
				# Store transition
				color_name = image_path + "color_{:06}.jpg".format(iteration)
				depth_name = depth_path + "depth_data_{:06}.txt".format(iteration)
				next_color_name = image_path + "next_color_{:06}.jpg".format(iteration)
				next_depth_name = depth_path + "next_depth_data_{:06}.txt".format(iteration)
				transition = Transition(color_name, depth_name, pixel_index, action_success, is_valid, next_color_name, next_depth_name, is_empty)
				experience_buffer.append(transition)
				iteration += 1
				t += 1
				if iteration % learning_freq == 0:
					print "[%f] Computing TD target array..." %(time.time()-program_ts))
					td_target_array = np.zeros(len(experience_buffer))
					for i in range(len(experience_buffer)):
						color = cv2.imread(experience_buffer[i].color)
						depth = np.loadtxt(experience_buffer[i].depth)
						next_color = cv2.imread(experience_buffer[i].next_color)
						next_depth = np.loadtxt(experience_buffer[i].next_depth)
						td_target, _ = trainer.get_label_value(experience_buffer[i].is_valid, \
						                                       experience_buffer[i].is_success, \
						                                       next_color, next_depth, \
						                                       experience_buffer[i].is_empty)
						td_target_array[i] = td_target
					print "[%f] Complete" %(time.time()-program_ts)
					sampling_weight = utils.softmax(td_target_array)
					for j in range(mini_batch_size):
						sample_idx = np.random.choice(list(range(len(experience_buffer))), p=sampling_weight)
						color = cv2.imread(experience_buffer[sample_idx].color)
						depth = np.loadtxt(experience_buffer[sample_idx].depth)
						action_pix_idx = experience_buffer[sample_idx].pixel_idx
						label_value = td_target_array[sample_idx]
						loss_value = trainer.backprop(color, depth, action_pix_idx, label_value)
						loss_list.append(loss_value)
					
				if iteration % updating_freq == 0:
					print "[%f] Replace target network to behavior network" %(time.time()-program_ts)
					trainer.target_net = trainer.behavior_net
				if iteration % save_every == 0 and not testing:
					model_name = model_path + "{}.pth".format(iteration)
					torch.save(trainer.behavior_net.state_dict(), model_name)
					print "[%f] Model: %s saved" %(time.time()-program_ts, model_name)
					
except KeyboardInterrupt:
	utils.saveFiles(action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, csv_path)
	print "Shutdown since keyboard interrupt"
	sys.exit(0)
