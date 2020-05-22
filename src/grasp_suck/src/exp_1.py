import os
import sys
import time
import numpy as np
import cv2
import argparse
import torch
import rospy
import multiprocessing as mp
import rospkg
import serial
import sensor_msgs.point_cloud2 as pc2
from trainer import Trainer
from ctypes import c_bool, c_char_p
import utils
from utils import Transition

# srv
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, \
                         Empty, EmptyRequest, EmptyResponse
from arm_operation.srv import agent_abb_action, agent_abb_actionRequest, agent_abb_actionResponse, \
                              publish_info, publish_infoRequest, publish_infoResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse, \
                              check_grasp_success, check_grasp_successRequest, check_grasp_successResponse
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse
from grasp_suck.srv import recorder, recorderRequest, recorderResponse

def prediction_process(args, action_queue, experience_queue, work, ready, can_predict, should_reset, iteration, path_queue):
	# Setup model
	ts = time.time()
	first = True
	reward = 5.0
	discount_factor = 0.5
	path = path_queue.get()
	image_path = path[0]; depth_path = path[1]; pc_path = path[2]; vis_path = path[3];  mixed_paths = path[4]; feat_paths = path[5]
	trainer = Trainer(reward, discount_factor, False, args.primitive_lr, args.densenet_lr)
	trainer.behavior_net.load_state_dict(torch.load(args.model))
	trainer.target_net.load_state_dict(trainer.behavior_net.state_dict())
	ready.value = True
	if args.specific_tool == 3: specific_tool = None
	else: specific_tool = args.specific_tool
	cv2.namedWindow("prediction")
	print("[Prediction Thread] Load model took %f seconds. Start prediction thread" %(time.time()-ts))
	while work.value:
		if should_reset.value:
			print("[Prediction Thread] Receive reset command")
			if first: 
				print("[Prediction Thread] Already in initial state, abort reset request...")
				should_reset.value = False
				ready.value = True
				continue
			ts = time.time()
			ready.value = False
			trainer.behavior_net.load_state_dict(torch.load(args.model))
			first = True
			path = path_queue.get()
			image_path = path[0]; depth_path = path[1]; pc_path = path[2]; vis_path = path[3];  mixed_paths = path[4]; feat_paths = path[5]
			print("[Prediction Thread] Reset complete! Took {} seconds".format(time.time()-ts))
			should_reset.value = False
			ready.value = True
			continue
		if not first:
			while experience_queue.empty() and not should_reset.value and work.value:
				pass
		if not experience_queue.empty():
			transition = experience_queue.get()
			if transition.reward!=reward: # on;y update if fail
				print("[Prediction Thread] Got experience, updating network...")
				color = cv2.imread(transition.color)
				depth = np.load(transition.depth)
				next_color = cv2.imread(transition.next_color)
				next_depth = np.load(transition.next_depth)
				pixel_index = transition.pixel_idx
				td_target = trainer.get_label_value(transition.reward, next_color, next_depth, transition.is_empty, pixel_index[0])
				trainer.backprop(color, depth, pixel_index, td_target, 1.0, 1, True, True)
		if can_predict.value:
			if first: first = False
			print("[Prediction Thread] Start prediction")
			pc_response = _get_pc(iteration.value, True, pc_path)
			color, depth, points = utils.get_heightmap(pc_response.pc, image_path, depth_path, iteration.value)
			suck_1_prediction, suck_2_prediction, grasp_prediction = trainer.forward(color, depth, is_volatile=True)
			heatmaps, mixed_imgs = utils.save_heatmap_and_mixed(suck_1_prediction, suck_2_prediction, grasp_prediction, feat_paths, mixed_paths, color, iteration.value)
			action, action_str, pixel_index, angle = utils.greedy_policy(suck_1_prediction, suck_2_prediction, grasp_prediction, specific_tool)
			visual_img = utils.draw_image(mixed_imgs[pixel_index[0]], False, pixel_index, vis_path + "vis_{:06}.jpg".format(iteration.value))
			cv2.imshow("prediction", cv2.resize(visual_img, None, fx=2, fy=2)); cv2.waitKey(33)
			utils.print_action(action_str, pixel_index, points[pixel_index[1], pixel_index[2]])
			action_queue.put([action, action_str, points[pixel_index[1], pixel_index[2]], angle, pixel_index])
			can_predict.value = False
	print("[Prediction Thread] Prediction thread stop")

def create_directories(pkg_path, model_idx, epi, run, specific_tool):
	if specific_tool == 0: tool = "tool_3"
	elif specific_tool == 1: tool = "tool_2"
	elif specific_tool == 2: tool = "tool_1"
	else: tool = "tool_all"
	root_path = pkg_path + "/exp_1/model_{}/ep{}_run{}_{}/".format(model_idx, epi, run, tool)
	image_path = root_path + "images/"
	depth_path = root_path + "depth_data/"
	pc_path    = root_path + "pc/"
	vis_path   = root_path + "vis/"
	grasp_path = root_path + "grasp/"
	mixed_paths_root = root_path + "mixed_img/"
	feat_paths_root  = root_path + "feats/"
	mixed_paths = []
	feat_paths  = []
	primitives = ["suck_1/", "suck_2/", "grasp_0/", "grasp_1/", "grasp_2/", "grasp_3/"]
	for primitive in primitives:
		mixed_paths.append(mixed_paths_root+primitive)
		feat_paths.append(feat_paths_root+primitive)
		if not os.path.exists(mixed_paths[-1]): os.makedirs(mixed_paths[-1])
		if not os.path.exists(feat_paths[-1]): os.makedirs(feat_paths[-1])
	for path in list([root_path, image_path, depth_path, pc_path, vis_path, grasp_path]):
		if not os.path.exists(path):
			os.makedirs(path)
	return root_path, image_path, depth_path, pc_path, vis_path, grasp_path, mixed_paths, feat_paths

def encode_index(model_idx, epi, run, specific_tool):
	return 10000+1000*model_idx+100*epi+10*run+specific_tool # (0, 0, 0) -> 1000; (0, 1, 0) -> 1005; (1, 1, 1) -> 1106

def main():
	# Parse input
	parser = argparse.ArgumentParser(prog="exp_1", description="Code for Exp 1., testing the model capacity for grasping")
	parser.add_argument("model", type=str, help="model path for testing")
	parser.add_argument("model_idx", type=int, help="model index")
	parser.add_argument("run", type=int, help="Which number is this run")
	parser.add_argument("episode", type=int, help="Which episode is this run")
	parser.add_argument("specific_tool", type=int, help="only use specific tool? 0->small suction cup; 1->medium suction cup; 2->gripper; 3->all")
	parser.add_argument("--obj_nums", type=int, default=8, help="Number of object, default is 8")
	parser.add_argument("--port", type=str, default="/dev/ttylight", help="Port for arduino, which controls the alram lamp, default is /dev/ttylight")
	parser.add_argument("--densenet_lr", type=float, default=1e-5, help="Learning rate for feature extraction part, default is 1e-5")
	parser.add_argument("--primitive_lr", type=float, default=5e-5, help="Learning rate for motion primitive subnetworks, default is 1e-4")
	args = parser.parse_args()
	utils.show_args(args)
	# Create directories
	r = rospkg.RosPack()
	package_path = r.get_path("grasp_suck")
	root_path, image_path, depth_path, pc_path, vis_path, grasp_path, mixed_paths, feat_paths = create_directories(package_path, args.model_idx, args.episode, args.run, args.specific_tool)
	connect_to_alarm = False
	if args.port!="":
		connect_to_alarm = True
		arduino = serial.Serial(args.port, 115200)
	reward = 5.0
	discount_factor = 0.5
	return_ = 0.0
	pick_item = 0
	# Service clients
	vacuum_pump_control      = rospy.ServiceProxy("/vacuum_pump_control_node/vacuum_control", SetBool)
	check_suck_success       = rospy.ServiceProxy("/vacuum_pump_control_node/check_suck_success", SetBool)
	go_home                  = rospy.ServiceProxy("/agent_server_node/go_home", Empty)
	go_place                 = rospy.ServiceProxy("/agent_server_node/go_place", Empty)
	fixed_home               = rospy.ServiceProxy("/agent_server_node/go_home_fix_orientation", Empty)
	publish_data_client      = rospy.ServiceProxy("/agent_server_node/publish_data", publish_info)
	record_bag_client        = rospy.ServiceProxy("/autonomous_recording_node/start_recording", recorder)
	stop_record_client       = rospy.ServiceProxy("/autonomous_recording_node/stop_recording", Empty)
	# Shared data between processes
	work = mp.Value(c_bool, True) # Can prediction thread continue working? <bool>
	ready = mp.Value(c_bool, False) # Is prediction thread ready? <bool>
	can_predict = mp.Value(c_bool, False) # Can prediction thread do predict? <bool>
	should_reset = mp.Value(c_bool, False) # Should prediction thread reset model? <bool>
	iteration = mp.Value("i", 0) # What iteration is this action? <int>
	path_queue = mp.Queue()
	path_queue.put([image_path, depth_path, pc_path, vis_path, feat_paths, mixed_paths])
	action_queue = mp.Queue() # Action placeholder, prediction thread will generate an action and main thread will consume it
	experience_queue = mp.Queue() # Transition placeholder, main thread will generate a transition and prediction thread will consume it
	# Start prediction thread
	p = mp.Process(target=prediction_process, args=(args, \
													action_queue, experience_queue, \
													work, ready, can_predict, should_reset, \
													iteration, \
													path_queue, ))
	p.start()
	# Initialize
	while not ready.value:
		pass
	go_home()
	vacuum_pump_control(SetBoolRequest(False))
	is_empty = False
	cmd = raw_input("[Main Thread] Press any key to continue...")
	program_ts = time.time()
	can_predict.value = True
	while 1:
		action_target = []
		is_empty_list = []
		record_bag_client(recorderRequest(encode_index(args.model_idx, args.episode, args.run, args.specific_tool))) # Start recording
		while not is_empty and iteration.value<args.obj_nums*2:
			print("\033[1;32m[{}] Iteration: {}\033[0m".format(time.time()-program_ts, iteration.value))
			if connect_to_alarm: arduino.write("b 1000")
			# Wait until there is action in the queue
			while action_queue.empty():
				pass
			action_obj = action_queue.get() # [action, action_str, points, angle, pixel_index]
			is_valid = utils.check_if_valid(action_obj[2])
			_viz(action_obj[2], action_obj[0], action_obj[3], is_valid)
			will_collide = None
			if is_valid:
				tool_id = (3-action_obj[0]) if action_obj[0] <2 else 1
				if tool_id == 1:
					will_collide = _check_collide(action_obj[2], action_obj[3])
				if not will_collide or tool_id!=1:
					_take_action(tool_id, action_obj[2], action_obj[3])
				else:
					print("[Main Thread] Will collide, abort request!")
			else:
				if connect_to_alarm: arduino.write("r 1000")
				action_success = False
			if is_valid:
				if action_obj[0] < 2:
					action_success = check_suck_success().success
				else:
					if not will_collide:
						action_success = _check_grasp_success(iteration.value, grasp_path)
					else:
						action_success = False
			if action_success: pick_item += 1
			info = publish_infoRequest(); info.execution = utils.wrap_execution_info(iteration.value, is_valid, action_obj[0], action_success); publish_data_client(info)
			empty_state = mp.Value(c_bool, False)
			iteration.value += 1
			next_state_thread = mp.Process(target=get_next_state, args=(empty_state, iteration.value-1, (pick_item==args.obj_nums), pc_path, image_path, depth_path))
			next_state_thread.start()
			if action_success:
				if connect_to_alarm: arduino.write("g 1000")
				go_place(); fixed_home(); 
			else:
				fixed_home(); vacuum_pump_control(SetBoolRequest(False));
			current_reward = utils.reward_judgement(reward, is_valid, action_success)
			return_ += current_reward * np.power(discount_factor, iteration.value-1)
			print "\033[1;33mCurrent reward: {} \t Return: {}\033[0m".format(current_reward, return_)
			color_name, depth_name, next_color_name, next_depth_name = utils.wrap_strings(image_path, depth_path, iteration.value-1)
			next_state_thread.join(); is_empty = empty_state.value
			action_target.append(action_obj[4]); is_empty_list.append(is_empty)
			transition = Transition(color_name, depth_name, action_obj[4], current_reward, next_color_name, next_depth_name, is_empty)
			experience_queue.put(transition)
			if not is_empty and iteration.value < args.obj_nums*2: can_predict.value = True
		stop_record_client()
		
		if is_empty: 
			print("\033[1;33m[{}] Pass test with return: {}\033[0m".format(time.time()-program_ts, return_)) 
		else: 
			print("\033[1;31m[{}] Failed with return: {}\033[0m".format(time.time()-program_ts, return_))
		np.savetxt(root_path+"action_target.csv", action_target, delimiter=",")
		np.savetxt(root_path+"is_empty.csv", is_empty_list, delimiter=",")
		f = open(root_path+"{}.txt".format(encode_index(args.model_idx, args.episode, args.run, args.specific_tool)), 'w')
		f.write("{}\n".format(is_empty)); f.write("{}".format(return_)); f.close()
		action_target = []; is_empty_list = []
		cmd = raw_input("Press 'r' to reset, 'e' to exit: ")
		if cmd == 'e' or cmd == 'E':
			break
			
		elif cmd == 'r' or cmd == 'R':
			print("[Main Thread] Receive reset command")
			ready.value = False
			should_reset.value = True
			args.run += 1
			root_path, image_path, depth_path, pc_path, vis_path, grasp_path, mixed_paths, feat_paths = create_directories(package_path, args.model_idx, args.episode, args.run, args.specific_tool)
			path_queue.put([image_path, depth_path, pc_path, vis_path, feat_paths, mixed_paths])
			is_empty = False
			pick_item = 0
			return_ = 0.0
			iteration.value = 0
			# Wait until prediction thread ready
			while not ready.value: pass
			program_ts = time.time()
			can_predict.value = True # Tell prediction thread we can start
			
	# Stop prediction thread
	work.value = False
	p.join()
	print("Main thread stop")
	
# Thread to get next state
def get_next_state(empty_state, iteration, all_picked_state, pc_path, image_path, depth_path):
	next_pc = _get_pc(iteration, False, pc_path)
	gen = pc2.read_points(next_pc.pc, skip_nans=False)
	while len(list(gen))<180000: # prevent to less points in view
		next_pc = _get_pc(iteration, False, pc_path)
		gen = pc2.read_points(next_pc.pc, skip_nans=False)
	next_color, next_depth, next_points = utils.get_heightmap(next_pc.pc, image_path + "next_", depth_path + "next_", iteration)
	empty_state.value = _check_if_empty(next_pc.pc)
	if all_picked_state: # The agent thinks that he has picked all items
		for i in range(2):
			next_pc = _get_pc(iteration, False, pc_path)
			empty_state.value = empty_state.value or _check_if_empty(next_pc.pc) # If one judgement says is empty, then it IS empty

# Code for calling service
def _get_pc(iteration, is_before, path, check_grasp=False):
	get_pc_client = rospy.ServiceProxy("/combine_pc_node/get_pc", get_pc)
	get_pc_req = get_pcRequest()
	if check_grasp:
		get_pc_req.file_name = path + "{:06}.pcd".format(iteration)
	elif is_before:
		get_pc_req.file_name = path + "{:06}_before.pcd".format(iteration)
	else:
		get_pc_req.file_name = path + "{:06}_after.pcd".format(iteration)
	return get_pc_client(get_pc_req)
def _viz(point, action, angle, valid):
	viz = rospy.ServiceProxy("/viz_marker_node/viz_marker", viz_marker)
	viz_req = viz_markerRequest()
	viz_req.point = utils.point_np_to_ros(point)
	viz_req.primitive = action
	viz_req.angle = angle
	viz_req.valid = valid
	viz(viz_req)
def _take_action(tool_id, point, angle):
	agent_take_action_client = rospy.ServiceProxy("/agent_server_node/agent_take_action", agent_abb_action)
	agent_action = agent_abb_actionRequest()
	agent_action.tool_id = tool_id
	agent_action.position = utils.point_np_to_ros(point)
	agent_action.angle = angle
	_ = agent_take_action_client(agent_action)
def _check_collide(point, angle):
	check_if_collide_client = rospy.ServiceProxy("/agent_server_node/check_if_collide", agent_abb_action)
	agent_action = agent_abb_actionRequest()
	agent_action.tool_id = 1
	agent_action.position = utils.point_np_to_ros(point)
	agent_action.angle = angle
	res = check_if_collide_client(agent_action)
	if res.result == "true":
		return True # Will collide
	else:
		return False # Won't collide
def _check_if_empty(pc):
	empty_checker = rospy.ServiceProxy("/combine_pc_node/empty_state", pc_is_empty)
	empty_checker_req = pc_is_emptyRequest()
	empty_checker_req.input_pc = pc
	return empty_checker(empty_checker_req).is_empty.data
def _check_grasp_success(iter, path):
	go_home = rospy.ServiceProxy("/agent_server_node/go_home", Empty)
	calibrate_gripper_client = rospy.ServiceProxy("/change_tool_service/calibrate_gripper", SetBool)
	check_grasp_client = rospy.ServiceProxy("/combine_pc_node/grasp_state", check_grasp_success)
	go_home(); rospy.sleep(0.5)
	calibrate_req = SetBoolRequest()
	calibrate_res = calibrate_gripper_client(calibrate_req)
	if not calibrate_res.success: return calibrate_res.success
	check_grasp_success_request = check_grasp_successRequest()
	# Get temporary pc and save file
	_ = _get_pc(iter, False, path, True)
	check_grasp_success_request.pcd_str = path + "{:06}.pcd".format(iter)
	return check_grasp_client(check_grasp_success_request).is_success
	
if __name__ == "__main__":
	main()
	
