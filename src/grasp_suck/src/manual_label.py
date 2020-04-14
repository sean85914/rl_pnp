import utils
from utils import Transition
import os
import sys
import cv2
import time
import numpy as np
import yaml
import torch
import rospy
import rospkg
import serial
from trainer import Trainer
from prioritized_memory import Memory
from geometry_msgs.msg import PointStamped

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

# Data from YAML
yaml_path = "/home/sean/Documents/flip_obj/src/visual_system/config/param_config.yaml"
with open(yaml_path, "r") as stream:
	data = yaml.load(stream)

x_lower = data['x_lower']
x_upper = data['x_upper']
y_lower = data['y_lower']
y_upper = data['y_upper']
z_lower = data['z_lower']
z_upper = data['z_upper']
resolution = data['resolution']
workspace_limits = np.asarray([[x_lower, x_upper], [y_lower, y_upper], [z_lower, z_upper]]) # X Y Z
heightmap_resolution = (workspace_limits[0][1]-workspace_limits[0][0])/resolution

# Constant
reward = 5.0
discount_factor = 0.5
iteration = 0
learned_times = 0
sufficient_exp = 0
buffer_size = 500

gripper_memory = Memory(buffer_size)
suction_1_memory = Memory(buffer_size)
suction_2_memory = Memory(buffer_size)

gripper_memory.load_memory("../training/mannual_label/gripper_memory.pkl")
suction_1_memory.load_memory("../training/mannual_label/suction_1_memory.pkl")
suction_2_memory.load_memory("../training/manual_label/suction_2_memory.pkl")

# Service clients
vacuum_pump_control      = rospy.ServiceProxy("/vacuum_pump_control_node/vacuum_control", SetBool)
check_suck_success       = rospy.ServiceProxy("/vacuum_pump_control_node/check_suck_success", SetBool)
agent_take_action_client = rospy.ServiceProxy("/agent_server_node/agent_take_action", agent_abb_action)
calibrate_gripper_client = rospy.ServiceProxy("/change_tool_service/calibrate_gripper", SetBool)
get_pc_client            = rospy.ServiceProxy("/combine_pc_node/get_pc", get_pc)
empty_checker            = rospy.ServiceProxy("/combine_pc_node/empty_state", pc_is_empty)
check_grasp_success      = rospy.ServiceProxy("/combine_pc_node/grasp_state", check_grasp_success)
go_home                  = rospy.ServiceProxy("/agent_server_node/go_home", Empty)
go_place                 = rospy.ServiceProxy("/agent_server_node/go_place", Empty)
fixed_home               = rospy.ServiceProxy("/agent_server_node/go_home_fix_orientation", Empty)
check_if_collide_client  = rospy.ServiceProxy("/agent_server_node/check_if_collide", agent_abb_action)
publish_data_client      = rospy.ServiceProxy("/agent_server_node/publish_data", publish_info)
viz                      = rospy.ServiceProxy("/viz_marker_node/viz_marker", viz_marker)
record_bag_client        = rospy.ServiceProxy("/autonomous_recording_node/start_recording", recorder)
stop_record_client       = rospy.ServiceProxy("/autonomous_recording_node/stop_recording", Empty)

r = rospkg.RosPack()
pkg_path = r.get_path("grasp_suck")
root_path = pkg_path + "/training/manual_label/"
image_path = pkg_path + "/training/manual_label/images/"
depth_path = pkg_path + "/training/manual_label/depth_data/"
pc_path = pkg_path + "/training/manual_label/pc/"
check_grasp_path = pkg_path + "/training/manual_label/grasp/"
angle_list = [-np.pi/2, -np.pi/4, 0, np.pi/4]
pixel_idx_list = []
is_empty_list = []
is_success_list = []

for path in list([image_path, depth_path, pc_path, check_grasp_path]):
	if not os.path.exists(path):
		os.makedirs(path)

# Code for calling service
def _get_pc(iteration, is_before, check_grasp=False):
	get_pc_req = get_pcRequest()
	if check_grasp:
		get_pc_req.file_name = check_grasp_path + "{:06}.pcd".format(iteration)
	elif is_before:
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
	
def _check_if_empty(pc):
	empty_checker_req = pc_is_emptyRequest()
	empty_checker_req.input_pc = pc
	return empty_checker(empty_checker_req).is_empty.data
def _check_grasp_success():
	go_home(); rospy.sleep(0.5)
	calibrate_req = SetBoolRequest()
	calibrate_res = calibrate_gripper_client(calibrate_req)
	if not calibrate_res.success: return calibrate_res.success
	check_grasp_success_request = check_grasp_successRequest()
	# Get temporary pc and save file
	_ = _get_pc(iteration, False, True)
	check_grasp_success_request.pcd_str = check_grasp_path + "{:06}.pcd".format(iteration)
	return check_grasp_success(check_grasp_success_request).is_success
	

def cb(msg):
	global iteration
	if msg.header.frame_id != "base_link":
		return
	if msg.point.x<=x_lower or msg.point.x>=x_upper or \
	   msg.point.y<=y_lower or msg.point.y>=y_upper:
		print "Out of range, abort..."
		return
	action_srv = agent_abb_actionRequest()
	req_tool_id = int(raw_input("Please input which tool you want to use: [1: gripper, 2:suction cup II, 3: suction cup III]"))
	if not req_tool_id<=3 and not req_tool_id>=1:
		print "Invalid input, abort..."
		return
	# State before action
	pc_response = _get_pc(iteration, True)
	color, depth, points = utils.get_heightmap(pc_response.pc, image_path, depth_path, iteration)
	action_srv.tool_id = req_tool_id
	action_srv.position.x = msg.point.x
	action_srv.position.y = msg.point.y
	action_srv.position.z = msg.point.z-0.05
	if req_tool_id==1:
		idx = int(raw_input("Angle index:\n0: -90, 1: -45, 2: 0, 3: 45 (degree)"))
		action_srv.angle = angle_list[idx]
	else:
		action_srv.angle = angle_list[2]
	if check_if_collide_client(action_srv).result == "true":
		print "Will collide, abort request..."
		return
	# Take action
	agent_take_action_client(action_srv)
	rospy.sleep(0.2)
	if req_tool_id==1:
		action_success = _check_grasp_success()
	else:
		action_success = check_suck_success().success
	is_success_list.append(action_success)
	if action_success:
		print "Success"
		go_place()
	else:
		print "Failed"
		fixed_home()
		vacuum_pump_control(SetBoolRequest(False))
	# State after action
	next_pc = _get_pc(iteration, False)
	next_color, next_depth, next_points = utils.get_heightmap(next_pc.pc, image_path + "next_", depth_path + "next_", iteration)
	is_empty = _check_if_empty(next_pc.pc)
	is_empty_list.append(is_empty)
	current_reward = utils.reward_judgement(reward, True, action_success)
	color_name, depth_name, next_color_name, next_depth_name = utils.wrap_strings(image_path, depth_path, iteration)
	pixel_x = np.floor((workspace_limits[0][1]-msg.point.x)/heightmap_resolution).astype(int)
	pixel_y = np.floor((workspace_limits[1][1]-msg.point.y)/heightmap_resolution).astype(int)
	print "@({}, {})".format(pixel_x, pixel_y)
	print "is empty: {}".format(is_empty)
	if req_tool_id==1: # Gripper
		pixel_index = [2+idx, pixel_x, pixel_y]
		transition = Transition(color_name, depth_name, pixel_index, current_reward, next_color_name, next_depth_name, is_empty)
		gripper_memory.add(transition)
		success_cnt = [0, 0, 0, 0]
		for i in range(gripper_memory.length):
			if gripper_memory.tree.data[i].reward == reward:
				success_cnt[gripper_memory.tree.data[i].pixel_idx[0]-2] += 1
		print success_cnt, sum(success_cnt)
		pixel_idx_list.append(pixel_index)
		
	elif req_tool_id==2: # Medium suction cup
		pixel_index = [1, pixel_x, pixel_y]
		transition = Transition(color_name, depth_name, pixel_index, current_reward, next_color_name, next_depth_name, is_empty)
		suction_2_memory.add(transition)
		success_cnt = 0
		for i in range(suction_2_memory.length):
			if suction_2_memory.tree.data[i].reward == reward:
				success_cnt += 1
		print success_cnt
		pixel_idx_list.append(pixel_index)
		
	else:
		pixel_index = [0, pixel_x, pixel_y]
		transition = Transition(color_name, depth_name, pixel_index, current_reward, next_color_name, next_depth_name, is_empty)
		suction_1_memory.add(transition)
		success_cnt = 0
		for i in range(suction_1_memory.length):
			if suction_1_memory.tree.data[i].reward == reward:
				success_cnt += 1
		print success_cnt
		pixel_idx_list.append(pixel_index)
	iteration += 1
	

if __name__ == "__main__":
	rospy.init_node("manual_collect_data")
	go_home()
	vacuum_pump_control(SetBoolRequest(False))
	sub = rospy.Subscriber("/clicked_point", PointStamped, cb)
	while not rospy.is_shutdown():
		rospy.spin()
	gripper_memory.save_memory(root_path, "gripper_memory.pkl")
	suction_2_memory.save_memory(root_path, "suction_2_memory.pkl")
	suction_1_memory.save_memory(root_path, "suction_1_memory.pkl")
	np.savetxt("action_target.csv", pixel_idx_list, delimiter=",")
	np.savetxt("empty_result.csv", is_empty_list, delimiter=",")
	np.savetxt("action_result.csv", is_success_list, delimiter=",")
