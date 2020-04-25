import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, \
                         Empty, EmptyRequest, EmptyResponse
from arm_operation.srv import agent_abb_action, agent_abb_actionRequest, agent_abb_actionResponse, \
							  publish_info, publish_infoRequest, publish_infoResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
							  pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse, \
							  check_grasp_success, check_grasp_successRequest, check_grasp_successResponse
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse

class ActionWrapper:
	def __init__(self):
		self.vacuum_pump_control = rospy.ServiceProxy("/vacuum_pump_control_node/vacuum_control", SetBool)
		self.check_suck_success = rospy.ServiceProxy("/vacuum_pump_control_node/check_suck_success", SetBool)
		self.agent_take_action_client = rospy.ServiceProxy("/agent_server_node/agent_take_action", agent_abb_action)
		self.calibrate_gripper_client = rospy.ServiceProxy("/change_tool_service/calibrate_gripper", SetBool)
		self.empty_checker = rospy.ServiceProxy("/combine_pc_node/empty_state", pc_is_empty)
		self.check_grasp_success = rospy.ServiceProxy("/combine_pc_node/grasp_state", check_grasp_success)
		self.get_pc_client = rospy.ServiceProxy("/combine_pc_node/get_pc", get_pc)
		self.go_home = rospy.ServiceProxy("/agent_server_node/go_home", Empty)
		self.go_place = rospy.ServiceProxy("/agent_server_node/go_place", Empty)
		self.fixed_home = rospy.ServiceProxy("/agent_server_node/go_home_fix_orientation", Empty)
		self.check_if_collide_client = rospy.ServiceProxy("/agent_server_node/check_if_collide", agent_abb_action)
		self.viz = rospy.ServiceProxy("/viz_marker_node/viz_marker", viz_marker)
		self.publish_data_client = rospy.ServiceProxy("/agent_server_node/publish_data", publish_info)
	def get_pc(self, filename):
		# Save pointcloud as pcd file
		# @filename: saved file name
		get_pc_req = get_pcRequest()
		get_pc_req.file_name = filename
		self.get_pc_client(get_pc_req)
		
	def take_action(self, tool_id, point, angle):
		# Take action
		# @param tool_id: which tool to use, 1: gripper; 2: bagcup suction; 3: small suction
		# @param point: target position in arm coordinate, in numpy.ndarray, with size (3, ), with order x, y and z
		# @param angle: tool angle, in radians
		agent_action = agent_abb_actionRequest()
		agent_action.tool_id = tool_id
		agent_action.position.x = point[0]
		agent_action.position.y = point[1]
		agent_action.position.z = point[2]
		agent_action.angle = angle
		viz_req = viz_markerRequest()
		viz_req.point = agent_action.position
		viz_req.primitive = 0 if tool_id==3 else 2
		viz_req.angle = angle
		viz_req.valid = True
		self.viz(viz_req)
		_ = self.agent_take_action_client(agent_action)
	def check_if_collide(self, point, angle):
		# Check if gripper will collide with the tote
		# @param point: target position in arm coordinate, in numpy.ndarray, with size (3, ), with order x, y and z
		# @param angle: tool angle, in radians
		agent_action = agent_abb_actionRequest()
		agent_action.tool_id = 1 # Hard code to gripper
		agent_action.position.x = point[0]
		agent_action.position.y = point[1]
		agent_action.position.z = point[2]
		agent_action.angle = angle
		res = self.check_if_collide_client(agent_action)
		return True if res.result=="true" else False
	def check_if_success(self, tool_id, filename):
		# Check if action success
		# @param tool_id: which tool is used in last execution
		# @param filename: if using gripper, save the pcd with given filename and check
		if tool_id==1: # gripper
			self.go_home(); rospy.sleep(0.5)
			calibrate_req = SetBoolRequest()
			calibrate_res = self.calibrate_gripper_client(calibrate_req)
			if not calibrate_res.success: return calibrate_res.success
			self.get_pc(filename)
			check_grasp_success_request = check_grasp_successRequest()
			check_grasp_success_request.pcd_str = filename
			action_success = self.check_grasp_success(check_grasp_success_request).is_success
		else: # suction
			action_success = self.check_suck_success().success
		return action_success
	def check_if_empty(self, filename):
		# Check if workspace is empty
		# @param filename: pcd file to check
		empty_checker_req = pc_is_emptyRequest()
		empty_checker_req.pcd_file = filename
		return self.empty_checker(empty_checker_req).is_empty.data
	def publish_data(self, iteration, primitive_idx, is_success):
		info = publish_infoRequest()
		info.execution.iteration = iteration
		info.execution.is_success = is_success
		if primitive_idx == 0:
			info.execution.primitive = "suck_1"
		elif primitive_idx == 1:
			info.execution.primitive = "grasp_0"
		elif primitive_idx == 2:
			info.execution.primitive = "grasp_neg_45"
		elif primitive_idx == 3:
			info.execution.primitive = "grasp_neg_90"
		elif primitive_idx == 4:
			info.execution.primitive = "grasp_45"
		elif primitive_idx == -1:
			info.execution.primitive = "invalid"
		self.publish_data_client(info)
	def reset(self):
		self.fixed_home()
		self.vacuum_pump_control(SetBoolRequest(False))
	def place(self):
		self.go_place()
		self.fixed_home()
