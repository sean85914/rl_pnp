#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from grasp_suck.srv import get_pose, get_poseRequest, get_poseResponse
from vacuum_conveyor_control.srv import vacuum_control, vacuum_controlRequest

s = rospy.ServiceProxy("/helper_services_node/goto_target", get_pose)
h = rospy.ServiceProxy("/helper_services_node/robot_go_home", Empty)
p = rospy.ServiceProxy("/helper_services_node/robot_goto_place", Empty)
o = rospy.ServiceProxy("/robotiq_finger_control_node/open_gripper", Empty)
v = rospy.ServiceProxy("/arduino_control/vacuum_control", vacuum_control)
r = rospy.ServiceProxy("/arduino_control/pheumatic_control", SetBool)

pub = rospy.Publisher("/test_marker", Marker, queue_size=1)

def pub_marker(msg):
	marker = Marker()
	marker.action = Marker.ADD
	marker.type = Marker.SPHERE
	marker.header.frame_id = msg.header.frame_id
	marker.scale.x = marker.scale.y = marker.scale.z = 0.015
	marker.color.r = marker.color.a = 1.0
	marker.pose.position = msg.point
	marker.pose.orientation.w = 1.0
	pub.publish(marker)

def cb(msg):
	if msg.header.frame_id != "base_link":
		rospy.logwarn("Invalid frame, ignore...")
		return
	pub_marker(msg)
	req = get_poseRequest()
	req.point_in_hand = msg.point
	pri = int(raw_input("Get message, which primitive to do? [0: grasp, 1: suck] \n"))
	if pri != 0 and pri != 1:
		rospy.logwarn("Invalid input, ignore...")
		return
	req.primitive = pri
	if pri == 0:
		yaw = int(raw_input("Input index for grasp: [0: -90; 1: -45; 2:0; 3: 45]\n"))
		if yaw != 0 and yaw != 1 and yaw != 2 and yaw != 3:
			rospy.logwarn("Invalid input, ignore...")
			return
		req.yaw = np.radians(-90+45*yaw)
	s(req)
	command = raw_input("Next action: [h: go home; p: go to place; o: open gripper; r: retract the cylinder; v: turn off vacuum; e: exit]")
	while command is not 'e':
		if command is 'h':
			h()
		elif command is 'p':
			p()
		elif command is 'o':
			o()
		elif command is 'r':
			r(False)
		elif command is 'v':
			v(1)
			rospy.sleep(0.5)
			v(0)
		elif command is 'e':
			break
		command = raw_input("Next action: [h: go home; p: go to place; o: open gripper; r: retract the cylinder; v: turn off vacuum; e: exit]")
	
	
rospy.init_node("test_calibration_node")
sub = rospy.Subscriber("/clicked_point", PointStamped, cb)

rospy.spin()

