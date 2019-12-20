#!/usr/bin/env python
import rospy
from abb_node.srv import robot_SetCartesian, robot_SetCartesianRequest, \
                         robot_GetJoints, robot_GetJointsResponse, \
                         robot_SetJoints, robot_SetJointsRequest
from geometry_msgs.msg import PointStamped

def cb(point):
	if point.header.frame_id == "base_link":
		get_joints = rospy.ServiceProxy("/abb/GetJoints", robot_GetJoints)
		now = get_joints()
		set_cartesian = rospy.ServiceProxy("/abb/SetCartesian", robot_SetCartesian)
		goal = robot_SetCartesianRequest()
		goal.cartesian = [point.point.x*1000.0, point.point.y*1000.0, (point.point.z+0.555)*1000.0]
		goal.quaternion = [0.0, 0.0, 1.0, 0.0] # Top-down
		set_cartesian(goal)
		s = raw_input("Press any key to go back...")
		set_joints = rospy.ServiceProxy("/abb/SetJoints", robot_SetJoints)
		goal = robot_SetJointsRequest()
		goal.position = [now.j1, now.j2, now.j3, now.j4, now.j5, now.j6]
		set_joints(goal)
	else:
		print "Please make sure RViz frame in `base_link`, abort request..."

def main():
	rospy.init_node("test_calibration")
	sub = rospy.Subscriber("/clicked_point", PointStamped, cb)
	rospy.spin()
	
if __name__ == "__main__":
	main()
