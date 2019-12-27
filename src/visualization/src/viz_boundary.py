#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

def main():
	rospy.init_node("visual_boundary_node")
	pub = rospy.Publisher("visual_boundary", Marker, queue_size=1)
	marker = Marker()
	x_lower = rospy.get_param("/combine_pc_node/x_lower")
	x_upper = rospy.get_param("/combine_pc_node/x_upper")
	y_lower = rospy.get_param("/combine_pc_node/y_lower")
	y_upper = rospy.get_param("/combine_pc_node/y_upper")
	p = [Point(x_lower, y_lower, 0.0), Point(x_lower,  y_upper, 0.0), \
		 Point(x_upper,  y_upper, 0.0), Point(x_upper, y_lower, 0.0)]
	marker.header.frame_id = "base_link"
	marker.type = Marker.LINE_STRIP
	marker.action = Marker.ADD
	marker.pose.orientation.w = 1.0
	marker.color.b = marker.color.a = 0.8
	marker.scale.x = 0.01
	marker.scale.y = 0.01
	marker.scale.z = 0.01
	for i in range(5):
		marker.points.append(p[i%4])
	r = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		pub.publish(marker)
		r.sleep()
	rospy.spin()
		  

if __name__ == "__main__":
	main()
