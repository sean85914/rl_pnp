#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

def main():
	rospy.init_node("visual_boundary_node")
	pub = rospy.Publisher("visual_boundary", Marker, queue_size=1)
	marker = Marker()
	p = [Point(-0.659, -0.269, 0.0), Point(-0.659,  0.117, 0.0), \
		 Point(-0.273,  0.117, 0.0), Point(-0.273, -0.269, 0.0)]
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
