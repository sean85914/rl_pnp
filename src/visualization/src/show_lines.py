#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

x_upper = 1.00
x_lower = 0.80
y_upper = -0.27
y_lower = -0.42
z_upper = 0.27
z_lower = 0.14

point_list = list([Point(x_upper, y_upper, z_upper), 
                  Point(x_lower, y_upper, z_upper),
                  Point(x_lower, y_lower, z_upper), 
                  Point(x_upper, y_lower, z_upper), 
                  Point(x_upper, y_upper, z_lower), 
                  Point(x_lower, y_upper, z_lower),
                  Point(x_lower, y_lower, z_lower), 
                  Point(x_upper, y_lower, z_lower)])

def form_lines():
	marker = Marker()
	marker.header.frame_id = "base_link"
	marker.action = Marker.ADD
	marker.type = Marker.LINE_STRIP
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.01; marker.scale.y = 0.01; marker.scale.z = 0.01
	marker.color.r = 1.0; marker.color.a = 1.0
	marker.points.append(point_list[0])
	marker.points.append(point_list[1])
	marker.points.append(point_list[2])
	marker.points.append(point_list[3])
	marker.points.append(point_list[0])
	marker.points.append(point_list[4])
	marker.points.append(point_list[5])
	marker.points.append(point_list[1])
	marker.points.append(point_list[5])
	marker.points.append(point_list[6])
	marker.points.append(point_list[2])
	marker.points.append(point_list[6])
	marker.points.append(point_list[7])
	marker.points.append(point_list[3])
	marker.points.append(point_list[7])
	marker.points.append(point_list[4])
	return marker
	
def main():
	rospy.init_node("show_range_node")
	pub = rospy.Publisher("detect_range", Marker, queue_size=1)
	marker = form_lines()
	while not rospy.is_shutdown():
		pub.publish(marker)
		
if __name__ == "__main__":
	main()
