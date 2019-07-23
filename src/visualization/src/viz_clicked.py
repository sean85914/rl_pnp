#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

def cb(msg):
	global pub
	print "%f, %f, %f" %(msg.point.x, msg.point.y, msg.point.z)
	marker = Marker()
	marker.header.frame_id = msg.header.frame_id
	marker.action = Marker.ADD
	marker.type = Marker.SPHERE
	marker.scale.x = 0.01
	marker.scale.y = 0.01
	marker.scale.z = 0.01
	marker.color.r = 1.0
	marker.color.a = 1.0
	marker.pose.position = msg.point
	marker.pose.orientation.w = 1.0
	pub.publish(marker)

def main():
	global pub
	rospy.init_node("viz_clicked_point_node")
	sub = rospy.Subscriber("/clicked_point", PointStamped, cb)
	pub = rospy.Publisher("~clicked_point_marker", Marker, queue_size=1)
	rospy.spin()

if __name__ == "__main__":
	main()
