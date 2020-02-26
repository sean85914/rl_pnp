#!/usr/bin/env python
from math import pi
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization.srv import viz_marker, viz_markerRequest, viz_markerResponse

def service_cb(req):
	marker = Marker() # Sphere
	text_marker = Marker() # Text
	marker.header.stamp = rospy.Time.now()
	marker.header.frame_id = "base_link"
	text_marker.header.stamp = rospy.Time.now()
	text_marker.header.frame_id = "base_link"
	marker.action = Marker.ADD
	marker.type = Marker.SPHERE
	text_marker.type = Marker.TEXT_VIEW_FACING
	marker.pose.position = req.point
	marker.pose.orientation.w = 1.0
	marker.scale.x = marker.scale.y = marker.scale.z = 0.02
	text_marker.pose.position.y = 0.4
	text_marker.scale.z = 0.06
	if req.primitive==0: # suck 1
		marker.color.b = marker.color.a = 1.0 # blue
	elif req.primitive==1: # suck 2
		marker.color.g = marker.color.a = 1.0 # green
	else: # grasp
		marker.color.r = marker.color.a = 1.0
	text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0
	if req.primitive == 0 and req.valid: # suck 1
		text_marker.text = "suck_1"
	elif req.primitive == 1 and req.valid: # suck 2
		text_marker.text = "suck_2"
	elif req.primitive == 2 and req.valid: # GRASP
		angle = round(req.angle * 180 / pi)
		text_marker.text = "grasp, " + str(angle)
	elif not req.valid: # INVALID
		text_marker.text = "invalid target"
	pub_marker.publish(marker)
	pub_text_marker.publish(text_marker)
	return viz_markerResponse()

def main():
	rospy.init_node("viz_marker_node")
	global pub_marker, pub_text_marker
	pub_marker = rospy.Publisher("~marker", Marker, queue_size=1) # Sphere
	pub_text_marker = rospy.Publisher("~text", Marker, queue_size=1) # Text
	s = rospy.Service("~viz_marker", viz_marker, service_cb)
	rospy.spin()

if __name__ == "__main__":
	main()
