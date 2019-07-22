#!/usr/bin/env python
import numpy as np
import rospy
from robotiq_ft_sensor.msg import ft_sensor
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

fx = []
fy = []
fz = []
lock = True
last_force = 0.0
has_last = False
wait_time = 0.1
G = 9.806650000000272

def set_prior(req):
	global fx, fy, fz, lock, last_force, has_last
	fx = []
	fy = []
	fz = []
	lock = False
	rospy.sleep(wait_time)
	lock = True
	norm = np.linalg.norm(np.array([fx, fy, fz]))
	print "Prior: %f (N)" %(norm)
	last_force = norm
	has_last = True
	return EmptyResponse()

def get_diff(req):
	global fx, fy, fz, lock, last_force, has_last
	if not has_last:
		print "Not set prior yet, ignore..."
		return EmptyResponse()
	fx = []
	fy = []
	fz = []
	lock = False
	rospy.sleep(wait_time)
	lock = True
	norm = np.linalg.norm(np.array([fx, fy, fz]))
	print "Post: %f (N)" %(norm)
	diff = abs(last_force-norm)
	diff_gm = diff*1000/G
	print "Diff: %f (N), %f (gm)" %(diff, diff_gm)
	has_last = False
	return EmptyResponse()

def cb_sub(msg):
	global fx, fy, fz, lock
	if not lock:
		fx.append(msg.Fx)
		fy.append(msg.Fy)
		fz.append(msg.Fz)
		

def main():
	rospy.init_node("get_weight_node")
	sub = rospy.Subscriber("/robotiq_ft_sensor", ft_sensor, cb_sub, queue_size=1)
	s_1 = rospy.Service("~set_prior_data", Empty, set_prior)
	s_2 = rospy.Service("~get_diff", Empty, get_diff)
	rospy.spin()

if __name__ == "__main__":
	main()
