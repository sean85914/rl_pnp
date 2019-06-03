#!/usr/bin/env python

'''
  Check if there is any barcode in an image
'''

import os
import sys
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visual_system.srv import barcode_detect, barcode_detectRequest, barcode_detectResponse
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.backends.cudnn as cudnn
from torch.autograd import Variable

if torch.cuda.is_available():
	torch.set_default_tensor_type('torch.cuda.FloatTensor')
else:
	torch.set_default_tensor_type('torch.FloatTensor')
from ssd import build_ssd 

class SSDPrediction(object):
	def __init__(self):
		self.net = build_ssd('test', 300, 2)
		self.bridge = CvBridge()
		self.image1 = None
		self.image2 = None
		self.image_to_plot = None
		self.rospack = rospkg.RosPack()
		pkg_path = self.rospack.get_path('visual_system')
		self.model_name = rospy.get_param("~model_name", "ssd300_BARCODE_21500.pth")
		rospy.loginfo("model: %s"%(self.model_name))
		# Only considered as target when confidence greater than
		self.confidence_thres = rospy.get_param("~confidence_thres", 0.5) 
		rospy.loginfo("confidence_threshold: %f"%(self.confidence_thres))
		self.model_path = pkg_path + "/model/" + self.model_name
		self.net.load_weights(self.model_path)
		if torch.cuda.is_available():
			self.net.cuda()
		self.sub_img1 = rospy.Subscriber("/camera1/color/image_raw", Image, self.img1_cb)
		self.sub_img2 = rospy.Subscriber("/camera2/color/image_raw", Image, self.img2_cb)
		self.pub_img_with_bb = rospy.Publisher("~prediction_res", Image, queue_size = 10)
		self.detection_service = rospy.Service("~barcode_detection", barcode_detect, self.service_cb)

		rospy.loginfo("Finish loading weights...")
		
	# Subscriber callback for camera 1, save image in self.image1
	def img1_cb(self, msg):
		try:
			self.image1 = self.bridge.imgmsg_to_cv2(msg, "rgb8") # In RGB
		except CvBridgeError as e:
			print(e)
	# Subscriber callback for camera 2, save image in self.image2
	def img2_cb(self, msg):
		try:
			self.image2 = self.bridge.imgmsg_to_cv2(msg, "rgb8") # In RGB
		except CvBridgeError as e:
			print(e)
	# Service callback
	def service_cb(self, req):
		res = barcode_detectResponse()
		res.result = False
		if(req.camera_id): # camera1
			if(self.image1 is not None):
				self.img_to_net = self.img_preprocessing(self.image1)
				self.image_to_plot = self.image1
			else:
				rospy.logerr("No image receive from camera 1 yet, aborting...")
				return res
		else: # camera 2
			if(self.image2 is not None):
				self.img_to_net = self.img_preprocessing(self.image2)
				self.image_to_plot = self.image2
			else:
				rospy.logerr("No image receive from camera 2 yet, aborting...")
				return res
		var = Variable(self.img_to_net.unsqueeze(0)) # Wrap tensor in Variable
		if torch.cuda.is_available():
			var = var.cuda()
		prediction_res = self.net(var) # Forward pass
		detection = prediction_res.data
		scale = torch.Tensor(self.image_to_plot.shape[1::-1]).repeat(2)
		for i in range(detection.size(1)):
			j = 0
			while detection[0, i, j, 0].cpu().numpy() >= self.confidence_thres:
				res.result = True
				score = detection[0, i, j, 0]
				display_txt = '%d'%(score*100.)
				pt = (detection[0, i, j, 1:]*scale).cpu().numpy()
				pt = [int(p) for p in pt]
				# Draw rectangle
				cv2.rectangle(self.image_to_plot, (pt[0], pt[1]), (pt[2], pt[3]), \
						(0, 0, 255), 3)
				# Put text
				font = cv2.FONT_HERSHEY_SIMPLEX
				cv2.putText(self.image_to_plot, display_txt, (pt[0], pt[1]), font, \
						3, (255, 0, 0), cv2.LINE_AA)
				j += 1
		if(res.result == True):
			rospy.loginfo("Detect barcode")
		else:	rospy.loginfo("No barcode detected")
		# Publish drawed image
		try:
			self.image_to_plot = cv2.cvtColor(self.image_to_plot, cv2.COLOR_BGR2RGB)
			self.pub_img_with_bb.publish(self.bridge.cv2_to_imgmsg(self.image_to_plot))
		except CvBridgeError as e:
			print(e)

		return res

	# Preprocessing for SSD
	def img_preprocessing(self, img):
		res = cv2.resize(img, (300, 300)).astype(np.float32)
		res -= (104., 117., 123.)
		res = res.astype(np.float32)
		res = res[:, :, ::-1].copy()
		res = torch.from_numpy(res).permute(2, 0, 1)
		return res

def main(args):
	rospy.init_node("barcode_prediction_node", anonymous = False)
	barcode = SSDPrediction()
	try:
		rospy.spin()
	except KeyBoardInterrupt:
		rospy.loginfo("Shutting down")

if __name__ == "__main__":
	main(sys.argv)
