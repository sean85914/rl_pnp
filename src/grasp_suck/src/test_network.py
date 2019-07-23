import os
import sys
import time
#from threading import Thread, currentThread
import numpy as np
import cv2
import rospy
import argparse
import torch
import utils
from visual_system.srv import get_pc

from trainer import Trainer

# Parameter
testing      = True
use_cpu      = False
episode      = 0
iteration    = 0
suck_reward  = 1.0
grasp_reward = 1.0
discount     = 0.5

get_pc_client = rospy.ServiceProxy("/pixel_to_xyz/get_pc", get_pc)

trainer = Trainer(suck_reward, grasp_reward, discount, testing, use_cpu)
print "Loading model..."
trainer.model.load_state_dict(torch.load("../training_log/logger_64/models/final.pth"))
print "Complete!"

total = 0
ts = time.time()
res = get_pc_client()
print "get result: {}".format(time.time()-ts)
total += time.time()-ts
ts = time.time()
color, depth, points = utils.get_heightmap(res.pc, "", 0)
print "get heightmap: {}".format(time.time()-ts)
total += time.time()-ts
ts = time.time()

suck_predictions, grasp_predictions, state_feat = \
                          trainer.forward(color, depth, is_volatile=True)
print "Forward: {}".format(time.time()-ts)
total += time.time()-ts

suck_predictions, grasp_predictions = utils.standarization(suck_predictions, grasp_predictions)
suck_heatmap = utils.vis_affordance(suck_predictions[0])
cv2.imwrite("suck.jpg", suck_heatmap)
for i in range(len(grasp_predictions)):
	grasp_heatmap = utils.vis_affordance(grasp_predictions[i])
	name = "grasp_{}.jpg".format(i)
	cv2.imwrite(name, grasp_heatmap)

cv2.imshow("color", color)
cv2.waitKey(0)
