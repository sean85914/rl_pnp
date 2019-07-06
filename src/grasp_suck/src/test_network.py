import os
import sys
import time
#from threading import Thread, currentThread
import numpy as np
import cv2
import argparse
import torch

from trainer import Trainer

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="using color and depth images to do pick and place")
parser.add_argument("--is_testing", action="store_true", default=False)
parser.add_argument("--force_cpu",  action="store_true", default=False)
parser.add_argument("--last_model", type=str, default="", help="If provided, continue training with provided model file")

args = parser.parse_args()

# Parameter
testing      = args.is_testing
use_cpu      = args.force_cpu
episode      = 0
iteration    = 0
suck_reward  = 1.0
grasp_reward = 1.0
discount     = 0.5
Z_THRES      = 0.645
num_of_items = 10
save_every   = 5

trainer = Trainer(suck_reward, grasp_reward, discount, testing, use_cpu)

color = cv2.imread("test_img/color_000000.jpg")
next_color = cv2.imread("test_img/color_000001.jpg")
depth = cv2.imread("test_img/depth_000000.png", -1)
next_depth = cv2.imread("test_img/depth_000001.png", -1)

total = 0
ts = time.time()
suck_predictions, grasp_predictions, state_feat = \
                          trainer.forward(color, depth, is_volatile=True)
print "Forward: {}".format(time.time()-ts)
total += time.time()-ts

action_str = "grasp"
action_success = False

ts = time.time()
label_value, prev_reward_value = trainer.get_label_value(action_str, action_success, \
		                                                         next_color, next_depth)
print "Get label: {}".format(time.time()-ts)
total += time.time()-ts
pixel_index = [1, 112, 112]
ts = time.time()
trainer.backprop(color, depth, action_str, pixel_index, label_value)
print "Backpropagation: {}".format(time.time()-ts)
total += time.time()-ts

print "Total time: {}".format(total)
