import os
import sys
import time
import numpy as np
import cv2
import rospy
import argparse
import torch
import utils
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse

from trainer import Trainer

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="Testing trained model")
parser.add_argument("--model", type=str, help="Model to test")
parser.add_argument("--online", action="store_true", default=False)
parser.add_argument("--color_img_path", type=str)
parser.add_argument("--depth_img_path", type=str)
args = parser.parse_args()
# Parameter
testing      = True
use_cpu      = False
episode      = 0
iteration    = 0
suck_reward  = 1.0
grasp_reward = 1.0
discount     = 0.5

trainer = Trainer(suck_reward, grasp_reward, discount, testing, use_cpu)
print "Loading model..."
trainer.model.load_state_dict(torch.load(args.model))
print "Complete!"

color_heightmap = None
depth_heightmap = None

if args.online:
	print "hello"
	get_pc_client = rospy.ServiceProxy("/pc_transform/get_pc", get_pc)
	pc_req = get_pcRequest()
	pc_res = get_pc_client()
	color, depth, points, depth_img_msg = utils.get_heightmap(pc_res.pc, "", 0)
	
else:
	color = cv2.imread(args.color_img_path)
	depth = cv2.imread(args.depth_img_path, -1)
suck_predictions, grasp_predictions, state_feat = \
                          trainer.forward(color, depth, is_volatile=True)
suck_predictions, grasp_predictions = utils.standarization(suck_predictions, grasp_predictions)
suck_heatmap = utils.vis_affordance(suck_predictions[0])
suck_mixed = cv2.addWeighted(color, 1.0, suck_heatmap, 0.4, 0)
cv2.imwrite("suck.jpg", suck_heatmap)
grasp_mixed = []
for i in range(len(grasp_predictions)):
	grasp_heatmap = utils.vis_affordance(grasp_predictions[i])
	name = "grasp_{}.jpg".format(i)
	cv2.imwrite(name, grasp_heatmap)
	grasp_mixed_idx = cv2.addWeighted(color, 1.0, grasp_heatmap, 0.4, 0)
	grasp_mixed.append(grasp_mixed_idx)

action, action_str, pixel_index, angle = utils.greedy_policy(suck_predictions, grasp_predictions)
visual_img = None
if action:
	visual_img = utils.draw_image(suck_mixed, action, pixel_index)
else:
	visual_img = utils.draw_image(grasp_mixed[pixel_index[0]], action, pixel_index)
cv2.imwrite("vis_0.jpg", visual_img)
