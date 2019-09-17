import os
import sys
sys.path.insert(1, os.path.join(sys.path[0], '..'))
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
parser.add_argument("--model", type=str, default="", help="Model to test")
parser.add_argument("--online", action="store_true", default=False)
parser.add_argument("--color_img_path", type=str)
args = parser.parse_args()

if args.model == "":
	print "\033[1;31mNo model provided, exit...\033[0m"
	sys.exit()

# Parameter
testing      = True
use_cpu      = False
episode      = 0
iteration    = 0
suck_reward  = 2.0
grasp_reward = 2.0
discount     = 0.5

trainer = Trainer(suck_reward, grasp_reward, discount, testing, use_cpu)
print "Loading model..."
trainer.model.load_state_dict(torch.load(args.model))
print "Complete!"

if args.online:
	get_pc_client = rospy.ServiceProxy("/pc_transform/get_pc", get_pc)
	pc_req = get_pcRequest()
	pc_res = get_pc_client()
	color, depth, points, depth_img_msg = utils.get_heightmap(pc_res.pc, "", 0)
	
else:
	_, depth_img_path, _, _, _, _, _ = utils.get_file_path(args.color_img_path)
	color = cv2.imread(args.color_img_path)
	depth = cv2.imread(depth_img_path, -1)
suck_predictions, grasp_predictions, state_feat = \
                          trainer.forward(color, depth, is_volatile=True)
suck_predictions, grasp_predictions = utils.standarization(suck_predictions, grasp_predictions)
suck_heatmap = utils.vis_affordance(suck_predictions[0])
suck_mixed = cv2.addWeighted(color, 1.0, suck_heatmap, 0.4, 0)
tmp = np.where(suck_predictions == np.max(suck_predictions))
best_pixel = [tmp[0][0], tmp[1][0], tmp[2][0]]
suck_img = utils.draw_image(suck_mixed, 1, best_pixel)
cv2.imwrite("suck.jpg", suck_img)
grasp_mixed = []
for i in range(len(grasp_predictions)):
	grasp_heatmap = utils.vis_affordance(grasp_predictions[i])
	name = "grasp_{}.jpg".format(i)
	grasp_mixed_idx = cv2.addWeighted(color, 1.0, grasp_heatmap, 0.4, 0)
	tmp = np.where(grasp_predictions == np.max(grasp_predictions[i]))
	best_pixel = [tmp[0][0], tmp[1][0], tmp[2][0]]
	grasp_img = utils.draw_image(grasp_mixed_idx, 0, best_pixel)
	cv2.imwrite("grasp_{}.jpg".format(i), grasp_img)
	grasp_mixed.append(grasp_mixed_idx)

#action, action_str, pixel_index, angle = utils.greedy_policy(suck_predictions, grasp_predictions)
#visual_img = None
#if action:
#	visual_img = utils.draw_image(suck_mixed, action, pixel_index)
#else:
#	visual_img = utils.draw_image(grasp_mixed[pixel_index[0]], action, pixel_index)

print "Grasp max: %f %f %f %f" % (np.max(grasp_predictions[0]), np.max(grasp_predictions[1]), np.max(grasp_predictions[2]), np.max(grasp_predictions[3]))
print "Suck max:  %f" % np.max(suck_predictions)
