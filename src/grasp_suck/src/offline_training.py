import os
import sys
import time
import numpy as np
import random
import re
import cv2
import argparse
import torch
import rospy
import rospkg
import utils
# Network
from trainer import Trainer

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="learning pick and place by trial and error using DQN")
parser.add_argument("--force_cpu",  action="store_true",  default=False, help="True if using CPU, default is false")
parser.add_argument("--update_target", type=int,          default=3,    help="After how much iterations should update target network")
parser.add_argument("--max_iter", type=int, default=1000)
parser.add_argument("--save_every", type=int, default=10, help="After how much iterations should save the network")
parser.add_argument("--model",         type=str,          default="",    help="If provided, continue training the model or using this model for testing, default is empty string")
parser.add_argument("--save_path", type=str, default="", help="Where to save the models")
parser.add_argument("--data_path", type=str, help="Data for training")

args = parser.parse_args()

use_cpu      = args.force_cpu
iteration    = 0
suck_reward  = 2.0
grasp_reward = 2.0
discount     = 0.5
update_fre = args.update_target
save_path  = args.save_path
data_path  = args.data_path
save_every = args.save_every

# Logger path
logger_dir = save_path + "/"
image_path =  logger_dir + 'images/'
mixed_path =  logger_dir + 'mixed_img/'
feat_path  =  logger_dir + 'feat/'
grasp_path = [feat_path + 'rotate_idx_0/',
              feat_path + 'rotate_idx_1/',
              feat_path + 'rotate_idx_2/',
              feat_path + 'rotate_idx_3/',
              feat_path + 'rotate_idx_4/']
model_path =  logger_dir + 'models/'
vis_path   =  logger_dir + 'vis/'

enum_dir = os.listdir(data_path)
color_matcher = re.compile("color_.*.jpg")

if not os.path.exists(logger_dir):
	os.makedirs(logger_dir)
if not os.path.exists(image_path):
	os.makedirs(image_path)
if not os.path.exists(mixed_path):
	os.makedirs(mixed_path)
if not os.path.exists(feat_path):
	os.makedirs(feat_path)
if not os.path.exists(save_path):
	os.makedirs(save_path)
if not os.path.exists(model_path):
	os.makedirs(model_path)
for i in range(5):
	if not os.path.exists(grasp_path[i]):
		os.makedirs(grasp_path[i])
# trainer
trainer = Trainer(suck_reward, grasp_reward, discount, False, use_cpu)
# Load model if provided last model
if args.model != "":
	print "[%f]: Loading provided model..." %(time.time())
	trainer.model.load_state_dict(torch.load(args.model))

while iteration < args.max_iter:
	print "[%f] Iteration: %d"%(time.time(), iteration)
	select_dir = random.choice(enum_dir)
	image_list = os.listdir(data_path+"/"+select_dir+"/images")
	color_images = list(filter(color_matcher.match, image_list))
	select_image = random.choice(color_images)
	depth_image  = select_image.replace("color", "depth"); depth_image = depth_image.replace("jpg", "png")
	next_color_image = select_image.replace("color", "next_color")
	next_depth_image = select_image.replace("color", "next_depth"); next_depth_image = next_depth_image.replace("jpg", "png")
	color_image_name = data_path+"/"+select_dir+"/images/"+select_image
	depth_image_name = data_path+"/"+select_dir+"/images/"+depth_image
	next_color_image_name = data_path+"/"+select_dir+"/images/"+next_color_image
	next_depth_image_name = data_path+"/"+select_dir+"/images/"+next_depth_image
	primitive_csv = data_path+"/"+select_dir+"/action_primitive.csv"
	result_csv    = data_path+"/"+select_dir+"/action_result.csv"
	target_csv    = data_path+"/"+select_dir+"/action_target.csv"
	color = cv2.imread(color_image_name)
	depth = cv2.imread(depth_image_name, -1)
	next_color = cv2.imread(next_color_image_name)
	next_depth = cv2.imread(next_depth_image_name, -1)
	iter_ = select_image.replace("color_", ""); iter_ = iter_.replace(".jpg", ""); iter_ = int(iter_)
	action_primitive = np.loadtxt(primitive_csv, delimiter=",")[iter_]
	pixel_index = np.loadtxt(target_csv, delimiter=",")[iter_]
	pixel_index = list([int(pixel_index[0]), int(pixel_index[1]), int(pixel_index[2])])
	action_result = np.loadtxt(result_csv, delimiter=",")[iter_]
	if action_result == -1: action_str = "invalid"
	elif action_result == 1: action_str = "suck"
	else: action_str = "grasp"
	label_value, prev_reward_value = trainer.get_label_value(action_str, action_result, next_color, next_depth, False)
	loss_value = trainer.backprop(color, depth, action_str, pixel_index, label_value)
	iteration+=1
	if iteration % update_fre == 0:
		trainer.target = trainer.model
	if iteration % save_every == 0:
		model_name = model_path + "/offline_{}.pth".format(iteration)
		torch.save(trainer.model.state_dict(), model_name)
		print "Model: {} saved".format(model_name) 
