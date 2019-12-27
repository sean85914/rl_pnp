import os
import sys
from collections import deque
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
parser.add_argument("--update_target", type=int,          default=3,    help="After how much iterations should update target network, default is 3")
parser.add_argument("--max_iter", type=int, default=1000, help="How many iterations should train, default is 1000")
parser.add_argument("--save_every", type=int, default=10, help="After how much iterations should save the network, default is 10")
parser.add_argument("--model",         type=str,          default="",    help="If provided, continue training the model or using this model for testing, default is empty string")
parser.add_argument("--save_path", type=str, default="offline", help="Where to save the models, default is offline")
parser.add_argument("--data_path", type=str, help="Data for training")
parser.add_argument("--learn_every", type=int, default=10, help="Frequency for updating network, default is 10")
parser.add_argument("--learn_amount", type=int, default=3, help="How many trajectories to train for each learn, default is 3")

args = parser.parse_args()

if args.data_path == "":
	print "\033[1;31mNo data path provided, exit...\033[0m"
	sys.exit()

use_cpu      = args.force_cpu
iteration    = 0
suck_reward  = 4.0
grasp_reward = 4.0
discount     = 0.5
max_len      = 1000 
update_fre   = args.update_target
save_path    = args.save_path
data_path    = args.data_path
save_every   = args.save_every
learn_every  = args.learn_every
learn_amount = args.learn_amount

td_target_deque = deque(maxlen=max_len)
string_deque    = deque(maxlen=max_len)

# Logger path
logger_dir = save_path + "/"
image_path =  logger_dir + 'images/'
mixed_path =  logger_dir + 'mixed_img/'
feat_path  =  logger_dir + 'feat/'
grasp_path = [feat_path + 'rotate_idx_0/',
              feat_path + 'rotate_idx_1/',
              feat_path + 'rotate_idx_2/',
              feat_path + 'rotate_idx_3/']
model_path =  logger_dir + 'models/'
vis_path   =  logger_dir + 'vis/'

enum_dir = os.listdir(data_path)
color_matcher = re.compile("color_.*.jpg")

if not os.path.exists(save_path):
	os.makedirs(save_path)
if not os.path.exists(logger_dir):
	os.makedirs(logger_dir)
if not os.path.exists(model_path):
	os.makedirs(model_path)

# trainer
trainer = Trainer(suck_reward, grasp_reward, discount, False, use_cpu)
# Load model if provided last model
if args.model != "":
	print "[%f]: Loading provided model..." %(time.time())
	trainer.model.load_state_dict(torch.load(args.model))
	trainer.copyNetwork()

first_ts = time.time()

while iteration < args.max_iter:
	print "[%f, %f] Iteration: %d"%(time.time(), time.time()-first_ts, iteration)
	select_dir = random.choice(enum_dir)
	image_list = os.listdir(data_path+"/"+select_dir+"/images")
	color_images = list(filter(color_matcher.match, image_list))
	select_image = random.choice(color_images)
	color_image_name = data_path+"/"+select_dir+"/images/"+select_image
	iter_, depth_image_name, next_color_image_name, next_depth_image_name, \
        primitive_csv, result_csv, target_csv = utils.get_file_path(color_image_name)
	color = cv2.imread(color_image_name)
	depth = cv2.imread(depth_image_name, -1) # uint16
	depth = (depth.astype(float)/1000).astype(float)
	next_color = cv2.imread(next_color_image_name)
	next_depth = cv2.imread(next_depth_image_name, -1)
	next_depth = (next_depth.astype(float)/1000).astype(float)
	action_primitive = np.loadtxt(primitive_csv, delimiter=",")[iter_]
	pixel_index = np.loadtxt(target_csv, delimiter=",")[iter_]
	pixel_index = list([int(pixel_index[0]), int(pixel_index[1]), int(pixel_index[2])])
	action_result = np.loadtxt(result_csv, delimiter=",")[iter_]
	if action_primitive == -1: action_str = "invalid"
	elif action_primitive == 1: action_str = "suck"
	else: action_str = "grasp"
	label_value, prev_reward_value = trainer.get_label_value(action_str, action_result, next_color, next_depth, False)
	td_target_deque.append(label_value)
	string_deque.append(color_image_name)
	iteration+=1
	if iteration % learn_every == 0:
		index_list = np.arange(len(td_target_deque))
		random_sample_index = np.random.choice(index_list, learn_amount)
		tmp = np.sort(td_target_deque)
		print "Top five: %f, %f, %f, %f, %f" %(tmp[-1], tmp[-2], tmp[-3], tmp[-4], tmp[-5])
		random_sample_index[0] = np.argmax(td_target_deque)
		for i in range(learn_amount):
			sample_idx = random_sample_index[i]
			idx, depth_str, _, _, primitive_csv_str, result_csv_str, target_csv_str = \
				utils.get_file_path(string_deque[sample_idx])
			print "Using %s" %string_deque[sample_idx]
			color = cv2.imread(string_deque[sample_idx])
			depth = cv2.imread(depth_str, -1)
			depth = (depth.astype(float)/1000).astype(float)
			action_primitive = np.loadtxt(primitive_csv_str, delimiter=",")[idx]
			pixel_index = np.loadtxt(target_csv_str, delimiter=",")[idx]
			label_value = td_target_deque[sample_idx]
			if action_primitive == -1: action_str = "invalid"
			elif action_primitive == 1: action_str = "suck"
			else: action_str = "grasp"
			loss_value = trainer.backprop(color, depth, action_str, pixel_index, label_value)
		# remove the maximum one from deque
		td_target_deque.remove(td_target_deque[random_sample_index[0]])
		string_deque.remove(string_deque[random_sample_index[0]])
	if iteration % (update_fre*learn_every) == 0:
		trainer.target = trainer.model
	if iteration % (save_every*learn_every) == 0:
		model_name = model_path + "offline_{}.pth".format(iteration)
		torch.save(trainer.model.state_dict(), model_name)
		print "Model: {} saved".format(model_name) 
