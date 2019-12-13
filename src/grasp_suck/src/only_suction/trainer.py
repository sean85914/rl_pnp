import os
import time
import copy
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
from model import reinforcement_net
from scipy import ndimage

class Trainer(object):
	def __init__(self, reward, discount_factor, force_cpu, learning_rate = 1e-4):
		
		if torch.cuda.is_available() and not force_cpu:
			print "CUDA detected, use GPU"
			self.use_cuda = True
		elif force_cpu:
			print "Force CPU"
			self.use_cuda = False
		else:
			print "No CUDA detected, use CPU"
			self.use_cuda = False
			
		self.behavior_net = reinforcement_net(self.use_cuda)
		self.target_net   = copy.deepcopy(self.behavior_net)
		
		if self.use_cuda:
			self.behavior_net = self.behavior_net.cuda()
			self.target_net   = self.target_net.cuda()
		
		self.reward = reward
		self.discount_factor = discount_factor
		
		# Huber Loss
		self.criterion = torch.nn.SmoothL1Loss(reduce = False)
		if self.use_cuda:
			self.criterion = self.criterion.cuda()
		# Set model to train moed
		self.behavior_net.train()
		self.target_net.train()
		
		# Initialize optimizer
		self.optimizer = torch.optim.SGD(self.behavior_net.parameters(), lr = learning_rate, momentum = 0.9, weight_decay = 2e-5)
	
	# Copy network from behavior to target
	def copyNetwork(self):
		self.target_net = copy.deepcopy(self.behavior_net)
		
	# Forward pass through image, get Q value nad features
	def forward(self, color_img, depth_img, is_volatile = False, network = "behavior"):
		# Preprocessing
		# Zoom 2X
		color_img_2x = ndimage.zoom(color_img, zoom=[2, 2, 1], order = 0)
		depth_img_2x = ndimage.zoom(depth_img, zoom=[2, 2], order = 0)
		
		# BGR to RGB
		color_img_2x_b = color_img_2x[:, :, 0]
		color_img_2x_b.shape = (color_img_2x_b.shape[0], color_img_2x_b.shape[1], 1)
		color_img_2x_g = color_img_2x[:, :, 1]
		color_img_2x_g.shape = (color_img_2x_g.shape[0], color_img_2x_g.shape[1], 1)
		color_img_2x_r = color_img_2x[:, :, 2]
		color_img_2x_r.shape = (color_img_2x_r.shape[0], color_img_2x_r.shape[1], 1)
		color_img_2x = np.concatenate((color_img_2x_r, color_img_2x_g, color_img_2x_b), axis = 2)
		
		# Normalize color image with ImageNet data
		image_mean = [0.485, 0.456, 0.406]
		image_std  = [0.229, 0.224, 0.225]
		input_color_img = color_img_2x.astype(float)/255 # np.uint8
		for c in range(3):
			input_color_img[:, :, c] = (input_color_img[:, :, c] - image_mean[c]) / image_std[c]
		# Normalize depth image
		depth_mean = 0.01
		depth_std  = 0.005
		tmp = depth_img_2x.astype(float)
		tmp = (tmp-depth_mean)/depth_std
		# Duplicate channel to DDD
		tmp.shape = (tmp.shape[0], tmp.shape[1], 1)
		input_depth_img = np.concatenate((tmp, tmp, tmp), axis = 2)
		# Convert to tensor
		# H, W, C - > N, C, H, W
		input_color_img.shape = (input_color_img.shape[0], input_color_img.shape[1], input_color_img.shape[2], 1)
		input_depth_img.shape = (input_depth_img.shape[0], input_depth_img.shape[1], input_depth_img.shape[2], 1)
		input_color_data = torch.from_numpy(input_color_img.astype(np.float32)).permute(3, 2, 0, 1)
		input_depth_data = torch.from_numpy(input_depth_img.astype(np.float32)).permute(3, 2, 0, 1)
		# end preprocessing
		# Pass input data to model
		if network == "behavior":
			output_prob, state_feat = self.behavior_net.forward(input_color_data, input_depth_data, is_volatile)
		else: # Target
			output_prob, state_feat = self.target_net.forward(input_color_data, input_depth_data, is_volatile, clear_grad = True)
		
		output_prob = output_prob[0].cpu().detach().numpy()
		return output_prob, state_feat
	def get_label_value(self, valid_action, is_success, next_color, next_depth, is_empty):
		# Compute current reward
		current_reward = 0.0
		if is_success:
			current_reward = self.reward
		elif not valid_action:
			current_reward = -self.reward # Penalty
		# Compute TD target
		''' 
		Double DQN 
		TD target = R + discount * Q_target(next state, argmax(Q_behavior(next state, a)))
		'''
		next_prediction, next_state_feat = self.forward(next_color, next_depth, is_volatile = True, network = "behavior")
		tmp = np.where(next_prediction == np.max(next_prediction))
		del next_prediction, next_state_feat
		next_prediction, next_state_feat = self.forward(next_color, next_depth, is_volatile = False, network = "target")
		future_reward = 0.0
		if not is_empty:
			future_reward = next_prediction[0, tmp[1][0], tmp[2][0]]
		td_target = current_reward + self.discount_factor * future_reward
		
		del next_prediction, next_state_feat
		
		return td_target, current_reward
		
	def backprop(self, color_img, depth_img, action_pix_idx, label_value):
		label = np.zeros((1, 320, 320))
		label[0, action_pix_idx[0], action_pix_idx[1]] = label_value
		label_weight = np.zeros((1, 320, 320))
		label_weight[0, action_pix_idx[0], action_pix_idx[1]] = 1
		
		self.optimizer.zero_grad()
		loss_value = 0.0
		# Forward pass to save gradient
		prediction, state_feat = self.forward(color_img, depth_img, is_volatile = False)
		if self.use_cuda:
			loss = self.criterion(self.behavior_net.output_prob.view(1, 320, 320), Variable(torch.from_numpy(label).float().cuda()))* \
					Variable(torch.from_numpy(label_weight).float().cuda(), requires_grad = False)
		else:
			loss = self.criterion(self.behavior_net.output_prob.view(1, 320, 320), Variable(torch.from_numpy(label).float()))* \
					Variable(torch.from_numpy(label_weight).float(), requires_grad = False)
		loss = loss.sum()
		loss.backward()
		loss_value = loss.cpu().data.numpy()
		print "Training loss: %f" % loss_value
		self.optimizer.step()
		return loss_value
