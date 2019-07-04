import os
import time
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
from models import reinforcement_net
from scipy import ndimage

class Trainer(object):
    def __init__(self, suck_rewards, grasp_rewards, discount_factor, testing, force_cpu):
    
        if torch.cuda.is_available() and not force_cpu:
            print "CUDA detected, use GPU."
            self.use_cuda = True
        elif force_cpu:
            print "Force CPU."
            self.use_cuda = False
        else:
            print "No CUDA detected, use CPU."
            self.use_cuda = False
        
        # Model
        self.model = reinforcement_net(self.use_cuda, 4)
        if self.use_cuda:
            self.model = self.model.cuda()
        self.suck_rewards = suck_rewards
        self.grasp_rewards = grasp_rewards
        self.discount_factor = discount_factor
        
        # Huber loss 
        self.criterion = torch.nn.SmoothL1Loss(reduce=False)
        if self.use_cuda:
            self.criterion = self.criterion.cuda()
        # Set model to train mode
        self.model.train()
        
        # Initialize optimizer
        self.optimizer = torch.optim.SGD(self.model.parameters(), lr=1e-4, momentum=0.9, weight_decay=2e-5)
        self.iteration = 0
        
    # Forward pass through image, get affordance (Q value) and features
    def forward(self, color_img, depth_img, is_volatile=False, specific_rotation=-1):
    
        # Zoom 2X
        color_img_2x = ndimage.zoom(color_img, zoom=[2, 2, 1], order=0)
        depth_img_2x = ndimage.zoom(depth_img, zoom=[2, 2], order=0)
        
        # Add extra padding to handle rotations inside network
        diag_length = float(color_img_2x.shape[0])*np.sqrt(2)
        diag_length = np.ceil(diag_length/32)*32 # Shrink 32 times in network
        padding_width = int((diag_length - color_img_2x.shape[0])/2)
        color_img_2x_b = np.pad(color_img_2x[:, :, 0], padding_width, 'constant', constant_values=0)
        color_img_2x_b.shape = (color_img_2x_b.shape[0], color_img_2x_b.shape[1], 1)
        color_img_2x_g = np.pad(color_img_2x[:, :, 1], padding_width, 'constant', constant_values=0)
        color_img_2x_g.shape = (color_img_2x_g.shape[0], color_img_2x_g.shape[1], 1)
        color_img_2x_r = np.pad(color_img_2x[:, :, 2], padding_width, 'constant', constant_values=0)
        color_img_2x_r.shape = (color_img_2x_r.shape[0], color_img_2x_r.shape[1], 1)
        color_img_2x = np.concatenate((color_img_2x_b, color_img_2x_g, color_img_2x_r), axis=2)
        
        depth_img_2x = np.pad(depth_img_2x, padding_width, 'constant', constant_values=0)
        
        # Normalize color image
        image_mean = [0.485, 0.456, 0.406]
        image_std  = [0.229, 0.224, 0.225]
        input_color_img = color_img_2x.astype(float)/255 # np.uint8
        for c in range(3):
            input_color_img[:, :, c] = (input_color_img[:, :, c] - image_mean[c]) / image_std[c]
        # Normalize depth image
        image_mean = [0.005, 0.005, 0.005]
        image_std =  [0.002, 0.002, 0.002]
        input_depth_img = depth_img_2x.astype(float)/65535 # np.uint16
        # Duplicate channel to DDD
        depth_img_2x.shape = (depth_img_2x.shape[0], depth_img_2x.shape[1], 1)
        input_depth_img = np.concatenate((depth_img_2x, depth_img_2x, depth_img_2x), axis=2)
        for c in range(3):
            input_depth_img[:, :, c] = (input_depth_img[:, :, c] - image_mean[c]) / image_std[c]
            
        # Convert to tensor
        # H, W, C -> N, C, H, W
        input_color_img.shape = (input_color_img.shape[0], input_color_img.shape[1], input_color_img.shape[2], 1)
        input_depth_img.shape = (input_depth_img.shape[0], input_depth_img.shape[1], input_depth_img.shape[2], 1)
        input_color_data = torch.from_numpy(input_color_img.astype(np.float32)).permute(3, 2, 0, 1)
        input_depth_data = torch.from_numpy(input_depth_img.astype(np.float32)).permute(3, 2, 0, 1)
        if self.use_cuda:
            input_color_data = input_color_data.cuda()
            input_depth_data = input_depth_data.cuda()
        # Pass input data to model
        output_prob, state_feat = self.model.forward(input_color_data, input_depth_data, is_volatile, specific_rotation)
        
        # Return affordance and remove extra padding
        suck_predictions = output_prob[0].cpu().detach().numpy()[:, 0, int(padding_width/2):int(color_img_2x.shape[0]/2 - padding_width/2),
                                                                       int(padding_width/2):int(color_img_2x.shape[0]/2 - padding_width/2)]
        for rotate_idx in range(len(output_prob)-1):
            if rotate_idx == 0:
                grasp_predictions = output_prob[rotate_idx+1].cpu().detach().numpy()[:, 0, int(padding_width/2):int(color_img_2x.shape[0]/2 - padding_width/2),
                                                                                           int(padding_width/2):int(color_img_2x.shape[0]/2 - padding_width/2)]
            else:
                grasp_predictions = np.concatenate((grasp_predictions, output_prob[rotate_idx+1].cpu().detach().numpy()[:, 0, int(padding_width/2):int(color_img_2x.shape[0]/2 - padding_width/2),
                                                                                                                              int(padding_width/2):int(color_img_2x.shape[0]/2 - padding_width/2)]))
        suck_max_Q = np.max(suck_predictions)
        suck_max_index = np.where(suck_predictions == suck_max_Q)
        print "MAX SUCK: {}: ({}, {})".format(suck_max_Q, suck_max_index[2], suck_max_index[1])
        grasp_max_Q = np.max(grasp_predictions)
        grasp_max_index = np.where(grasp_predictions == grasp_max_Q)
        print "MAX GRASP: {}: ({}, {}) with rotation_idx: {}".format(grasp_max_Q, grasp_max_index[2], grasp_max_index[1], grasp_max_index[0])
        return suck_predictions, grasp_predictions, state_feat
        
    def get_label_value(self, primitive, action_success, next_color, next_depth):
        # Compute current reward
        current_reward = 0
        if primitive == "grasp":
            if action_success:
                current_reward = self.grasp_rewards
        if primitive == "suck":
            if action_success:
                current_reward = self.suck_rewards
        # Compute future reward
        next_suck_predictions, next_grasp_predictions, next_state_feat = self.forward(next_color, next_depth, is_volatile=True)
        future_reward = max(np.max(next_suck_predictions), np.max(next_grasp_predictions))
        
        # Compute expected reward
        expected_reward = current_reward + self.discount_factor * future_reward
        
        # Print information
        print "Current reward: %f" % current_reward
        print "Future reward:  %f" % future_reward
        
        return expected_reward, current_reward
        
    def backprop(self, color_img, depth_img, primitive, best_pix_idx, label_value):
        
        # Compute labels
        label = np.zeros((1, 320, 320))
        action_area = np.zeros((224, 224))
        action_area[best_pix_idx[1]][[best_pix_idx[2]]] = 1
        tmp_label = np.zeros((224, 224))
        tmp_label[action_area>0] = label_value
        label[0, 48: (320-48), 48: (320-48)] = tmp_label
       
        # Computelabel mask
        label_weights = np.zeros(label.shape)
        tmp_label_weights = np.zeros((224, 224))
        tmp_label_weights[action_area>0] = 1
        label_weights[0, 48: (320-48), 48: (320-48)] = tmp_label_weights
       
        # Compute loss and backward pass
        self.optimizer.zero_grad()
        loss_value = 0
       
        if primitive == "suck":
            # Forward pass to save gradients
            suck_predictions, grasp_predictions, state_feat = self.forward(color_img, depth_img, is_volatile=False)
            
            if self.use_cuda:
                loss = self.criterion(self.model.output_prob[0].view(1, 320, 320), Variable(torch.from_numpy(label).float().cuda())) * \
                                                                                   Variable(torch.from_numpy(label_weights).float().cuda(), requires_grad=False)
            else:
                loss = self.criterion(self.model.output_prob[0].view(1, 320, 320), Variable(torch.from_numpy(label).float())) * \
                                                                                   Variable(torch.from_numpy(label_weights).float(), requires_grad=False)
            loss = loss.sum()
            loss.backward()
            loss_value = loss.cpu().data.numpy()
        elif primitive == "grasp":
            # Forward pass with specific rotation to save gradients
            suck_predictions, grasp_predicrtions, state_feat = self.forward(color_img, depth_img, is_volatile=False, specific_rotation=best_pix_idx[0])
            
            if self.use_cuda:
                loss = self.criterion(self.model.output_prob[1].view(1, 320, 320), Variable(torch.from_numpy(label).float().cuda())) * \
                                                                                   Variable(torch.from_numpy(label_weights).float().cuda(), requires_grad=False)
            else:
                loss = self.criterion(self.model.output_prob[1].view(1, 320, 320), Variable(torch.from_numpy(label).float())) * \
                                                                                   Variable(torch.from_numpy(label_weights).float(), requires_grad=False)
                                                                                   
            loss = loss.sum()
            loss.backward()
            loss_value = loss.cpu().data.numpy()
            
            # Symmetric
            opposite_rotate_idx = (best_pix_idx[0] + self.model.num_rotations/2) % self.model.num_rotations
            suck_predictions, grasp_predicrtions, state_feat = self.forward(color_img, depth_img, is_volatile=False, specific_rotation=opposite_rotate_idx)
            if self.use_cuda:
                loss = self.criterion(self.model.output_prob[1].view(1, 320, 320), Variable(torch.from_numpy(label).float().cuda())) * \
                                                                                   Variable(torch.from_numpy(label_weights).float().cuda(), requires_grad=False)
            else:
                loss = self.criterion(self.model.output_prob[1].view(1, 320, 320), Variable(torch.from_numpy(label).float())) * \
                                                                                   Variable(torch.from_numpy(label_weights).float(), requires_grad=False)
            loss = loss_sum()
            loss.backward()
            loss_value = loss.cpu().data.numpy()
            
            loss_value = loss_value/2
        print "Training loss: %f" % loss_value
        self.optimizer.step()
