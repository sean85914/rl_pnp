import os
import sys
import numpy as np
import yaml
import cv2
import struct
import ctypes
import argparse
import multiprocessing as mp
from scipy import ndimage
from collections import namedtuple
import matplotlib.pyplot as plt
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse
from arm_operation.msg import execution
import torch
from torchvision import transforms
from prioritized_memory import Memory

# Define transition tuple
Transition = namedtuple('Transition', ['color', 'depth', 'pixel_idx', 'reward', 'next_color', 'next_depth', 'is_empty'])

yaml_path = "/home/sean/Documents/flip_obj/src/visual_system/config/param_config.yaml"
background_data = "/home/sean/Documents/flip_obj/src/grasp_suck/data/background.txt"

with open(yaml_path, "r") as stream:
	data = yaml.load(stream)

x_lower = data['x_lower']
x_upper = data['x_upper']
y_lower = data['y_lower']
y_upper = data['y_upper']
z_lower = data['z_lower']
z_upper = data['z_upper']
resolution = data['resolution']

workspace_limits = np.asarray([[x_lower, x_upper], [y_lower, y_upper], [z_lower, z_upper]]) # X Y Z
heightmap_resolution = (workspace_limits[0][1]-workspace_limits[0][0])/resolution

'''
  _____                                       _       _           _ 
 |_   _|                                     | |     | |         | |
   | |  _ __ ___   __ _  __ _  ___   _ __ ___| | __ _| |_ ___  __| |
   | | | '_ ` _ \ / _` |/ _` |/ _ \ | '__/ _ \ |/ _` | __/ _ \/ _` |
  _| |_| | | | | | (_| | (_| |  __/ | | |  __/ | (_| | ||  __/ (_| |
 |_____|_| |_| |_|\__,_|\__, |\___| |_|  \___|_|\__,_|\__\___|\__,_|
                         __/ |                                      
                        |___/                                       
'''

def workers(id, data_list, out_q):
	outdict = {}
	counter = 0
	result_arr = np.zeros((len(data_list), 3), dtype=np.uint8)
	for data in data_list:
		s = struct.pack(">f", data)
		i = struct.unpack(">l", s)[0]
		pack = ctypes.c_uint32(i).value
		r = (pack & 0x00FF0000) >> 16
		g = (pack & 0x0000FF00) >> 8
		b = (pack & 0x000000FF)
		result_arr[counter][:] = b, g, r
		counter += 1
	outdict[id] = result_arr
	out_q.put(outdict)

def unpack_rgb(rgb_uint32_list):
	result = np.zeros((len(rgb_uint32_list), 3), dtype=np.uint8)
	out_q = mp.Queue()
	process = []
	process_num = 16
	mount = len(rgb_uint32_list)/float(process_num)
	for i in range(process_num):
		p = mp.Process(target=workers, \
		               args=(i, \
		                     rgb_uint32_list[int(mount*i):int(mount*(i+1))], \
		                     out_q))
		process.append(p)
		p.start()
	for i in range(process_num):
		temp = out_q.get()
		for key in temp:
			result[int(mount*key):int(mount*(key+1))][:] = temp[key][:]
	for p in process:
		p.join()
	return result

def get_heightmap(pc, img_path, depth_path, iteration):
	color_heightmap = np.zeros((resolution, resolution, 3), dtype=np.uint8)
	depth_heightmap = np.zeros((resolution, resolution))
	points = np.zeros((resolution, resolution, 3)) 
	gen = pc2.read_points(pc, skip_nans=False)
	int_data = list(gen)
	np_data = np.array(int_data); np_data = np_data.T
	heightmap_x = np.floor((workspace_limits[0][1]-np_data[0])/heightmap_resolution).astype(int)
	heightmap_y = np.floor((workspace_limits[1][1]-np_data[1])/heightmap_resolution).astype(int)
	heightmap_x[heightmap_x>=resolution] = resolution-1
	heightmap_y[heightmap_y>=resolution] = resolution-1
	rgb = unpack_rgb(np_data[3])
	color_heightmap[heightmap_x, heightmap_y, :] = rgb
	depth_heightmap[heightmap_x, heightmap_y] = np_data[2]
	points[heightmap_x, heightmap_y] = np_data[:3].T
	depth_heightmap = depth_heightmap - workspace_limits[2][0]
	depth_heightmap[depth_heightmap==-workspace_limits[2][0]] = 0
	#depth_heightmap[depth_heightmap == -z_bot] = np.nan
	color_name = img_path + "color_{:06}.jpg".format(iteration)
	cv2.imwrite(color_name, color_heightmap)
	np.save(depth_path+"depth_data_{:06}.npy".format(iteration), depth_heightmap) # for saving space of hard disk
	return color_heightmap, depth_heightmap, points

def _depth_to_heatmap(depth):
	depth_hm = np.zeros_like(depth, np.uint8)
	depth_hm = (depth/np.max(depth)*255).astype(np.uint8)
	depth_hm = cv2.applyColorMap(depth_hm, cv2.COLORMAP_JET)
	return depth_hm
	
def preprocessing(name, dir_, color=None, depth=None):
	has_color = None; has_depth = None
	try:
		shape = color.shape
		has_color = True
	except AttributeError:
		has_color = False
	try:
		shape = depth.shape
		has_depth = True
	except AttributeError:
		has_depth = False
	if has_color==False and has_depth==False:
		return
	def _tensor_to_PIL(tensor):
		image = tensor.cpu().clone()
		image = image.squeeze(0)
		image = transforms.ToPILImage()(image)
		return image
	if has_color:
		color_2x = ndimage.zoom(color, zoom=[2, 2, 1], order=0)
		cv2.imwrite("color_2x.jpg", color_2x)
		diag_length = float(color_2x.shape[0])*np.sqrt(2)
		diag_length = np.ceil(diag_length/32)*32
		padding_width = int((diag_length-color_2x.shape[0])/2)
		color_padded = cv2.copyMakeBorder(color_2x, padding_width, padding_width, padding_width, padding_width, cv2.BORDER_CONSTANT, 0)
		cv2.imwrite("color_2x_padded.jpg", color_padded)
		image_mean = [0.406, 0.456, 0.485] # BGR
		image_std  = [0.225, 0.224, 0.229]
		color_float = color_padded.astype(float)/255
		color_norm = np.zeros_like(color_padded, dtype=float)
		# Change to RGB
		for i in range(3):
			color_norm[:, :, i] = (color_float[:, :, i] - image_mean[i]) / image_std[i]
		color_norm_ = (color_norm*255).astype(np.uint8)
		color_norm_.shape = (color_norm_.shape[0], color_norm_.shape[1], color_norm_.shape[2], 1)
		color_tensor = torch.from_numpy(color_norm_.astype(np.float32)).permute(3, 2, 0, 1)
		_tensor_to_PIL(color_tensor).save(dir_+"color_processed_"+name+".jpg")
	if has_depth:
		cv2.imwrite("depth.jpg", viz_depth_heightmap(depth))
		depth_2x = ndimage.zoom(depth, zoom=[2, 2],    order=0)
		cv2.imwrite("depth_2x.jpg", viz_depth_heightmap(depth_2x))
		diag_length = float(depth_2x.shape[0])*np.sqrt(2)
		diag_length = np.ceil(diag_length/32)*32
		padding_width = int((diag_length-depth_2x.shape[0])/2)
		depth_padded = cv2.copyMakeBorder(depth_2x, padding_width, padding_width, padding_width, padding_width, cv2.BORDER_CONSTANT, 0)
		cv2.imwrite("depth_2x_padded.jpg", viz_depth_heightmap(depth_padded))
		depth_mean = 0.0909769548291
		depth_std  = 0.0397293901695
		depth_norm = (depth_padded - depth_mean) / depth_std
		depth_norm.shape  = (depth_norm.shape[0], depth_norm.shape[1], 1)
		depth_3c = np.concatenate((depth_norm, depth_norm, depth_norm), axis = 2)
		depth_3c.shape = (depth_3c.shape[0], depth_3c.shape[1], depth_3c.shape[2], 1)
		depth_tensor = torch.from_numpy(depth_3c.astype(np.float32)).permute(3, 2, 0, 1)
		_tensor_to_PIL(depth_tensor).save(dir_+"depth_processed_"+name+".jpg")
	
def viz_depth_heightmap(depth):
	norm = (depth/np.max(depth)*255).astype(np.uint8)
	#norm_color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
	return norm
	
# Draw symbols in the color image with different primitive
def draw_image(image, explore, pixel_index, image_name):
	'''
		pixel_index[0] == 0: suck_1
		pixel_index[0] == 1: suck_2
		pixel_index[0] == 2: grasp, -90
		pixel_index[0] == 3: grasp, -45
		pixel_index[0] == 4: grasp,   0
		pixel_index[0] == 5: grasp,  45
	'''
	center = (pixel_index[2], pixel_index[1])
	if explore: color = (0, 0, 255) # Red for exploring
	else: color = (0, 0, 0) # Black for exploiting
	if pixel_index[0] == 0:
		result = cv2.circle(image, center, 4, color, 2)
	if pixel_index[0] == 1:
		result = cv2.circle(image, center, 7, color, 2)
	if pixel_index[0] != 0 and pixel_index[0] != 1: # grasp
		rotate_idx = pixel_index[0] - 2
		theta = np.radians(-90.0+45.0*rotate_idx)
		X = 20
		Y = 7
		x_unit = [ np.cos(theta), np.sin(theta)]
		y_unit = [-np.sin(theta), np.cos(theta)]
		p1  = (center[0]+np.ceil(x_unit[0]*X), center[1]+np.ceil(x_unit[1]*X))
		p2  = (center[0]-np.ceil(x_unit[0]*X), center[1]-np.ceil(x_unit[1]*X))
		p11 = (p1[0]-np.ceil(y_unit[0]*Y), p1[1]-np.ceil(y_unit[1]*Y))
		p12 = (p1[0]+np.ceil(y_unit[0]*Y), p1[1]+np.ceil(y_unit[1]*Y))
		p21 = (p2[0]-np.ceil(y_unit[0]*Y), p2[1]-np.ceil(y_unit[1]*Y))
		p22 = (p2[0]+np.ceil(y_unit[0]*Y), p2[1]+np.ceil(y_unit[1]*Y))
		p11 = (int(p11[0]), int(p11[1]))
		p12 = (int(p12[0]), int(p12[1]))
		p21 = (int(p21[0]), int(p21[1]))
		p22 = (int(p22[0]), int(p22[1]))
		result = cv2.circle(image, center, 3, color, 2)
		result = cv2.line(result, p11, p12, color, 2)
		result = cv2.line(result, p21, p22, color, 2)
	cv2.imwrite(image_name, result)
	return result

def vis_affordance(predictions):
	tmp = np.copy(predictions)
	# View the value as probability
	tmp[tmp<0] = 0
	tmp /= 5
	tmp[tmp>1] = 1
	tmp = (tmp*255).astype(np.uint8)
	tmp.shape = (tmp.shape[0], tmp.shape[1], 1)
	heatmap = cv2.applyColorMap(tmp, cv2.COLORMAP_JET)
	return heatmap

def save_heatmap_and_mixed(suck_1_prediction, suck_2_prediction, grasp_predictions, feat_paths, mixed_paths, color, iteration):
	heatmaps   = []
	mixed_imgs = []
	heatmaps.append(vis_affordance(suck_1_prediction[0]))
	heatmaps.append(vis_affordance(suck_2_prediction[0]))
	for grasp_prediction in grasp_predictions:
		heatmaps.append(vis_affordance(grasp_prediction))
	for heatmap_idx in range(len(heatmaps)):
		img_name = feat_paths[heatmap_idx] + "{:06}.jpg".format(iteration)
		cv2.imwrite(img_name, heatmaps[heatmap_idx])
		img_name = mixed_paths[heatmap_idx] + "{:06}.jpg".format(iteration)
		mixed = cv2.addWeighted(color, 1.0, heatmaps[heatmap_idx], 0.4, 0)
		mixed_imgs.append(mixed)
		cv2.imwrite(img_name, mixed)
	return heatmaps, mixed_imgs

'''
  _____      _ _                       _       _           _ 
 |  __ \    | (_)                     | |     | |         | |
 | |__) |__ | |_  ___ _   _   _ __ ___| | __ _| |_ ___  __| |
 |  ___/ _ \| | |/ __| | | | | '__/ _ \ |/ _` | __/ _ \/ _` |
 | |  | (_) | | | (__| |_| | | | |  __/ | (_| | ||  __/ (_| |
 |_|   \___/|_|_|\___|\__, | |_|  \___|_|\__,_|\__\___|\__,_|
                       __/ |                                 
                      |___/                                  
'''
# Return probability for sampling
def get_prob(mask, ratio=9):
	# ratio: ratio of sampling on white and on black is `ratio`:1
	# Greater than threshold will be considered as white (255)
	mask[mask>=126] = 255
	mask[mask<126]  = 0
	x = len(np.where(mask==255)[0])
	y = ratio*(resolution*resolution-x)/float(x)
	prob = np.zeros_like(mask, dtype=float)
	prob[mask==255] = y
	prob[mask==0]   = 1
	prob = prob/np.sum(prob) # Normalized
	return prob.reshape(-1) # Return flattened array
	
def generate_mask(depth):
	# Constant
	kernel = np.ones((2, 2), np.uint8) # Kernel for erosion and dilation
	depth_background = np.loadtxt(background_data)
	diff_threshold = 0.015
	diff = depth - depth_background
	mask = np.empty_like(diff, np.uint8)
	mask[diff>diff_threshold] = 255; mask[diff<=diff_threshold] = 0
	cv2.imwrite("raw.jpg", mask)
	erode  = cv2.erode(mask,  kernel, iterations=3)
	dilate = cv2.dilate(erode, kernel, iterations=3)
	return dilate

def standarization(prediction):
	if prediction.shape[0] is not 1:
		for i in range(prediction.shape[0]):
			mean = np.nanmean(prediction[i])
			std  = np.nanstd(prediction[i])
			prediction[i] = (prediction[i]-mean)/std
	else:
		mean = np.nanmean(prediction)
		std = np.nanstd(prediction)
		prediction = (prediction-mean)/std
	return prediction

# Choose action using epsilon-greedy policy
def epsilon_greedy_policy(epsilon, suck_1_prediction, suck_2_prediction, grasp_prediction, depth, loggerPath, iteration, specific_tool=None):
	explore = np.random.uniform() < epsilon # explore with probability epsilon
	action = 0
	angle = 0
	action_str = "suck_1"
	pixel_index = [] # primitive index, y, x
	out_str = ""
	print np.max(suck_1_prediction), np.max(suck_2_prediction), np.max(grasp_prediction[0]), np.max(grasp_prediction[1]), np.max(grasp_prediction[2]), np.max(grasp_prediction[3])
	if not explore: # Choose max Q
		out_str += "|Exploit| "
		if specific_tool is not None:
			max_q_index = specific_tool
			out_str += "(Specific: {}) ".format(specific_tool)
		else:
			primitives_max = [np.max(suck_1_prediction), np.max(suck_2_prediction), np.max(grasp_prediction)]
			max_q_index = np.where(primitives_max==np.max(primitives_max))[0][0]
		if max_q_index == 0: # suck_1
			tmp = np.where(suck_1_prediction == np.max(suck_1_prediction))
			pixel_index = [0, tmp[1][0], tmp[2][0]]
			out_str += "Select suck_1 at ({}, {}) with Q value {:.3f}\n".format(pixel_index[1], pixel_index[2], suck_1_prediction[0, pixel_index[1], pixel_index[2]])
		elif max_q_index == 1: # suck_2
			tmp = np.where(suck_2_prediction == np.max(suck_2_prediction))
			pixel_index = [1, tmp[1][0], tmp[2][0]]
			action_str = "suck_2"
			action = 1
			out_str += "Select suck_2 at ({}, {}) with Q value {:.3f}\n".format(pixel_index[1], pixel_index[2], suck_2_prediction[0, pixel_index[1], pixel_index[2]])
		elif max_q_index == 2:
			tmp = np.where(grasp_prediction == np.max(grasp_prediction))
			pixel_index = [tmp[0][0]+2, tmp[1][0], tmp[2][0]]
			angle = -90.0+45.0*tmp[0][0]
			action_str = "grasp " + str(int(angle))
			angle = np.radians(angle)
			action = tmp[0][0]+2
			out_str += "Select grasp with angle {} at ({}, {}) with Q value {:.3f}\n".format(angle, pixel_index[1], pixel_index[2], grasp_prediction[action-2, pixel_index[1], pixel_index[2]])
	else: # Random 
		out_str += "|Explore| "
		mask = generate_mask(depth)
		mask_name = loggerPath + "diff_{:06}.jpg".format(iteration)
		cv2.imwrite(mask_name, mask)
		idx = np.random.choice(np.arange(resolution*resolution), p=get_prob(mask))
		x = int(idx/resolution); y = int(idx%resolution)
		if specific_tool is not None:
			primitive = specific_tool
			out_str += "(Specific: {}) ".format(specific_tool)
		else:
			primitive = np.random.choice(np.arange(3), p=[0.4, 0.4, 0.2]) # suck_1: suck_2: grasp = 2:2:1
		if primitive == 0:
			out_str += "Select suck_1 at ({}, {}) with Q value {:.3f}\n".format(x, y, suck_1_prediction[0, x, y])
		if primitive == 1:
			action = 1
			action_str = "suck_2"
			out_str += "Select suck_2 at ({}, {}) with Q value {:.3f}\n".format(x, y, suck_2_prediction[0, x, y])
		if primitive == 2:
			rotate_idx = np.random.choice(np.arange(4)) # grasp_-90, grasp_-45, grasp_0, grasp_45
			angle = np.radians(-90.0+45.0*rotate_idx); angle_deg = -90.0+45.0*rotate_idx
			primitive += rotate_idx
			action = primitive
			action_str = "grasp " + str(int(angle_deg))
			out_str += "Select grasp with angle {} at ({}, {}) with Q value {:.3f}\n".format(angle, x, y, grasp_prediction[rotate_idx, x, y])
		pixel_index = [primitive, x, y]
	print out_str
	return explore, action, action_str, pixel_index, angle

# Choose action using greedy policy
def greedy_policy(suck_1_prediction, suck_2_prediction, grasp_prediction, specific_tool=None):
	print np.max(suck_1_prediction), np.max(suck_2_prediction), np.max(grasp_prediction[0]), np.max(grasp_prediction[1]), np.max(grasp_prediction[2]), np.max(grasp_prediction[3])
	action = 0
	action_str = 'suck_1'
	angle = 0
	pixel_index = [] # rotate_idx, y, x
	if specific_tool is not None:
		max_q_index = specific_tool
	else:
		primitives_max = [np.max(suck_1_prediction), np.max(suck_2_prediction), np.max(grasp_prediction)/2]
		max_q_index = np.where(primitives_max==np.max(primitives_max))[0][0]
	if max_q_index == 0: # suck_1
		tmp = np.where(suck_1_prediction == np.max(suck_1_prediction))
		pixel_index = [0, tmp[1][0], tmp[2][0]]
	elif max_q_index == 1: # suck_2
		tmp = np.where(suck_2_prediction == np.max(suck_2_prediction))
		pixel_index = [1, tmp[1][0], tmp[2][0]]
		action_str = "suck_2"
		action = 1
	elif max_q_index == 2: # Grasp
		tmp = np.where(grasp_prediction == np.max(grasp_prediction))
		pixel_index = [tmp[0][0]+2, tmp[1][0], tmp[2][0]]
		angle = -90.0+45.0*tmp[0][0]
		action_str = "grasp " + str(int(angle))
		angle = np.radians(angle)
		action = tmp[0][0]+2
	
	return action, action_str, pixel_index, angle

'''
   ____  _   _                   
  / __ \| | | |                  
 | |  | | |_| |__   ___ _ __ ___ 
 | |  | | __| '_ \ / _ \ '__/ __|
 | |__| | |_| | | |  __/ |  \__ \
  \____/ \__|_| |_|\___|_|  |___/
                                                               
'''
def create_argparser():
	parser = argparse.ArgumentParser(prog="reinforcement_grasping", description="Reinforcement learning for robot arm grasping")
	parser.add_argument("run", type=int, help="Which number is this run?")
	parser.add_argument("--is_testing", action="store_true", default=False, help="True if testing, default is false")
	parser.add_argument("--force_cpu", action="store_true", default=False, help="True if using CPU, default is false")
	parser.add_argument("--model", type=str, default="", help="If provided, continue training the model, or using this model for testing, 	default is empty srting")
	parser.add_argument("--buffer_file", type=str, default="", help="If provided, will read the given file to construct the experience buffer, default is empty string")
	parser.add_argument("--epsilon", type=float, default=0.5, help="Probability to choose random action, default is 0.5")
	parser.add_argument("--port", type=str, default="/dev/ttylight", help="Port for arduino, which controls the alram lamp, default is /dev/ttylight")
	parser.add_argument("--buffer_size", type=int, default=1000, help="Experience buffer size, default is 500") # N
	parser.add_argument("--learning_freq", type=int, default=5, help="Frequency for updating behavior network, default is 5") # M
	parser.add_argument("--updating_freq", type=int, default=10, help="Frequency for updating target network, default is 10") # C
	parser.add_argument("--mini_batch_size", type=int, default=3, help="How many transitions should used for learning, default is 3") # K
	parser.add_argument("--save_every", type=int, default=5, help="Every how many update should save the model, default is 5")
	parser.add_argument("--learning_rate", type=float, default=5e-4, help="Learning rate for the trainer, default is 5e-4")
	parser.add_argument("--densenet_lr", type=float, default=1e-4, help="Learning rate for the densenet block, default is 1e-4")
	parser.add_argument("--run_episode", type=int, default=0, help="index for recording bag")
	parser.add_argument("--specific_tool", type=int, default=None, help="If use specific tool only?")
	parser.add_argument("--suction_1_memory", type=str, default="")
	parser.add_argument("--suction_2_memory", type=str, default="")
	parser.add_argument("--gripper_memory", type=str, default="")
	return parser

def show_args(args):
	d = vars(args) # Convert to dictionary
	print "================================================"
	for key in d:
		print "{}:\t\t {}".format(key, d[key])
	print "================================================"

def parse_input(args):
	testing = args.is_testing
	use_cpu = args.force_cpu
	model_str = args.model
	buffer_str = args.buffer_file
	run = args.run
	epsilon = args.epsilon
	run_episode = args.run_episode
	port = args.port
	buffer_size = args.buffer_size
	learning_freq = args.learning_freq if not testing else 1e-5 # Still using small learning rate to backpropagate when testing
	updating_freq = args.updating_freq
	mini_batch_size = args.mini_batch_size
	save_every = args.save_every
	learning_rate = args.learning_rate
	densenet_lr = args.densenet_lr
	specific_tool = args.specific_tool
	suction_1_memory = args.suction_1_memory
	suction_2_memory = args.suction_2_memory
	gripper_memory = args.gripper_memory
	return testing, run, use_cpu, model_str, buffer_str, epsilon, port, \
	       buffer_size, learning_freq, updating_freq, mini_batch_size, save_every, learning_rate, run_episode, densenet_lr, specific_tool, suction_1_memory, suction_2_memory, gripper_memory

def get_file_path(color_img_path_str):
	idx = color_img_path_str[-16:]; idx = idx.replace("color_", ""); idx = idx.replace(".jpg", "")
	idx = int(idx)
	depth_image_path_str = color_img_path_str.replace("color", "depth")
	depth_image_path_str = depth_image_path_str.replace("jpg", "png")
	next_color_image_path_str = color_img_path_str.replace("color", "next_color")
	next_depth_image_path_str = color_img_path_str.replace("color", "depth")
	next_depth_image_path_str = next_depth_image_path_str.replace("jpg", "png")
	sub = color_img_path_str.split("images")[0]
	primitive_csv = sub+"/action_primitive.csv"
	result_csv = sub+"/action_result.csv"
	target_csv = sub+"/action_target.csv"
	return idx, depth_image_path_str, next_color_image_path_str, next_depth_image_path_str, \
			primitive_csv, result_csv, target_csv
			
def getLoggerPath(testing, root_path, run):
	if not testing: logger_dir = "/training/logger_{:03}/".format(run)
	else: logger_dir = "/testing/logger_{:03}/".format(run)
	csv_path         = root_path + logger_dir
	image_path       = csv_path + "images/"
	depth_path       = csv_path + "depth_data/"
	mixed_paths_root = csv_path + "mixed_img/"
	feat_paths_root  = csv_path + "feat/"
	pc_path          = csv_path + "pc/"
	model_path       = csv_path + "models/"
	vis_path         = csv_path + "vis/"
	diff_path        = csv_path + "diff/"
	check_grasp_path = csv_path + "grasp/"
	mixed_paths      = []
	feat_paths       = []
	primitives       = ["suck_1/", "suck_2/", "grasp_0/", "grasp_1/", "grasp_2/", "grasp_3/"]
	for primitive in primitives:
		mixed_paths.append(mixed_paths_root+primitive)
		feat_paths.append(feat_paths_root+primitive)
		if not os.path.exists(mixed_paths[-1]):
			os.makedirs(mixed_paths[-1])
		if not os.path.exists(feat_paths[-1]):
			os.makedirs(feat_paths[-1])
	# Check if directionary exist
	for path in list([csv_path, image_path, depth_path, pc_path, model_path, vis_path, check_grasp_path]):
		if not os.path.exists(path):
			os.makedirs(path)
	if not testing:
		if not os.path.exists(diff_path):
			os.makedirs(diff_path)
	return csv_path, image_path, depth_path, mixed_paths, feat_paths, pc_path, model_path, vis_path, diff_path, check_grasp_path

def saveFiles(runtime, action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, path):
	def _count_success(usage_iter, result_list):
		num = 0
		for idx in usage_iter:
			if result_list[idx]==1:
				num+=1
		return num
	def _get_rate(num, den):
		try:
			rate = float(num)/den
		except ZeroDivisionError:
			rate = float('nan')
		return rate
	action_list = np.array(action_list)
	result_list = np.array(result_list)
	np.savetxt(path+"action_primitive.csv", action_list, delimiter=",")
	np.savetxt(path+"action_target.csv", target_list, delimiter=",")
	np.savetxt(path+"action_result.csv", result_list, delimiter=",")
	np.savetxt(path+"loss.csv", loss_list, delimiter=",")
	np.savetxt(path+"explore.csv", explore_list, delimiter=",")
	np.savetxt(path+"return.csv", return_list, delimiter=",")
	np.savetxt(path+"episode.csv", episode_list, delimiter=",")
	np.savetxt(path+"position.csv", position_list, delimiter=",")
	f = open(path+"episode_result.txt", "w")
	total_iter = action_list.size
	invalid_iter = np.where(action_list==-1)[0].size
	success_iter = np.where(result_list==1)[0].size
	gripper_usage_iter = np.where(action_list>=2)[0]
	suction_1_usage_iter = np.where(action_list==0)[0]
	suction_2_usage_iter = np.where(action_list==1)[0]
	gripper_success = _count_success(gripper_usage_iter, result_list)
	suction_1_success = _count_success(suction_1_usage_iter, result_list)
	suction_2_success = _count_success(suction_2_usage_iter, result_list)
	valid_iter = total_iter-invalid_iter
	invalid_action_rate = _get_rate(invalid_iter, total_iter)
	success_rate = _get_rate(success_iter, total_iter)
	gripper_success_rate = _get_rate(gripper_success, gripper_usage_iter.size)
	suction_1_success_rate = _get_rate(suction_1_success, suction_1_usage_iter.size)
	suction_2_success_rate = _get_rate(suction_2_success, suction_2_usage_iter.size)
	gripper_usage_rate = _get_rate(gripper_usage_iter.size, valid_iter)
	suction_1_usage_rate = _get_rate(suction_1_usage_iter.size, valid_iter)
	suction_2_usage_rate = _get_rate(suction_2_usage_iter.size, valid_iter)
	# Write data
	f.write("runtime: {}\n".format(runtime))
	f.write("return: {}\n".format(return_list[0]))
	f.write("mean loss: {}\n".format(np.mean(loss_list)))
	f.write("invalid action rate: {}\n".format(invalid_action_rate))
	f.write("success_rate: {}\n".format(success_rate))
	f.write("gripper success rate: {}\n".format(gripper_success_rate))
	f.write("suction I success rate: {}\n".format(suction_1_success_rate))
	f.write("suction II success rate: {}\n".format(suction_2_success_rate))
	f.write("gripper usage rate: {}\n".format(gripper_usage_rate))
	f.write("suction I usage rate: {}\n".format(suction_1_usage_rate))
	f.write("suction II usage rate: {}\n".format(suction_2_usage_rate))
	f.close()

def check_if_valid(position):
	if (position[0] > x_lower and position[0] < x_upper) and \
	   (position[1] > y_lower and position[1] < y_upper) and \
	   (position[2] > z_lower and position[2] < z_upper):
	   return True # valid
	else: return False # invalid
	
def getModelCapacity(model):
	return sum(p.numel() for p in model.parameters() if p.requires_grad)
	
def wrap_strings(image_path, depth_path, iteration):
	color_name = image_path + "color_{:06}.jpg".format(iteration)
	next_color_name = image_path + "next_color_{:06}.jpg".format(iteration)
	depth_name = depth_path + "depth_data_{:06}.npy".format(iteration)
	next_depth_name = depth_path + "next_depth_data_{:06}.npy".format(iteration) # for saving space of hard disk
	return color_name, depth_name, next_color_name, next_depth_name

def shutdown_process(runtime, action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, path, s_1_mem, s_2_mem, gri_mem, regular=True):
	s_1_mem.save_memory(path, "suction_1_memory.pkl")
	s_2_mem.save_memory(path, "suction_2_memory.pkl")
	gri_mem.save_memory(path, "gripper_memory.pkl")
	saveFiles(runtime, action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, path)
	if regular: print "Regular shutdown"
	else: print "Shutdown since user interrupt"
	sys.exit(0)
	
def wrap_execution_info(iteration, is_valid, primitive, is_success):
	res = execution()
	res.iteration = iteration
	if not is_valid:
		res.primitive = "invalid"
	else:
		if primitive == 0:
			res.primitive = "suck_1"
		elif primitive == 1:
			res.primitive = "suck_2"
		elif primitive == 2:
			res.primitive = "grasp_neg_90"
		elif primitive == 3:
			res.primitive = "grasp_neg_45"
		elif primitive == 4:
			res.primitive = "grasp_0"
		else:
			res.primitive = "grasp_45"
	res.is_success = is_success
	return res

def get_action_info(pixel_index):
	if pixel_index[0] == 0:
		action_str = "suck_1"; rotate_idx = -1
	elif pixel_index[0] == 1:
		action_str = "suck_2"; rotate_idx = -1
	else:
		action_str = "grasp"; rotate_idx = pixel_index[0]-2
	return action_str, rotate_idx

def sample_data(memory, batch_size):
	done = False
	mini_batch = []; idxs = []; is_weight = []
	while not done:
		success = True
		mini_batch, idxs, is_weight = memory.sample(batch_size)
		for transition in mini_batch:
			success = success and isinstance(transition, Transition)
		if success: done = True
	return mini_batch, idxs, is_weight

'''
  ____                                         _ 
 |  _ \    ___  __      __   __ _   _ __    __| |
 | |_) |  / _ \ \ \ /\ / /  / _` | | '__|  / _` |
 |  _ <  |  __/  \ V  V /  | (_| | | |    | (_| |
 |_| \_\  \___|   \_/\_/    \__,_| |_|     \__,_|
                                                 
'''
def reward_judgement(reward_unit, action_valid, action_success):
	if not action_valid:
		return -3*reward_unit # Invalid
	if action_success:
		return reward_unit # Valid and success
	else:
		return -reward_unit # Valid and failed
		
'''
   ____                                               _                 
  / ___|   ___    _ __   __   __   ___   _ __   ___  (_)   ___    _ __  
 | |      / _ \  | '_ \  \ \ / /  / _ \ | '__| / __| | |  / _ \  | '_ \ 
 | |___  | (_) | | | | |  \ V /  |  __/ | |    \__ \ | | | (_) | | | | |
  \____|  \___/  |_| |_|   \_/    \___| |_|    |___/ |_|  \___/  |_| |_|
                                                                        
'''

def point_np_to_ros(point):
	return Point(point[0], point[1], point[2])

'''
  __  __           _     _     
 |  \/  |   __ _  | |_  | |__  
 | |\/| |  / _` | | __| | '_ \ 
 | |  | | | (_| | | |_  | | | |
 |_|  |_|  \__,_|  \__| |_| |_|
                               
'''

def softmax(x):
	return np.exp(x)/np.sum(np.exp(x))
	
'''
  ___            __                                      _     _                 
 |_ _|  _ __    / _|   ___    _ __   _ __ ___     __ _  | |_  (_)   ___    _ __  
  | |  | '_ \  | |_   / _ \  | '__| | '_ ` _ \   / _` | | __| | |  / _ \  | '_ \ 
  | |  | | | | |  _| | (_) | | |    | | | | | | | (_| | | |_  | | | (_) | | | | |
 |___| |_| |_| |_|    \___/  |_|    |_| |_| |_|  \__,_|  \__| |_|  \___/  |_| |_|
                                                                                 
'''

def print_action(action_str, pixel_index, point):
	print "Take action [\033[1;31m%s\033[0m] at (%d, %d) -> (%f, %f, %f)" %(action_str, \
	                                                                        pixel_index[1], pixel_index[2], \
	                                                                        point[0], point[1], point[2])

def parse_string(input_str):
	def _substring(input_str, key, length):
		pos = input_str.find(key)
		return input_str[pos+len(key):pos+len(key)+length]
	episode_str = _substring(input_str, "logger_", 3); episode = int(episode_str)
	iter_str = _substring(input_str, "color_", 6); iter_ = int(iter_str)
	print "Sample at episode: {} iteration: {}".format(episode, iter_)
	return episode, iter_
