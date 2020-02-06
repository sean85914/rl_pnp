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
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse

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
# cv bridge
br = CvBridge()

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
	np.savetxt(depth_path+"depth_data_{:06}.txt".format(iteration), depth_heightmap)
	return color_heightmap, depth_heightmap, points

def _depth_to_heatmap(depth):
	depth_hm = np.zeros_like(depth, np.uint8)
	depth_hm = (depth/np.max(depth)*255).astype(np.uint8)
	depth_hm = cv2.applyColorMap(depth_hm, cv2.COLORMAP_JET)
	return depth_hm
	
def preprocessing(color, depth):
	# Zoom 2X
	color_2x = ndimage.zoom(color, zoom=[2, 2, 1], order=0)
	depth_2x = ndimage.zoom(depth, zoom=[2, 2],    order=0)
	cv2.imwrite("color_2x.jpg", color_2x)
	cv2.imwrite("depth_2x.jpg", _depth_to_heatmap(depth_2x))
	# Zero padding
	diag_length = float(color_2x.shape[0])*np.sqrt(2)
	diag_length = np.ceil(diag_length/32)*32
	padding_width = int((diag_length-color_2x.shape[0])/2)
	color_padded = cv2.copyMakeBorder(color_2x, padding_width, padding_width, padding_width, padding_width, cv2.BORDER_CONSTANT, 0)
	depth_padded = cv2.copyMakeBorder(depth_2x, padding_width, padding_width, padding_width, padding_width, cv2.BORDER_CONSTANT, 0)
	cv2.imwrite("color_padded.jpg", color_padded)
	cv2.imwrite("depth_padded.jpg", _depth_to_heatmap(depth_padded))
	# Normalization
	image_mean = [0.406, 0.456, 0.485] # BGR
	image_std  = [0.229, 0.224, 0.225]
	color_float = color_padded.astype(float)/255
	color_norm = np.zeros_like(color_padded, dtype=float)
	for i in range(3):
		color_norm[:, :, i] = (color_float[:, :, i] - image_mean[i]) / image_std[i]
	color_norm_ = (color_norm*255).astype(np.uint8)
	depth_mean = 0.07
	depth_std  = 0.0005
	depth_norm = (depth_padded - depth_mean) / depth_std
	fig = plt.figure(figsize=(640./100, 640./100), frameon=False)
	ax = plt.Axes(fig, [0., 0., 1., 1.])
	ax.set_axis_off()
	fig.add_axes(ax)
	ax.imshow(color_norm_)
	plt.savefig('color_normalized.jpg', aspect="normal")
	ax.imshow(depth_norm)
	plt.savefig('depth_normalized.jpg', aspect="normal")
	
# Draw symbols in the color image with different primitive
def draw_image(image, explore, pixel_index):
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
	return result

def vis_affordance(predictions):
	tmp = np.copy(predictions)
	# View the value as probability
	tmp[tmp<0] = 0
	tmp[tmp>1] = 1
	tmp = (tmp*255).astype(np.uint8)
	tmp.shape = (tmp.shape[0], tmp.shape[1], 1)
	heatmap = cv2.applyColorMap(tmp, cv2.COLORMAP_JET)
	return heatmap

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
def get_prob(mask, ratio=4):
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

# Choose action using epsilon-greedy policy
def epsilon_greedy_policy(epsilon, suck_1_prediction, suck_2_prediction, grasp_prediction, depth, loggerPath, iteration):
	explore = np.random.uniform() < epsilon # explore with probability epsilon
	action = 0
	angle = 0
	action_str = "suck_1"
	pixel_index = [] # primitive index, y, x
	if not explore: # Choose max Q
		print "|Exploit|"
		primitives_max = [np.max(suck_1_prediction), np.max(suck_2_prediction), np.max(grasp_prediction)]
		max_q_index = np.where(primitives_max==np.max(primitives_max))
		if max_q_index == 0: # suck_1
			tmp = np.where(suck_1_prediction == np.max(suck_1_prediction))
			pixel_index = [0, tmp[1][0], tmp[2][0]]
		elif max_q_index == 1: # suck_2
			tmp = np.where(suck_2_prediction == np.max(suck_2_prediction))
			pixel_index = [1, tmp[1][0], tmp[2][0]]
			action_str = "suck_2"
			action = 1
		else:
			tmp = np.where(grasp_prediction == np.max(grasp_prediction))
			pixel_index = [tmp[0][0]+2, tmp[1][0], tmp[2][0]]
			angle = -90.0+45.0*tmp[0][0]
			action_str = "grasp " + str(int(angle))
			angle = np.radians(angle)
			action = tmp[0][0]+2
	else: # Random 
		print "|Explore|"
		mask = generate_mask(depth)
		mask_name = loggerPath + "diff_{:06}.jpg".format(iteration)
		cv2.imwrite(mask_name, mask)
		idx = np.random.choice(np.arange(resolution*resolution), p=get_prob(mask))
		primitive = np.random.choice(np.arange(3)) # suck_1, suck_2, grasp
		if primitive == 1:
			action = 1
			action_str = "suck_2"
		if primitive == 2:
			rotate_idx = np.random.choice(np.arange(4)) # grasp_-90, grasp_-45, grasp_0, grasp_45
			angle = -90.0+45.0*rotate_idx
			primitive += rotate_idx
			action = primitive
			action_str = "grasp " + str(int(angle))
		pixel_index = [primitive, int(idx/resolution), int(idx%resolution)]
	return explore, action, action_str, pixel_index, angle

# Choose action using greedy policy
def greedy_policy(suck_1_prediction, suck_2_prediction, grasp_prediction):
	action = 0
	action_str = 'suck_1'
	angle = 0
	pixel_index = [] # rotate_idx, y, x
	primitives_max = [np.max(suck_1_prediction), np.max(suck_2_prediction), np.max(grasp_prediction)]
	max_q_index = np.where(primitives_max==np.max(primitives_max))
	if max_q_index == 0: # suck_1
		tmp = np.where(suck_1_prediction == np.max(suck_1_prediction))
		pixel_index = [0, tmp[1][0], tmp[2][0]]
	elif max_q_index == 1: # suck_2
		tmp = np.where(suck_2_prediction == np.max(suck_2_prediction))
		pixel_index = [1, tmp[1][0], tmp[2][0]]
		action_str = "suck_2"
		action = 1
	else:
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
	parser.add_argument("episode", type=int, help="Which episode is this run?")
	parser.add_argument("--is_testing", action="store_true", default=False, help="True if testing, default is false")
	parser.add_argument("--force_cpu", action="store_true", default=False, help="True if using CPU, default is false")
	parser.add_argument("--model", type=str, default="", help="If provided, continue training the model, or using this model for testing, 	default is empty srting")
	parser.add_argument("--buffer_file", type=str, default="", help="If provided, will read the given file to construct the experience buffer, default is empty string")
	parser.add_argument("--epsilon", type=float, default=0.5, help="Probability to choose random action, default is 0.5")
	parser.add_argument("--port", type=str, default="/dev/ttylight", help="Port for arduino, which controls the alram lamp, default is /dev/ttylight")
	parser.add_argument("--buffer_size", type=int, default=500, help="Experience buffer size, default is 500") # N
	parser.add_argument("--learning_freq", type=int, default=10, help="Frequency for updating behavior network, default is 10") # M
	parser.add_argument("--updating_freq", type=int, default=40, help="Frequency for updating target network, default is 40") # C
	parser.add_argument("--mini_batch_size", type=int, default=4, help="How many transitions should used for learning, default is 4") # K
	parser.add_argument("--save_every", type=int, default=10, help="Every how many steps should save the model, default is 10")
	parser.add_argument("--learning_rate", type=float, default=1e-4, help="Learning rate for the trainer, default is 1e-4")
	return parser

def show_args(args):
	d = vars(args) # Convert to dictionary
	print "================================================"
	for key in d:
		print "{}:\t\t {}".format(key, d[key])
	print "================================================"

def parse_input(args):
	testing = args.is_testing
	episode = args.episode
	use_cpu = args.force_cpu
	model_str = args.model
	buffer_str = args.buffer_file
	epsilon = args.epsilon
	port = args.port
	buffer_size = args.buffer_size
	learning_freq = args.learning_freq if not testing else 1e-5 # Still using small learning rate to backpropagate when testing
	updating_freq = args.updating_freq
	mini_batch_size = args.mini_batch_size
	save_every = args.save_every
	learning_rate = args.learning_rate
	return testing, episode, use_cpu, model_str, buffer_str, epsilon, port, \
	       buffer_size, learning_freq, updating_freq, mini_batch_size, save_every, learning_rate
	

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
			
def getLoggerPath(testing, root_path, episode):
	if not testing: logger_dir = "/training/logger_{:03}/".format(episode)
	else: logger_dir = "/testing/logger_{:03}/".format(episode)
	csv_path         = root_path + logger_dir
	image_path       = csv_path + "images/"
	depth_path       = csv_path + "depth_data/"
	mixed_paths_root = csv_path + "mixed_img/"
	feat_paths_root  = csv_path + "feat/"
	pc_path          = csv_path + "pc/"
	model_path       = csv_path + "models/"
	vis_path         = csv_path + "vis/"
	diff_path        = csv_path + "diff/"
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
	for path in list([csv_path, image_path, depth_path, pc_path, model_path, vis_path]):
		if not os.path.exists(path):
			os.makedirs(path)
	if not testing:
		if not os.path.exists(diff_path):
			os.makedirs(diff_path)
	return csv_path, image_path, depth_path, mixed_paths, feat_paths, pc_path, model_path, vis_path, diff_path

def saveFiles(action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, path):
	np.savetxt(path+"action_primitive.csv", action_list, delimiter=",")
	np.savetxt(path+"action_target.csv", target_list, delimiter=",")
	np.savetxt(path+"action_result.csv", result_list, delimiter=",")
	np.savetxt(path+"loss.csv", loss_list, delimiter=",")
	np.savetxt(path+"explore.csv", explore_list, delimiter=",")
	np.savetxt(path+"return.csv", return_list, delimiter=",")
	np.savetxt(path+"episode.csv", episode_list, delimiter=",")
	np.savetxt(path+"position.csv", position_list, delimiter=",")

def check_if_valid(position):
	if (position[0] > x_lower and position[0] < x_upper) and \
	   (position[1] > y_lower and position[1] < y_upper) and \
	   (position[2] > z_lower and position[2] < z_upper):
	   return True # valid
	else: return False # invalid

def softmax(x):
	return np.exp(x)/np.sum(np.exp(x))
	
def getModelCapacity(model):
	return sum(p.numel() for p in model.parameters() if p.requires_grad)
	
def print_action(action_str, pixel_index, point):
	print "Take action [\033[1;31m%s\033[0m] at (%d, %d) -> (%f, %f, %f)" %(action_str, \
	                                                                        pixel_index[1], pixel_index[2], \
	                                                                        point[0], point[1], point[2])

def point_np_to_ros(point):
	return Point(point[0], point[1], point[2])
	
def wrap_strings(image_path, depth_path, iteration):
	color_name = image_path + "color_{:06}.jpg".format(iteration)
	next_color_name = image_path + "next_color_{:06}.jpg".format(iteration)
	depth_name = depth_path + "depth_data_{:06}.txt".format(iteration)
	next_depth_name = depth_path + "next_depth_data_{:06}.txt".format(iteration)
	return color_name, depth_name, next_color_name, next_depth_name

def shutdown_process(action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, path, memory, regular=True):
	saveFiles(action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, position_list, path)
	memory.save_memory(path)
	if regular: print "Regular shutdown"
	else: print "Shutdown since user interrupt"
	sys.exit(0)
	
def get_action_info(pixel_index):
	if pixel_index[0] == 0:
		action_str = "suck_1"; rotate_idx = -1
	elif pixel_index[0] == 1:
		action_str = "suck_2"; rotate_idx = -1
	else:
		action_str = "grasp"; rotate_idx = pixel_index[0]-2
	return action_str, rotate_idx

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
	
# Reward shaping
def reward_judgement(reward_unit, action_valid, action_success):
	if not action_valid:
		return -3*reward_unit # Invalid
	if action_success:
		return reward_unit # Valid and success
	else:
		return -reward_unit # Valid and failed
