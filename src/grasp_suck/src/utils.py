import os
import numpy as np
import yaml
import cv2
import struct
import ctypes
import multiprocessing as mp
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse

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

# Choose action using epsilon-greedy policy
def epsilon_greedy_policy(epsilon, suck_1_prediction, suck_2_prediction, grasp_prediction, depth, loggerPath, iteration):
	explore = np.random.uniform() < epsilon # explore with probability epsilon
	action = 0
	angle = 0
	action_str = "suck_1"
	pixel_index = [] # rotate_idx, y, x
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
		depth_background = np.loadtxt(background_data)
		diff = depth - depth_background
		mask = np.empty_like(diff, np.uint8)
		mask[diff>0.015] = 255; mask[diff<=0.015] = 0
		kernel = np.ones((2, 2), np.uint8) # Kernel for erosion and dilation
		erode  = cv2.erode(mask,  kernel, iterations=3)
		dilate = cv2.dilate(erode, kernel, iterations=3)
		dilate_name = loggerPath + "diff_{:06}.jpg".format(iteration)
		cv2.imwrite(dilate_name, dilate)
		idx = np.random.choice(np.arange(resolution*resolution), p=get_prob(dilate))
		primitive = np.random.choice(np.arange(3))
		if primitive == 1:
			action = 1
			action_str = "suck_2"
		if primitive == 2:
			rotate_idx = np.random.choice(np.arange(4))
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

def show_args(args):
	args_list = ['epsilon', 'force_cpu', 'is_testing', 'model', 'buffer_size', 'learning_freq', 'updating_freq', 'mini_batch_size', 'save_every', 'learning_rate', 'episode', 'port']
	d = vars(args)
	print "================================================"
	for i in range(len(args_list)):
		print "{}:\t\t {}".format(args_list[i], d[args_list[i]])
	print "================================================"

def standarization(prediction):
	mean = np.nanmean(prediction)
	std  = np.nanstd(prediction)
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
