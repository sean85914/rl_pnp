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
angle_map = [np.radians(0.), np.radians(-45.), np.radians(-90.), np.radians(45.)] # FIXME
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
	depth_img = np.copy(depth_heightmap)
	depth_img = (depth_img*1000).astype(np.uint16)
	depth_img_msg = br.cv2_to_imgmsg(depth_img)
	#depth_heightmap[depth_heightmap == -z_bot] = np.nan
	depth_name = img_path + "depth_{:06}.png".format(iteration)
	cv2.imwrite(depth_name, depth_img)
	color_name = img_path + "color_{:06}.jpg".format(iteration)
	cv2.imwrite(color_name, color_heightmap)
	np.savetxt(depth_path+"depth_data_{:06}.txt".format(iteration), depth_heightmap)
	return color_heightmap, depth_heightmap, points, depth_img_msg

# Draw symbols in the color image with different primitive
def draw_image(image, explore, pixel_index):
	center = (pixel_index[2], pixel_index[1])
	if explore: color = (0, 0, 255) # Red for exploring
	else: color = (0, 0, 0) # Black for exploiting
	result = cv2.circle(image, center, 7, color, 2)
	
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
# Choose action using epsilon-greedy policy
def epsilon_greedy_policy(epsilon, prediction, depth, loggerPath, iteration):
	explore = np.random.uniform() < epsilon # explore with probability epsilon
	action = 1
	action_str = 'suck'
	angle = 0
	pixel_index = [] # rotate_idx, y, x
	if not explore: # Choose max Q
		print "|Exploit|"
		tmp = np.where(prediction == np.max(prediction))
		pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
	else: # Random 
		print "|Explore|"
		depth_background = np.loadtxt(background_data, delimiter=",")
		diff = depth - depth_background
		np.savetxt("diff.csv", diff, delimiter=",")
		mask = np.empty_like(diff, np.uint8)
		mask[diff>0.015] = 255; mask[diff<=0.015] = 0
		kernel = np.ones((2, 2), np.uint8)
		erode  = cv2.erode(mask,  kernel, iterations=3)
		dilate = cv2.dilate(erode, kernel, iterations=3)
		dilate_name = loggerPath + "diff_{:06}.jpg".format(iteration)
		cv2.imwrite(dilate_name, dilate)
		candidate = np.where(dilate==255)
		random   = np.random.choice(list(range(len(candidate[0]))))
		random_x = candidate[0][random]
		random_y = candidate[1][random]
		pixel_index = [0, random_x, random_y]
	return explore, action, action_str, pixel_index, angle

# Choose action using greedy policy
def greedy_policy(prediction):
	action = 1
	action_str = 'suck'
	angle = 0
	pixel_index = [] # rotate_idx, y, x
	tmp = np.where(prediction == np.max(prediction))
	pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]

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
	csv_path   = root_path + logger_dir
	image_path = csv_path + "images/"
	depth_path = csv_path + "depth_data/"
	mixed_path = csv_path + "mixed_img/"
	feat_path  = csv_path + "feat/"
	pc_path    = csv_path + "pc/"
	model_path = csv_path + "models/"
	vis_path   = csv_path + "vis/"
	diff_path  = csv_path + "diff/"
	# Check if directionary exist
	for path in list([csv_path, image_path, depth_path, mixed_path, feat_path, pc_path, model_path, vis_path, diff_path]):
		if not os.path.exists(path):
			os.makedirs(path)
	return csv_path, image_path, depth_path, mixed_path, feat_path, pc_path, model_path, vis_path, diff_path

def saveFiles(action_list, target_list, result_list, loss_list, explore_list, return_list, episode_list, path):
	np.savetxt(path+"action_primitive.csv", action_list, delimiter=",")
	np.savetxt(path+"action_target.csv", target_list, delimiter=",")
	np.savetxt(path+"action_result.csv", result_list, delimiter=",")
	np.savetxt(path+"loss.csv", loss_list, delimiter=",")
	np.savetxt(path+"explore.csv", explore_list, delimiter=",")
	np.savetxt(path+"return.csv", return_list, delimiter=",")
	np.savetxt(path+"episode.csv", episode_list, delimiter=",")

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
