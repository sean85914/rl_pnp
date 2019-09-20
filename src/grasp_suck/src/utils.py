import numpy as np
import cv2
import struct
import ctypes
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse

workspace_limits = np.asarray([[-0.659, -0.273], [-0.269, 0.117], [-0.01, 0.2]])
angle_map = [np.radians(0.), np.radians(-45.), np.radians(-90.), np.radians(45.)]
heightmap_resolution = (workspace_limits[0][1]-workspace_limits[0][0])/224
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

def get_heightmap(pc, img_path, iteration):
	color_heightmap = np.zeros((224, 224, 3), dtype=np.uint8)
	depth_heightmap = np.zeros((224, 224))
	points = np.zeros((224, 224, 3)) 
	gen = pc2.read_points(pc, skip_nans=True)
	int_data = list(gen)
	for p in int_data:
		rgb = p[3]
		s = struct.pack(">f", rgb)
		i = struct.unpack(">l", s)[0]
		pack = ctypes.c_uint32(i).value
		r = (pack & 0x00FF0000) >> 16
		g = (pack & 0x0000FF00) >> 8
		b = (pack & 0x000000FF)
		heightmap_x = np.floor((p[0]-workspace_limits[0][0])/heightmap_resolution).astype(int)
		heightmap_y = np.floor((p[1]-workspace_limits[1][0])/heightmap_resolution).astype(int)
		color_heightmap[heightmap_y, heightmap_x, 0] = b
		color_heightmap[heightmap_y, heightmap_x, 1] = g
		color_heightmap[heightmap_y, heightmap_x, 2] = r
		depth_heightmap[heightmap_y, heightmap_x] = p[2]
		points[heightmap_y, heightmap_x] = np.array([p[0], p[1], p[2]])
	z_bot = workspace_limits[2][0]
	depth_heightmap = depth_heightmap - z_bot
	depth_heightmap[depth_heightmap<0] = 0
	depth_img = np.copy(depth_heightmap)
	depth_img = (depth_img*1000).astype(np.uint16)
	depth_img_msg = br.cv2_to_imgmsg(depth_img)
	#depth_heightmap[depth_heightmap == -z_bot] = np.nan
	depth_name = img_path + "depth_{:06}.png".format(iteration)
	cv2.imwrite(depth_name, depth_img)
	color_name = img_path + "color_{:06}.jpg".format(iteration)
	cv2.imwrite(color_name, color_heightmap)
	return color_heightmap, depth_heightmap, points, depth_img_msg

# Draw symbols in the color image with different primitive
def draw_image(image, primitive, pixel_index):
	center = (pixel_index[2], pixel_index[1])
	result = cv2.circle(image, center, 7, (0, 0, 0), 2)
	X = 20
	Y = 7
	theta = np.radians(-90+45*pixel_index[0])
	x_unit = [ np.cos(theta), np.sin(theta)]
	y_unit = [-np.sin(theta), np.cos(theta)]
	if not primitive: # GRASP
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
		result = cv2.line(result, p11, p12, (0, 0, 0), 2) # Black
		result = cv2.line(result, p21, p22, (0, 0, 0), 2) # Black
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
# Choose action using grasp-only epsilon-greedy policy
def grasp_epsilon_greedy_policy(epsilon, grasp_predictions):
	explore = np.random.uniform() < epsilon
	action = 0
	action_str = 'grasp'
	angle = 0
	pixel_index = [] # rotate_idx, y, x
	if not explore:
		tmp = np.where(grasp_predictions == np.max(grasp_predictions))
		pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
		angle = angle_map[pixel_index[0]]
	else: # explore, use second best
		flat = grasp_predictions.flatten()
		flat = np.sort(flat)
		tmp = np.where(grasp_predictions == flat[-2])
		pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
		angle = angle_map[pixel_index[0]]
	return explore, action, action_str, pixel_index, angle
		
# Choose action using epsilon-greedy policy
def epsilon_greedy_policy(epsilon, suck_predictions, grasp_predictions):
	explore = np.random.uniform() < epsilon
	action = 0
	action_str = 'grasp'
	angle = 0
	pixel_index = [] # rotate_idx, y, x
	if np.max(suck_predictions) > np.max(grasp_predictions):
		if not explore:
			action = 1 # SUCK
			action_str = 'suck'
			tmp = np.where(suck_predictions == np.max(suck_predictions))
			pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
		else: # GRASP
			tmp = np.where(grasp_predictions == np.max(grasp_predictions))
			pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
			angle = angle_map[pixel_index[0]]
	else:
		if not explore: # GRASP
			tmp = np.where(grasp_predictions == np.max(grasp_predictions))
			pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
			angle = angle_map[pixel_index[0]]
		else:
			action = 1 # SUCK
			action_str = 'suck'
			tmp = np.where(suck_predictions == np.max(suck_predictions))
			pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
	return explore, action, action_str, pixel_index, angle

# Choose action using greedy policy
def greedy_policy(suck_predictions, grasp_predictions):
	action = 0
	action_str = 'grasp'
	angle = 0
	pixel_index = [] # rotate_idx, y, x
	if np.max(suck_predictions) > np.max(grasp_predictions):
		action = 1 # SUCK
		action_str = 'suck'
		tmp = np.where(suck_predictions == np.max(suck_predictions))
		pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
	else: # GRASP
		tmp = np.where(grasp_predictions == np.max(grasp_predictions))
		pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
		angle = angle_map[pixel_index[0]]
	return action, action_str, pixel_index, angle

# Policy that only choose grasp
def grasp_only_policy(grasp_predictions):
	action = 0
	action_str = 'grasp'
	tmp = np.where(grasp_predictions == np.max(grasp_predictions))
	pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
	angle = angle_map[pixel_index[0]]
	return pixel_index, angle

'''
   ____  _   _                   
  / __ \| | | |                  
 | |  | | |_| |__   ___ _ __ ___ 
 | |  | | __| '_ \ / _ \ '__/ __|
 | |__| | |_| | | |  __/ |  \__ \
  \____/ \__|_| |_|\___|_|  |___/
                                                               
'''

def show_args(args):
	args_list = ['episode', 'epsilon', 'force_cpu', 'grasp_only', 'is_testing', 'model', 'update_target', 'learn_every']
	d = vars(args)
	print "================================================"
	for i in range(len(args_list)):
		print "{}:\t\t {}".format(args_list[i], d[args_list[i]])
	print "================================================"

def standarization(suck_predictions, grasp_predictions):
	mean = np.nanmean(suck_predictions)
	std  = np.nanstd(suck_predictions)
	suck_predictions = (suck_predictions-mean)/std
	for i in range(len(grasp_predictions)):
		mean = np.nanmean(grasp_predictions[i])
		std  = np.nanstd(grasp_predictions[i])
		grasp_predictions[i] = (grasp_predictions[i]-mean)/std
	return suck_predictions, grasp_predictions

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
