import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from visual_system.srv import get_image, get_imageRequest, get_imageResponse

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

# Get color and depth images from service response and save them
def get_imgs_from_msg(response, img_path, iteration):
	color = br.imgmsg_to_cv2(response.crop_color_img)
	depth = br.imgmsg_to_cv2(response.crop_depth_img)
	color_name = img_path + "color_{:06}.jpg".format(iteration)
	depth_name = img_path + "depth_{:06}.png".format(iteration)
	cv2.imwrite(color_name, color)
	cv2.imwrite(depth_name, depth)
	return color, depth

# Draw symbols in the color image with different primitive
def draw_image(image, primitive, pixel_index):
	center = (pixel_index[2], pixel_index[1])
	result = cv2.circle(image, center, 7, (0, 0, 0), 2)
	X = 20
	Y = 7
	theta = np.radians(-90.0+45.0*pixel_index[0])
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
			angle = np.radians(-90+45*pixel_index[0])
	else:
		if not explore: # GRASP
			tmp = np.where(grasp_predictions == np.max(grasp_predictions))
			pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
			angle = np.radians(-90+45*pixel_index[0])
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
		angle = np.radians(-90+45*pixel_index[0])
	return action, action_str, pixel_index, angle

# Policy that only choose grasp
def grasp_only_policy(grasp_predictions):
	action = 0
	action_str = 'grasp'
	tmp = np.where(grasp_predictions == np.max(grasp_predictions))
	pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
	angle = np.radians(-90+45*pixel_index[0])
	return pixel_index, angle

def standarization(suck_predictions, grasp_predictions):
	mean = np.mean(suck_predictions)
	std  = np.std(suck_predictions)
	suck_predictions = (suck_predictions-mean)/std
	for i in range(len(grasp_predictions)):
		mean = np.mean(grasp_predictions[i])
		std  = np.std(grasp_predictions[i])
		grasp_predictions[i] = (grasp_predictions[i]-mean)/std
	return suck_predictions, grasp_predictions
