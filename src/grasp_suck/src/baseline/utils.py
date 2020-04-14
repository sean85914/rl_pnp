import os
import numpy as np
import numpy.matlib
import cv2
import copy
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import pypcd

image_net_mean = np.array([0.485, 0.456, 0.406])
image_net_std  = np.array([0.229, 0.224, 0.225])

def preprocessing(colorImg, depthImg): # Expected color in BGR order
	color = (colorImg/255.).astype(np.float32)
	depth = (depthImg/1000.).astype(np.float32) # millimeter to meter
	# Change to RGB channel and normalize
	color_rgb = np.zeros(color.shape)
	for i in range(3):
		color_rgb[:, :, i] = (color[:, :, 2-i]-image_net_mean[i])/image_net_std[i]
	depth = np.clip(depth, 0.0, 10.0) # D435 depth range
	# Duplicate channel and normalize
	depth_3c = np.zeros(color.shape)
	for i in range(3):
		depth_3c[:, :, i] = (depth[:, :]-image_net_mean[i])/image_net_std[i]
	# Convert to tensor
	# H, W, C, (N) -> N, C, H, W
	color_rgb.shape = (color_rgb.shape[0], color_rgb.shape[1], color_rgb.shape[2], 1)
	depth_3c.shape = (depth_3c.shape[0], depth_3c.shape[1], depth_3c.shape[2], 1)
	color_tensor = torch.from_numpy(color_rgb.astype(np.float32)).permute(3, 2, 0, 1)
	depth_tensor = torch.from_numpy(depth_3c.astype(np.float32)).permute(3, 2, 0, 1)
	return color_tensor, depth_tensor
	
def background_subtraction(color, depth, background_color, background_depth):
	color_bg = cv2.imread(background_color)
	depth_bg = cv2.imread(background_depth, -1)
	color_bg = (color_bg/255.).astype(np.float32)
	depth_bg = (depth_bg/1000.).astype(np.float32)
	depth_bg = np.clip(depth_bg, 0.0, 10.0)
	color = (color/255.).astype(np.float32)
	depth = (depth/1000.).astype(np.float32)
	depth = np.clip(depth, 0.0, 10.0)
	
	fg_color_mask = (np.sum(np.abs(color - color_bg) < 0.3, axis = 2) != 3)
	fg_depth_mask = np.bitwise_and(depth_bg!=0, np.abs(depth - depth_bg) > 0.02)
	fg_depth_mask = np.bitwise_and(fg_depth_mask, depth!=0)
	fg_mask = np.bitwise_or(fg_color_mask, fg_depth_mask)
	# Erosion & dilation
	fg_mask = fg_mask.astype(np.uint8)*255
	kernel = np.ones((7, 7), np.uint8)
	fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
	return fg_mask

def generate_heightmap(color, depth, camera_info, background_color, background_depth, voxel_size):
	# The following coordinate has been rotated by [[0, -1, 0], [-1, 0, 0], [0, 0, -1]]
	binMiddleBottom = np.loadtxt(os.path.dirname(os.path.abspath(__file__))+"/bin_pose.txt")
	grid_origin = [binMiddleBottom[0]-voxel_size*150, binMiddleBottom[1]-voxel_size*100, binMiddleBottom[2]]
	# Background subtraction
	color_bg = cv2.imread(background_color)
	depth_bg = cv2.imread(background_depth, -1)
	color_bg = (color_bg/255.).astype(np.float32)
	depth_bg = (depth_bg/1000.).astype(np.float32)
	depth_bg = np.clip(depth_bg, 0.0, 10.0)
	color = (color/255.).astype(np.float32)
	depth = (depth/1000.).astype(np.float32)
	depth = np.clip(depth, 0.0, 10.0)
	fg_color_mask = (np.sum(np.abs(color - color_bg) < 0.3, axis = 2) != 3)
	fg_depth_mask = np.bitwise_and(depth_bg!=0, np.abs(depth - depth_bg) > 0.02)
	fg_depth_mask = np.bitwise_and(fg_depth_mask, depth!=0)
	fg_mask = np.bitwise_or(fg_color_mask, fg_depth_mask)
	# Erosion & dilation
	fg_mask = fg_mask.astype(np.uint8)*255
	kernel = np.ones((7, 7), np.uint8)
	fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
	#cv2.imwrite(os.path.dirname(os.path.abspath(__file__))+"/foreground_mask.png", fg_mask)
	
	# Heightmap placeholder
	heightMap = np.zeros((200*300), dtype=np.float32)
	heightMapColor = np.zeros((200*300, 3), dtype=np.float32)
	
	# Pointcloud in camera frame
	x = np.linspace(0, color.shape[1]-1, color.shape[1])
	y = np.linspace(0, color.shape[0]-1, color.shape[0])
	xx, yy = np.meshgrid(x, y)
	camX = (xx - camera_info.K[2])*depth/camera_info.K[0]
	camY = (yy - camera_info.K[5])*depth/camera_info.K[4]
	camZ = depth
	camPts = np.hstack([camX.reshape(-1, 1), camY.reshape(-1, 1), camZ.reshape(-1, 1)])
	cam_pose = np.loadtxt(os.path.dirname(os.path.abspath(__file__))+"/camera_pose.txt")
	worldPts = (np.matmul(cam_pose[0:3, 0:3], camPts.T) + np.matlib.repmat(cam_pose[0:3, 3].reshape(3, 1), 1, color.shape[0]*color.shape[1])).T
	gridMapping = np.vstack([np.floor((worldPts[:, 0]-grid_origin[0])/voxel_size), 
							 np.floor((worldPts[:, 1]-grid_origin[1])/voxel_size), 
							 -worldPts[:, 2]+grid_origin[2]])
	# Compute height map color
	valid_x = np.intersect1d(np.where(gridMapping[0, :]>0), np.where(gridMapping[0, :]<300))
	valid_y = np.intersect1d(np.where(gridMapping[1, :]>0), np.where(gridMapping[1, :]<200))
	validPix = np.intersect1d(valid_x, valid_y)
	#color = color[:, :, ::-1] # BGR to RGB (network expects channel in RGB)
	colorPts = np.hstack([color[:, :, 0].reshape(-1, 1), color[:, :, 1].reshape(-1, 1), color[:, :, 2].reshape(-1, 1)])
	ind_color = np.array([gridMapping[1, validPix].astype(int), gridMapping[0, validPix].astype(int)])
	heightMapColor[np.ravel_multi_index(ind_color, (200, 300)), :] = colorPts[validPix, 0:3]
	# Compute real height map with background subtraction
	validDepth = np.bitwise_and(fg_mask, camZ!=0)
	validDepth = np.ravel_multi_index(np.where(validDepth==1), depth.shape)
	valid = np.intersect1d(validDepth, validPix)
	gridMapping_depth = gridMapping[:, valid]
	gridMapping_depth[gridMapping_depth<0.0] = 0.0
	ind_depth = np.array([gridMapping_depth[1, :].astype(int), gridMapping_depth[0, :].astype(int)])
	heightMap[np.ravel_multi_index(ind_depth, (200, 300))] = gridMapping_depth[2, :]
	# Find missing depth and project background depth into camera space
	missingDepth = np.array(np.where(np.bitwise_and(depth==0, depth_bg>0)==True))
	camX = (xx - camera_info.K[2])*depth_bg/camera_info.K[0]
	camY = (yy - camera_info.K[5])*depth_bg/camera_info.K[4]
	camZ = depth_bg
	missingCamPts = np.hstack([camX[missingDepth[0, :], missingDepth[1, :]].reshape(-1, 1), 
	                           camY[missingDepth[0, :], missingDepth[1, :]].reshape(-1, 1),
	                           camZ[missingDepth[0, :], missingDepth[1, :]].reshape(-1, 1)])
	missingWorldPts = (np.matmul(cam_pose[0:3, 0:3], missingCamPts.T) + np.matlib.repmat(cam_pose[0:3, 3].reshape(3, 1), 1, missingDepth.shape[1])).T
	# Get missing depth height map
	missingHeightMap = np.zeros(200*300, dtype=np.uint8)
	gridMapping = np.vstack([np.floor((missingWorldPts[:, 0]-grid_origin[0])/voxel_size), 
							 np.floor((missingWorldPts[:, 1]-grid_origin[1])/voxel_size), 
							 -missingWorldPts[:, 2]+grid_origin[2]])
	valid_x = np.intersect1d(np.where(gridMapping[0, :]>0), np.where(gridMapping[0, :]<300))
	valid_y = np.intersect1d(np.where(gridMapping[1, :]>0), np.where(gridMapping[1, :]<200))
	validPix = np.intersect1d(valid_x, valid_y)
	gridMapping = gridMapping[:, validPix]
	ind_missing = np.array([gridMapping[1, :].astype(int), gridMapping[0, :].astype(int)])
	missingHeightMap[np.ravel_multi_index(ind_missing, (200, 300))] = 255
	missingHeightMap = missingHeightMap.reshape((200, 300))
	# Denoise missing height map
	noisePix = np.bitwise_xor(missingHeightMap, bwareaopen(missingHeightMap, 50))
	missingHeightMap[noisePix==255] = 0
	# Denoise height map
	heightMap = heightMap.reshape((200, 300))
	heightMap_mask = np.zeros(heightMap.shape, dtype=np.uint8)
	heightMap_mask[heightMap>0] = 255
	noisePix = np.bitwise_xor(heightMap_mask, bwareaopen(heightMap_mask, 50))
	heightMap[noisePix==255] = 0
	# Fill in missing height map (assume height of 3 cm)
	heightMap[np.where(np.bitwise_and(heightMap==0, missingHeightMap==255))] = 0.03
	# Save height map with extra padding
	# +60px both side of y-axis, +10px both side of x-axis
	heightMapColor = heightMapColor.reshape((200, 300, 3))
	color_data = np.zeros((320, 320, 3), dtype=np.uint8)
	depth_data = np.zeros((320, 320), dtype=np.uint16)
	color_data[60: 260, 10: 310, :] = (heightMapColor*255).astype(np.uint8)
	depth_data[60: 260, 10: 310] = (heightMap*1000).astype(np.uint16)
	return color_data, depth_data # Color in BGR order
	'''
	# Draw
	heightMapCP = copy.deepcopy(heightMap)
	heightMapCP = (heightMapCP/0.3*255).astype(np.uint8)
	heightMapCP = heightMapCP.reshape((200, 300))
	heightMapCP = cv2.applyColorMap(heightMapCP, cv2.COLORMAP_JET)
	cv2.imwrite(os.path.dirname(os.path.abspath(__file__))+"/heightmap_depth.png", heightMapCP)
	heightMapCP = copy.deepcopy(heightMapColor)
	heightMapCP = (heightMapCP*255).astype(np.uint8)
	heightMapCP = heightMapCP.reshape((200, 300, 3))
	cv2.imwrite(os.path.dirname(os.path.abspath(__file__))+"/heightMapColor.jpg", heightMapCP[:, :, ::-1])
	'''

def viz_heightMap(heightMap):
	cp = copy.deepcopy(heightMap)
	cp = (cp/0.3*255).astype(np.uint8)
	cp = cv2.applyColorMap(cp, cv2.COLORMAP_JET)
	return cp

def viz_affordance(affordance):
	draw = copy.deepcopy(affordance)
	draw[draw>=1.0] = 0.999999
	draw[draw<0] = 0.0
	draw = (draw*255).astype(np.uint8)
	draw = cv2.applyColorMap(draw, cv2.COLORMAP_JET)
	return draw

def rotate_heightmap(color_heightmap, depth_heightmap, angle):
	(h, w) = color_heightmap.shape[:2]
	center = (w/2, h/2)
	M = cv2.getRotationMatrix2D(center, angle, 1.0)
	rotated_color_heightmap = cv2.warpAffine(color_heightmap, M, (w, h))
	rotated_depth_heightmap = cv2.warpAffine(depth_heightmap, M, (w, h))
	return rotated_color_heightmap, rotated_depth_heightmap

def rotate_img(img, angle):
	(h, w) = img.shape[:2]
	center = (w/2, h/2)
	M = cv2.getRotationMatrix2D(center, angle, 1.0)
	rotate_img = cv2.warpAffine(img, M, (w, h))
	return rotate_img

# For computing connected components label
class Graph:
	def __init__(self):
		self.connect = []
	def add_connect(self, con):
		if len(self.connect)==0:
			self.connect.append(con)
			return
		arr = []
		for idx in range(len(self.connect)):
			connect = self.connect[idx]
			if np.intersect1d(connect, con).shape[0] != 0: # If intersects, save the index
				arr.append(idx)
		if len(arr) == 0: # No intersect, add to graph
			self.connect.append(con)
		elif len(arr) == 1:
			self.connect[arr[0]] = np.union1d(self.connect[arr[0]], con)
		else: # More than 1, then combine the connectivity
			combined = self.connect[arr[0]]
			for idx in range(1, len(arr)):
				combined = np.union1d(combined, self.connect[arr[idx]])
				del self.connect[arr[idx]]
			self.connect[arr[0]] = combined

# Python implementation for `bwareaopen` in Matlab
def bwareaopen(mask, P):
	# Find which connected pixel is accessible
	def findVisitPix(i, j, shape):
		directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1)] # 8-connect
		visit = []
		for direction in directions:
			u = i+direction[0]; v = j+direction[1]
			if u>=0 and u<shape[0] and v>=0 and v<shape[1]:
				visit.append([u, v])
		return visit
	res = copy.deepcopy(mask)
	g = Graph()
	label = np.zeros(mask.shape, dtype=np.uint8)
	label_val = 1
	# First pass, create labels
	for i in range(0, mask.shape[0]):
		for j in range(0, mask.shape[1]):
			if mask[i][j]!=0: # Only handle if non-zero
				visit = findVisitPix(i, j, mask.shape)
				if len(visit)==0: # (0, 0) non-zero
					label[i, j] = label_val; label_val+=1; continue
				elif len(visit)==1: # First row
					if label[visit[0][0]][visit[0][1]]!=0: # left non-zero -> inherit from it
						label[i, j] = label[visit[0][0]][visit[0][1]]
					else: # left zero -> add new label
						label[i, j] = label_val; label_val+=1; continue
				else: # Pixels except first row
					val = [] # Non-zero placeholder
					for nn in visit:
						if label[nn[0]][nn[1]]!=0: # Only consider non-zero
							if len(val)==0: val.append(label[nn[0]][nn[1]])
							else:
								if not label[nn[0]][nn[1]] in val: val.append(label[nn[0]][nn[1]]); val.sort()# Prevent duplicated
					if len(val)==0: # all zero in 4 neighbors -> add new label
						label[i, j] = label_val; label_val+=1; continue
					elif len(val)==1: # same values appears in all visitable cells -> inherit the label
						label[i, j] = val[0]
					else: # More than 1 value around the cell -> choose the smallest, and the corresponding indices are connected
						label[i, j] = val[0]; g.add_connect(val)
	# end First pass
	# Second pass, replace connected labels
	final_label = []
	for i in range(0, mask.shape[0]):
		for j in range(0, mask.shape[1]):
			if label[i, j]!=0: # Only consider non-zero
				for connect in g.connect:
					if label[i, j] in connect: # The label appears in `connect`
						label[i, j] = min(connect)
						if not min(connect) in final_label: final_label.append(min(connect)); break # Will not appears in other `connect`
				if not label[i, j] in final_label: final_label.append(label[i, j]) # alone one
	# Remove pixels that number of connected pixels less than `P`
	for val in final_label:
		if len(np.where(label==val)[0]) < P:
			res[np.where(label==val)] = 0 # Change from 255 to 0
	return res

def draw_action(color_hm, pixel, primitive="suck"):
	res = copy.deepcopy(color_hm)
	if primitive=="suck":
		cv2.circle(res, tuple(pixel), 7, (0, 0, 0), 2)
	elif primitive=="grasp":
		x = 20
		y = 10
		line_1_upper = (pixel[0]-x, pixel[1]-y)
		line_1_lower = (pixel[0]-x, pixel[1]+y)
		line_2_upper = (pixel[0]+x, pixel[1]-y)
		line_2_lower = (pixel[0]+x, pixel[1]+y)
		cv2.circle(res, tuple(pixel), 7, (0, 0, 0), 2)
		cv2.line(res, line_1_upper, line_1_lower, (0, 0, 0), 3)
		cv2.line(res, line_2_upper, line_2_lower, (0, 0, 0), 3)
	return res
		
def draw_affordance(prediction):
	# @prediciton: background area set to 0
	draw = copy.deepcopy(prediction)
	draw[draw>=1.0] = 0.99999
	draw[draw<0.0]  = 0.0
	hm = (draw*255).astype(np.uint8) # float -> uint8
	hm = cv2.applyColorMap(hm, cv2.COLORMAP_JET)
	hm = cv2.GaussianBlur(hm, (29, 29), 7)
	return hm
		
class Net(nn.Module):
	def __init__(self, n_classes):
		super(Net, self).__init__()
		self.color_trunk = torchvision.models.resnet101(pretrained=True)
		del self.color_trunk.fc, self.color_trunk.avgpool, self.color_trunk.layer4
		self.depth_trunk = copy.deepcopy(self.color_trunk)
		self.conv1 = nn.Conv2d(2048, 512, 1)
		self.conv2 = nn.Conv2d(512, 128, 1)
		self.conv3 = nn.Conv2d(128, n_classes, 1)
	def forward(self, color, depth):
		# Color
		color_feat_1 = self.color_trunk.conv1(color) # 3 -> 64
		color_feat_1 = self.color_trunk.bn1(color_feat_1)
		color_feat_1 = self.color_trunk.relu(color_feat_1)
		color_feat_1 = self.color_trunk.maxpool(color_feat_1)
		color_feat_2 = self.color_trunk.layer1(color_feat_1) # 64 -> 256
		color_feat_3 = self.color_trunk.layer2(color_feat_2) # 256 -> 512
		color_feat_4 = self.color_trunk.layer3(color_feat_3) # 512 -> 1024
		# Depth
		depth_feat_1 = self.depth_trunk.conv1(depth) # 3 -> 64
		depth_feat_1 = self.color_trunk.bn1(depth_feat_1)
		depth_feat_1 = self.color_trunk.relu(depth_feat_1)
		depth_feat_1 = self.color_trunk.maxpool(depth_feat_1)
		depth_feat_2 = self.color_trunk.layer1(depth_feat_1) # 64 -> 256
		depth_feat_3 = self.color_trunk.layer2(depth_feat_2) # 256 -> 512
		depth_feat_4 = self.color_trunk.layer3(depth_feat_3) # 512 -> 1024depth_feat_1
		# Concatenate
		feat = torch.cat([color_feat_4, depth_feat_4], dim=1) # 2048
		feat_1 = self.conv1(feat)
		feat_2 = self.conv2(feat_1)
		feat_3 = self.conv3(feat_2)
		return F.interpolate(feat_3, scale_factor=2, mode="bilinear", align_corners=True)
