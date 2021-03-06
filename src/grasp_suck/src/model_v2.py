import numpy as np
from PIL import Image
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
from torchvision import transforms as TF
from collections import OrderedDict

def tensor_to_PIL(tensor):
	image = tensor.cpu().clone()
	image = image.squeeze(0)
	image = TF.ToPILImage()(image)
	return image # Can be visualized simply by image.show()

def rotate_heightmap(color_tensor, depth_tensor, theta, use_cuda):
	# theta in radian
	affine_mat_before = np.asarray([[ np.cos(-theta), -np.sin(-theta), 0],
	                                [ np.sin(-theta),  np.cos(-theta), 0]])
	affine_mat_before.shape = (2, 3, 1)
	affine_mat_before = torch.from_numpy(affine_mat_before).permute(2, 0, 1).float()
	if use_cuda:
		flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False).cuda(), color_tensor.size())
		rotate_color_tensor = F.grid_sample(Variable(color_tensor, volatile=True).cuda(), flow_grid_before, mode="nearest")
		rotate_depth_tensor = F.grid_sample(Variable(depth_tensor, volatile=True).cuda(), flow_grid_before, mode="nearest")
	else:
		flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False), color_tensor.size())
		rotate_color_tensor = F.grid_sample(Variable(color_tensor, volatile=True), flow_grid_before, mode="nearest")
		rotate_depth_tensor = F.grid_sample(Variable(depth_tensor, volatile=True), flow_grid_before, mode="nearest")
	return rotate_color_tensor, rotate_depth_tensor
	
def rotate_featuremap(feature_tensor, theta, use_cuda):
	# theta in radian
	affine_mat_after = np.asarray([[ np.cos(-theta), -np.sin(-theta), 0],
	                               [ np.sin(-theta),  np.cos(-theta), 0]])
	affine_mat_after.shape = (2, 3, 1)
	affine_mat_after = torch.from_numpy(affine_mat_after).permute(2, 0, 1).float()
	if use_cuda:
		flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False).cuda(), feature_tensor.size())
		rotate_feature = F.grid_sample(feature_tensor, flow_grid_after, mode="nearest")
	else:
		flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False), feature_tensor.size())
		rotate_feature = F.grid_sample(feature_tensor, flow_grid_after, mode="nearest")
	return rotate_feature
	
class reinforcement_net(nn.Module):
	def __init__(self, use_cuda, num_rotations=4):
		super(reinforcement_net, self).__init__()
		self.use_cuda = use_cuda
		self.num_rotations = num_rotations
		
		# Initialize Densenet pretrained on ImageNet
		self.suck_1_color_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		self.suck_1_depth_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		self.suck_2_color_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		self.suck_2_depth_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		self.grasp_color_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		self.grasp_depth_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		# We don't need there fully connected layers
		del self.suck_1_color_feat_extractor.classifier, self.suck_1_depth_feat_extractor.classifier, \
		    self.suck_2_color_feat_extractor.classifier, self.suck_2_depth_feat_extractor.classifier, \
		    self.grasp_color_feat_extractor.classifier,  self.grasp_depth_feat_extractor.classifier
		
		# Suck 1, corresponding to tool ID 3
		self.suck_1_net = nn.Sequential(OrderedDict([
		  ('suck1-norm0', nn.BatchNorm2d(2048)),
		  ('suck1-relu0', nn.ReLU(inplace = True)),
		  ('suck1-conv0', nn.Conv2d(2048, 64, kernel_size = 1, stride = 1, bias = False)),
		  ('suck1-norm1', nn.BatchNorm2d(64)),
		  ('suck1-relu1', nn.ReLU(inplace = True)),
		  ('suck1-upsample0', nn.Upsample(scale_factor = 4, mode="bilinear")),
		  ("suck1-conv1", nn.Conv2d(64, 1, kernel_size = 1, stride = 1, bias = False)),
		  ('suck1-norm2', nn.BatchNorm2d(1)),
		  ("suck1-upsample1", nn.Upsample(scale_factor = 4, mode="bilinear"))
		]))
		# Suck 2, corresponding to tool ID 2
		self.suck_2_net = nn.Sequential(OrderedDict([
		  ('suck2-norm0', nn.BatchNorm2d(2048)),
		  ('suck2-relu0', nn.ReLU(inplace = True)),
		  ('suck2-conv0', nn.Conv2d(2048, 64, kernel_size = 1, stride = 1, bias = False)),
		  ('suck2-norm1', nn.BatchNorm2d(64)),
		  ('suck2-relu1', nn.ReLU(inplace = True)),
		  ('suck2-upsample0', nn.Upsample(scale_factor = 4, mode="bilinear")),
		  ("suck2-conv1", nn.Conv2d(64, 1, kernel_size = 1, stride = 1, bias = False)),
		  ('suck2-norm2', nn.BatchNorm2d(1)),
		  ("suck2-upsample1", nn.Upsample(scale_factor = 4, mode="bilinear"))
		]))
		# Gripper, corresponding to tool ID 1
		self.grasp_net = nn.Sequential(OrderedDict([
		  ('grasp-norm0', nn.BatchNorm2d(2048)),
		  ('grasp-relu0', nn.ReLU(inplace = True)),
		  ('grasp-conv0', nn.Conv2d(2048, 64, kernel_size = 1, stride = 1, bias = False)),
		  ('grasp-norm1', nn.BatchNorm2d(64)),
		  ('grasp-relu1', nn.ReLU(inplace = True)),
		  ('grasp-upsample0', nn.Upsample(scale_factor = 4, mode="bilinear")),
		  ("grasp-conv1", nn.Conv2d(64, 1, kernel_size = 1, stride = 1, bias = False)),
		  ('grasp-norm2', nn.BatchNorm2d(1)),
		  ("grasp-upsample1", nn.Upsample(scale_factor = 4, mode="bilinear"))
		]))
		
		# Initialize network weights
		for m in self.named_modules():
			#if 'suck1-' in m[0] or 'suck2-' in m[0] or 'grasp-' in m[0]:
			if 'suck1-' in m[0]:
				if isinstance(m[1], nn.Conv2d):
					nn.init.kaiming_normal_(m[1].weight.data)
					if 'conv0' in m[0]:
						self.suck_2_net[2].load_state_dict(self.suck_1_net[2].state_dict())
						self.grasp_net[2].load_state_dict(self.suck_1_net[2].state_dict())
					elif 'conv1' in m[0]:
						self.suck_2_net[6].load_state_dict(self.suck_1_net[6].state_dict())
						self.grasp_net[6].load_state_dict(self.suck_1_net[6].state_dict())
					#m[1].bias.data.zero_()
				elif isinstance(m[1], nn.BatchNorm2d):
					m[1].weight.data.fill_(1)
					m[1].bias.data.zero_()
		# Initialize output variables (for back propagation)
		self.output_prob = None
		
	def forward(self, input_color_data, input_depth_data, action_str = None, is_volatile = False, specific_rotation = -1, clear_grad = False):
		if is_volatile: # For choosing action
			output_prob = []
			if self.use_cuda:
				input_color_data = input_color_data.cuda()
				input_depth_data = input_depth_data.cuda()
			with torch.no_grad():
				suck_1_color_feat = self.suck_1_color_feat_extractor.features(input_color_data)
				suck_1_depth_feat = self.suck_1_depth_feat_extractor.features(input_depth_data)
				suck_1_feat = torch.cat((suck_1_color_feat, suck_1_depth_feat), dim = 1)
				output_prob.append(self.suck_1_net(suck_1_feat))
				del suck_1_color_feat, suck_1_depth_feat, suck_1_feat
				suck_2_color_feat = self.suck_2_color_feat_extractor.features(input_color_data)
				suck_2_depth_feat = self.suck_2_depth_feat_extractor.features(input_depth_data)
				suck_2_feat = torch.cat((suck_2_color_feat, suck_2_depth_feat), dim = 1)
				output_prob.append(self.suck_2_net(suck_2_feat))
				del suck_2_color_feat, suck_2_depth_feat, suck_2_feat
				
			# Rotation
			for rotate_idx in range(self.num_rotations):
				theta = np.radians(-90.0+(180.0/self.num_rotations)*rotate_idx)
				rotate_color, rotate_depth = rotate_heightmap(input_color_data, input_depth_data, theta, self.use_cuda)
				with torch.no_grad():
					interm_color_feat_rotated = self.grasp_color_feat_extractor.features(rotate_color)
					interm_depth_feat_rotated = self.grasp_depth_feat_extractor.features(rotate_depth)
					interm_feat_rotated = torch.cat((interm_color_feat_rotated, interm_depth_feat_rotated), dim=1)
					#interm_feat = rotate_featuremap(interm_feat_rotated, -theta, self.use_cuda)
					output_prob.append(rotate_featuremap(self.grasp_net(interm_feat_rotated), -theta, self.use_cuda))
			return output_prob
		else: # For backpropagation, or computing TD target
			self.output_prob = None
			if self.use_cuda:
				input_color_data = input_color_data.cuda()
				input_depth_data = input_depth_data.cuda()
			if "suck_1" in action_str:
				suck_1_color_feat = self.suck_1_color_feat_extractor.features(input_color_data)
				suck_1_depth_feat = self.suck_1_depth_feat_extractor.features(input_depth_data)
				if clear_grad: # Target doesn't need gradient
					suck_1_color_feat.detach()
					suck_1_depth_feat.detach()
				suck_1_feat = torch.cat((suck_1_color_feat, suck_1_depth_feat), dim = 1)
				self.output_prob = self.suck_1_net(suck_1_feat)
			elif "suck_2" in action_str:
				suck_2_color_feat = self.suck_2_color_feat_extractor.features(input_color_data)
				suck_2_depth_feat = self.suck_2_depth_feat_extractor.features(input_depth_data)
				if clear_grad:
					suck_2_color_feat.detach()
					suck_2_depth_feat.detach()
				suck_2_feat = torch.cat((suck_2_color_feat, suck_2_depth_feat), dim = 1)
				self.output_prob = self.suck_2_net(suck_2_feat)
			elif "grasp" in action_str: # "grasp"
				rotate_idx = specific_rotation
				theta = np.radians(-90.0+(180.0/self.num_rotations)*rotate_idx)
				rotate_color, rotate_depth = rotate_heightmap(input_color_data, input_depth_data, theta, self.use_cuda)
				interm_color_feat_rotated = self.grasp_color_feat_extractor.features(rotate_color)
				interm_depth_feat_rotated = self.grasp_depth_feat_extractor.features(rotate_depth)
				if clear_grad:
					interm_color_feat_rotated.detach()
					interm_depth_feat_rotated.detach()
				interm_feat_rotated = torch.cat((interm_color_feat_rotated, interm_depth_feat_rotated), dim=1)
				#interm_feat = rotate_featuremap(interm_feat_rotated, -theta, self.use_cuda)
				self.output_prob = rotate_featuremap(self.grasp_net(interm_feat_rotated), -theta, self.use_cuda)
			return self.output_prob
