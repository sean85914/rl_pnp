import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
from torchvision import transforms as TF
from collections import OrderedDict

class reinforcement_net(nn.Module):

	def __init__(self, use_cuda):
		super(reinforcement_net, self).__init__()
		self.use_cuda = use_cuda
		
		# Initialize Densenet pretrained on ImageNet
		self.color_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		self.depth_feat_extractor = torchvision.models.densenet.densenet121(pretrained = True)
		# We don't need there fully connected layers
		del self.color_feat_extractor.classifier, self.depth_feat_extractor.classifier
		
		self.primitive_net = nn.Sequential(OrderedDict([
		  ('suck-conv0', nn.Conv2d(2048, 64, kernel_size = 1, stride = 1, bias = True)),
		  ('suck-relu0', nn.ReLU(inplace = True)),
		  ('suck-norm0', nn.BatchNorm2d(64)),
		  ('suck-upsample0', nn.Upsample(scale_factor = 4, mode = "bilinear")),
		  ("suck-conv1", nn.Conv2d(64, 1, kernel_size = 1, stride = 1, bias = True)),
		  ("suck-relu1", nn.ReLU(inplace = True)),
		  ('suck-norm1', nn.BatchNorm2d(1)),
		  ("suck-upsample1", nn.Upsample(scale_factor = 4, mode="bilinear"))
		]))
		
		# Initialize network weights
		for m in self.named_modules():
			if 'suck-' in m[0]:
				if isinstance(m[1], nn.Conv2d):
					nn.init.kaiming_normal_(m[1].weight.data)
				elif isinstance(m[1], nn.BatchNorm2d):
					m[1].weight.data.fill_(1)
					m[1].bias.data.zero_()
		
		# Initialize output variables (for back propagation)
		self.interm_feat = []
		self.output_prob = []
		
	def forward(self, input_color_data, input_depth_data, is_volatile = False, clear_grad = False):
		if is_volatile:
			if self.use_cuda:
				input_color_data = input_color_data.cuda()
				input_depth_data = input_depth_data.cuda()
			interm_color_feat = self.color_feat_extractor.features(input_color_data)
			interm_depth_feat = self.depth_feat_extractor.features(input_depth_data)
			interm_color_feat.detach()
			interm_depth_feat.detach()
			interm_feat = torch.cat((interm_color_feat, interm_depth_feat), dim = 1)
			output_prob = self.primitive_net(interm_feat)
			return output_prob, interm_feat
		else:
			self.output_prob = []
			self.interm_feat = []
			if self.use_cuda:
				input_color_data = input_color_data.cuda()
				input_depth_data = input_depth_data.cuda()
			interm_color_feat = self.color_feat_extractor.features(input_color_data)
			interm_depth_feat = self.depth_feat_extractor.features(input_depth_data)
			if clear_grad:
				interm_color_feat.detach()
				interm_depth_feat.detach()
			self.interm_feat = torch.cat((interm_color_feat, interm_depth_feat), dim = 1)
			self.output_prob = self.primitive_net(self.interm_feat)
			return self.output_prob, self.interm_feat
