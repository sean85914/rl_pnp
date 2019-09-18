from collections import OrderedDict
import numpy as np
import torch  
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
from torchvision import transforms as TF

ToPIL = TF.ToPILImage()
ToTensor = TF.ToTensor()

class reinforcement_net(nn.Module):

    def __init__(self, use_cuda, num_rotations=8):
        super(reinforcement_net, self).__init__()
        self.use_cuda = use_cuda

        # Initialize network trunks with DenseNet pre-trained on ImageNet
        self.suck_color_trunk = torchvision.models.densenet.densenet121(pretrained=True)
        self.suck_depth_trunk = torchvision.models.densenet.densenet121(pretrained=True)
        self.grasp_color_trunk = torchvision.models.densenet.densenet121(pretrained=True)
        self.grasp_depth_trunk = torchvision.models.densenet.densenet121(pretrained=True)

        self.num_rotations = num_rotations

        # Construct network branches for sucking and grasping
        self.sucknet = nn.Sequential(OrderedDict([
            ('suck-norm0', nn.BatchNorm2d(2048)),
            ('suck-relu0', nn.ReLU(inplace=True)),
            ('suck-conv0', nn.Conv2d(2048, 64, kernel_size=1, stride=1, bias=True)),
            ('suck-norm1', nn.BatchNorm2d(64)),
            ('suck-relu1', nn.ReLU(inplace=True)),
            ('suck-conv1', nn.Conv2d(64, 1, kernel_size=1, stride=1, bias=True)),
            ('suck-norm2', nn.BatchNorm2d(1)),
            ('suck-relu2', nn.ReLU(inplace=True))
            # ('suck-upsample2', nn.Upsample(scale_factor=4, mode='bilinear'))
        ]))
        self.graspnet = nn.Sequential(OrderedDict([
            ('grasp-norm0', nn.BatchNorm2d(2048)),
            ('grasp-relu0', nn.ReLU(inplace=True)),
            ('grasp-conv0', nn.Conv2d(2048, 64, kernel_size=1, stride=1, bias=True)),
            ('grasp-norm1', nn.BatchNorm2d(64)),
            ('grasp-relu1', nn.ReLU(inplace=True)),
            ('grasp-conv1', nn.Conv2d(64, 1, kernel_size=1, stride=1, bias=True)),
            ('grasp-norm2', nn.BatchNorm2d(1)),
            ('grasp-relu2', nn.ReLU(inplace=True))
            # ('grasp-upsample2', nn.Upsample(scale_factor=4, mode='bilinear'))
        ]))

        # Initialize network weights
        for m in self.named_modules():
            if 'suck-' in m[0] or 'grasp-' in m[0]:
                if isinstance(m[1], nn.Conv2d):
                    nn.init.kaiming_normal_(m[1].weight.data)
                elif isinstance(m[1], nn.BatchNorm2d):
                    m[1].weight.data.fill_(1)
                    m[1].bias.data.zero_()

        # Initialize output variable (for backprop)
        self.interm_feat = []
        self.output_prob = []


    def forward(self, input_color_data, input_depth_data, is_volatile=False, specific_rotation=-1, clear_grad = False):
        if is_volatile:
            output_prob = []
            interm_feat = []
            if self.use_cuda:
                input_color_data = input_color_data.cuda()
                input_depth_data = input_depth_data.cuda()
            # suck is undirectional, so don't have to rotate
            interm_suck_color_feat = self.suck_color_trunk.features(input_color_data)
            interm_suck_depth_feat = self.suck_depth_trunk.features(input_depth_data)
            interm_suck_color_feat = interm_suck_color_feat.detach()
            interm_suck_depth_feat = interm_suck_depth_feat.detach()
            interm_suck_feat = torch.cat((interm_suck_color_feat, interm_suck_depth_feat), dim=1)
            interm_feat.append(interm_suck_feat.cpu())
            output_prob.append(nn.Upsample(scale_factor=16, mode='bilinear').forward(self.sucknet(interm_suck_feat)).cpu())
            if self.use_cuda: # Back to CPU
                input_color_data = input_color_data.cpu()
                input_depth_data = input_depth_data.cpu()
            # Apply rotations to images
            for rotate_idx in range(self.num_rotations):
                # -90, -45, 0, 45, 90
                rotate_theta = np.radians(-90.0+(180.0/self.num_rotations)*rotate_idx)
                
                # Compute sample grid for rotation BEFORE neural network
                affine_mat_before = np.asarray([[np.cos(rotate_theta), np.sin(rotate_theta), 0],[-np.sin(rotate_theta), np.cos(rotate_theta), 0]])
                affine_mat_before.shape = (2,3,1)
                affine_mat_before = torch.from_numpy(affine_mat_before).permute(2,0,1).float()
                if self.use_cuda:
                    flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False).cuda(), input_color_data.size())
                else:
                    flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False), input_color_data.size())
                # Rotate images 'counterclockwise'
                if self.use_cuda:
                    with torch.no_grad():
                        rotate_color = F.grid_sample(Variable(input_color_data).cuda(), flow_grid_before, mode='nearest')
                        rotate_depth = F.grid_sample(Variable(input_depth_data).cuda(), flow_grid_before, mode='nearest')
                else:
                    with torch.no_grad():
                        rotate_color = F.grid_sample(Variable(input_color_data), flow_grid_before, mode='nearest')
                        rotate_depth = F.grid_sample(Variable(input_depth_data), flow_grid_before, mode='nearest')
                
                # Compute intermediate features
                interm_grasp_color_feat = self.grasp_color_trunk.features(rotate_color)
                interm_grasp_depth_feat = self.grasp_depth_trunk.features(rotate_depth)
                interm_grasp_color_feat = interm_grasp_color_feat.detach()
                interm_grasp_depth_feat = interm_grasp_depth_feat.detach()
                interm_grasp_feat = torch.cat((interm_grasp_color_feat, interm_grasp_depth_feat), dim=1)
                interm_feat.append(interm_grasp_feat.cpu())

                # Compute sample grid for rotation AFTER branches
                affine_mat_after = np.asarray([[np.cos(-rotate_theta), np.sin(-rotate_theta), 0],[-np.sin(-rotate_theta), np.cos(-rotate_theta), 0]])
                affine_mat_after.shape = (2,3,1)
                affine_mat_after = torch.from_numpy(affine_mat_after).permute(2,0,1).float()
                if self.use_cuda:
                    flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False).cuda(), interm_suck_feat.data.size())
                else:
                    flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False), interm_suck_feat.data.size())
                
                # Forward pass through branches, undo rotation on output predictions, upsample results
                output_prob.append(nn.Upsample(scale_factor=16, mode='bilinear').forward(F.grid_sample(self.graspnet(interm_grasp_feat), flow_grid_after, mode='nearest')).cpu())
                # Clear old memory
                del interm_grasp_color_feat, interm_grasp_depth_feat, interm_grasp_feat, rotate_color, rotate_depth, flow_grid_after, flow_grid_before
                torch.cuda.empty_cache()
            return output_prob, interm_feat

        else: # volatile is false
            self.output_prob = []
            self.interm_feat = []
            if self.use_cuda:
                input_color_data = input_color_data.cuda()
                input_depth_data = input_depth_data.cuda()

            # Suck is undirectional, so don't have to rotate
            interm_suck_color_feat = self.suck_color_trunk.features(input_color_data)
            interm_suck_depth_feat = self.suck_depth_trunk.features(input_depth_data)
            if clear_grad:
                interm_suck_color_feat = interm_suck_color_feat.detach()
                interm_suck_depth_feat = interm_suck_depth_feat.detach()
            interm_suck_feat = torch.cat((interm_suck_color_feat, interm_suck_depth_feat), dim=1)

            self.interm_feat.append(interm_suck_feat)
            self.output_prob.append(nn.Upsample(scale_factor=16, mode='bilinear').forward(self.sucknet(interm_suck_feat)))

            # Apply rotations to intermediate features
            # for rotate_idx in range(self.num_rotations):
            rotate_idx = specific_rotation
            rotate_theta = np.radians(-90.0+(180.0/self.num_rotations-1)*rotate_idx)
            
            # Compute sample grid for rotation BEFORE branches
            affine_mat_before = np.asarray([[np.cos(rotate_theta), np.sin(rotate_theta), 0],[-np.sin(rotate_theta), np.cos(rotate_theta), 0]])
            affine_mat_before.shape = (2,3,1)
            affine_mat_before = torch.from_numpy(affine_mat_before).permute(2,0,1).float()
            if self.use_cuda:
                flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False).cuda(), input_color_data.size())
            else:
                flow_grid_before = F.affine_grid(Variable(affine_mat_before, requires_grad=False), input_color_data.size())
             
            # Rotate images 'counterclockwise'
            if self.use_cuda:
                rotate_color = F.grid_sample(Variable(input_color_data, requires_grad=False).cuda(), flow_grid_before, mode='nearest')
                rotate_depth = F.grid_sample(Variable(input_depth_data, requires_grad=False).cuda(), flow_grid_before, mode='nearest')
            else:
                rotate_color = F.grid_sample(Variable(input_color_data, requires_grad=False), flow_grid_before, mode='nearest')
                rotate_depth = F.grid_sample(Variable(input_depth_data, requires_grad=False), flow_grid_before, mode='nearest')

            # Compute intermediate features
            
            interm_grasp_color_feat = self.grasp_color_trunk.features(rotate_color)
            interm_grasp_depth_feat = self.grasp_depth_trunk.features(rotate_depth)
            if clear_grad:
                interm_grasp_color_feat = interm_grasp_color_feat.detach()
                interm_grasp_depth_feat = interm_grasp_depth_feat.detach()
            interm_grasp_feat = torch.cat((interm_grasp_color_feat, interm_grasp_depth_feat), dim=1)
            self.interm_feat.append(interm_grasp_feat)
            
            # Compute sample grid for rotation AFTER branches
            affine_mat_after = np.asarray([[np.cos(-rotate_theta), np.sin(-rotate_theta), 0],[-np.sin(-rotate_theta), np.cos(-rotate_theta), 0]])
            affine_mat_after.shape = (2,3,1)
            affine_mat_after = torch.from_numpy(affine_mat_after).permute(2,0,1).float()
            if self.use_cuda:
                flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False).cuda(), interm_suck_feat.data.size())
            else:
                flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False), interm_suck_feat.data.size())
            
            # Forward pass through branches, undo rotation on output predictions, upsample results
            self.output_prob.append(nn.Upsample(scale_factor=16, mode='bilinear').forward(F.grid_sample(self.graspnet(interm_grasp_feat), flow_grid_after, mode='nearest')))
                
            return self.output_prob, self.interm_feat
