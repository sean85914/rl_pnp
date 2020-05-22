import os
import sys
import copy
import time
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
from torch.utils.data import Dataset, DataLoader

# Dataloader
image_net_mean = np.array([0.485, 0.456, 0.406])
image_net_std  = np.array([0.229, 0.224, 0.225])

class suction_based_grasping_dataset(Dataset):
    name = []
    def __init__(self, n_classes, data_dir, scale=1./8):
        self.n_classes = n_classes
        self.data_dir = data_dir
        self.scale = scale
        f = open(self.data_dir+"train-split.txt", "r")
        for i, line in enumerate(f):
            self.name.append(line.replace("\n", ""))
    def __len__(self):
        return len(self.name)
    def __getitem__(self, idx):
        idx_name = self.name[idx]
        color_img = cv2.imread(self.data_dir+"color-input/"+idx_name+".png")
        depth_img = cv2.imread(self.data_dir+"depth-input/"+idx_name+".png", -1)
        label_img = cv2.imread(self.data_dir+"label/"+idx_name+".png", cv2.IMREAD_GRAYSCALE)
        # uint8 -> float
        color = (color_img/255.).astype(float)
        # BGR -> RGB and normalize
        color_rgb = np.zeros(color.shape)
        for i in range(3):
            color_rgb[:, :, i] = (color[:, :, 2-i]-image_net_mean[i])/image_net_std[i]
        depth = (depth_img/1000.).astype(float) # to meters
        # SR300 depth range
        depth = np.clip(depth, 0.0, 1.2)
        # Duplicate channel and normalize
        depth_3c = np.zeros(color.shape)
        for i in range(3):
            depth_3c[:, :, i] = (depth[:, :]-image_net_mean[i])/image_net_std[i]
        # Unlabeled -> 2; unsuctionable -> 0; suctionable -> 1
        label = np.round(label_img/255.*2.).astype(float)
        label = cv2.resize(label, (int(label.shape[1]*self.scale), int(label.shape[0]*self.scale)))
        transform = transforms.Compose([
                        transforms.ToTensor(),
                    ])
        color_tensor = transform(color_rgb).float()
        depth_tensor = transform(depth_3c).float()
        label_tensor = transform(label).float()
        sample = {"color": color_tensor, "depth": depth_tensor, "label": label_tensor}
        return sample
        
# Network
class SuckNet(nn.Module):
    def __init__(self, n_classes):
        super(SuckNet, self).__init__()
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
        depth_feat_1 = self.depth_trunk.bn1(depth_feat_1)
        depth_feat_1 = self.depth_trunk.relu(depth_feat_1)
        depth_feat_1 = self.depth_trunk.maxpool(depth_feat_1) 
        depth_feat_2 = self.depth_trunk.layer1(depth_feat_1) # 64 -> 256
        depth_feat_3 = self.depth_trunk.layer2(depth_feat_2) # 256 -> 512
        depth_feat_4 = self.depth_trunk.layer3(depth_feat_3) # 512 -> 1024
        # Concatenate
        feat = torch.cat([color_feat_4, depth_feat_4], dim=1) # 2048
        feat_1 = self.conv1(feat)
        feat_2 = self.conv2(feat_1)
        feat_3 = self.conv3(feat_2)
        return nn.Upsample(scale_factor=2, mode="bilinear")(feat_3)
        
# Train
#FIXME
epochs = 400
save_every = 10
batch_size = 3
class_weight = torch.ones(3)
class_weight[2] = 0
criterion = nn.CrossEntropyLoss(class_weight).cuda()
optimizer = optim.SGD(net.parameters(), lr = 1e-3, momentum=0.99)
scheduler = optim.lr_scheduler.StepLR(optimizer, step_size = 25, gamma = 0.1)
dataset = suction_based_grasping_dataset(3, "data/")
dataloader = DataLoader(dataset, batch_size = batch_size, shuffle = True, num_workers = 8)

loss_l = []
for epoch in range(epochs):
    loss_sum = 0.0
    ts = time.time()
    for i_batch, sampled_batched in enumerate(dataloader):
        print("\r[{:03.2f} %]".format(i_batch/float(len(dataloader))*100.0), end="\r")
        optimizer.zero_grad()
        color = sampled_batched['color'].cuda()
        depth = sampled_batched['depth'].cuda()
        label = sampled_batched['label'].cuda().long()
        predict = net(color, depth)
        loss = criterion(predict.view(len(sampled_batched['color']), 3,80*60), label.view(len(sampled_batched['color']), 80*60))
        loss.backward()
        loss_sum += loss.detach().cpu().numpy()
        optimizer.step()
    scheduler.step()
    if (epoch+1)%save_every==0:
        torch.save(net.state_dict(), "models_batch_3/net_{}.pth".format(epoch+1))
    loss_l.append(loss_sum/len(dataloader))
    print("Epoch: {}| Loss: {}| Time elasped: {}".format(epoch+1, loss_l[-1], time.time()-ts))
    
np.savetxt("suck_net_loss.csv", loss_l, delimiter=",")
