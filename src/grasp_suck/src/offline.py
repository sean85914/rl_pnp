import os
import time
import numpy as np
import cv2
import torch
from trainer import Trainer
from prioritized_memory import Memory
import utils

memory_capacity = 1500
reward = 5.0
discount_factor = 0.5
mini_batch_size = 10
copy_target_net = 100
train_iter = 1000
save_freq = 10

suction_1_sampled = np.zeros(memory_capacity)
suction_2_sampled = np.zeros(memory_capacity)
gripper_sampled = np.zeros(memory_capacity)
#model_str  = "../training/logger_010/models/10_121.pth"
model_str = "evaluate_model/600.pth"

suction_1_memory = Memory(memory_capacity)
suction_2_memory = Memory(memory_capacity)
gripper_memory   = Memory(memory_capacity)
suction_1_memory.load_memory("suction_1_augment.pkl")
suction_2_memory.load_memory("suction_2_augment.pkl")
gripper_memory.load_memory("gripper_augment.pkl")
suction_1_memory.tree.reset_priority()
suction_2_memory.tree.reset_priority()
gripper_memory.tree.reset_priority()

compare_color = "../training/logger_009/images/color_000005.jpg"
compare_depth = "../training/logger_009/depth_data/depth_data_000005.npy"

def create_path():
	cwd = os.getcwd() + "/models"
	if not os.path.exists(cwd):
		os.makedirs(cwd)
	primitives = ['suck_1/', 'suck_2/', 'grasp_0/', 'grasp_1/', 'grasp_2/', 'grasp_3/']
	feat_path = [cwd+'/feat/']*6
	mixed_path = [cwd+'/mixed/']*6
	for i in range(len(primitives)):
		feat_path[i] += primitives[i]
		mixed_path[i] += primitives[i]
		if not os.path.exists(feat_path[i]):
			os.makedirs(feat_path[i])
		if not os.path.exists(mixed_path[i]):
			os.makedirs(mixed_path[i])
	return feat_path, mixed_path
	
def load_and_predict(logger_idx, iter_idx):
	color = cv2.imread("../training/logger_{:03}/images/color_{:06}.jpg".format(logger_idx, iter_idx))
	depth = np.load("../training/logger_{:03}/depth_data/depth_data_{:06}.npy".format(logger_idx, iter_idx))
	suck_1_prediction, suck_2_prediction, grasp_prediction = trainer.forward(color, depth, is_volatile=True)
	heatmaps, mixed_imgs = utils.save_heatmap_and_mixed(suck_1_prediction, suck_2_prediction, grasp_prediction, feat_path, mixed_path, color, 8888+logger_idx*1000+iter_idx)
	print "Code: {}".format(8888+logger_idx*1000+iter_idx)
	best = np.where(np.max(suck_1_prediction)==suck_1_prediction)
	print "Suck 1 max: {} @({}, {})".format(np.max(suck_1_prediction), best[1][0], best[2][0])
	best = np.where(np.max(suck_2_prediction)==suck_2_prediction)
	print "Suck 2 max: {} @({}, {})".format(np.max(suck_2_prediction), best[1][0], best[2][0])
	best = np.where(np.max(grasp_prediction[0])==grasp_prediction[0])
	print "Grasp -90 max: {} @({}, {})".format(np.max(grasp_prediction[0]), best[0][0], best[1][0])
	best = np.where(np.max(grasp_prediction[1])==grasp_prediction[1])
	print "Grasp -45 max: {} @({}, {})".format(np.max(grasp_prediction[1]), best[0][0], best[1][0])
	best = np.where(np.max(grasp_prediction[2])==grasp_prediction[2])
	print "Grasp 0 max: {} @({}, {})".format(np.max(grasp_prediction[2]), best[0][0], best[1][0])
	best = np.where(np.max(grasp_prediction[3])==grasp_prediction[3])
	print "Grasp 45 max: {} @({}, {})".format(np.max(grasp_prediction[3]), best[0][0], best[1][0])
	
feat_path, mixed_path = create_path()
trainer = Trainer(reward, discount_factor, False, 5e-4, 1e-4)
#torch.save(trainer.behavior_net.state_dict(), "models/initial.pth") # From scratch
trainer.behavior_net.load_state_dict(torch.load(model_str))
trainer.target_net.load_state_dict(trainer.behavior_net.state_dict())

def train():
	for i in range(train_iter):
		ts = time.time()
		print "[{}%]".format(i/float(train_iter)*100)
		mini_batch = []; idxs = []; is_weight = []; old_q = []
		_mini_batch, _idxs, _is_weight = utils.sample_data(suction_1_memory, mini_batch_size); mini_batch += _mini_batch; idxs += _idxs; is_weight += list(_is_weight); tmp = [idx-memory_capacity-1 for idx in _idxs]; suction_1_sampled[tmp] += 1
		_mini_batch, _idxs, _is_weight = utils.sample_data(suction_2_memory, mini_batch_size); mini_batch += _mini_batch; idxs += _idxs; is_weight += list(_is_weight); tmp = [idx-memory_capacity-1 for idx in _idxs]; suction_2_sampled[tmp] += 1
		_mini_batch, _idxs, _is_weight = utils.sample_data(gripper_memory, mini_batch_size);   mini_batch += _mini_batch; idxs += _idxs; is_weight += list(_is_weight); tmp = [idx-memory_capacity-1 for idx in _idxs]; gripper_sampled[tmp] += 1
		for j in range(len(mini_batch)):
			color = cv2.imread(mini_batch[j].color)
			depth = np.load(mini_batch[j].depth)
			pixel_index = mini_batch[j].pixel_idx
			next_color = cv2.imread(mini_batch[j].next_color)
			next_depth = np.load(mini_batch[j].next_depth)
			action_str, rotate_idx = utils.get_action_info(pixel_index)
			old_q.append(trainer.forward(color, depth, action_str, False, rotate_idx, clear_grad=True)[0, pixel_index[1], pixel_index[2]])
			reward = mini_batch[j].reward
			td_target = trainer.get_label_value(reward, next_color, next_depth, mini_batch[j].is_empty, pixel_index[0])
			loss_ = trainer.backprop(color, depth, pixel_index, td_target, is_weight[j], mini_batch_size, j==0, j==len(mini_batch)-1)
		# Update priority
		for j in range(len(mini_batch)):
			color = cv2.imread(mini_batch[j].color)
			depth = np.load(mini_batch[j].depth)
			pixel_index = mini_batch[j].pixel_idx
			next_color = cv2.imread(mini_batch[j].next_color)
			next_depth = np.load(mini_batch[j].next_depth)
			reward = mini_batch[j].reward
			td_target = trainer.get_label_value(reward, next_color, next_depth, mini_batch[j].is_empty, pixel_index[0])
			action_str, rotate_idx = utils.get_action_info(pixel_index)
			new_value = trainer.forward(color, depth, action_str, False, rotate_idx, clear_grad=True)[0, pixel_index[1], pixel_index[2]]
			if j/mini_batch_size==0: suction_1_memory.update(idxs[j], td_target-new_value)
			elif j/mini_batch_size==1: suction_2_memory.update(idxs[j], td_target-new_value)
			else: gripper_memory.update(idxs[j], td_target-new_value)
			#print "Q value: {} -> {}| TD target: {}".format(old_q[j], new_value, td_target)
		if (i+1)%save_freq==0:
			print "Save model"
			torch.save(trainer.behavior_net.state_dict(), "models/{}.pth".format(i+1))
			color = cv2.imread(compare_color)
			depth = np.load(compare_depth)
			suck_1_prediction, suck_2_prediction, grasp_prediction = trainer.forward(color, depth, is_volatile=True)
			heatmaps, mixed_imgs = utils.save_heatmap_and_mixed(suck_1_prediction, suck_2_prediction, grasp_prediction, feat_path, mixed_path, color, i+1)
			np.savetxt("models/suction_1_sampled.csv", suction_1_sampled, delimiter=",")
			np.savetxt("models/suction_2_sampled.csv", suction_2_sampled, delimiter=",")
			np.savetxt("models/gripper_sampled.csv", gripper_sampled, delimiter=",")
		if (i+1)%copy_target_net==0:
			trainer.target_net.load_state_dict(trainer.behavior_net.state_dict())
		print "Took {} seconds".format(time.time()-ts)
		
if __name__ == "__main__":
	train()
