import numpy as np
import random
import pickle
from SumTree import SumTree

# Revised from: https://github.com/rlcode/per and https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow/blob/master/contents/5.2_Prioritized_Replay_DQN/RL_brain.py

class Memory:
	epsilon = 0.01
	alpha = 0.6
	beta = 0.4
	beta_increment = 0.001
	abs_err_upper = 1.
	
	def __init__(self, capacity):
		self.tree = SumTree(capacity)
	
	def _get_priority(self, error):
		return (np.abs(error) + self.epsilon) ** self.alpha
		
	def add(self, transition):
		max_p = np.max(self.tree.tree[-self.tree.capacity:])
		if max_p == 0:
			max_p = self.abs_err_upper
		self.tree.add(max_p, transition)
		
	def sample(self, n):
		batch = []
		idxs = []
		segment = self.tree.total / n
		priorities = []
		
		self.beta = np.min([1., self.beta + self.beta_increment])
		
		for i in range(n):
			a, b = segment * i, segment * (i + 1)
			s = random.uniform(a, b)
			idx, p, data = self.tree.get(s)
			priorities.append(p)
			batch.append(data)
			idxs.append(idx)
		sampling_probabilities = priorities / self.tree.total # (1)
		is_weight = np.power(self.tree.n_entries * sampling_probabilities, -self.beta)
		is_weight /= is_weight.max() # 3.4
		
		return batch, idxs, is_weight
		
	def update(self, idx, error):
		p = self._get_priority(error)
		#print("Data index {} priority set from {} to {}".format(idx - self.tree.capacity + 1, self.tree.tree[idx], p))
		self.tree.update(idx, p)
		
	def save_memory(self, root_path, name):
		f = open(root_path+name, 'wb')
		pickle.dump(self.tree, f)
		f.close()
	
	def load_memory(self, file_path):
		with open(file_path, 'rb') as file:
			self.tree = pickle.load(file)
			print("Loaded {} data".format(self.tree.length))
	
	@property
	def length(self):
		return self.tree.n_entries
