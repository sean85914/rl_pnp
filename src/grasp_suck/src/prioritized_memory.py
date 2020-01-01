import numpy as np
import random
import pickle
from SumTree import SumTree

class Memory:
	epsilon = 0.01
	alpha = 0.6
	beta = 0.4
	beta_increment = 0.001
	
	def __init__(self, capacity):
		self.tree = SumTree(capacity)
		self.capacity = capacity
	
	def _get_priority(self, error):
		return (np.abs(error) + self.epsilon) ** self.alpha
		
	def add(self, error, transition):
		p = self._get_priority(error)
		self.tree.add(p, transition)
		
	def sample(self, n):
		batch = []
		idxs = []
		segment = self.tree.total() / n
		priorities = []
		
		self.beta = np.min([1., self.beta + self.beta_increment])
		
		for i in range(n):
			a = segment * i
			b = segment * (i + 1)
			s = random.uniform(a, b)
			idx, p, data = self.tree.get(s)
			priorities.append(p)
			batch.append(data)
			idxs.append(idx)
		sampling_probabilities = priorities / self.tree.total() # (1)
		is_weight = np.power(self.tree.n_entries * sampling_probabilities, -self.beta)
		is_weight /= is_weight.max() # 3.4
		
		return batch, idxs, is_weight
		
	def update(self, idx, error):
		p = self._get_priority(error)
		self.tree.update(idx, p)
		
	def save_memory(self, root_path):
		f = open(root_path+"sum_tree.pkl", 'wb')
		pickle.dump(self.tree, f)
		f.close()
	
	def load_memory(self, file_path):
		with open(buffer_str, 'rb') as file:
			self.tree = pickle.load(file)
