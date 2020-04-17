# Generate 3 groups with 8 items, 4 from train set and 4 from test set

import numpy as np

group = 3
num_items = 8

train_set = np.loadtxt("train_set.csv", delimiter=",", dtype=str)
test_set  = np.loadtxt("test_set.csv",  delimiter=",", dtype=str)

train_candidate = np.random.choice(train_set, num_items/2*group, replace=False)
test_candidate  = np.random.choice(test_set, num_items/2*group,  replace=False)

res = []
for i in range(group):
	train_item = np.random.choice(train_candidate, num_items/2, replace=False)
	test_item  = np.random.choice(test_candidate,  num_items/2, replace=False)
	res.append(np.union1d(train_item, test_item))
	train_candidate = np.setdiff1d(train_candidate, train_item)
	test_candidate = np.setdiff1d(test_candidate, test_item)
	
for subset in res:
	print(subset)
