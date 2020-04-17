# Split test set into three groups, 8 items each

import numpy as np

num_items = 8

test_set = np.loadtxt("test_set.csv", delimiter=",", dtype=str)
test_set.shape = test_set.shape[0]
np.random.shuffle(test_set)

res = np.split(test_set, test_set.shape[0]/num_items)
for subset in res:
	print(subset)
