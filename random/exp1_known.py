# Split train set into six groups, 8 items each

import numpy as np

num_item = 8
times = 6

train_set = np.loadtxt("train_set.csv", delimiter=",", dtype=str)
train_set_rm = train_set
train_set.shape = train_set.shape[0]
times_list = np.zeros(train_set.shape, dtype=int)

for i in range(times):
	subset = np.random.choice(train_set_rm, num_item, replace=False)
	for obj in subset:
		train_set_rm = np.delete(train_set_rm, np.where(train_set_rm==obj)[0][0])
	print subset
	for item in subset:
		idx = np.where(train_set==item)[0][0]
		times_list[idx] += 1

for i in range(train_set.shape[0]):
	print "%s: %d" %(train_set[i], times_list[i])
	
print "Sum: %d" %(np.sum(times_list))
