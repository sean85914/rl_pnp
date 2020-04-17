import numpy as np

SELECT_NUM = 7
group_soft = group_other = np.arange(15)

soft_train_set  = np.random.choice(group_soft, SELECT_NUM, replace=False)
other_train_set = np.random.choice(group_other, SELECT_NUM, replace=False)

print "Soft: ", np.sort(soft_train_set)
print "Other: ", np.sort(other_train_set)
