from prioritized_memory import Memory
import numpy as np
import cv2
from utils import Transition

R = 5.0

m1 = Memory(1000)
m2 = Memory(1000)
m3 = Memory(1000)
M1 = Memory(1500)
M2 = Memory(1500)
M3 = Memory(1500)

m1.load_memory("../training/logger_013/suction_1_memory.pkl")
m2.load_memory("../training/logger_013/suction_2_memory.pkl")
m3.load_memory("../training/logger_013/gripper_memory.pkl")

empty_color = []
empty_depth = []

for i in range(m1.length):
	M1.add(m1.tree.data[i])
	M2.add(m2.tree.data[i])
	M3.add(m3.tree.data[i])
	
for i in range(m1.length):
	# Invalid point is common
	if m1.tree.data[i].reward == -3*R:
		transition = m1.tree.data[i]
		pixel_index = transition.pixel_idx
		pixel_index[0] = 1
		transition_2 = Transition(transition.color, transition.depth, pixel_index, transition.reward, transition.next_color, transition.next_depth, transition.is_empty)
		M2.add(transition_2)
		pixel_index[0] = np.random.choice(range(2, 6))
		transition_3 = Transition(transition.color, transition.depth, pixel_index, transition.reward, transition.next_color, transition.next_depth, transition.is_empty)
		M3.add(transition_3)
	if m2.tree.data[i].reward == -3*R:
		transition = m2.tree.data[i]
		pixel_index = transition.pixel_idx
		pixel_index[0] = 0
		transition_1 = Transition(transition.color, transition.depth, pixel_index, transition.reward, transition.next_color, transition.next_depth, transition.is_empty)
		M1.add(transition_2)
		pixel_index[0] = np.random.choice(range(2, 6))
		transition_3 = Transition(transition.color, transition.depth, pixel_index, transition.reward, transition.next_color, transition.next_depth, transition.is_empty)
		M3.add(transition_3)
	if m3.tree.data[i].reward == -3*R:
		transition = m3.tree.data[i]
		pixel_index = transition.pixel_idx
		pixel_index[0] = 0
		transition_1 = Transition(transition.color, transition.depth, pixel_index, transition.reward, transition.next_color, transition.next_depth, transition.is_empty)
		M1.add(transition_1)
		pixel_index[0] = 1
		transition_2 = Transition(transition.color, transition.depth, pixel_index, transition.reward, transition.next_color, transition.next_depth, transition.is_empty)
		M2.add(transition_2)
	# Final state is common
	if m1.tree.data[i].is_empty == True:
		empty_color.append(m1.tree.data[i].next_color)
		empty_depth.append(m1.tree.data[i].next_depth)
	if m2.tree.data[i].is_empty == True:
		empty_color.append(m2.tree.data[i].next_color)
		empty_depth.append(m2.tree.data[i].next_depth)
	if m3.tree.data[i].is_empty == True:
		empty_color.append(m2.tree.data[i].next_color)
		empty_depth.append(m2.tree.data[i].next_depth)

cnt = [0, 0, 0]
for i in range(M1.length, 1500):
	idx = np.random.choice(len(empty_color))
	pixel_index = [0, np.random.choice(224), np.random.choice(224)]
	t1 = Transition(empty_color[idx], empty_depth[idx], pixel_index, -R, empty_color[idx], empty_depth[idx], True)
	M1.add(t1)
	cnt[0] += 1
	
for i in range(M2.length, 1500):
	idx = np.random.choice(len(empty_color))
	pixel_index = [1, np.random.choice(224), np.random.choice(224)]
	t2 = Transition(empty_color[idx], empty_depth[idx], pixel_index, -R, empty_color[idx], empty_depth[idx], True)
	M2.add(t2)
	cnt[1] += 1
	
for i in range(M3.length, 1500):
	idx = np.random.choice(len(empty_color))
	pixel_index = [np.random.choice(range(2, 6)), np.random.choice(224), np.random.choice(224)]
	t3 = Transition(empty_color[idx], empty_depth[idx], pixel_index, -R, empty_color[idx], empty_depth[idx], True)
	M3.add(t3)
	cnt[2] += 1
	
print "Collect %d final state" %len(empty_color)
print "Suction 1 memory synthesized %d data" %cnt[0]
print "Suction 2 memory synthesized %d data" %cnt[1]
print "Gripper memory synthesized %d data" %cnt[2]
print M1.length, M2.length, M3.length
M1.save_memory("", "suction_1_augment.pkl")
M2.save_memory("", "suction_2_augment.pkl")
M3.save_memory("", "gripper_augment.pkl")
