import os
import numpy as np
import cv2
import time
import h5py
import argparse

def conversion(path_to_exp, hdf5_dir, dir):
	total_iter = len(os.listdir(path_to_exp+dir+"/images/"))/2
	if not hdf5_dir:
		filename = "{}.hdf5".format(dir)
	else:
		filename = "{}/{}.hdf5".format(hdf5_dir, dir)
	f = h5py.File(filename, "w")
	primitive = np.loadtxt(path_to_exp+dir+"/action_primitive.csv", delimiter=",")
	result = np.loadtxt(path_to_exp+dir+"/action_result.csv", delimiter=",")
	target = np.loadtxt(path_to_exp+dir+"/action_target.csv", delimiter=",")
	empty = np.loadtxt(path_to_exp+dir+"/empty_result.csv", delimiter=",")
	reward_unit = 5.0
	# attribute
	total_success = 0
	total_fail = 0
	total_invalid = 0
	gripper_used = 0
	small_suction_used = 0
	medium_suction_used = 0
	gripper_angle = [0, 0, 0, 0]
	total_empty = 0
	gripper_success = 0
	small_suction_success = 0
	medium_suction_success = 0
	ts = time.time()
	for i in range(0, total_iter):
		color = cv2.imread(path_to_exp+dir+"/images/color_{:06}.jpg".format(i))
		depth = np.load(path_to_exp+dir+"/depth_data/depth_data_{:06}.npy".format(i))
		next_color = cv2.imread(path_to_exp+dir+"/images/next_color_{:06}.jpg".format(i))
		next_depth = np.load(path_to_exp+dir+"/depth_data/depth_data_{:06}.npy".format(i))
		action = target[i]
		invalid = (primitive[i]==-1)
		fail = (result[i]==0)
		# hdf5
		grp = f.create_group("iter_{:04}".format(i))
		state = grp.create_group("state")
		next_state = grp.create_group("next_state")
		_ = state.create_dataset("color", color.shape, dtype=color.dtype)
		_ = state.create_dataset("depth", depth.shape, dtype=np.float32)
		_ = next_state.create_dataset("color", color.shape, dtype=color.dtype)
		_ = next_state.create_dataset("depth", depth.shape, dtype=np.float32)
		_ = next_state.create_dataset("empty", (1, ), dtype=bool)
		state["color"][:] = color
		state["depth"][:] = depth
		next_state["color"][:] = next_color
		next_state["depth"][:] = next_depth
		next_state["empty"][:] = int(empty[i])
		if empty[i]:
			total_empty+=1
		if action[0] == 0:
			small_suction_used+=1
			if not fail:
				small_suction_success+=1
		elif action[0] == 1:
			medium_suction_used+=1
			if not fail:
				medium_suction_success+=1
		else:
			gripper_used+=1
			gripper_angle[int(action[0])-2]+=1
			if not fail:
				gripper_success+=1
		_ = grp.create_dataset("action", action.shape, dtype=np.uint8)
		grp["action"][:] = action
		_ = grp.create_dataset("reward", (1, ), dtype=np.int8)
		if invalid:
			grp["reward"][:] = -3*reward_unit
			total_invalid+=1
		elif fail:
			grp["reward"][:] = -reward_unit
			total_fail += 1
		else:
			grp["reward"][:] = reward_unit
			total_success += 1	
	# attributes
	f.attrs.create("total_iter", total_iter, dtype=np.uint16)
	f.attrs.create("total_success", total_success, dtype=np.uint16)
	f.attrs.create("total_fail", total_fail, dtype=np.uint16)
	f.attrs.create("total_invalid", total_invalid, dtype=np.uint16)
	f.attrs.create("total_empty", total_empty, dtype=np.uint16)
	f.attrs.create("tool_gripper_used", gripper_used, dtype=np.uint16)
	f.attrs.create("tool_small_suction_used", small_suction_used, dtype=np.uint16)
	f.attrs.create("tool_medium_suction_used", medium_suction_used, dtype=np.uint16)
	f.attrs.create("tool_gripper_success", gripper_success, dtype=np.uint16)
	f.attrs.create("tool_small_suction_success", small_suction_success, dtype=np.uint16)
	f.attrs.create("tool_medium_suction_success", medium_suction_success, dtype=np.uint16)
	f.attrs.create("tool_gripper_angle_used", gripper_angle, dtype=np.uint16)
	f.attrs.create("reward_unit", reward_unit, dtype=np.float16)
	f.close()
	print "{}: conversion takes {} seconds | {} experiences added".format(filename, time.time()-ts, total_iter)
	
def main():
	parser = argparse.ArgumentParser(prog="train_exp_to_hdf5", description="Convert training data to hdf5 format")
	parser.add_argument("path_to_exp", type=str, help="path to experience")
	parser.add_argument("--hdf5_root", type=str, default="hdf5", help="path to store the converted hdf5 files, default is hdf5")
	args = parser.parse_args()
	if not args.path_to_exp[-1]=="/":
		args.path_to_exp+="/"
	if args.hdf5_root is not "":
		if not os.path.exists(args.hdf5_root):
			os.makedirs(args.hdf5_root)
	for dir in os.listdir(args.path_to_exp):
		if "logger_" in dir:
			conversion(args.path_to_exp, args.hdf5_root, dir)
	
if __name__=="__main__":
	main()
