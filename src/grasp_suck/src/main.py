import os
import sys
import time
#from threading import Thread, currentThread
import numpy as np
import cv2
import argparse
import torch
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from trainer import Trainer
# SRV
from std_srvs.srv import Empty, SetBool, SetBoolRequest, SetBoolResponse, \
                         Trigger, TriggerRequest, TriggerResponse
from grasp_suck.srv import get_pose, get_poseRequest, get_poseResponse
from visual_system.srv import get_image, get_imageRequest, get_imageResponse, \
                              get_xyz, get_xyzRequest, get_xyzResponse
from vacuum_conveyor_control.srv import vacuum_control, vacuum_controlRequest

parser = argparse.ArgumentParser(prog="visual_suck_and_grasp", description="using color and depth images to do pick and place")
parser.add_argument("--is_testing", action="store_true", default=False)
parser.add_argument("--force_cpu",  action="store_true", default=False)
parser.add_argument("--last_model",   type=str, default="", help="If provided, continue training with provided model file")
parser.add_argument("--episode",      type=int)
parser.add_argument("--num_of_items", type=int, default=5)

args = parser.parse_args()

# Parameter
testing      = args.is_testing
use_cpu      = args.force_cpu
epsilon      = None
iteration    = 0
episode      = args.episode
suck_reward  = 2
grasp_reward = 2
discount     = 0.5
Z_THRES      = 0.645
num_of_items = args.num_of_items  ## Number of items when start
save_every   = 5

if testing:
	print "########TESTING MODE########"
	epsilon = 0.1
else:
	if episode < 5:
		epsilon = 0.75
	elif episode < 10:
		epsilon = 0.5
	elif episode < 15:
		epsilon = 0.3
	elif episode < 20:
		epsilon = 0.2

# Logger path
r = rospkg.RosPack()
package_path = r.get_path('grasp_suck')
logger_dir = '/logger_{}/'.format(episode)
csv_path   =  package_path + logger_dir
image_path =  package_path + logger_dir + 'images/'
mixed_path =  package_path + logger_dir + 'mixed_img/'
feat_path  =  package_path + logger_dir + 'feat/'
grasp_path = [feat_path + 'rotate_idx_0/',
              feat_path + 'rotate_idx_1/',
              feat_path + 'rotate_idx_2/',
              feat_path + 'rotate_idx_3/',
              feat_path + 'rotate_idx_4/']
model_path =  package_path + logger_dir + 'models/'
vis_path   =  package_path + logger_dir + 'vis/'

if not os.path.exists(image_path):
	os.makedirs(image_path)
if not os.path.exists(feat_path):
	os.makedirs(feat_path)
if not os.path.exists(model_path):
	os.makedirs(model_path)
if not os.path.exists(mixed_path):
	os.makedirs(mixed_path)
if not os.path.exists(vis_path):
	os.makedirs(vis_path)
for i in range(5):
	if not os.path.exists(grasp_path[i]):
		os.makedirs(grasp_path[i])

# cv bridge
br = CvBridge()

# trainer
trainer = Trainer(suck_reward, grasp_reward, discount, testing, use_cpu)

# Load model if provided last model
if args.last_model != "":
	print "[%f]: Loading provided model..." %(time.time())
	trainer.model.load_state_dict(torch.load(args.last_model))

# Service client
# Gripper
open_gripper      = rospy.ServiceProxy('/robotiq_finger_control_node/open_gripper', Empty)
grasp_state       = rospy.ServiceProxy('/robotiq_finger_control_node/get_grasp_state', Trigger)
pheumatic         = rospy.ServiceProxy('/arduino_control/pheumatic_control', SetBool)
vacuum            = rospy.ServiceProxy('/arduino_control/vacuum_control', vacuum_control)

# Get reward
set_prior_img     = rospy.ServiceProxy('/get_reward/set_prior', Empty)
set_posterior_img = rospy.ServiceProxy('/get_reward/set_posterior', Empty)
get_result        = rospy.ServiceProxy('/get_reward/get_result', SetBool)

# Helper
goto_target = rospy.ServiceProxy('/helper_services_node/goto_target', get_pose)
go_home     = rospy.ServiceProxy('/helper_services_node/robot_go_home', Empty)
go_place    = rospy.ServiceProxy('/helper_services_node/robot_goto_place', Empty)

# pixel_to_xyz
pixel_to_xyz = rospy.ServiceProxy('/pixel_to_xyz/pixel_to_xyz', get_xyz)
get_image    = rospy.ServiceProxy('/pixel_to_xyz/get_image', get_image)

# Result list
action_list  = []
target_list  = []
result_list  = []
loss_list    = []
explore_list = []
return_      = 0.0
# Result file name
action_file  = csv_path + "action_primitive.csv"
target_file  = csv_path + "action_target.csv"
result_file  = csv_path + "action_result.csv"
loss_file    = csv_path + "loss.csv"
explore_file = csv_path + "explore.csv"
curve_file   = csv_path + "curve.txt"

if not testing:	f = open(curve_file, "w")

# Get color and depth images from service response
def get_imgs_from_msg(response):
	color = br.imgmsg_to_cv2(response.crop_color_img)
	depth = br.imgmsg_to_cv2(response.crop_depth_img)
	return color, depth
# Draw symbols in the color image with different primitive
def draw_image(image, primitive, pixel_index):
	center = (pixel_index[2], pixel_index[1])
	result = cv2.circle(image, center, 7, (0, 0, 0), 2)
	X = 20
	Y = 7
	theta = np.radians(-90.0+45.0*pixel_index[0])
	x_unit = [ np.cos(theta), np.sin(theta)]
	y_unit = [-np.sin(theta), np.cos(theta)]
	if not primitive: # GRASP
		p1  = (center[0]+np.ceil(x_unit[0]*X), center[1]+np.ceil(x_unit[1]*X))
		p2  = (center[0]-np.ceil(x_unit[0]*X), center[1]-np.ceil(x_unit[1]*X))
		p11 = (p1[0]-np.ceil(y_unit[0]*Y), p1[1]-np.ceil(y_unit[1]*Y))
		p12 = (p1[0]+np.ceil(y_unit[0]*Y), p1[1]+np.ceil(y_unit[1]*Y))
		p21 = (p2[0]-np.ceil(y_unit[0]*Y), p2[1]-np.ceil(y_unit[1]*Y))
		p22 = (p2[0]+np.ceil(y_unit[0]*Y), p2[1]+np.ceil(y_unit[1]*Y))
		p11 = (int(p11[0]), int(p11[1]))
		p12 = (int(p12[0]), int(p12[1]))
		p21 = (int(p21[0]), int(p21[1]))
		p22 = (int(p22[0]), int(p22[1]))
		result = cv2.line(result, p11, p12, (0, 0, 0), 2) # Black
		result = cv2.line(result, p21, p22, (0, 0, 0), 2) # Black
	return result

print "Initializing..."
go_home()
open_gripper()
pheumatic(SetBoolRequest(False))
vacuum(vacuum_controlRequest(0))

total_time = 0.0

try:
	while num_of_items != 0:
		print "\033[0;31;46mIteration: {}\033[0m".format(iteration)
		epsilon = max(epsilon * np.power(0.9998, iteration), 0.1)
		iter_ts = time.time()	
		# Get color and depth images
		images = get_image()
		# Convert to cv2
		color, depth = get_imgs_from_msg(images)
		# Save images
		color_name = image_path + "color_{:06}.jpg".format(iteration)
		depth_name = image_path + "depth_{:06}.png".format(iteration)
		cv2.imwrite(color_name, color)
		cv2.imwrite(depth_name, depth)
		ts = time.time()
		print "[%f]: Forward pass..." %(time.time()), 
		sys.stdout.write('')
		suck_predictions, grasp_predictions, state_feat = \
                          trainer.forward(color, depth, is_volatile=True)
		print "  {} seconds".format(time.time() - ts)
		# Standardization
		mean = np.mean(suck_predictions)
		std  = np.std(suck_predictions)
		suck_predictions = (suck_predictions-mean)/std
		for i in range(len(grasp_predictions)):
			mean = np.mean(grasp_predictions[i])
			std  = np.std(grasp_predictions[i])
			grasp_predictions[i] = (grasp_predictions[i]-mean)/std
		# Convert feature to heatmap and save
		tmp = np.copy(suck_predictions)
		# View the value as probability
		tmp[tmp<0] = 0
		tmp[tmp>1] = 1
		tmp = (tmp*255).astype(np.uint8)
		tmp.shape = (tmp.shape[1], tmp.shape[2])
		suck_heatmap = cv2.applyColorMap(tmp, cv2.COLORMAP_JET)
		suck_name = feat_path + "suck_{:06}.jpg".format(iteration)
		cv2.imwrite(suck_name, suck_heatmap)
		suck_mixed = cv2.addWeighted(color, 1.0, suck_heatmap, 0.4, 0)
		suck_name = mixed_path + "suck_{:06}.jpg".format(iteration)
		grasp_mixed = []
		cv2.imwrite(suck_name, suck_mixed)
		for rotate_idx in range(len(grasp_predictions)):
			tmp = np.copy(grasp_predictions[rotate_idx])
			tmp[grasp_predictions[rotate_idx]<0] = 0
			tmp[grasp_predictions[rotate_idx]>1] = 1
			tmp = (tmp*255).astype(np.uint8)
			tmp.resize((tmp.shape[0], tmp.shape[1], 1))
			grasp_heatmap = cv2.applyColorMap(tmp, cv2.COLORMAP_JET)
			grasp_name = grasp_path[rotate_idx] + "grasp_{:06}.jpg".format(iteration, rotate_idx)
			cv2.imwrite(grasp_name, grasp_heatmap)
			grasp_mixed_idx = cv2.addWeighted(color, 1.0, grasp_heatmap, 0.4, 0)
			grasp_mixed.append(grasp_mixed_idx)
			grasp_name = mixed_path + "grasp_{:06}_idx_{}.jpg".format(iteration, rotate_idx)
			cv2.imwrite(grasp_name, grasp_mixed_idx)
		action = 0 # GRASP
		action_str = 'grasp'
		angle = 0
		pixel_index = [] # rotate_idx, x, y
		print "[{:.6f}]: suck max: \033[0;34m{}\033[0m| grasp max: \033[0;35m{}\033[0m".format(time.time(), np.max(suck_predictions), \
                                                   np.max(grasp_predictions))
		explore = np.random.uniform() < epsilon
		explore_list.append(explore)
		if explore: print "Using explore..."
		if np.max(suck_predictions) > np.max(grasp_predictions):
			if not explore:
				action = 1 # SUCK
				action_str = 'suck'
				tmp = np.where(suck_predictions == np.max(suck_predictions))
				pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
		
			else: # GRASP
				tmp = np.where(grasp_predictions == np.max(grasp_predictions))
				pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
				angle = np.radians(-90+45*pixel_index[0])
		else:
			if not explore:
				tmp = np.where(grasp_predictions == np.max(grasp_predictions))
				pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
				angle = np.radians(-90+45*pixel_index[0])
			else:
				action = 1 # SUCK
				action_str = 'suck'
				tmp = np.where(suck_predictions == np.max(suck_predictions))
				pixel_index = [tmp[0][0], tmp[1][0], tmp[2][0]]
		'''prediction_file = csv_path + "prediction_{:06}.csv".format(iteration)
		if action: #SUCK
			np.savetxt(prediction_file, suck_predictions[0], delimiter=",")
		else: #GRASP
			np.savetxt(prediction_file, grasp_predictions[pixel_index[0]], delimiter=",")'''
		del suck_predictions, grasp_predictions, state_feat
		print "[%f]: Take action: \033[0;31m %s\033[0m at \
\033[0;32m(%d, %d)\033[0m with theta \033[0;33m%f \033[0m" %(time.time(), action_str, pixel_index[1], \
                                                             pixel_index[2], angle)
		set_prior_img()
		getXYZ = get_xyzRequest()
		getXYZ.point[0] = pixel_index[2] # X
		getXYZ.point[1] = pixel_index[1] # Y
		getXYZ.color = images.crop_color_img
		getXYZ.depth = images.crop_depth_img
		getXYZResult = pixel_to_xyz(getXYZ)
		
		is_valid = True
		# Draw color + heatmap + motion
		visual_img = None
		if action: # SUCK
			visual_img = draw_image(suck_mixed, action, pixel_index)
		else:
			visual_img = draw_image(grasp_mixed[pixel_index[0]], action, pixel_index)
		vis_name = vis_path + "vis_{:06}.jpg".format(iteration)
		cv2.imwrite(vis_name, visual_img)
		# Invalid conditions:
		# 1. NaN point
		# 2. Point with z too far, which presents the plane of convayor
		# TODO 3. Gripper collision with object
		if np.isnan(getXYZResult.result.x):
			print "\033[0;31;44mInvalid pointcloud. Ignore...\033[0m"
			is_valid = False
			
		if getXYZResult.result.z >= Z_THRES:
			print "\033[0;31;44mTarget on converyor! Ignore...\033[0m"
			is_valid = False
		
		if is_valid:
			action_list.append(action)
			gotoTarget = get_poseRequest()
			gotoTarget.point_in_cam = getXYZResult.result
			gotoTarget.yaw = angle
			gotoTarget.primmitive = action
		
			goto_target(gotoTarget)
			time.sleep(0.5)

			go_home() # TODO: service maybe block
		else:
			action_list.append(-1)
		target_list.append(pixel_index)
		
		if is_valid:
			if not action: # GRASP
				action_success = grasp_state().success #and get_result(SetBoolRequest()).success 
			else: # SUCK
				action_success = get_result(SetBoolRequest()).success
		else:
			action_success = False

		result_list.append(action_success)
		
		if action_success:
			go_place()
			num_of_items -= 1
			print "%d items remain..." % num_of_items
		else:
			if not action: # GRASP
				open_gripper()
			else:
				pheumatic(SetBoolRequest(False))
				vacuum(vacuum_controlRequest(0))
		iteration += 1
		if not testing:
			# Get images after action
			images = get_image()
			next_color, next_depth = get_imgs_from_msg(images)
			# Save images
			color_name = image_path + "next_color_{:06}.jpg".format(iteration)
			depth_name = image_path + "next_depth_{:06}.png".format(iteration)
			cv2.imwrite(color_name, next_color)
			cv2.imwrite(depth_name, next_depth)
			ts = time.time()
			if is_valid:
				label_value, prev_reward_value = trainer.get_label_value(action_str, action_success, \
		    	                                                     next_color, next_depth, num_of_items)
			else: # invalid
				label_value, prev_reward_value = trainer.get_label_value("invalid", False, next_color, next_depth, num_of_items)
			print "Get label: {} seconds".format(time.time()-ts)
			return_ += prev_reward_value * np.power(discount, iteration)
			ts = time.time()
			loss_value = trainer.backprop(color, depth, action_str, pixel_index, label_value)
			loss_list.append(loss_value)
			print "Backpropagation {} seconds".format(time.time() - ts)
			if iteration % save_every == 0:
				model_name = model_path + "iter_{:06}.pth".format(iteration)
				torch.save(trainer.model.state_dict(), model_name)
				print "Model: %s saved" %model_name
		iter_time = time.time() - iter_ts
		total_time += iter_time
		print "[Total time] \033[0;31m{}\033[0m s| [Iter time] \033[0;32m{}\033[0m s".format \
	                                                                    (total_time, iter_time)
		time.sleep(0.5) # Sleep 0.5 s for next iteration	
		
	# Num of item = 0
	if not testing:
		model_name = model_path + "final.pth"
		torch.save(trainer.model.state_dict(), model_name)
		print "Model: %s saved" % model_name
		np.savetxt(loss_file   , loss_list   , delimiter=",")
		num_of_invalid = np.where(np.array(action_list) == -1)[0].size
		num_of_grasp   = np.where(np.array(action_list) == 0)[0].size
		num_of_suck    = np.where(np.array(action_list) == 1)[0].size
		ratio          = float(num_of_grasp)/num_of_suck
		result_str = str("Iteration: %d\nReturn: %f\nMean loss: %f\nNum of invalid action: %d\nNum of valid action: %d\nRatio: %f" 
             	    % (iteration, return_, np.mean(loss_list), num_of_invalid, iteration-num_of_invalid, ratio))
		f.write(result_str)
		f.close()
	np.savetxt(action_file , action_list , delimiter=",")
	np.savetxt(target_file , target_list , delimiter=",")
	np.savetxt(result_file , result_list , delimiter=",")
	np.savetxt(explore_file, explore_list, delimiter=",")
	
	

except KeyboardInterrupt:
	print "Force terminate"
	model_name = model_path + "force_terminate_{}.pth".format(iteration)
	print "Model: %s saved" % model_name
	torch.save(trainer.model.state_dict(), model_name)
	np.savetxt(action_file, action_list, delimiter=",")
	np.savetxt(target_file, target_list, delimiter=",")
	np.savetxt(result_file, result_list, delimiter=",")
	if not testing:
		num_of_invalid = np.where(np.array(action_list) == -1)[0].size
		result_str = str("Iteration: %d\nReturn: %f\nMean loss: %f\nNum of invalid action: %d\nNum of valid action: %d" 
        	         % (iteration, return_, np.mean(loss_list), num_of_invalid, iteration-num_of_invalid))
		f.write(result_str)
		f.close()
		np.savetxt(loss_file  , loss_list  , delimiter=",")
