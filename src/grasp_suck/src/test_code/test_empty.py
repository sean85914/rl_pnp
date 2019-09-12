import sys
import os
import numpy as np
import rospy
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import utils
from grasp_suck.srv import get_result, get_resultRequest, get_resultResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse

'''
  1. Test if workspace is empty
'''

get_pc_client = rospy.ServiceProxy("/pc_transform/get_pc", get_pc)
is_empty_client = rospy.ServiceProxy("/pc_transform/empty_state", pc_is_empty)

test_no_move = True 
#test_no_move = False

while 1:
	req = get_pc_client()
	color, depth, points, depth_img = utils.get_heightmap(req.pc, "", 0)
	empty_res = is_empty_client(pc_is_emptyRequest(req.pc))
	print "is_empty: ", empty_res.is_empty.data

os.remove("color_000000.jpg")
os.remove("depth_000000.png")
