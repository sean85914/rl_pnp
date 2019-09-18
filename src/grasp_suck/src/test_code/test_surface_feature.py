import sys
import os
import numpy as np
import rospy
import time
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import utils
from geometry_msgs.msg import Point
from grasp_suck.srv import get_result, get_resultRequest, get_resultResponse
from visual_system.srv import get_pc, get_pcRequest, get_pcResponse, \
                              pc_is_empty, pc_is_emptyRequest, pc_is_emptyResponse, \
                              get_surface_feature, get_surface_featureRequest, get_surface_featureResponse

get_pc_client = rospy.ServiceProxy("/pc_transform/get_pc", get_pc)
get_feature_client = rospy.ServiceProxy("/pc_transform/get_surface_feature", get_surface_feature)

pc_res = get_pc_client()
color_heightmap, depth_heightmap, points, depth_img_msg = utils.get_heightmap(pc_res.pc, "", 100)
feature_req = get_surface_featureRequest()
feature_req.pc = pc_res.pc
p = Point()
x = 99#int(raw_input("Input x: "))
y = 99#int(raw_input("Input y: "))
p.x = points[x, y, 0]
p.y = points[x, y, 1]
p.z = points[x, y, 2]
feature_req.p = p
feature_req.type = 1
feature_req.radius = 0.015
print "[%f] Call service..." %(time.time())
get_feature_client(feature_req)
