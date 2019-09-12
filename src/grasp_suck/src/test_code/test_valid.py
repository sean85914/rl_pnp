import os
import sys
sys.path.insert(1, os.path.join(sys.path[0], '..'))
import rospy
import utils
from geometry_msgs.msg import Point
from visual_system.srv import check_valid, check_validRequest, \
                              get_pc, get_pcRequest, get_pcResponse
from visualization.srv import viz_marker, viz_markerRequest

get_pc_client = rospy.ServiceProxy("/pc_transform/get_pc", get_pc)
check_valid_client = rospy.ServiceProxy("/pc_transform/check_valid", check_valid)
viz_client = rospy.ServiceProxy("/viz_marker_node/viz_marker", viz_marker)

target = int(sys.argv[1])

pc_res = get_pc_client()
color_heightmap, depth_heightmap, points, depth_img_msg = utils.get_heightmap(pc_res.pc, "", 100)
check_validReq = check_validRequest()
check_validReq.pc = pc_res.pc
check_validReq.p.x = points[target, target, 0]
check_validReq.p.y = points[target, target, 1]
check_validReq.p.z = points[target, target, 2]

print "(%f, %f, %f)" %(points[target, target, 0], points[target, target, 1], points[target, target, 2])

res = check_valid_client(check_validReq).is_valid
print res
viz_req = viz_markerRequest()
viz_req.point = check_validReq.p
viz_req.primitive = 1
viz_req.angle = 0
viz_req.valid = res


viz_client(viz_req)
