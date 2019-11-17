`cameras_calibration.cpp`: transform calibration between two cameras
`cam_on_hand_calibration.cpp`: hand-eye calibration with eye equipped on hand, using calibrated static camera **Deprecated**
`cam_on_hand_navy.cpp`: hand-eye calibration with eye equipped on hand, refer to *Robot sensor calibration: solving AX=XB on the Euclidean group*
`static_cam_by_on_hand.cpp`: hand-eye calibration with static-mounted camera, using calibrated on-hand camera
`static_cam_calibration.cpp`: hand-eye calibration with static-mounted camera, we should mount calibration board on the robot arm
`get_transform.cpp`: After you recorded data (tag position w.r.t. robot arm frame & target camera frame), you can use this node to get the transform
`set_urdf.cpp`: set joint between hand and eye coordinate with given input data, we assume that user put the URDF file in package:robot_description/urdf with name *system.urdf.xacro*
