DIR="$HOME/sean/flip_object"
SET_VARIABLES="source devel/setup.bash"
OPEN_ARM="roslaunch arm_operation ur5_real.launch robot_ip:=192.168.50.11 tool_length:=0.0"
OPEN_CAM="roslaunch realsense2_camera rs_rgbd.launch camera:=camera"
OPEN_TF="roslaunch hand_eye_calibration fake_tcp_publisher.launch"
OPEN_TAG="roslaunch apriltags_ros cali_cam1.launch"
OPEN_RVIZ="rosrun rviz rviz -d src/visualization/config/system.rviz"
OPEN_SYS="roslaunch grasp_suck ict_demo.launch"
PROCESS="rosrun hand_eye_calibration static_hand_eye_calibration _file_name:=demo"
gnome-terminal --geometry=100x50 --working-directory=$DIR \
                                 --tab -e "bash -c \"$SET_VARIABLES && roscore; exec bash\"" \
                                 --tab -e "bash -c \"sleep 1s; $SET_VARIABLES; $OPEN_ARM; exec bash\"" \
                                 --tab -e "bash -c \"sleep 1s; $SET_VARIABLES; $OPEN_CAM; exec bash\"" \
                                 --tab -e "bash -c \"sleep 2s; $SET_VARIABLES; $OPEN_RVIZ; exec bash\"" \
                                 --tab -e "bash -c \"sleep 3s; $SET_VARIABLES; $OPEN_SYS; exec bash\"" \
                                 --tab -e "bash -c \"sleep 5s; $SET_VARIABLES; $PROCESS; exec bash\""
