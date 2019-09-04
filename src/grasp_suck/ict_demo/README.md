$ cd ~/sean/flip_object
$ source devel/setup.bash # DO THIS EVERY TIME YOU OPEN NEW TERMINAL
[Terminal 1] $ roslaunch realsense2_ros rs_rgbd.launch camera:=camera1
[Terminal 2] $ roslaunch arm_operation ur5_real.launch robot_ip:=192.168.50.11 tool_length:=0
[Terminal 3] $ roslaunch grasp_suck ict_demo.launch
[Terminal 4] $ rosrun rviz rviz -d src/visualization/config/system.rviz
[Terminal 5] $ roscd grasp_suck/src && python ict_demo.py --model ict_demo.pth

1. Press `s` after you setup the enviroment, the agent will start to empty the workspace
2. After the workspace is empty, the information will show again, press `s` to continue or `e` to exit

It is recommended that put the blocks separately to ensure high efficiency
