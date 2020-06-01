# Abstract
<p align="center">The increasing logistics work of E-commerce and cost for human labor come with a <b>great need for semi- to fully autonomous robotic picking task.</b> The Amazon Picking Challenge and the Amazon Robotics Challenge facilitate the developments of pick-and-place problems, commonly by the use of suction cup and parallel-jaw gripper. However, due to the diversity of object geometry, material, and weight, it is still challenging to successfully grasp all objects using single end effector. Existing affordance predictions were carried out by either human labelling or only learned for the geometry aspect of small unrealistic objects. This work mainly focus on <b>learning novel object bin-picking with proper tool selection for realistic objects with varieties of size, weight, geometry, and deformable objects.</b> A set of changeable pneumatic tool, including different types of suction cups and vacuum-based parallel-jaw gripper, is used for handling wide varieties of objects. The run-time tool selection is carried out by <b>model-free deep Q-learning that learns the mapping from a heightmap captured by a RGB-D camera to the action-value of each tool at each pixel.</b> The training data are collected with real robot system in a self-supervised fashion. We found the learnt policy trained from <b> 3,000 experiences collected in 18 hours</b> outperformed previous work by human labeled affordance. The grasping performance of novel objects were improved by a self-supervised finetune. All data are publicly available in RoboNet.
<p align="center"><img src="https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/img/motivation.png" width=830 height=250/>
<p align="center"><img src="https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/img/teaser.png" width=1185 height=435/>

# Table of Contents
1. [Hardware](#Hardware)
2. [How to Start](#Start)
3. [Launch](#Launch)
4. [System Pipeline](#System)
5. [Services List](#Services)
6. [Network](#Network)

## Hardware <a name="Hardware"></a>
| Hardware | Image |
| :---: | :---: |
| ABB IRB1660ID| <img src="https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/img/abb.jpg" width=222 height=296/>|
| RealSense D435/415 <br> One required, one optinal | <img src="https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/img/d435.jpeg" width=237 height=105/> |
| Vacuum Pumb <br> Should can be controlled by an Arduino and a valve| <img src="https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/img/vacuum_pump.png" width=244 height=186/> |
| Tools from [XYZ Robotics](http://en.xyzrobotics.ai/)| <img src="https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/img/tools.png" width=299 height=159/> |

## How to Start <a name="Start"></a>
```
$ cd && git clone https://github.com/sean85914/rl_pnp.git && cd rl_pnp && catkin_make
$ source devel/setup.bash # do this every time you open a new terminal
[Terminal 1] $ roscore  
[Terminal 2] $ roslaunch grasp_suck actuator.launch  
[Terminal 3] $ roslaunch grasp_suck sensors.launch serial_no:=[Camera Serial No.] side_record_no:=[Another Camera Serial No.]  
[Terminal 4] $ roslaunch visualization viz.launch rviz:=true  
[Terminal 5] $ roscd grasp_suck/src && python main.py [integer number]  
```

## Launch <a name="Launch"></a>
| Name | Nodes | Description |
| :---: | :---: | :---: |
| grasp_suck/actuator.launch | abb_node/abb_node <br> arm_operation/change_tool_service arm_operation/agent_server_node | Turn on robot arm logger and server <br> Services for changing tools <br> Services for agent behavior |
| grasp_suck/sensors.launch | -- <br> visual_system/combine_pc_node  grasp_suck/autonomous_recording_node visual_system/two_cams_tf </br>| Turn on realsense camera(s) <br> Get pointcloud in workspace <br> Autonomous recording for ROS bag <br> Broadcast extrinsic transformation between the arm and the camera |
| visualization/viz.launch| rviz/rviz <br> visualization/show_lines <br> visualization/viz_boundary <br> visualization/viz_marker_node | Launch RViz <br> Show gripper detect range <br> Show worksapce range <br> Service to visualize the selected action |

## System Pipeline <a name="System"></a>
<p align="center"> <img src="https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/img/system.png" width=730 height=413/> </p>
The system consists three part:

1. <b>Environment Perception</b>: The agent retrieves pointcloud and convert to arm coordinate with calibrated extrinsic matrix. We then reproject the pointcloud in the pre-defined workspace to a top-down viewpoint heightmap, which includes both color and height information.
2. <b>Network Prediction</b>: The heightmap then passes through a preprocessing procedure, feeds into a fully convolution neural network and get affordance prediction, which maps the heightmap to the Q-value of each tool.
3. <b>Action Execution</b>: The agent selects the action among the predictions according to its policy (either <sub>&epsilon;-greedy</sub> or greedy policy) and executes the action and the envioronment will tell the agent whether it successfully grasp the object.



## Services List <a name="Services"></a>

| Service Name| Service Type | Description |
| :---: | :---: | :---: |
|<tr><td colspan=3><p align="center">**Arm Related**</p></td></tr>|
| /abb/GetCartesian | [abb_node/robot_GetCartesian](https://github.com/sean85914/rl_pnp/blob/master/src/open_abb/abb_node/srv/robot_GetCartesian.srv) | Get robot end-effector pose w.r.t. the base |
| /abb/GetJoints | [abb_node/robot_GetJoints](https://github.com/sean85914/rl_pnp/blob/master/src/open_abb/abb_node/srv/robot_GetJoints.srv) | Get robot six joints angle |
| /abb/SetCartesian | [abb_node/robot_SetCartesian](https://github.com/sean85914/rl_pnp/blob/master/src/open_abb/abb_node/srv/robot_SetCartesian.srv) | Set robot end-effector to assigned pose |
| /abb/SetJoints | [abb_node/robot_SetJoints](https://github.com/sean85914/rl_pnp/blob/master/src/open_abb/abb_node/srv/robot_SetJoints.srv) | Set robot six joints to assigned angle |
| /abb/SetSpeed | [abb_node/robot_SetSpeed](https://github.com/sean85914/rl_pnp/blob/master/src/open_abb/abb_node/srv/robot_SetSpeed.srv) | Set robot end-effector tranlation and rotation speed |
| /abb/SetZone | [abb_node/robot_SetCartesian](https://github.com/sean85914/rl_pnp/blob/master/src/open_abb/abb_node/srv/robot_SetZone.srv)| Set accuracy of the end-effector when moving to the designated pose |
|<tr><td colspan=3><p align="center">**Vacuum Related**</p></td></tr>|
| /vacuum_pump_control_node/vacuum_control | [std_srvs/SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html) | Turn on the pneumatic tool |
| /vacuum_pump_control_node/check_suck_success | [std_srvs/SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html) | Check if current suction cup is grasping object |
|<tr><td colspan=3><p align="center">**Visaul Related**</p></td></tr>|
| /combine_pc_node/get_pc | [visual_system/get_pc](https://github.com/sean85914/rl_pnp/blob/master/src/visual_system/srv/get_pc.srv)| Get pointcloud in the workspace |
| /combine_pc_node/grasp_state | [visual_system/check_grasp_success](https://github.com/sean85914/rl_pnp/blob/master/src/visual_system/srv/check_grasp_success.srv)| Check if current parallel-jaw gripper is grasping object |
| /combine_pc_node/empty_state | [visual_system/pc_is_empty](https://github.com/sean85914/rl_pnp/blob/master/src/visual_system/srv/pc_is_empty.srv)| Check if current workspace is empty, <br> i.e., contains no other object |
|<tr><td colspan=3><p align="center">**Agent Related**</p></td></tr>|
| /change_tool_service/change_tool_service| [arm_operation/change_tool](https://github.com/sean85914/rl_pnp/blob/master/src/arm_operation/srv/change_tool.srv)| Change mounted tool to designated one |
| /change_tool_service/calibrate_gripper | [std_srvs/SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html) | Calibrate parallel-jaw gripper to zero-degree |
| /agent_server_node/go_home | [std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html) | Make robot arm return to predefined joint angles |
| /agent_server_node/go_home_fix_orientation | [std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html) | Make robot arm return to predefined position with current orientation |
| /agent_server_node/go_place | [std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html) | Transport grasped object to another bin then return |
| /agent_server_node/agent_take_action | [arm_operation/agent_abb_action](https://github.com/sean85914/rl_pnp/blob/master/src/arm_operation/srv/agent_abb_action.srv) | Execute the action with selected tool and position |
| /agent_server_node/light_vibrate | [std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html) | Shake gently from the end-effector to prevent bad grasping |
| /agent_server_node/fast_vibrate | [std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html) | Quickly vibrate up and down to avoid object gets stuck in the gripper |
| /agent_server_node/check_if_collide | [arm_operation/agent_abb_action](https://github.com/sean85914/rl_pnp/blob/master/src/arm_operation/srv/agent_abb_action.srv) | Check if the gripper will collide with the bin and camera |
| /agent_server_node/publish_data | [arm_operation/publish_info](https://github.com/sean85914/rl_pnp/blob/master/src/arm_operation/srv/publish_info.srv) | Log execution data for future usage |
|<tr><td colspan=3><p align="center">**Visualization Related**</p></td></tr>|
| /viz_marker_node/viz_marker | [visualization/viz_marker](https://github.com/sean85914/rl_pnp/blob/master/src/visualization/srv/viz_marker.srv)| Visualize the selected action in RViz |
| /autonomous_recording_node/start_recording | [grasp_suck/recorder](https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/srv/recorder.srv) | Start recording with given index |
| /autonomous_recording_node/stop_recording| [std_srvs/Empty](http://docs.ros.org/melodic/api/std_srvs/html/srv/Empty.html) | Stop recording |



## Network <a name="Network"></a>
* [Model Architecture](https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/src/model_v2.py)
* [Training](https://github.com/sean85914/rl_pnp/blob/master/src/grasp_suck/src/trainer.py)

