grasp_suck_system.launch
| Type | Path | Description |
| ：---：    | ：---： | ：---： |
| launch | arm_operation/tcp_publisher.launch                    | Suction and 2-finger gripper transformation information |
| node   | vacuum_conveyor_control/arduino_control               | Turn on vacuum control services |
| node   | robotiq_2f_gripper_control/Robotiq2FGripperRtuNode.py | Turn on robotiq 2-finger gripper |
| node   | grasp_suck/robotiq_gripper_control                    | Trun on robotiq gripper control services |
| node   | visual_system/pixel_to_xyz                            | Convert pixel to 3D coordinate  Get cropped color and depth images |
| node   | grasp_suck/get_reward                                 | Using consecutive depth images to judge if action succeed |
| launch | grasp_suck/helper_services.launch                     | High-level services, including homing, picking and placing |

