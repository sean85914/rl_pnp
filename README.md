## Dependencies
* [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [librealsense2](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
* [Serial](http://wjwwood.io/serial/)

## How to Start
```
$ cd && git clone https://github.com/sean85914/flip_object.git
$ cd flip_object && catkin_make
```

## [DLP 2019 Final Project](https://github.com/sean85914/flip_object/blob/master/src/grasp_suck/README.md)
![](https://github.com/sean85914/flip_object/blob/master/src/grasp_suck/img/system.png)
In this project, we try to use reinforcement learning to do pick and place with custom made gripper. The gripper is composed of Robotiq 2F-85 and a retractable suction. The method is inspired by "Learning Synergies between Pushing and Grasping with Self-supervised Deep Reinforcement Learning", from Andy Zeng.

## [My Master Thesis](Still in Progress)
Extend from the project, I try to use three different tools to do bin picking job with ABB robot arm. Specially thanks to XYZ Robotics for sponsoring the robot arm and the tooling system.
![](https://github.com/sean85914/flip_object/blob/master/src/grasp_suck/img/teaser_v1.png)
