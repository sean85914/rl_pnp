cmake_minimum_required(VERSION 2.8.3)
project(grasp_suck)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules
  roscpp
  rosbag
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  tf
  actionlib
  actionlib_msgs
  message_generation
  cv_bridge
  arm_operation
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  get_pose.srv
  get_result.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
 #INCLUDE_DIRS include
 #LIBRARIES grasp_suck
 CATKIN_DEPENDS roscpp rospy std_msgs roslib message_runtime
 #DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(helper_services src/helper_services.cpp)
add_dependencies(helper_services ${catkin_EXPORTED_TARGETS} grasp_suck_gencpp arm_operation_gencpp)
target_link_libraries(helper_services ${catkin_LIBRARIES})

add_executable(robotiq_gripper_control src/robotiq_gripper_control.cpp)
add_dependencies(robotiq_gripper_control ${catkin_EXPORTED_TARGETS} robotiq_2f_gripper_control_gencpp)
target_link_libraries(robotiq_gripper_control ${catkin_LIBRARIES})
