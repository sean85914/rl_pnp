cmake_minimum_required(VERSION 2.8.3)
project(grasp_suck)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules
  roscpp
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
)

add_service_files(
  FILES
  get_pose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 #INCLUDE_DIRS include
 #LIBRARIES grasp_suck
 CATKIN_DEPENDS roscpp rospy std_msgs roslib message_runtime
 #DEPENDS 
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_pose_service src/get_pose_service.cpp)
add_dependencies(get_pose_service ${catkin_EXPORTED_TARGETS} grasp_suck_gencpp)
target_link_libraries(get_pose_service ${catkin_LIBRARIES})
