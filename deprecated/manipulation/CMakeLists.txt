cmake_minimum_required(VERSION 2.8.3)
project(manipulation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  arm_operation
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs visual_system
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(suck_object src/suck_object.cpp)
target_link_libraries(suck_object ${catkin_LIBRARIES})
