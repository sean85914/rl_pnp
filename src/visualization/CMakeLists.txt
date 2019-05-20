cmake_minimum_required(VERSION 2.8.3)
project(visualization)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(draw_path src/draw_path.cpp)
target_link_libraries(draw_path ${catkin_LIBRARIES})
