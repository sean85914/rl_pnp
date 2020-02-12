cmake_minimum_required(VERSION 2.8.3)
project(visualization)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  roslib
  rosbag
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
  cv_bridge
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  viz_marker.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs visualization_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(draw_path src/draw_path.cpp)
target_link_libraries(draw_path ${catkin_LIBRARIES})

add_executable(bag_to_video src/bag_to_video.cpp)
target_link_libraries(bag_to_video ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} stdc++fs)
