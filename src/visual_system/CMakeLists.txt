cmake_minimum_required(VERSION 2.8.3)
project(visual_system)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  pcl_conversions
  pcl_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs cv_bridge tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(get_click_position src/get_click_position.cpp)
target_link_libraries(get_click_position ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(get_conveyor_plane src/get_conveyor_plane.cpp)
target_link_libraries(get_conveyor_plane ${catkin_LIBRARIES})
