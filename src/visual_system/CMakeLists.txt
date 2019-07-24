cmake_minimum_required(VERSION 2.8.3)
project(visual_system)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  message_generation
  message_filters
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  target_pose.srv
  barcode_detect.srv
  get_xyz.srv
  get_pc.srv
  pc_is_empty.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES visual_system
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

add_executable(get_click_position src/get_click_position.cpp)
target_link_libraries(get_click_position ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(get_conveyor_plane src/get_conveyor_plane.cpp)
target_link_libraries(get_conveyor_plane ${catkin_LIBRARIES})

add_executable(get_target_pc src/get_target_pc.cpp)
add_dependencies(get_target_pc visual_system_gencpp)
target_link_libraries(get_target_pc ${catkin_LIBRARIES})

add_executable(take_picture src/take_picture.cpp)
target_link_libraries(take_picture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(save_all src/save_all.cpp)
target_link_libraries(save_all ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pc_transform src/pc_transform.cpp)
add_dependencies(pc_transform visual_system_gencpp)
target_link_libraries(pc_transform ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
