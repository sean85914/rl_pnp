cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calibration)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  tf
  std_msgs
  geometry_msgs
  arm_operation
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS arm_operation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

message(STATUS "${Eigen3_INCLUDE_DIRS}")

add_executable(static_hand_eye_calibration src/static_cam_calibration.cpp)
add_dependencies(static_hand_eye_calibration ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_hand_eye_calibration ${catkin_LIBRARIES})

add_executable(static_cam_by_on_hand src/static_cam_by_on_hand.cpp)
target_link_libraries(static_cam_by_on_hand ${catkin_LIBRARIES})

add_executable(cam_on_hand_calibration src/cam_on_hand_calibration.cpp)
add_dependencies(cam_on_hand_calibration ${catkin_EXPORTED_TARGETS})
target_link_libraries(cam_on_hand_calibration ${catkin_LIBRARIES})

add_executable(get_transform src/get_transform.cpp)
target_link_libraries(get_transform ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

add_executable(eyes_calibration src/eyes_calibration.cpp)
target_link_libraries(eyes_calibration ${catkin_LIBRARIES})

add_executable(cam_on_hand_navy src/cam_on_hand_navy.cpp)
target_link_libraries(cam_on_hand_navy ${catkin_LIBRARIES})

add_executable(set_urdf src/set_urdf.cpp)
target_link_libraries(set_urdf ${catkin_LIBRARIES} tinyxml2)
