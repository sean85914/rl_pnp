cmake_minimum_required(VERSION 2.8.3)
project(abb_node)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
 FILES
 joints.msg
)

add_service_files(
 FILES
 robot_GetCartesian.srv
 robot_GetJoints.srv
 robot_IsMoving.srv
 robot_Ping.srv
 robot_SetCartesian.srv
 robot_SetComm.srv
 robot_SetDIO.srv
 robot_SetJoints.srv
 robot_SetSpeed.srv
 robot_SetTool.srv
 robot_SetTrackDist.srv
 robot_SetVacuum.srv
 robot_SetWorkObject.srv
 robot_SetZone.srv
 robot_SpecialCommand.srv
 robot_Stop.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs roslib message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} packages/abb_communications packages/matVec)

add_library(matVec packages/matVec/HomogTransf.cpp
                   packages/matVec/Mat.cpp
                   packages/matVec/Polynom.cpp
                   packages/matVec/Quaternion.cpp
                   packages/matVec/RotMat.cpp
                   packages/matVec/Vec.cpp)
                   
add_library(abb_comm packages/abb_communications/abb_comm.cpp)

add_executable(abb_node src/abb_node.cpp)
add_dependencies(abb_node abb_node_gencpp)
target_link_libraries(abb_node ${catkin_LIBRARIES} abb_comm matVec)                   

add_executable(abb_logger src/abb_logger.cpp)
add_dependencies(abb_logger abb_node_gencpp)
target_link_libraries(abb_logger ${catkin_LIBRARIES}) 
