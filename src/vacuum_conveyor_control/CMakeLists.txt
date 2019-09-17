cmake_minimum_required(VERSION 2.8.3)
project(vacuum_conveyor_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

find_package(serial REQUIRED)

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)

add_service_files(
  FILES
  vacuum_control.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS rt pthread
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(arduino_control src/vacuum_and_conveyor_control.cpp)
add_dependencies(arduino_control ${catkin_EXPORTED_TARGETS} vacuum_conveyor_control_gencpp)
target_link_libraries(arduino_control ${catkin_LIBRARIES} ${serial_LIBRARIES})
