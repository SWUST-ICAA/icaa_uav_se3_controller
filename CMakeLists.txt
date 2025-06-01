cmake_minimum_required(VERSION 2.8.3)
project(icaa_uav_se3_controller)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  geometry_msgs
  sensor_msgs
  mavros
  message_generation
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen3_LIBRARIES ${EIGEN3_LIBRARIES})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Px4ctrlDebug.msg
  TakeoffLand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    nav_msgs
    geometry_msgs 
    sensor_msgs
    mavros
    message_runtime
  DEPENDS EIGEN3
)

include_directories(
  include/${PROJECT_NAME}
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(icaa_uav_se3_controller_node 
  src/icaa_uav_se3_controller_node.cpp
  src/Se3CtrlFSM.cpp
  src/Se3CtrlParam.cpp
  src/se3_controller.cpp
  src/input.cpp
  src/common.cpp
)

add_dependencies(icaa_uav_se3_controller_node 
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(icaa_uav_se3_controller_node
  ${catkin_LIBRARIES}
)

install(PROGRAMS scripts/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)