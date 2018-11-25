cmake_minimum_required(VERSION 2.8.3)
project(robotis_op_cpwalk)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
)

find_package(conoid REQUIRED)

set(CPGEN_LIBRARIES /usr/local/lib/libcpgen.so)
set(CPGEN_INCLUDE_DIRS /usr/local/include/cpgen)

catkin_package(
 LIBRARIES robotis_op_cpwalk
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs tf
 DEPENDS cpgen conoid
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CPGEN_INCLUDE_DIRS}
  ${CONOID_INCLUDE_DIRS}
)
link_directories(${CONOID_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/robotis_op_cpwalk.cc)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${CPGEN_LIBRARIES}
  ${CONOID_LIBRARIES}
)
