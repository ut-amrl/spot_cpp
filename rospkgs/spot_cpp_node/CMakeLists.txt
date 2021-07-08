cmake_minimum_required(VERSION 3.0.2)
project(spot_ros_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  spot_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
find_package(gRPC CONFIG REQUIRED)
set(_GRPC_GRPCPP gRPC::grpc++)
find_library(UTSPOT_LIBRARY utspot REQUIRED)
find_package(Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spot_test
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

add_executable(spot_ros_cpp
  src/spot_ros_cpp.cpp
  src/helpers.cpp
)

add_dependencies(spot_ros_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(spot_ros_cpp
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
  ${UTSPOT_LIBRARY}
  Eigen3::Eigen
)