cmake_minimum_required(VERSION 3.0.2)
project(bodytrack)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf CONFIG REQUIRED)


set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)

find_package(gRPC CONFIG REQUIRED)
set(_GRPC_GRPCPP gRPC::grpc++)

find_package(Threads REQUIRED)

find_package(OpenPose REQUIRED)

find_library(UTSPOT_LIBRARY utspot REQUIRED)



include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
)


add_executable(bodytrack
bodytrack.cpp
)


target_link_libraries(bodytrack
  ${Protobuf_LIBRARIES}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
  ${OpenCV_LIBRARIES}
  ${UTSPOT_LIBRARY}
  ${OpenPose_LIBS}
  Eigen3::Eigen
)