cmake_minimum_required(VERSION 3.0.2)
project(spot_teleop)

find_package(Eigen3 REQUIRED)
find_package(Protobuf CONFIG REQUIRED)


set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)

find_package(gRPC CONFIG REQUIRED)
set(_GRPC_GRPCPP gRPC::grpc++)

find_package(Threads REQUIRED)

find_library(UTSPOT_LIBRARY utspot REQUIRED)



include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
)


add_executable(spot_teleop
spot_teleop.cpp
)


target_link_libraries(spot_teleop
  ${Protobuf_LIBRARIES}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
  ${UTSPOT_LIBRARY}
  Eigen3::Eigen
)