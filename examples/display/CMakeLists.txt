cmake_minimum_required(VERSION 3.0.2)
project(spot_frame)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)

find_package(gRPC CONFIG REQUIRED)
set(_GRPC_GRPCPP gRPC::grpc++)

find_package(Threads REQUIRED)

find_library(UTSPOT_LIBRARY utspot REQUIRED)



include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${GTK3_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
)


add_executable(spot_display 
spot_display.cpp
)


target_link_libraries(spot_display
  ${Protobuf_LIBRARIES}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
  ${UTSPOT_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${GTK3_LIBRARIES}
  ${CAIRO_LIBRARIES}
  Eigen3::Eigen
)