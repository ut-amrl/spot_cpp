cmake_minimum_required(VERSION 3.0.2)
project(spot_comm)
add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${protobuf_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
find_package(gRPC CONFIG REQUIRED)
set(_REFLECTION gRPC::grpc++_reflection)
set(_GRPC_GRPCPP gRPC::grpc++)
message(STATUS "Using gRPC ${gRPC_VERSION}")

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${GTK3_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

function(include_files name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/${name}.pb.cc 
    		      include/bosdyn/api/${name}.grpc.pb.cc 
		      PARENT_SCOPE)
endfunction()

function(include_files_service name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/${name}.pb.cc 
    		      include/bosdyn/api/${name}.grpc.pb.cc 
		      include/bosdyn/api/${name}_service.pb.cc 
		      include/bosdyn/api/${name}_service.grpc.pb.cc 
		      PARENT_SCOPE)
endfunction()

function(include_service name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/${name}_service.pb.cc 
		      include/bosdyn/api/${name}_service.grpc.pb.cc 
		      PARENT_SCOPE)
endfunction()


function(include_files_spot name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/spot/${name}.pb.cc 
    		      include/bosdyn/api/spot/${name}.grpc.pb.cc 
		      PARENT_SCOPE)
endfunction()

function(include_files_spot_service name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/spot/${name}.pb.cc 
    		      include/bosdyn/api/spot/${name}.grpc.pb.cc 
		      include/bosdyn/api/spot/${name}_service.pb.cc 
		      include/bosdyn/api/spot/${name}_service.grpc.pb.cc 
		      PARENT_SCOPE)
endfunction()

function(include_files_docking_service name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/docking/${name}.pb.cc 
    		      include/bosdyn/api/docking/${name}.grpc.pb.cc 
		      include/bosdyn/api/docking/${name}_service.pb.cc 
		      include/bosdyn/api/docking/${name}_service.grpc.pb.cc 
		      PARENT_SCOPE)
endfunction()

# deps
include_files(header)
include_files(geometry)
include_files(trajectory)
include_files(arm_command)
include_files(full_body_command)
include_files(mobility_command)
include_files(basic_command)
include_files(gripper_command)
include_files(synchronized_command)
include_files(parameter)
include_files(service_fault)
include_files(payload_estimation)
include_files(time_range)

# base
include_files_service(auth)
include_files_service(directory)
include_files_service(log_annotation)
include_files_service(time_sync)
include_files_service(robot_id)

# payloads
include_files_service(directory_registration)
include_files_service(payload)

# state
include_files_service(image)
include_files_service(local_grid)
include_files_service(robot_state)
include_files_service(world_object)

# control
include_files_service(estop)
include_files_service(lease)
include_files_service(power)
include_files_service(robot_command)
include_files_spot_service(spot_check)
include_files_service(license)

# data
include_service(data)
include_files(data_index)
include_files(data_chunk)
include_files(data_acquisition_plugin_service)
include_files_service(data_acquisition)
include_files_service(data_store)
include_files_service(data_buffer)

# docking
include_files_docking_service(docking)

add_executable(spot
  src/clients/auth.cpp
  src/clients/image.cpp
  src/clients/power.cpp
  src/clients/robot_id.cpp
  src/clients/spot_check.cpp
  src/clients/timesync.cpp
  src/clients/estop.cpp
  src/clients/lease.cpp
  src/clients/robot_command.cpp
  src/clients/robot_state.cpp
  src/clients/directory.cpp
  src/clients/directory_registration.cpp
  src/clients/local_grid.cpp
  src/display.cpp
  src/spot.cpp
  src/robot.cpp

  # deps
  ${HEADER}
  ${FULL_BODY_COMMAND}
  ${MOBILITY_COMMAND}
  ${BASIC_COMMAND}
  ${ARM_COMMAND}
  ${GRIPPER_COMMAND}
  ${SYNCHRONIZED_COMMAND}
  ${TIME_RANGE}
  ${PAYLOAD_ESTIMATION}
  ${TRAJECTORY}
  ${PARAMETER}
  ${GEOMETRY}
  ${TIME_SYNC}
  ${LICENSE}
  ${SERVICE_FAULT}

  # temp
  ${DOCKING}

  # base
  ${AUTH}
  ${DIRECTORY}
  ${LOG_ANNOTATION}
  ${ROBOT_ID}
  ${TIME_SYNC}

  # payloads
  ${DIRECTORY_REGISTRATION}
  ${PAYLOAD}

  # state
  ${IMAGE}
  ${LOCAL_GRID}
  ${ROBOT_STATE}
  ${WORLD_OBJECT}

  # control
  ${ESTOP}
  ${LEASE}
  ${POWER}
  ${ROBOT_COMMAND}
  ${SPOT_CHECK}

  # data
  ${DATA}
  ${DATA_INDEX}
  ${DATA_CHUNK}
  ${DATA_ACQUISITION_PLUGIN_SERVICE}
  ${DATA_ACQUISITION}
  ${DATA_ACQUISITION_STORE}
  ${DATA_BUFFER})

set(LINK_LIBRARIES 
  ${Protobuf_LIBRARIES}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF}
  ${OpenCV_LIBRARIES}
  ${GTK3_LIBRARIES}
  ${CAIRO_LIBRARIES}
)

target_link_libraries(spot ${LINK_LIBRARIES})
