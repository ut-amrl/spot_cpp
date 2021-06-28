/*
    common.h: common information needed throughout software
*/
#ifndef COMMON_H
#define COMMON_H

#include <grpc++/grpc++.h>
#include <google/protobuf/util/time_util.h>

#include "bosdyn/api/header.grpc.pb.h"

#include <spot/utils.h>

// variables for connecting to Spot
const std::string DEFAULT_SPOT_SERVER = "192.168.80.3";
const std::string DEFAULT_SECURE_PORT = ":443";
const std::string DEFAULT_ROOT_CERT_FILEPATH = "../src/resources/robot.pem";
const std::string DEFAULT_ROOT_CERT = read_file(DEFAULT_ROOT_CERT_FILEPATH);

/* standard client names used when creating clients */
const std::string AUTH_CLIENT_NAME = "auth";
const std::string DIRECTORY_REGISTRATION_CLIENT_NAME = "directory-registration";
const std::string DIRECTORY_CLIENT_NAME = "directory";
const std::string ESTOP_CLIENT_NAME = "estop";
const std::string IMAGE_CLIENT_NAME = "image";
const std::string LEASE_CLIENT_NAME = "lease";
const std::string LOCAL_GRID_CLIENT_NAME = "local-grid";
const std::string POWER_CLIENT_NAME = "power";
const std::string ROBOT_COMMAND_CLIENT_NAME = "robot-command";
const std::string ROBOT_ID_CLIENT_NAME = "robot-id";
const std::string ROBOT_STATE_CLIENT_NAME = "robot-state";
const std::string SPOT_CHECK_CLIENT_NAME = "spot-check";
const std::string TIMESYNC_CLIENT_NAME = "time-sync";
const std::string WORLD_OBJECTS_CLIENT_NAME = "world-objects";

#endif