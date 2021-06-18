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

// types of clients used
enum class CLIENT_TYPES {
    AUTH = 1,
    DIRECTORY,
    ESTOP,
    IMAGE,
    LEASE,
    POWER,
    ROBOT_COMMAND,
    ROBOT_ID,
    ROBOT_STATE,
    SPOT_CHECK,
    TIMESYNC,
};

#endif