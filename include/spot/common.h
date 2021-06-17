/*
    common.h: common information needed throughout software
*/
#ifndef COMMON_H
#define COMMON_H

#include <grpc++/grpc++.h>
#include <google/protobuf/util/time_util.h>

#include "bosdyn/api/header.grpc.pb.h"

#include <variant>

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