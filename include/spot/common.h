/*
    common.h: common information needed throughout software
*/
#ifndef COMMON_H
#define COMMON_H

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

// variant type that holds clients, to be used in containers that store clients
typedef std::variant<
    AuthClient,
    DirectoryClient,
    EstopClient,
    ImageClient,
    LeaseClient,
    PowerClient,
    RobotCommandClient,
    RobotIdClient,
    RobotStateClient,
    SpotCheckClient,
    TimeSyncClient> CLIENT_CONTAINER;

#endif