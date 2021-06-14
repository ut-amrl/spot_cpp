/*
    utils.h: contains common variables and utility functions (should probably split into common and utils later)
*/

#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

// CLIENT_TYPES: types for clients (used throughout lib)
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

// DEFAULT_SPOT_SERVER(): default server hosted on Spot
extern std::string DEFAULT_SPOT_SERVER;

// DEFAULT_ROOT_CERT(): default root certificate found inside 'robot.pem' in resources
extern std::string DEFAULT_ROOT_CERT;

// DEFAULT_SECURE_PORT(): default port
extern std::string DEFAULT_SECURE_PORT;

// read_file(): reads the contents of a file and return as string
std::string read_file(const std::string &filename);

#endif 
