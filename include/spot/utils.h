#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

// temporary stuff for now to get robot_id working, refactor into classes later

std::map<std::string, std::string> BOOTSTRAP_SERVICE_AUTHORITIES;

// DEFAULT_SECURE_CHANNEL_PORT: default port for channel creation by gRPC
extern std::string DEFAULT_SECURE_CHANNEL_PORT;

// read_file(): reads the contents of a file into a string
void read_file(const std::string &filename, std::string &data);

#endif 