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

// read_file(): reads the contents of a file and return as string
inline const std::string read_file(const std::string &filename) {
    std::ifstream file(filename);
    std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return str;
}

#endif 
