#ifndef SPOT_ROS_CPP_H
#define SPOT_ROS_CPP_H

#include "helpers.h"

class SpotROSNode {
public:
    SpotROSNode(const std::string &username, const std::string &password);

    void run(int argc, char **argv);
private:
    Spot _spot;
};

#endif