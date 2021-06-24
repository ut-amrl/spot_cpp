/*
    frame.h: robot frame helper methods and values 
*/

#ifndef FRAME_H
#define FRAME_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include "bosdyn/api/robot_command_service.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include "bosdyn/api/lease_service.grpc.pb.h"
#include <spot/base.h>

using bosdyn::api::Vec2;
using bosdyn::api::SE2Pose;
using bosdyn::api::Vec3;
using bosdyn::api::SE3Pose;

enum gravAlignedFrame {
    ODOM,
    VISION,
    FLAT_BODY
};

const std::string VISION_FRAME_NAME = "vision";
const std::string BODY_FRAME_NAME = "body";
const std::string FLAT_BODY_FRAME_NAME = "flat_body";
const std::string GRAV_ALIGNED_BODY_FRAME_NAME = "gpe";
const std::string ODOM_FRAME_NAME = "odom";
const std::string GROUND_PLANE_FRAME_NAME = "gpe";
const std::string HAND_FRAME_NAME = "hand";
const std::string UNKNOWN_FRAME_NAME = "unknown";
const std::string RAYCAST_FRAME_NAME = "walkto_raycast_intersection";

class Vector3{
public:
    Vector3(double x, double y, double z);
    double x;
    double y;
    double z;
};

class Quaternion{
public:
    Quaternion(double x, double y, double z, double w);

    Quaternion inverse();
    Quaternion mult(Quaternion otherQuat);
    Vector3 transformPoint(double x, double y, double z);

    double w();
    double x();
    double y();
    double z();

private:
    double _w; 
    double _x;
    double _y;
    double _z;
};

class Pose3{
public:
    Pose3(double x, double y, double z, Quaternion quat);
    Pose3(SE3Pose pose);

    Pose3 mult(Pose3 otherPose);

    double x();
    double y();
    double z();
    Quaternion quat();

private:
    double _x;
    double _y;
    double _z;
    Quaternion _q;
};

std::string frameNameGravAligned(gravAlignedFrame frame);

#endif
