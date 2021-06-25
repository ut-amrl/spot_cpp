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
#include <spot/clients/base.h>

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

std::string frameNameGravAligned(gravAlignedFrame frame);

#endif