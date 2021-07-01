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

class FrameTree {
public:
    FrameTree(bosdyn::api::FrameTreeSnapshot snapshot);

    class Parent {
    public: 
        Parent(Math::SE3Pose parent_tf_child, std::string parentFrameName);

        Math::SE3Pose parent_tf_child();
        std::string parentFrameName();
    private:
        Math::SE3Pose _parent_tf_child;
        std::string _parentFrameName;
    };

    std::map<std::string, Parent> childToParentEdges();

    void addEdge(Math::SE3Pose parent_tf_child, std::string parentFrame, std::string childFrame);

    Math::SE3Pose a_tf_b(std::string frameA, std::string frameB);

private:
    std::vector<Parent> listParentEdges(std::string leafFrame);
    Math::SE3Pose accumulateTransforms(std::vector<Parent> parents);

    std::map<std::string, Parent> _childToParentEdges;
};

std::string frameNameGravAligned(gravAlignedFrame frame);

#endif
