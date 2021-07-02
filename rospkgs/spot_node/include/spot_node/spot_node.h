#ifndef SPOT_NODE_H
#define SPOT_NODE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "spot/spot.h"

// messages
#include "nav_msgs/Odometry.h"
#include "spot_ros_msgs/KinematicState.h"

// services (todo: velocity and traj)
#include <std_srvs/SetBool.h>
#include <spot_ros_srvs/Stand.h> 

class SpotNode{
public:
	SpotNode(Spot spot);

	void start(int, char **);
	
	static Math::SE3Pose getVisionTFormBody(bosdyn::api::FrameTreeSnapshot);

	static Math::SE3Pose getOdomTFormBody(bosdyn::api::FrameTreeSnapshot);

	static spot_ros_msgs::KinematicState createKinematicMessage(Math::SE3Pose, Math::SE3Pose, bosdyn::api::SE3Velocity, bosdyn::api::SE3Velocity, bosdyn::api::KinematicState kinematicState);

	static nav_msgs::Odometry createOdometryMessage(spot_ros_msgs::KinematicState);

private:
	/* Callback functions */
	bool sitCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
	bool standCallback(spot_ros_srvs::Stand::Request &req, spot_ros_srvs::Stand::Response &resp);

	/* Other functions */
	void twistCallback(const geometry_msgs::Twist &msg);	

private:
	ros::Subscriber _sub;
	Spot _spot;
};

#endif
