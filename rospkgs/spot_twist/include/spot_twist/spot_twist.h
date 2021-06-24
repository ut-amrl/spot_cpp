#ifndef SPOT_TWIST_H
#define SPOT_TWIST_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "spot/robot.h"

class SpotTwist{
public:
	SpotTwist(ros::NodeHandle &n, Robot spot);
	void twistCallback(const geometry_msgs::Twist &msg);
protected:
	ros::Subscriber _sub;
	Robot _spot;
};

#endif