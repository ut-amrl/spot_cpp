#ifndef SPOT_TWIST_H
#define SPOT_TWIST_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "spot/spot.h"

class SpotTwist{
public:
	SpotTwist(ros::NodeHandle &n, Spot spot);
	void twistCallback(const geometry_msgs::Twist &msg);
protected:
	ros::Subscriber _sub;
	Spot _spot;
};

#endif