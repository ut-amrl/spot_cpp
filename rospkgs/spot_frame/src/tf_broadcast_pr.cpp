#include "ros/ros.h"
#include "spot_frame/tf_broadcast_pr.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void TFBroadcastPR::receivePose(geometry_msgs::Pose &pose){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = parent;
	transformStamped.child_frame_id = child;
	
	transformStamped.transform.translation.x = pose.position.x;
	transformStamped.transform.translation.y = pose.position.y;
	transformStamped.transform.translation.z = pose.position.z;
	
	transformStamped.transform.rotation.x = pose.orientation.x;
	transformStamped.transform.rotation.y = pose.orientation.y;
	transformStamped.transform.rotation.z = pose.orientation.z;
	transformStamped.transform.rotation.w = pose.orientation.w;

	br.sendTransform(transformStamped);
}

void TFBroadcastPR::setFrames(std::string parent_in, std::string child_in){
	parent = parent_in;
	child = child_in;
}