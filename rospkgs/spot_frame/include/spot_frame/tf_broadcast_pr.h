#ifndef TF_BROADCAST_POSE_RECIPIENT_H
#define TF_BROADCAST_POSE_RECIPIENT_H

#include "pose_recipient.h"

class TFBroadcastPR : public PoseRecipient{
	std::string parent;
	std::string child;
public:
	void receivePose(geometry_msgs::Pose &pose);
	void setFrames(std::string parent_in, std::string child_in);
};

#endif