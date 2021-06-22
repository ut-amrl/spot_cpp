#include "spot_frame/spot_frame.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "spot_frame_client");
	ros::NodeHandle node_handle;

	Robot spot("spot");

	
	ros::spin();

	return 0;
}