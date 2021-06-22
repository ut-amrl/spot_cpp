#include "spot_frame/spot_frame.h"

int main(int argc, char **argv) {
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	ros::init(argc, argv, "spot_frame_client");
	ros::NodeHandle node_handle;

	Robot spot("spot");

	
	ros::spin();

	return 0;
}