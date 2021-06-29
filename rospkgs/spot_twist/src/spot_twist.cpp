#include "spot_twist/spot_twist.h"

SpotTwist::SpotTwist(ros::NodeHandle &n, Spot spot) :
	_spot(spot) {
	_sub = n.subscribe("cmd_vel", 1000, &SpotTwist::twistCallback, this);
}

void SpotTwist::twistCallback(const geometry_msgs::Twist &msg){
	ROS_INFO("\nReceived Twist:");
	ROS_INFO("Linear:");
	ROS_INFO("x: %f  y: %f  z: %f", msg.linear.x, msg.linear.y, msg.linear.z);
	ROS_INFO("Angular:");
	ROS_INFO("x: %f  y: %f  z: %f", msg.angular.x, msg.angular.y, msg.angular.z);

	if(msg.linear.x != 0 || msg.linear.y != 0 || msg.angular.z != 0){
		_spot.velocityMove(msg.linear.x, msg.linear.y, msg.angular.z, 5000, FLAT_BODY);
	}
	else{
		_spot.stand();
	}
}

int main(int argc, char **argv) {
	//assert(argc == 3);
	std::cout << argv[0] << std::endl;
	std::cout << argv[1] << std::endl;
	std::cout << argv[2] << std::endl;
	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	ros::init(argc, argv, "spot_twist_client");
	ros::NodeHandle node_handle;

	Spot spot;
	spot.basicInit(username, password);
	

	//std::cout << spot.getId() << std::endl;

	// // authenticate robot
	// spot.authenticate(username, password);

	// // setup robot (initialize clients)
	// spot.setup();

	// // create estop and lease threads
	// spot.initBasicEstop();
	// std::cout << "Estop initialized" << std::endl;
	// spot.initBasicLease();
	// std::cout << "Lease initialized" << std::endl;
	// spot.performTimesync();
	// std::cout << "Timesync initialized" << std::endl;

	// spot.powerOn();
	// std::cout << "Powered on" << std::endl;

	Trajectory3D trajPose;
	trajPose.addPointRPY(0, 0, 0, 0, 0, 0, 1);
	spot.setBodyPose(trajPose, true);

	spot.stand();
	sleep(1);
	std::cout << "Entering teleop control" << std::endl;
	

	SpotTwist spotTwist(node_handle, spot);


	ros::spin();

	return 0;
}