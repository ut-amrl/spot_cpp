#include "spot_node/spot_node.h"

#include <string>

SpotNode::SpotNode(Spot spot) :
	_spot(spot),
	_t_last_non_zero_cmd() {
}

bool SpotNode::sit_cmd_srv(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp){
	if (req.data) {
		// sit
		_spot.sit();
		resp.success = true;
		resp.message = "sit success";
		ROS_INFO("Spot sit request received, Spot should now be sitting.");
		return true;
	} else {
		// do nothing if false (shouldn't be used)
		return false;
	}
}

bool SpotNode::stand_cmd_srv(spot_ros_srvs::Stand::Request &req, spot_ros_srvs::Stand::Response &resp){
	// TODO: add body height parameter in utspot lib
	_spot.stand();
	ROS_INFO("Spot stand request received, Spot should now be standing.");
	return true;
}

bool SpotNode::trajectory_cmd_srv(spot_ros_srvs::Trajectory::Request &req, spot_ros_srvs::Trajectory::Response &resp) {

}

bool SpotNode::velocity_cmd_srv(spot_ros_srvs::Velocity::Request &req, spot_ros_srvs::Velocity::Response &resp) {

}

void SpotNode::cmd_vel_listener(const geometry_msgs::Twist &msg) {
	// 600 ms
	int vel_cmd_duration = 600;
	std::chrono::seconds max_marching_time(5);

	ROS_INFO("\nReceived Twist:");
	ROS_INFO("Linear:");
	ROS_INFO("x: %f  y: %f  z: %f", msg.linear.x, msg.linear.y, msg.linear.z);
	ROS_INFO("Angular:");
	ROS_INFO("x: %f  y: %f  z: %f", msg.angular.x, msg.angular.y, msg.angular.z);

	bool all_zero = (msg.linear.x == 0) && (msg.linear.y == 0) && (msg.linear.z == 0);

	if (all_zero) {
		// vel command still executing
		if (_t_last_non_zero_cmd < std::chrono::steady_clock::now() - max_marching_time) {
			return;
		}
	} else {
		// set time of last nonzero command
		_t_last_non_zero_cmd = std::chrono::steady_clock::now();
		_spot.velocityMove(msg.linear.x, msg.linear.y, msg.linear.z, vel_cmd_duration, FLAT_BODY);
	}

// 	if(msg.linear.x != 0 || msg.linear.y != 0 || msg.angular.z != 0){
// 		_spot.velocityMove(msg.linear.x, msg.linear.y, msg.angular.z, vel_cmd_duration, FLAT_BODY);
// 	}
// 	else{
// 		_spot.stand();
// 	}
// }
}

void SpotNode::start(int argc, char **argv){
	ros::init(argc, argv, "spot_node");
	ros::NodeHandle node_handle;
	ros::Rate rate(200);

	// subscribers
	_sub = node_handle.subscribe("cmd_vel", 1000, &SpotNode::cmd_vel_listener, this);

	// publishers
	// TODO: implement other publishers
	ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("odom", 1000, false);
	
	// services
	// TODO: implement other services
	ros::ServiceServer sit_service = node_handle.advertiseService("sit_cmd", &SpotNode::sit_cmd_srv, this);
	ros::ServiceServer stand_server = node_handle.advertiseService("stand_cmd", &SpotNode::stand_cmd_srv, this);

	while (ros::ok()) {
		// get robot state from spot
		bosdyn::api::RobotState state = _spot.getSpotState()->robotState();

		// populate odometry message from params in kinematic state in robot state
		bosdyn::api::KinematicState kin_state = state.kinematic_state();
		bosdyn::api::FrameTreeSnapshot ft_snapshot = kin_state.transforms_snapshot();				

		// send
		spot_ros_msgs::KinematicState kin_state_msg = createKinematicMessage(getVisionTFormBody(ft_snapshot), getOdomTFormBody(ft_snapshot), kin_state.velocity_of_body_in_vision(), kin_state.velocity_of_body_in_odom(), kin_state);		
		nav_msgs::Odometry odom_msg = createOdometryMessage(kin_state_msg);

		ROS_INFO("odom_out.pose.pose.position.x: %f\n", odom_msg.pose.pose.position.x);
		ROS_INFO("odom_out.pose.pose.position.y: %f\n", odom_msg.pose.pose.position.y);
		ROS_INFO("odom_out.pose.pose.position.z: %f\n", odom_msg.pose.pose.position.z);
		ROS_INFO("odom_out.pose.pose.orientation.x: %f\n", odom_msg.pose.pose.orientation.x);
		ROS_INFO("odom_out.pose.pose.orientation.y: %f\n", odom_msg.pose.pose.orientation.y);
		ROS_INFO("odom_out.pose.pose.orientation.z: %f\n", odom_msg.pose.pose.orientation.z);
		ROS_INFO("odom_out.pose.pose.orientation.w: %f\n", odom_msg.pose.pose.orientation.w);

		odom_pub.publish(odom_msg);
		ros::spinOnce();
		rate.sleep();
	}
}

Math::SE3Pose SpotNode::getVisionTFormBody(bosdyn::api::FrameTreeSnapshot ftSnapshot){
	// create utspot frametree object
	FrameTree tree(ftSnapshot);
	
	// get transform
	Math::SE3Pose ret = tree.a_tf_b(VISION_FRAME_NAME, BODY_FRAME_NAME);
	return ret;
}

Math::SE3Pose SpotNode::getOdomTFormBody(bosdyn::api::FrameTreeSnapshot ftSnapshot){
	// create utspot ft
	FrameTree tree(ftSnapshot);

	// get transform
	Math::SE3Pose ret = tree.a_tf_b(ODOM_FRAME_NAME, BODY_FRAME_NAME);
	return ret;
}

spot_ros_msgs::KinematicState SpotNode::createKinematicMessage(Math::SE3Pose visionTFormBody, Math::SE3Pose odomTFormBody, bosdyn::api::SE3Velocity visionBodyVelocity, bosdyn::api::SE3Velocity odomBodyVelocity, bosdyn::api::KinematicState kinematicState) {
	// create msg
	spot_ros_msgs::KinematicState ret;

	// vision_tform_body
	ret.vision_tform_body.translation.x = visionTFormBody.x();
	ret.vision_tform_body.translation.y = visionTFormBody.y();
	ret.vision_tform_body.translation.z = visionTFormBody.z();

	ret.vision_tform_body.rotation.x = visionTFormBody.quat().x();
	ret.vision_tform_body.rotation.y = visionTFormBody.quat().y();
	ret.vision_tform_body.rotation.z = visionTFormBody.quat().z();
	ret.vision_tform_body.rotation.w = visionTFormBody.quat().w();

	// odom_tform_body
        ret.odom_tform_body.translation.x = odomTFormBody.x();
        ret.odom_tform_body.translation.y = odomTFormBody.y();
        ret.odom_tform_body.translation.z = odomTFormBody.z();

        ret.odom_tform_body.rotation.x = odomTFormBody.quat().x();
        ret.odom_tform_body.rotation.y = odomTFormBody.quat().y();
        ret.odom_tform_body.rotation.z = odomTFormBody.quat().z();
        ret.odom_tform_body.rotation.w = odomTFormBody.quat().w();

	// velocity of body in vision
	ret.velocity_of_body_in_vision.linear.x = visionBodyVelocity.linear().x();
	ret.velocity_of_body_in_vision.linear.y = visionBodyVelocity.linear().y();
	ret.velocity_of_body_in_vision.linear.z = visionBodyVelocity.linear().z();

	ret.velocity_of_body_in_vision.angular.x = visionBodyVelocity.angular().x();
	ret.velocity_of_body_in_vision.angular.y = visionBodyVelocity.angular().y();
	ret.velocity_of_body_in_vision.angular.z = visionBodyVelocity.angular().z();

	// velocity of body in odom

	ret.velocity_of_body_in_odom.linear.x = odomBodyVelocity.linear().x();
	ret.velocity_of_body_in_odom.linear.y = odomBodyVelocity.linear().y();
	ret.velocity_of_body_in_odom.linear.z = odomBodyVelocity.linear().z();

	ret.velocity_of_body_in_odom.angular.x = odomBodyVelocity.angular().x();
	ret.velocity_of_body_in_odom.angular.y = odomBodyVelocity.angular().y();
	ret.velocity_of_body_in_odom.angular.z = odomBodyVelocity.angular().z();

	return ret;
}


nav_msgs::Odometry SpotNode::createOdometryMessage(spot_ros_msgs::KinematicState kinematic_state){
	nav_msgs::Odometry odom_out;

	odom_out.header = kinematic_state.header;
	odom_out.header.frame_id = "odom";
	odom_out.child_frame_id = "base_link";
	odom_out.pose.pose.position.x = kinematic_state.vision_tform_body.translation.x;
	odom_out.pose.pose.position.y = kinematic_state.vision_tform_body.translation.y;
	odom_out.pose.pose.position.z = kinematic_state.vision_tform_body.translation.z;

	odom_out.pose.pose.orientation.x = kinematic_state.vision_tform_body.rotation.x;
	odom_out.pose.pose.orientation.y = kinematic_state.vision_tform_body.rotation.y;
	odom_out.pose.pose.orientation.z = kinematic_state.vision_tform_body.rotation.z;
	odom_out.pose.pose.orientation.w = kinematic_state.vision_tform_body.rotation.w;

	return odom_out;
}

int main(int argc, char **argv) {
	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	Spot spot;
	spot.basicInit(username, password);
	
	Trajectory3D trajPose;
	trajPose.addPointRPY(0, 0, 0, 0, 0, 0, 1);
	spot.setBodyPose(trajPose, true);

	//spot.stand();
	sleep(1);
	std::cout << "Entering teleop control" << std::endl;

	// create spotnode
	SpotNode spotNode(spot);

	spotNode.start(argc, argv);

	// ros::spin();

	return 0;
}
