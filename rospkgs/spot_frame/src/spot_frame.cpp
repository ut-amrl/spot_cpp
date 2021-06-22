#include "spot_frame/spot_frame.h"

int main(int argc, char **argv) {
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	ros::init(argc, argv, "spot_frame_client");
	ros::NodeHandle node_handle;

	TFBroadcastPR tfbr;
	tfbr.setFrames("origin", "test");
	geometry_msgs::Pose pose;
	pose.position.x = 1;
	pose.position.y = 1;
	pose.position.z = 1;
	pose.orientation.w = 1;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	while(true){
		tfbr.receivePose(pose);
		sleep(1);
	}
	

	Robot spot("spot");

	std::cout << spot.getId() << std::endl;

	// authenticate robot
	spot.authenticate(username, password);

	// setup robot (initialize clients)
	spot.setup();

	ListImageSourcesResponse imgSrcs = spot.getImageClientPtr()->listImageSources();
	for(int i = 0; i < imgSrcs.image_sources().size(); i++){
		std::cout << "Image Source Name: " << imgSrcs.image_sources(i).name() << std::endl;
	}
	std::cout << std::endl << std::endl;

	std::vector<ImageRequest> imgReqs;
	ImageRequest imgReq1;
	imgReq1.set_image_source_name("frontleft_fisheye_image");
	imgReq1.set_quality_percent(100);
	imgReq1.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
	imgReqs.push_back(imgReq1);
	ImageResponse imgResp = spot.getImageClientPtr()->getImage(imgReqs).image_responses(0);
	google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge> frameMap = imgResp.shot().transforms_snapshot().child_to_parent_edge_map();
	std::cout << "Image frames: " << std::endl;
	for (google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge>::const_iterator it=frameMap.begin(); it!=frameMap.end(); ++it){
    	std::cout << "Parent-Child combo:" << std::endl;
		std::cout << "Child Name: " << it->first << std::endl;
		std::cout << "Parent Name: " << it->second.parent_frame_name() << std::endl << std::endl;
	}
	std::cout << std::endl << std::endl << std::endl;

	std::cout << "Kinematic frames:" << std::endl;
	RobotStateResponse rState = spot.getRobotStateClientPtr()->getRobotState();
	frameMap = rState.robot_state().kinematic_state().transforms_snapshot().child_to_parent_edge_map();
	for (google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge>::const_iterator it=frameMap.begin(); it!=frameMap.end(); ++it){
    	std::cout << "Parent-Child combo:" << std::endl;
		std::cout << "Child Name: " << it->first << std::endl;
		std::cout << "Parent Name: " << it->second.parent_frame_name() << std::endl << std::endl;
	}

	ros::spin();

	return 0;
}