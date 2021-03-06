#include "spot_frame/spot_frame.h"

int main(int argc, char **argv) {
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	ros::init(argc, argv, "spot_frame_client");
	ros::NodeHandle node_handle;

	// TFBroadcastPR tfbr;
	// tfbr.setFrames("origin", "test");
	// geometry_msgs::Pose pose;
	// pose.position.x = 1;
	// pose.position.y = 1;
	// pose.position.z = 1;
	// pose.orientation.w = 1;
	// pose.orientation.x = 0;
	// pose.orientation.y = 0;
	// pose.orientation.z = 0;
	// while(true){
	// 	tfbr.receivePose(pose);
	// 	sleep(1);
	// }

	std::vector<geometry_msgs::Pose> tfbrList;
	

	Spot spot;

	//std::cout << spot.getId() << std::endl;

	// authenticate robot
	spot.authenticate(username, password);

	// setup robot (initialize clients)
	spot.initClients();
	spot.initBasicEstop();
	spot.initBasicLease();
	spot.initBasicTimeSync();

	std::vector<ImageRequest> imgReqs;
	ImageRequest imgReq1;
	imgReq1.set_image_source_name("frontright_depth_in_visual_frame");
	imgReq1.set_quality_percent(100);
	imgReq1.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
	ImageRequest imgReq2;
	imgReq2.set_image_source_name("frontright_fisheye_image");
	imgReq2.set_quality_percent(100);
	imgReq2.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
	ImageRequest imgReq3;
	imgReq3.set_image_source_name("back_fisheye_image");
	imgReq3.set_quality_percent(100);
	imgReq3.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
	ImageRequest imgReq4;
	imgReq4.set_image_source_name("left_fisheye_image");
	imgReq4.set_quality_percent(100);
	imgReq4.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
	ImageRequest imgReq5;
	imgReq5.set_image_source_name("right_fisheye_image");
	imgReq5.set_quality_percent(100);
	imgReq5.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
	imgReqs.push_back(imgReq1);
	//imgReqs.push_back(imgReq2);
	//imgReqs.push_back(imgReq3);
	//imgReqs.push_back(imgReq4);
	//imgReqs.push_back(imgReq5);
	while(true){
		GetImageResponse getImgResp = spot.getImageClient()->getImage(imgReqs);
		for(int imgNum = 0; imgNum < getImgResp.image_responses_size(); imgNum++){
			ImageResponse imgResp = spot.getImageClient()->getImage(imgReqs).image_responses(imgNum);
			google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge> frameMap = imgResp.shot().transforms_snapshot().child_to_parent_edge_map();

			if(imgResp.source().name().compare("frontright_depth_in_visual_frame") == 0){
				FrameTree ft(imgResp.shot().transforms_snapshot());
				
				Math::SE3Pose frontleft_fisheye_tform_vision_obj(-1.14565, -0.504815, 1.582, Math::Quaternion(0, 0, 0, 1));
				ft.addEdge(frontleft_fisheye_tform_vision_obj, "frontright", "vision_obj");
				Math::SE3Pose body_tf_vision_obj = ft.a_tf_b("body", "vision_obj");

				TFBroadcastPR tfbr;
				tfbr.setFrames("frontright", "vision_obj");
				geometry_msgs::Pose pose;
				pose.position.x = frontleft_fisheye_tform_vision_obj.x();
				pose.position.y = frontleft_fisheye_tform_vision_obj.y();
				pose.position.z = frontleft_fisheye_tform_vision_obj.z();
				pose.orientation.w = frontleft_fisheye_tform_vision_obj.quat().w();
				pose.orientation.x = frontleft_fisheye_tform_vision_obj.quat().x();
				pose.orientation.y = frontleft_fisheye_tform_vision_obj.quat().y();
				pose.orientation.z = frontleft_fisheye_tform_vision_obj.quat().z();
				tfbr.receivePose(pose);

				TFBroadcastPR tfbr2;
				tfbr2.setFrames("body", "vision_obj_test");
				pose.position.x = body_tf_vision_obj.x();
				pose.position.y = body_tf_vision_obj.y();
				pose.position.z = body_tf_vision_obj.z();
				pose.orientation.w = body_tf_vision_obj.quat().w();
				pose.orientation.x = body_tf_vision_obj.quat().x();
				pose.orientation.y = body_tf_vision_obj.quat().y();
				pose.orientation.z = body_tf_vision_obj.quat().z();
				tfbr2.receivePose(pose);

				double pitch = atan2(body_tf_vision_obj.z(), body_tf_vision_obj.x()); // (x,y) is (tform.x, tform.z)
				double yaw = atan2(body_tf_vision_obj.y(), body_tf_vision_obj.x()); // (x,y) is (tform.x, tform.y)
				//std::cout << "Pitch: " << pitch << " rad, " << (pitch * 57.2958) << " deg" << std::endl;
				//std::cout << "Yaw: " << yaw << " rad, " << (yaw * 57.2958) << " deg" << std::endl << std::endl;
			}
			
			for (google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge>::const_iterator it=frameMap.begin(); it!=frameMap.end(); ++it){
				// std::cout << "Parent-Child combo:" << std::endl;
				// std::cout << "Child Name: " << it->first << std::endl;
				// std::cout << "Parent Name: " << it->second.parent_frame_name() << std::endl << std::endl;
				if(!it->second.parent_frame_name().empty()){
					TFBroadcastPR tfbr;
					tfbr.setFrames(it->second.parent_frame_name(), it->first);
					geometry_msgs::Pose pose;
					pose.position.x = it->second.parent_tform_child().position().x();
					pose.position.y = it->second.parent_tform_child().position().y();
					pose.position.z = it->second.parent_tform_child().position().z();
					pose.orientation.w = it->second.parent_tform_child().rotation().w();
					pose.orientation.x = it->second.parent_tform_child().rotation().x();
					pose.orientation.y = it->second.parent_tform_child().rotation().y();
					pose.orientation.z = it->second.parent_tform_child().rotation().z();
					tfbr.receivePose(pose);
				}
				
			}
			
		}
	}
	std::cout << std::endl << std::endl << std::endl;

	// std::cout << "Kinematic frames:" << std::endl;
	// RobotStateResponse rState = spot.getRobotStateClientPtr()->getRobotState();
	// frameMap = rState.robot_state().kinematic_state().transforms_snapshot().child_to_parent_edge_map();
	// for (google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge>::const_iterator it=frameMap.begin(); it!=frameMap.end(); ++it){
    // 	std::cout << "Parent-Child combo:" << std::endl;
	// 	std::cout << "Child Name: " << it->first << std::endl;
	// 	std::cout << "Parent Name: " << it->second.parent_frame_name() << std::endl << std::endl;
	// }

	ros::spin();

	return 0;
}