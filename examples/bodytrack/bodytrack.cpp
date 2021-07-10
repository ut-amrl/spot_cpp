#include "bodytrack.h"

std::string IMAGE_FILE = "bodyimage.jpg";

void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // User's displaying/saving/other processing here
            // datum.cvOutputData: rendered frame with pose or heatmaps
            // datum.poseKeypoints: Array<float> with the estimated pose
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
            if (!cvMat.empty())
            {
                cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
                cv::waitKey(0);
            }
            else
                op::opLog("Empty cv::Mat as output.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);

		
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
    try
    {
        // Example: How to use the pose keypoints
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Alternative 1
            //op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);

            // // Alternative 2
            // op::opLog(datumsPtr->at(0)->poseKeypoints, op::Priority::High);

            // // Alternative 3
            // std::cout << datumsPtr->at(0)->poseKeypoints << std::endl;

            // Alternative 4 - Accesing each element of the keypoints
            op::opLog("\nKeypoints:", op::Priority::High);
            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            op::opLog("Person pose keypoints:", op::Priority::High);
			const auto& poseBodyPartMappingBody25 = op::getPoseBodyPartMapping(op::PoseModel::BODY_25);
            for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
            {
                op::opLog("Person " + std::to_string(person) + " (x, y, score):", op::Priority::High);
                for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
                {
                    std::string valueToPrint;
					valueToPrint += poseBodyPartMappingBody25.at(bodyPart) + ": ";
                    for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                        valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
                    op::opLog(valueToPrint, op::Priority::High);
                }
            }
            op::opLog(" ", op::Priority::High);
        }
        else
            op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

int openPoseTest(Spot spot)
{
	try
	{
		// Configuring OpenPose
		op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};

		// Starting OpenPose
		opWrapper.start();

		// Loop while true
			// Get image from spot's camera (No need to save as file, keep in memory)
			// Process image with openPose to get keypoints
			// Move spot to center wrist keypoint in view
				// Try using PID
				// Try using depth cameras

		while(true){
			// Get image from spot
			ImageRequest request;
			request.set_image_source_name("frontright_fisheye_image");
			request.set_quality_percent(100.0);
			request.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
			std::vector<bosdyn::api::ImageRequest> imageRequests;
			imageRequests.push_back(request);
			request.set_image_source_name("frontright_depth_in_visual_frame");
			request.set_quality_percent(100.0);
			request.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
			imageRequests.push_back(request);
			GetImageResponse spotImages = spot.getImageClient()->getImage(imageRequests);

			// Create a cv mat with the image from spot
			std::string imgString = spotImages.image_responses(0).shot().image().data();
			unsigned char dataBuffer[imgString.length()];
			memcpy(dataBuffer, imgString.data(), imgString.length());
			cv::Mat rawCVImg = cv::Mat(imgString.length(), 1, CV_8U, dataBuffer).clone();
			cv::Mat cvImg = rawCVImg.reshape(0, spotImages.image_responses(0).shot().image().rows());
			cvtColor(cvImg, cvImg, CV_GRAY2BGR);
			cv::rotate(cvImg, cvImg, cv::ROTATE_90_CLOCKWISE);

			imgString = spotImages.image_responses(1).shot().image().data();
			unsigned char dataBuffer2[imgString.length()];
			memcpy(dataBuffer2, imgString.data(), imgString.length());
			unsigned short depthArrFlat[imgString.length()/2];
			for(int i = 0; i < imgString.length()/2; i++){
				depthArrFlat[i] = (((unsigned short)dataBuffer2[i*2+1]) << 8) | (0x00ff & dataBuffer2[i*2]);
			}
			// unsigned short depthArr[spotImages.image_responses(1).shot().image().rows()][spotImages.image_responses(1).shot().image().cols()];
			// for(int r = 0; r < spotImages.image_responses(1).shot().image().rows(); r++){
			// 	for(int c = 0; c < spotImages.image_responses(1).shot().image().cols(); c++){
			// 		depthArr[r][c] = depthArrFlat[spotImages.image_responses(1).shot().image().cols() * r + c];
			// 		std::cout << depthArr[r][c] << " ";
			// 	}
			// 	std::cout<< std::endl;
			// }
			cv::Mat rawCVImg2 = cv::Mat(imgString.length() / 2, 1, CV_16U, depthArrFlat).clone();
			cv::Mat cvImg2 = rawCVImg2.reshape(0, spotImages.image_responses(1).shot().image().rows());
			cv::rotate(cvImg2, cvImg2, cv::ROTATE_90_CLOCKWISE);
			cv::Mat normedDepthImg(spotImages.image_responses(1).shot().image().rows(), spotImages.image_responses(1).shot().image().cols(), CV_8U);
			cv::normalize(cvImg2, normedDepthImg, 255, 0, cv::NORM_MINMAX, CV_8U);

			// cv::imshow("Image", cvImg);
			// cv::imshow("Depth Image", normedDepthImg);
			// cv::waitKey();

			// Process image with openPose
			const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImg);
			auto poseDatum = opWrapper.emplaceAndPop(imageToProcess);
			if(poseDatum == nullptr){
				std::cout << "Image could not be processed" << std::endl;
				continue;
			}
			if(poseDatum->empty()){
				std::cout << "Datum is empty" << std::endl;
				continue;
			}

			// Get location of right wrist
			const auto& poseKeypoints = poseDatum->at(0)->poseKeypoints;
			if(poseKeypoints.getSize(0) > 0){
				const auto rWristX = poseKeypoints[{0, 4, 0}];
				const auto rWristY = poseKeypoints[{0, 4, 1}];
				if(rWristX != 0 && rWristY != 0){
					std::cout << "Right Wrist X: " << rWristX << std::endl;
					std::cout << "Right Wrist Y: " << rWristY << std::endl;
					cv::Point wristPoint(rWristX, rWristY);
					unsigned short depth = cvImg2.at<unsigned short>(wristPoint);
					if(depth > 0 && depth < 4500){
						std::cout << "Depth: " << depth << std::endl;
						double fx = spotImages.image_responses(1).source().pinhole().intrinsics().focal_length().x();
						double fy = spotImages.image_responses(1).source().pinhole().intrinsics().focal_length().y();
						double cx = spotImages.image_responses(1).source().pinhole().intrinsics().principal_point().x();
						double cy = spotImages.image_responses(1).source().pinhole().intrinsics().principal_point().y();
						double point3dX = ((wristPoint.x - cx) * (double)cvImg2.at<unsigned short>(wristPoint)) / fx;
						double point3dY = ((wristPoint.y - cy) * (double)cvImg2.at<unsigned short>(wristPoint)) / fy;
						double point3dZ = cvImg2.at<unsigned short>(wristPoint);
						point3dX /= 1000;
						point3dY /= 1000;
						point3dZ /= 1000;
						std::cout << "3D X: " << point3dX << std::endl;
						std::cout << "3D Y: " << point3dY << std::endl;
						std::cout << "3D Z: " << point3dZ << std::endl;

						// google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge> frameMap = spotImages.image_responses(1).shot().transforms_snapshot().child_to_parent_edge_map();
						// for (google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge>::const_iterator it=frameMap.begin(); it!=frameMap.end(); ++it){
						// 	std::cout << it->first << " : " << it->second.parent_frame_name() << std::endl;
						// }

						FrameTree ft(spotImages.image_responses(1).shot().transforms_snapshot());
						// std::map<std::string, FrameTree::Parent>::iterator it;
						// for(it = ft.childToParentEdges().begin(); it!=ft.childToParentEdges().end(); it++){
						// 	std::cout << it->first << " : " << it->second.parentFrameName() << std::endl;
						// }
						
				
						Math::SE3Pose frontright_depth_tform_rwrist(point3dX, point3dY, point3dZ, Math::Quaternion(0, 0, 0, 1));
						ft.addEdge(frontright_depth_tform_rwrist, "frontright", "rwrist");
						Math::SE3Pose body_tf_rwrist = ft.a_tf_b("body", "rwrist");
						std::cout << "Transform X: " << body_tf_rwrist.x() << std::endl;
						std::cout << "Transform Y: " << body_tf_rwrist.y() << std::endl;
						std::cout << "Transform Z: " << body_tf_rwrist.z() << std::endl;

						double pitch = atan2(body_tf_rwrist.z(), body_tf_rwrist.x());
						double yaw = atan2(body_tf_rwrist.y(), body_tf_rwrist.x());

						Trajectory3D trajPose;
						trajPose.addPointRPY(0, 0, 0, 0, -pitch, yaw, 1);
						spot.setBodyPose(trajPose, true);
						spot.stand();
					}
					std::cout<< std::endl;
					//display(poseDatum);
				}
			}
		}


		// Return
		return 0;
	}
	catch (const std::exception&)
	{
		return -1;
	}
}


int main(int argc, char *argv[]){
	assert(argc == 3);

	// get username and password for spot
	const std::string username = argv[1];
	const std::string password = argv[2];

	// Initialize spot
	Spot spot;
	spot.authenticate(username, password);
	std::cout << "Spot authenticated" << std::endl;
	spot.initClients();
	std::cout << "Spot authenticated" << std::endl;
	spot.initBasicEstop();
	std::cout << "Spot authenticated" << std::endl;
	spot.initBasicLease();
	std::cout << "Spot authenticated" << std::endl;
	spot.initBasicTimeSync();
	std::cout << "Spot authenticated" << std::endl;
	spot.powerOnBlocking();
	//spot.basicInit(username, password);
	std::cout << "Spot initialization complete" << std::endl;
	spot.stand();

	return openPoseTest(spot);
}