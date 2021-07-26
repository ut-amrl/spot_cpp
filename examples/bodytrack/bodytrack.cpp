#include "bodytrack.h"

const int KEYPOINT_INDEX_RWRIST = 4;
const int TRACKED_KEYPOINT_INDEX = KEYPOINT_INDEX_RWRIST;

const double ERROR_THRESHOLD = 0.1;

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

Math::Vector3 quatToRPY(Math::Quaternion q){
	double roll, pitch, yaw;

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
	roll = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
	if (std::abs(sinp) >= 1)
		pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = std::asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
	yaw = std::atan2(siny_cosp, cosy_cosp);

	return Math::Vector3(roll, pitch, yaw);
}

int openPosePID(Spot spot)
{
	try
	{
		// Configuring OpenPose
		op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};

		// Starting OpenPose
		opWrapper.start();

		// Values that will be adjusted to change spot's position
		double spotPitch = 0;
		double spotYaw = 0;

		
		FrameTree ftOrientation(spot.getRobotStateClient()->getRobotState().robot_state().kinematic_state().transforms_snapshot());
		Math::SE3Pose vision_tf_bodyInitial = ftOrientation.a_tf_b("vision", "body");

		// Main Loop that adjusts spot's position to look at the tracked keypoint
		while(true){
			// Get images from spot
			std::vector<bosdyn::api::ImageRequest> imageRequests;
			ImageRequest request;
			request.set_image_source_name("frontright_fisheye_image");
			request.set_quality_percent(100.0);
			request.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
			imageRequests.push_back(request);
			GetImageResponse spotImages = spot.getImageClient()->getImage(imageRequests);

			// Create a cv mat with the fisheye image from spot
			std::string imgString = spotImages.image_responses(0).shot().image().data();
			unsigned char dataBuffer[imgString.length()];
			memcpy(dataBuffer, imgString.data(), imgString.length());
			cv::Mat rawCVImg = cv::Mat(imgString.length(), 1, CV_8U, dataBuffer).clone();
			cv::Mat cvImg = rawCVImg.reshape(0, spotImages.image_responses(0).shot().image().rows());
			cvtColor(cvImg, cvImg, CV_GRAY2BGR);
			cv::rotate(cvImg, cvImg, cv::ROTATE_90_CLOCKWISE);
			// cv::imshow("Image", cvImg);
			// cv::waitKey();

			// Process spot's fisheye image with openPose
			const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImg);
			auto poseDatum = opWrapper.emplaceAndPop(imageToProcess);
			if(poseDatum == nullptr || poseDatum->empty()){
				std::cout << "Image could not be processed or datum is empty" << std::endl;
				continue;
			}

			// Get location of right wrist
			const auto& poseKeypoints = poseDatum->at(0)->poseKeypoints;
			if(poseKeypoints.getSize(0) < 0){
				continue;
			}
			const auto rWristX = poseKeypoints[{0, TRACKED_KEYPOINT_INDEX, 0}];
			const auto rWristY = poseKeypoints[{0, TRACKED_KEYPOINT_INDEX, 1}];
			if(rWristX == 0 || rWristY == 0){
				continue;
			}
			std::cout << "Right Wrist X: " << rWristX << std::endl;
			std::cout << "Right Wrist Y: " << rWristY << std::endl;
			cv::Point wristPoint(rWristX, rWristY);

			// Determine error between position of target point and center of image
			int imgRows = spotImages.image_responses(0).shot().image().rows();
			int imgCols = spotImages.image_responses(0).shot().image().cols();
			int errX = rWristX - (imgRows / 2); // positive error means target is to the right of image frame
			int errY = rWristY - (imgCols / 2); // positive error means target is to the bottom of image frame
			double pErrX = ((double)abs(errX)) / (imgRows / 2);
			double pErrY = ((double)abs(errY)) / (imgCols / 2);

			// Determine spot's current pitch and yaw
			FrameTree ft(spotImages.image_responses(0).shot().transforms_snapshot());
			ft.addEdge(vision_tf_bodyInitial, "vision", "orientation");
			Math::SE3Pose orientation_tf_body = ft.a_tf_b("orientation", "body");
			Math::Vector3 orientationRPY = quatToRPY(orientation_tf_body.quat());
			double curPitch = orientationRPY.y();
			double curYaw = orientationRPY.z();

			// Determine what pitch and yaw spot should aim for
			double targetPitch = 0; bool adjustPitch = false;
			double targetYaw = 0; bool adjustYaw = false;
			if(pErrX > ERROR_THRESHOLD){
				adjustPitch = true;
				targetYaw = 0.785398; // 45 degrees
				if (errX > 0){
					targetYaw *= -1; // Spin clockwise, negative yaw
				}
			}
			else{
				targetPitch = curPitch;
			}
			if(pErrY > ERROR_THRESHOLD){
				adjustYaw = true;
				targetPitch = 0.785398; // 45 degrees
				if (errY < 0){
					targetPitch *= -1; // Tilt up, negative pitch
				}
			}
			else{
				targetYaw = curYaw;
			}

			// Make spot move toward target pitch and yaw
			if(adjustPitch || adjustYaw){
				Trajectory3D trajPose;
				trajPose.addPointRPY(0, 0, 0, 0, targetPitch, targetYaw, 2000); // May be necessary to decrease this time value the closer spot gets to the target
				spot.setBodyPose(trajPose, true);
				spot.stand();
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

int openPoseTest(Spot spot)
{
	try
	{
		// Configuring OpenPose
		op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};

		// Starting OpenPose
		opWrapper.start();

		// Values that will be adjusted to change spot's position
		double spotPitch = 0;
		double spotYaw = 0;

		// Main Loop that adjusts spot's position to look at the tracked keypoint
		while(true){
			// Get images from spot
			std::vector<bosdyn::api::ImageRequest> imageRequests;
			ImageRequest request;
			request.set_image_source_name("frontright_fisheye_image");
			request.set_quality_percent(100.0);
			request.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
			imageRequests.push_back(request);
			request.set_image_source_name("frontright_depth_in_visual_frame");
			request.set_quality_percent(100.0);
			request.set_image_format(bosdyn::api::Image_Format_FORMAT_RAW);
			imageRequests.push_back(request);

			GetImageResponse spotImages = spot.getImageClient()->getImage(imageRequests);

			// std::cout << spotImages.image_responses(1).shot().image().rows() << std::endl; // 240
			// std::cout << spotImages.image_responses(1).shot().image().cols() << std::endl; // 424
			// #define X_TEST 260
			// #define Y_TEST 164

			// Create a cv mat with the fisheye image from spot
			std::string imgString = spotImages.image_responses(0).shot().image().data();
			unsigned char dataBuffer[imgString.length()];
			memcpy(dataBuffer, imgString.data(), imgString.length());
			cv::Mat rawCVImg = cv::Mat(imgString.length(), 1, CV_8U, dataBuffer).clone();
			cv::Mat cvImg = rawCVImg.reshape(0, spotImages.image_responses(0).shot().image().rows());
			cvtColor(cvImg, cvImg, CV_GRAY2BGR);
			cv::rotate(cvImg, cvImg, cv::ROTATE_90_CLOCKWISE);

			imgString = spotImages.image_responses(1).shot().image().data();
			unsigned char dataBufferDepth[imgString.length()];
			memcpy(dataBufferDepth, imgString.data(), imgString.length());
			unsigned short depthArrFlat[imgString.length()/2];
			for(int i = 0; i < imgString.length()/2; i++){
				depthArrFlat[i] = (((unsigned short)dataBufferDepth[i*2+1]) << 8) | (0x00ff & dataBufferDepth[i*2]);
			}

			// Create an array with the values from spot's depth camera image
			// TODO:: This doesn't work
			int depthRows = spotImages.image_responses(1).shot().image().rows();
			int depthCols = spotImages.image_responses(1).shot().image().cols();
			unsigned short depthArr[depthRows][depthCols];
			int depthPixel = 0;
			for(int r = 0; r < depthRows; r++){
				for(int c = 0; c < depthCols; c++){
					int depthVal = (((unsigned short)dataBufferDepth[depthPixel*2+1]) << 8) | (0x00ff & dataBufferDepth[depthPixel*2]);
					depthArr[r][c] = depthVal;
					depthPixel++;
				}
			}

			cv::Mat rawCVImg2 = cv::Mat(imgString.length() / 2, 1, CV_16U, depthArrFlat).clone();
			cv::Mat cvImg2 = rawCVImg2.reshape(0, spotImages.image_responses(1).shot().image().rows());
			cv::rotate(cvImg2, cvImg2, cv::ROTATE_90_CLOCKWISE);
			cv::Mat normedDepthImg(spotImages.image_responses(1).shot().image().rows(), spotImages.image_responses(1).shot().image().cols(), CV_8U);
			cv::normalize(cvImg2, normedDepthImg, 255, 0, cv::NORM_MINMAX, CV_8U);

			// std::cout << cvImg2.at<unsigned short>(cv::Point(X_TEST, Y_TEST)) << std::endl;
			// std::cout << depthArr[240][320] << std::endl;

			// cv::imshow("Image", cvImg);
			// //cv::imwrite("img.jpg", cvImg);
			// cv::imshow("Depth Image", normedDepthImg);
			// cv::waitKey();

			// Process spot's fisheye image with openPose
			const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImg);
			auto poseDatum = opWrapper.emplaceAndPop(imageToProcess);
			if(poseDatum == nullptr || poseDatum->empty()){
				std::cout << "Image could not be processed or datum is empty" << std::endl;
				continue;
			}

			// Get location of right wrist
			const auto& poseKeypoints = poseDatum->at(0)->poseKeypoints;
			if(poseKeypoints.getSize(0) < 0){
				continue;
			}
			const auto rWristX = poseKeypoints[{0, TRACKED_KEYPOINT_INDEX, 0}];
			const auto rWristY = poseKeypoints[{0, TRACKED_KEYPOINT_INDEX, 1}];
			if(rWristX == 0 || rWristY == 0){
				continue;
			}
			std::cout << "Right Wrist X: " << rWristX << std::endl;
			std::cout << "Right Wrist Y: " << rWristY << std::endl;
			cv::Point wristPoint(rWristX, rWristY);
			
			// Get 3D coordinates of right wrist
			unsigned short rWristDepth = cvImg2.at<unsigned short>(wristPoint);
			if(rWristDepth <= 0 || rWristDepth > 4500){
				continue;
			}
			std::cout << "Depth: " << rWristDepth << std::endl;
			double fx = spotImages.image_responses(1).source().pinhole().intrinsics().focal_length().x();
			double fy = spotImages.image_responses(1).source().pinhole().intrinsics().focal_length().y();
			double cx = spotImages.image_responses(1).source().pinhole().intrinsics().principal_point().x();
			double cy = spotImages.image_responses(1).source().pinhole().intrinsics().principal_point().y();
			double point3dX = ((wristPoint.x - cx) * (double)cvImg2.at<unsigned short>(wristPoint)) / fx;
			double point3dY = ((wristPoint.y - cy) * (double)cvImg2.at<unsigned short>(wristPoint)) / fy;
			double point3dZ = cvImg2.at<unsigned short>(wristPoint);
			// double point3dX = ((X_TEST - cx) * (double)cvImg2.at<unsigned short>(cv::Point(X_TEST, Y_TEST))) / fx; // TODO:: Unhardcode these test values
			// double point3dY = ((Y_TEST - cy) * (double)cvImg2.at<unsigned short>(cv::Point(X_TEST, Y_TEST))) / fy; // TODO:: Unhardcode these test values
			// double point3dZ = cvImg2.at<unsigned short>(cv::Point(X_TEST, Y_TEST));
			
			point3dX /= 1000;
			point3dY /= 1000;
			point3dZ /= 1000;
			std::cout << "3D X: " << point3dX << std::endl;
			std::cout << "3D Y: " << point3dY << std::endl;
			std::cout << "3D Z: " << point3dZ << std::endl;

			// Get transform between center of robot body and right wrist
			FrameTree ft(spotImages.image_responses(1).shot().transforms_snapshot());
			Math::SE3Pose frontright_tf_rwrist(point3dX, point3dY, point3dZ, Math::Quaternion(0, 0, 0, 1));
			ft.addEdge(frontright_tf_rwrist, "frontright", "rwrist");
			Math::SE3Pose body_tf_rwrist = ft.a_tf_b("body", "rwrist");
			std::cout << "Transform X: " << body_tf_rwrist.x() << std::endl;
			std::cout << "Transform Y: " << body_tf_rwrist.y() << std::endl;
			std::cout << "Transform Z: " << body_tf_rwrist.z() << std::endl;
			
			Math::SE3Pose vision_tf_body = ft.a_tf_b("vision", "body");
			// roll (x-axis rotation)
			Math::Quaternion q = vision_tf_body.quat();
			double rollV, pitchV, yawV;
			double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
			double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
			rollV = std::atan2(sinr_cosp, cosr_cosp);
			// pitch (y-axis rotation)
			double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
			if (std::abs(sinp) >= 1)
				pitchV = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
			else
				pitchV = std::asin(sinp);
			// yaw (z-axis rotation)
			double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
			double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
			yawV = std::atan2(siny_cosp, cosy_cosp);

			
			std::cout << "Vision Roll: " << rollV << std::endl;
			std::cout << "Vision Pitch: " << pitchV << std::endl;
			std::cout << "Vision Yaw: " << yawV << std::endl;
		
			

			// Get the change in pitch and yaw required to get the robot to face the right wrist
			double deltaPitch = -atan2(body_tf_rwrist.z(), body_tf_rwrist.x());
			double deltaYaw = atan2(body_tf_rwrist.y(), body_tf_rwrist.x());
			spotPitch += deltaPitch;
			spotYaw += deltaYaw;

			// Move spot into the desired orientation to face the tracked keypoint
			Trajectory3D trajPose;
			trajPose.addPointRPY(0, 0, 0, 0, spotPitch, spotYaw, 1);
			spot.setBodyPose(trajPose, true);
			spot.stand();
			std::cout << "Delta Pitch: " << deltaPitch << std::endl;
			std::cout << "Delta Yaw: " << deltaYaw << std::endl;
			std::cout << "Pitch: " << spotPitch << std::endl;
			std::cout << "Yaw: " << spotYaw << std::endl;
			std::cout << std::endl;
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
	spot.initClients();
	spot.initBasicEstop();
	spot.initBasicLease();
	spot.initBasicTimeSync();
	//spot.powerOnBlocking();
	//spot.basicInit(username, password);
	std::cout << "Spot initialization complete" << std::endl;
	Trajectory3D trajPose;
	trajPose.addPointRPY(0, 0, 0, 0, 0, 0, 1);
	spot.setBodyPose(trajPose, true);
	spot.stand();

	return openPoseTest(spot);
}