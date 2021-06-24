#include <spot/spot.h>

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <streambuf>
#include <assert.h>
#include <map>

#include <functional>
#include <sys/ioctl.h> //For FIONREAD.
#include <termios.h>

//Call this at program start to setup for kbhit.
void initTerminalInput()
{
	//Disable internal buffering.
	std::wcout << std::unitbuf;

	//Turn off line buffering.
	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag &= ~ICANON;
	tcsetattr(0, TCSANOW, &term);
	setbuf(stdin, NULL);
}

//Returns 0 if there's no input character to read.
int kbhit()
{
	static int nbbytes;
	ioctl(0, FIONREAD, &nbbytes);
	return nbbytes;
}

static wchar_t getWCharClean()
{
  static wchar_t inputWChar;
  do
  {
    //Wait until there's an input character.
    while (!kbhit())
    {
    }
    inputWChar = getwchar();
	//Erase the valid character.
	std::wcout << L"\b \b";
    break;
  } while (true);
  return inputWChar;
}


// main function for running Spot clients
int main(int argc, char *argv[]) {
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	// create robot
	Robot robot("spot");

	// print id information
	std::cout << robot.getId() << std::endl;

	// authenticate robot
	robot.authenticate(username, password);

	// setup robot (initialize clients)
	robot.setup();

	// create estop and lease threads
	robot.initBasicEstop();
	std::cout << "Estop initialized" << std::endl;
	robot.initBasicLease();
	std::cout << "Lease initialized" << std::endl;
	robot.performTimesync();
	std::cout << "Timesync initialized" << std::endl;

	// Spot check joint calibration and camera check
	// std::cout << "Starting joint calibration and camera check..." << std::endl;
	// robot.getSpotCheckClientPtr()->spotCheckCommand(robot.getLease(), bosdyn::api::spot::SpotCheckCommandRequest_Command_COMMAND_START);
	// SpotCheckFeedbackResponse checkFback = robot.getSpotCheckClientPtr()->spotCheckFeedback();
	// while(checkFback.state() != 7 && checkFback.state() != 10){
	// 	checkFback = robot.getSpotCheckClientPtr()->spotCheckFeedback();
	// 	std::cout << "Spot Check State: " << checkFback.state() << std::endl;
	// 	std::cout << "Spot Check Progress: " << checkFback.progress() << std::endl << std::endl;
	// 	if(checkFback.state() == 9){
	// 		std::cout << "Spot Check Error: " << checkFback.error() << std::endl;
	// 		break;
	// 	}
	// 	sleep(1);
	// }

	// checkFback = robot.getSpotCheckClientPtr()->spotCheckFeedback();

	// // Camera check results
	// std::cout << "Camera Results:" << std::endl;
	// for (google::protobuf::Map<std::string, DepthPlaneSpotCheckResult>::const_iterator it=checkFback.camera_results().begin(); it!=checkFback.camera_results().end(); ++it){
    // 	std::cout << "Camera: " << it->first << std::endl;
	// 	std::cout << "Camera Status: " << it->second.status() << std::endl;
	// 	std::cout << "Camera Severity: " << it->second.severity_score() << std::endl << std::endl;
	// }
	// std::cout << std::endl;

	// // Load cell results
	// std::cout << "Load Cell Results:" << std::endl;
	// for (google::protobuf::Map<std::string, LoadCellSpotCheckResult>::const_iterator it=checkFback.load_cell_results().begin(); it!=checkFback.load_cell_results().end(); ++it){
    // 	std::cout << "Joint Location: " << it->first << std::endl;
	// 	std::cout << "Error: " << it->second.error() << std::endl;
	// 	std::cout << "Zero: " << it->second.zero() << std::endl;
	// 	std::cout << "Old Zero: " << it->second.old_zero() << std::endl << std::endl;
	// }
	// std::cout << std::endl;

	// // Kinematic Calibration results
	// std::cout << "Kinematic Calibration Results:" << std::endl;
	// for (google::protobuf::Map<std::string, JointKinematicCheckResult>::const_iterator it=checkFback.kinematic_cal_results().begin(); it!=checkFback.kinematic_cal_results().end(); ++it){
    // 	std::cout << "Joint Location: " << it->first << std::endl;
	// 	std::cout << "Error: " << it->second.error() << std::endl;
	// 	std::cout << "Health Score: " << it->second.health_score() << std::endl;
	// 	std::cout << "Offset: " << it->second.offset() << std::endl;
	// 	std::cout << "Old Offset: " << it->second.old_offset() << std::endl << std::endl;
	// }
	// std::cout << std::endl;

	// // Payload result
	// std::cout << "Payload Check Results:" << std::endl;
	// std::cout << "Error: " << checkFback.payload_result().error() << std::endl;
	// std::cout << "Extra payload: " << checkFback.payload_result().extra_payload() << std::endl << std::endl << std::endl;

	// // Hip RoM results
	// std::cout << "Hip Range of Motion Results:" << std::endl;
	// for (google::protobuf::Map<std::string, HipRangeOfMotionResult>::const_iterator it=checkFback.hip_range_of_motion_results().begin(); it!=checkFback.hip_range_of_motion_results().end(); ++it){
    // 	std::cout << "Leg: " << it->first << std::endl;
	// 	std::cout << "Error: " << it->second.error() << std::endl;
	// 	for(int j = 0; j < it->second.hx_size(); j++){
	// 		std::cout << "hx obstruction angle: " << it->second.hx(j) << std::endl;
	// 	}
	// 	for(int j = 0; j < it->second.hy_size(); j++){
	// 		std::cout << "hy obstruction angle: " << it->second.hy(j) << std::endl;
	// 	}
	// 	std::cout << std::endl;
	// }
	// std::cout << "Spot check results complete" << std::endl;



	// while(true){}

	// power on
	robot.powerOn();
	std::cout << "Powered on" << std::endl;

	robot.stand();
	std::cout << "standing" << std::endl;
	sleep(3);

	Trajectory3D lookPose;
	
	lookPose.addPointRPY(0, 0, 0, 0, 0.173261, 0.39726, 1);
	//Pitch: -0.173261 rad, -9.9271 deg
	//Yaw: 0.39726 rad, 22.7613 deg
	robot.setBodyPose(lookPose, true);
	robot.stand();
	sleep(10);
	std::cout << "Starting terminal control"<< std::endl;

	// Trajectory2D trajTest;
	// trajTest.addPoint(0.5, 0, 0, 2);
	// //trajTest.addPoint(1, 0, 0, 2);
	// robot.trajectoryMove(trajTest, ODOM, 10);
	// sleep(10);

	// move
	initTerminalInput();
	bool keepRunning = true;
	
	double posX = 0;
	double posY = 0;
	double posZ = 0;
	double pitch = 0;
	double roll = 0;
	double yaw = 0;

	while(keepRunning){
		wchar_t wchar = getWCharClean();
		
		// initialize velocities and angular velocity (rot)
		double velX = 0;
		double velY = 0;
		double rot = 0;

		// xy translation

		if (wchar == L'w') {
			velX += 1.0;
		}
		if (wchar == L'a') {
			velY += 1.0;
		}
		if (wchar == L's') {
			velX -= 1.0;
		}
		if (wchar == L'd') {
			velY -= 1.0;
		}
		if (wchar == L'q') {
			rot += 0.5;
		}
		if (wchar == L'e') {
			rot -= 0.5;
		}

		// orientation (once we figure out)
		if (wchar == L'i') {
			pitch += 3.14/16;
		}
		if (wchar == L'j') {
			roll += 3.14/16;
		}
		if (wchar == L'k') {
			pitch -= 3.14/16;
		}
		if (wchar == L'l') {
			roll -= 3.14/16;
		}
		if (wchar == L'u') {
			yaw -= 3.14/16;
		}
		if (wchar == L'o') {
			yaw += 3.14/16;
		}

		// height / positions ?
		if (wchar == L'r') {
			posX += 0.2;
		}
		if (wchar == L'f') {
			posX -= 0.2;
		}
		if (wchar == L't') {
			posY += 0.2;
		}
		if (wchar == L'g') {
			posY -= 0.2;
		}
		if (wchar == L'y') {
			posZ += 0.2;
		}
		if (wchar == L'h') {
			posZ -= 0.2;
		}

		// exit
		if (wchar == L'x') {
			keepRunning = false;
			// robot.move(sit);
			robot.sit();
			break;
		}

		Trajectory3D trajPose;
		trajPose.addPointRPY(posX, posY, posZ, roll, pitch, yaw, 1);
		robot.setBodyPose(trajPose, true);

		// issue move
		if (velY == 0 && velX == 0 && rot == 0){
			robot.stand();
		}
		else {
			robot.velocityMove(velX, velY, rot, 0.5, FLAT_BODY);
			// robot.move(travelVelocity, velX, velY, rot, 0.5);
		}
	
	}
	return 0;
}
