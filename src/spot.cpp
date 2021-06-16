#include <spot/spot.h>

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <streambuf>
#include <assert.h>
#include <map>

// print_robot_id(): print robot id information
void print_robot_id() {
	RobotIdClient client;
	RobotIdResponse reply = client.getId();
	std::cout << "Robot Id Information" << std::endl;
	std::cout << "Species: " << reply.robot_id().species() << std::endl;
	std::cout << "Version: " << reply.robot_id().version() << std::endl;
	std::cout << "Serial Number: " << reply.robot_id().computer_serial_number() << "\n" << std::endl;
}

void estop(EstopClient &client){
	// Estop Configuration
	EstopConfig estopConfig;
	EstopEndpoint* endpoint = estopConfig.add_endpoints();
	endpoint->set_role("PDB_rooted");
	endpoint->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));

	SetEstopConfigResponse estopConfResp = client.setConfig(estopConfig);
	if(estopConfResp.status() == 2){
		estopConfResp = client.setConfig(estopConfig, estopConfResp.active_config().unique_id());
	}
	std::string activeConfigId = estopConfResp.active_config().unique_id();
	std::cout << "Config Status: " << estopConfResp.status() << std::endl;
	std::cout << "Active config ID: " << estopConfResp.active_config().unique_id() << std::endl;

	// Estop Registration
	EstopEndpoint newEndpoint;
	newEndpoint.set_role("PDB_rooted");
	newEndpoint.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
	EstopEndpoint targetEndpoint;
	targetEndpoint.set_role("PDB_rooted");
	RegisterEstopEndpointResponse regEndResp = client.registerEndpoint(activeConfigId, targetEndpoint, newEndpoint);
	std::cout << "Registration Status: " << regEndResp.status() << std::endl;
	std::cout << "Active endpoint ID: " << regEndResp.new_endpoint().unique_id() << std::endl;
	EstopEndpoint activeEndpoint = regEndResp.new_endpoint();

	// // Estop Check-in
	EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;
	EstopCheckInResponse checkInResp = client.checkIn(stopLevel, activeEndpoint, 0, 0, false);
	std::cout << "Check In Status " << checkInResp.status() << std::endl;
	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
	checkInResp = client.checkIn(stopLevel, activeEndpoint, checkInResp.challenge(), ~checkInResp.challenge(), false);
	std::cout << "Check In Status " << checkInResp.status() << std::endl;
	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
	checkInResp = client.checkIn(stopLevel, activeEndpoint, checkInResp.challenge(), ~checkInResp.challenge(), false);
	std::cout << "Check In Status " << checkInResp.status() << std::endl;
	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
}

enum movementType {sit, stand, travel};

// RobotCommandResponse move(movementType mType, LeaseClient& leaseClient, AcquireLeaseResponse leaseResp, RetainLeaseResponse retLeaseResp, RobotCommandClient& robotCommandClient, double x, double y, double rot, double time, int64_t clockSkew, std::__cxx11::string& timeSyncClockId){
// 	RobotCommand command;
// 	Lease *lease = new Lease(leaseResp.lease());
// 	retLeaseResp = leaseClient.retainLease(lease);

// 	switch (mType){
// 		case sit:
// 			command.mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();
// 			break;
// 		case stand:
// 			command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
// 			break;
// 		case travel:
// 			SE2VelocityCommand_Request se2VelocityCommand_Request;
// 			se2VelocityCommand_Request.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + clockSkew) + time*1000000000)));
// 			se2VelocityCommand_Request.set_se2_frame_name(BODY_FRAME_NAME);
// 			se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_x(x);
// 			se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_y(y);
// 			se2VelocityCommand_Request.mutable_velocity()->set_angular(rot);
// /*
// 			SE2Velocity velocity;
// 			velocity.mutable_linear()->set_x(x);
// 			velocity.mutable_linear()->set_y(y);
// 			velocity.set_angular(rot);
// */
// //			se2VelocityCommand_Request->mutable_velocity()->copyFrom(velocity);

// 			RobotCommand command2;
// 			command2.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);
// 			break;
// 	}

// 	RobotCommandResponse robCommResp = robotCommandClient.robotCommand(leaseResp.lease(), command, timeSyncClockId);
// 	return robCommResp;
// }

// main function for running Spot clients
int main(int argc, char *argv[]) {
	assert(argc == 3);
	std::string username = argv[1];
	std::string password = argv[2];
	Robot robot("spot");
	std::cout << robot.getId() << std::endl;
	robot.authenticate(username, password);
	robot.setup();
	robot.initBasicEstop();
	robot.initBasicLease();
	sleep(5);
	std::cout << robot.getEstopClientPtr()->getStatus().status().stop_level() << std::endl;
	return 0;
}
