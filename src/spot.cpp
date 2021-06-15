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

enum movementType {sit; stand; move};

RobotCommandResponse move(movementType mType, AcquireLeaseResponse leaseResp, RetainLeaseResponse retLeaseResp, double x, double y, double rot, double time, int64_t clock_skew){
	RobotCommand command;
	lease = new Lease(leaseResp.lease());
	retLeaseResp = leaseClient.retainLease(lease);

	switch (movementType){
		case sit:
			command.mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();
			break;
		case stand:
			command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
			break;
		case move:
			SE2VelocityCommand_Request se2VelocityCommand_Request;
			se2VelocityCommand_Request.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + clockSkew) + time*1000000000)));
			se2VelocityCommand_Request.set_se2_frame_name(BODY_FRAME_NAME);
			SE2Velocity velocity;
			velocity.mutable_linear()->set_x(x);
			velocity.mutable_linear()->set_y(y);
			velocity.set_angular(0.7);
			se2VelocityCommand_Request.mutable_velocity()->CopyFrom(rot);
			RobotCommand command2;
			command2.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);
			break;
	}

	RobotCommandResponse robCommResp = robotCommandClient.robotCommand(leaseResp.lease(), command, timeSyncClockId);
}

// main function for running Spot clients
int main(int argc, char *argv[]) {
	// make sure username and password are supplied
	assert(argc == 3);
	const std::string username = argv[1];
	const std::string password = argv[2];

	// do robot id check
	print_robot_id();

	// create auth client
	AuthClient authClient;
	std::string token = authClient.authenticate(username, password, false);
	std::cout << "Authentication Information:" << std::endl;
	std::cout << "Token: " << token << std::endl;

	// create directory client
	DirectoryClient directoryClient(token);

	// estop
	EstopClient estopClient(directoryClient.getEntry(ESTOP_CLIENT_NAME).service_entry().authority(), token);
	GetEstopSystemStatusResponse estopStatusResp = estopClient.getStatus();
	std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;
	estop(estopClient);
	estopStatusResp = estopClient.getStatus();
	std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;

	// lease
	LeaseClient leaseClient(directoryClient.getEntry(LEASE_CLIENT_NAME).service_entry().authority(), token);
	AcquireLeaseResponse leaseResp = leaseClient.acquire("body");
	Lease *lease = new Lease(leaseResp.lease());
	std::cout << "Lease Status: " << leaseResp.status() << std::endl;

	RetainLeaseResponse retLeaseResp = leaseClient.retainLease(lease);
	std::cout << "Retain Lease Status: " << retLeaseResp.lease_use_result().status() << std::endl;

	// TimeSync
	TimeSyncClient timeSyncClient(directoryClient.getEntry(TIMESYNC_CLIENT_NAME).service_entry().authority(), token);
	TimeSyncUpdateResponse timeSyncResp = timeSyncClient.getTimeSyncUpdate();
	std::string timeSyncClockId = timeSyncResp.clock_identifier();
	//std::cout << "Time Sync Status: " << timeSyncResp.state().status() << std::endl;
	while(timeSyncResp.state().status() != 1){
		TimeSyncRoundTrip prevRoundTrip;
		prevRoundTrip.mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
		prevRoundTrip.mutable_client_tx()->CopyFrom(timeSyncResp.header().request_header().request_timestamp());
		prevRoundTrip.mutable_server_tx()->CopyFrom(timeSyncResp.header().response_timestamp());
		prevRoundTrip.mutable_server_rx()->CopyFrom(timeSyncResp.header().request_received_timestamp());
		timeSyncResp = timeSyncClient.getTimeSyncUpdate(prevRoundTrip, timeSyncClockId);
	}
	int64_t clockSkew = TimeUtil::DurationToNanoseconds(timeSyncResp.state().best_estimate().clock_skew());
	std::cout << "Time Sync Status: " << timeSyncResp.state().status() << std::endl;
	std::cout << "Time Sync Completed" << std::endl;

	// Power
	PowerClient powerClient(directoryClient.getEntry(POWER_CLIENT_NAME).service_entry().authority(), token);
	PowerCommandRequest_Request pcr_r;
	pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_ON; // PowerCommandRequest_Request_REQUEST_OFF to turn off, change to _ON to turn on
	PowerCommandResponse powerCommResp = powerClient.PowerCommand(leaseResp.lease(), pcr_r); 
	uint32_t pcID = powerCommResp.power_command_id();
	std::cout << "Power Command Status: " << powerCommResp.status() << std::endl;
	std::cout << "Power Command Lease Use Result: " << powerCommResp.lease_use_result().status() << std::endl;
	

	// robot state client
	RobotStateClient robotStateClient(directoryClient.getEntry(ROBOT_STATE_CLIENT_NAME).service_entry().authority(), token);

	// robot command client
	RobotCommandClient robotCommandClient(directoryClient.getEntry(ROBOT_COMMAND_CLIENT_NAME).service_entry().authority(), token);

	PowerCommandFeedbackResponse pcfr = powerClient.PowerCommandFeedback(pcID);
	while(pcfr.status() != 2){
		lease = new Lease(leaseResp.lease());
		leaseClient.retainLease(lease);
		pcfr = powerClient.PowerCommandFeedback(pcID);
		RobotStateResponse stateReply = robotStateClient.getRobotState();
		std::cout << "Robot State Information" << std::endl;
		std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;
    	sleep(1);
	}
	std::cout << "Power on!" << std::endl;

	move(stand, leaseResp, retLeaseResp, 1, 0, 0.5, 0, clock_skew);

	// RobotStateResponse stateReply = handler.robotStateClient().getRobotState();
	// while(pcfr.status() != 2){
	// 	pcfr = handler.powerClient().PowerCommandFeedback(pcID);
	// 	std::cout << "Powering on..." << std::endl;
	// 	stateReply = handler.robotStateClient().getRobotState();
	// 	std::cout << "Robot State Information" << std::endl;
	// 	std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;
	// 	sleep(1);
	// }
	// std::cout << "Power on" << std::endl;
	// std::cout << "Power Command Status: " << pcfr.status() << std::endl;

	// stateReply = handler.robotStateClient().getRobotState();
	// std::cout << "Robot State Information" << std::endl;
	// std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;


	// lease = new Lease(leaseResp.lease());
	// handler.leaseClient().returnLease(lease);
	

//	RobotStateResponse stateReply = handler.robotStateClient().getRobotState();
//	std::cout << "Robot State Information" << std::endl;
//	std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;
	

	// Robot Command - Stand
/*	
	lease = new Lease(leaseResp.lease());
	retLeaseResp = leaseClient.retainLease(lease);
	std::cout << "Retain Lease Status: " << retLeaseResp.lease_use_result().status() << std::endl;

	RobotCommand command;
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();

	RobotCommandResponse robCommResp = robotCommandClient.robotCommand(leaseResp.lease(), command, timeSyncClockId);
*/	
	

/*	// // Robot Command Velocity
	lease = new Lease(leaseResp.lease());
	retLeaseResp = leaseClient.retainLease(lease);
	std::cout << "Retain Lease Status: " << retLeaseResp.lease_use_result().status() << std::endl;

	SE2VelocityCommand_Request se2VelocityCommand_Request;
	se2VelocityCommand_Request.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + clockSkew) + 3000000000)));
	se2VelocityCommand_Request.set_se2_frame_name(BODY_FRAME_NAME);
	SE2Velocity velocity;
	velocity.mutable_linear()->set_x(0.0);
	velocity.mutable_linear()->set_y(0.0);
	velocity.set_angular(0.7);
	se2VelocityCommand_Request.mutable_velocity()->CopyFrom(velocity);
	RobotCommand command2;
	command2.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);

	RobotCommandResponse robCommResp = robotCommandClient.robotCommand(leaseResp.lease(), command2, timeSyncClockId); 
	std::cout << "Move command status: " << robCommResp.status() << std::endl;
	sleep(6);*/

// Robot Command - Sit 
	// lease = new Lease(leaseResp.lease());
	// retLeaseResp = leaseClient.retainLease(lease);
	// std::cout << "Retain Lease Status: " << retLeaseResp.lease_use_result().status() << std::endl;

	// RobotCommand commandSit;
	// commandSit.mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();
	// robCommResp = robotCommandClient.robotCommand(leaseResp.lease(), commandSit, timeSyncClockId);


	// do rpcs
	//print_robot_id(handler);
	//std::string token = print_auth_token(handler, username, password);

	// // Robot State Test
	// RobotStateResponse stateReply = handler.robotStateClient().getRobotState();
	// std::cout << "Robot State Information" << std::endl;
	// std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;

	return 0;
}

// untested 
/*	// Robot Command Stance
	retLeaseResp = leaseClient.retainLease(lease);
	std::cout << "Retain Lease Status: " << retLeaseResp.lease_use_result().status() << std::endl;
	command;
	command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stance_request()->mutable_end_time()->CopyFrom(TimeUtil::GetCurrentTime());
	command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stance_request()->mutable_stance()->set_se2_frame_name(ODOM_FRAME_NAME);

	Vec2 fl; // foot positions 
	fl.set_x(0);
	fl.set_y(0);
	Vec2 fr;
	fr.set_x(0);
	fr.set_y(0);
	Vec2 bl;
	bl.set_x(0);
	bl.set_y(0);
	Vec2 br;
	br.set_x(0);
	br.set_y(0);
	std::map<std::string, Vec2> footPositions;
	footPositions["fl"] = fl;
	footPositions["fr"] = fr;
	footPositions["bl"] = bl;
	footPositions["br"] = br;
	command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stance_request()->mutable_stance()->clear_foot_positions(); // map
	command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stance_request()->mutable_stance()->set_accuracy(0.05);
	robCommResp = robotCommandClient.robotCommand(leaseResp.lease(), command, timeSyncClockId); */