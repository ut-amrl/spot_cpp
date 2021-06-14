#include <spot/spot.h>

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <streambuf>
#include <assert.h>

// print_robot_id(): print robot id information
void print_robot_id(ClientHandler &handler) {
	RobotIdResponse reply = handler.robotIdClient().getId();
	std::cout << "Robot Id Information" << std::endl;
	std::cout << "Species: " << reply.robot_id().species() << std::endl;
	std::cout << "Version: " << reply.robot_id().version() << std::endl;
	std::cout << "Serial Number: " << reply.robot_id().computer_serial_number() << "\n" << std::endl;
}

// print_auth_token(): print auth token information
std::string print_auth_token(ClientHandler &handler, std::string username, std::string password) {
	GetAuthTokenResponse reply = handler.authClient().auth(username, password);
	std::cout << "Authentication Information" << std::endl;
	std::cout << "Status: " << reply.status() << std::endl;
	std::cout << "Token: " << reply.token() << "\n" << std::endl;
	return reply.token();
}

void estop(ClientHandler &ch){
	// Estop Configuration
	EstopConfig estopConfig;
	EstopEndpoint* endpoint = estopConfig.add_endpoints();
	endpoint->set_role("PDB_rooted");
	endpoint->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));

	SetEstopConfigResponse estopConfResp = ch.estopClient().setConfig(estopConfig);
	if(estopConfResp.status() == 2){
		estopConfResp = ch.estopClient().setConfig(estopConfig, estopConfResp.active_config().unique_id());
	}
	std::string activeConfigId = estopConfResp.active_config().unique_id();
	std::cout << "Config Status: " << estopConfResp.status() << std::endl;
	std::cout << "Active config ID: " << estopConfResp.active_config().unique_id() << std::endl;


	// // Estop Registration
	EstopEndpoint newEndpoint;
	newEndpoint.set_role("PDB_rooted");
	newEndpoint.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
	EstopEndpoint targetEndpoint;
	targetEndpoint.set_role("PDB_rooted");
	RegisterEstopEndpointResponse regEndResp = ch.estopClient().registerEndpoint(activeConfigId, targetEndpoint, newEndpoint);
	std::cout << "Registration Status: " << regEndResp.status() << std::endl;
	std::cout << "Active endpoint ID: " << regEndResp.new_endpoint().unique_id() << std::endl;
	EstopEndpoint activeEndpoint = regEndResp.new_endpoint();

	// // Estop Check-in
	EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;
	EstopCheckInResponse checkInResp = ch.estopClient().checkIn(stopLevel, activeEndpoint, 0, 0, false);
	std::cout << "Check In Status " << checkInResp.status() << std::endl;
	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
	checkInResp = ch.estopClient().checkIn(stopLevel, activeEndpoint, checkInResp.challenge(), ~checkInResp.challenge(), false);
	std::cout << "Check In Status " << checkInResp.status() << std::endl;
	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
	checkInResp = ch.estopClient().checkIn(stopLevel, activeEndpoint, checkInResp.challenge(), ~checkInResp.challenge(), false);
	std::cout << "Check In Status " << checkInResp.status() << std::endl;
	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
}

void leaseCode(AcquireLeaseResponse leaseResp, ClientHandler handler){
	Lease *lease = new Lease(leaseResp.lease());
	handler.leaseClient().retainLease(lease);
}

// main function for running Spot clients
int main(int argc, char *argv[]) {
	// make sure username and password are supplied
	assert(argc == 3);
	std::string username = argv[1];
	std::string password = argv[2];
	
	// read in the root certificate
	std::string cert = read_file("../src/resources/robot.pem");

	// create client handler
	std::string host = DEFAULT_SERVER_ADDRESS + ":" + DEFAULT_SECURE_PORT;
	ClientHandler handler(host, cert);
	handler.setAuthToken(handler.authClient().auth(username, password).token());

	// Estop
	GetEstopSystemStatusResponse estopStatusResp = handler.estopClient().getStatus();
	std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;
	estop(handler);
	estopStatusResp = handler.estopClient().getStatus();
	std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;

	// Lease
	AcquireLeaseResponse leaseResp = handler.leaseClient().acquire("body");
	Lease *lease = new Lease(leaseResp.lease());
	std::cout << "Lease Status: " << leaseResp.status() << std::endl;

	RetainLeaseResponse retLeaseResp = handler.leaseClient().retainLease(lease);
	std::cout << "Retain Lease Status: " << retLeaseResp.lease_use_result().status() << std::endl;

	// // TimeSync
	TimeSyncUpdateResponse timeSyncResp = handler.timeSyncClient().getTimeSyncUpdate();
	std::string timeSyncClockId = timeSyncResp.clock_identifier();
	//std::cout << "Time Sync Status: " << timeSyncResp.state().status() << std::endl;
	while(timeSyncResp.state().status() != 1){
		TimeSyncRoundTrip prevRoundTrip;
		prevRoundTrip.mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
      	prevRoundTrip.mutable_client_tx()->CopyFrom(timeSyncResp.header().request_header().request_timestamp());
      	prevRoundTrip.mutable_server_tx()->CopyFrom(timeSyncResp.header().response_timestamp());
      	prevRoundTrip.mutable_server_rx()->CopyFrom(timeSyncResp.header().request_received_timestamp());
		timeSyncResp = handler.timeSyncClient().getTimeSyncUpdate(prevRoundTrip, timeSyncClockId);
	}
	std::cout << "Time Sync Status: " << timeSyncResp.state().status() << std::endl;
	std::cout << "Time Sync Completed" << std::endl;

	// Power
	PowerCommandRequest_Request pcr_r;
	pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_ON; // PowerCommandRequest_Request_REQUEST_OFF to turn off, change to _ON to turn on
	PowerCommandResponse powerCommResp = handler.powerClient().PowerCommand(leaseResp.lease(), pcr_r); 
	uint32_t pcID = powerCommResp.power_command_id();
	std::cout << "Power Command Status: " << powerCommResp.status() << std::endl;
	std::cout << "Power Command Lease Use Result: " << powerCommResp.lease_use_result().status() << std::endl;
	
	//std::thread maintainLease(leaseCode, leaseResp, handler);

	PowerCommandFeedbackResponse pcfr = handler.powerClient().PowerCommandFeedback(pcID);
	while(pcfr.status() != 2){
		lease = new Lease(leaseResp.lease());
		handler.leaseClient().retainLease(lease);
		pcfr = handler.powerClient().PowerCommandFeedback(pcID);
		RobotStateResponse stateReply = handler.robotStateClient().getRobotState();
		std::cout << "Robot State Information" << std::endl;
		std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;
		sleep(1);
	}
	std::cout << "Power on!" << std::endl;

	// PowerCommandFeedbackResponse pcfr = handler.powerClient().PowerCommandFeedback(pcID);
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
	

	RobotStateResponse stateReply = handler.robotStateClient().getRobotState();
	std::cout << "Robot State Information" << std::endl;
	std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;
	
	
	
	// Robot Command - Stand
	//leaseResp = handler.leaseClient().acquire("body");
	
	//std::cout << "Lease Status: " << leaseResp.status() << std::endl;
	
	lease = new Lease(leaseResp.lease());
	retLeaseResp = handler.leaseClient().retainLease(lease);
	std::cout << "Retain Lease Status: " << retLeaseResp.lease_use_result().status() << std::endl;

	RobotCommand command;
	command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
	RobotCommandResponse robCommResp = handler.robotCommandClient().robotCommand(leaseResp.lease(), command, timeSyncClockId);
	
	

	std::cout << "Command Status: " << robCommResp.status() << std::endl;


	// do rpcs
	//print_robot_id(handler);
	//std::string token = print_auth_token(handler, username, password);

	// // Robot State Test
	// RobotStateResponse stateReply = handler.robotStateClient().getRobotState();
	// std::cout << "Robot State Information" << std::endl;
	// std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;

	

	return 0;
}
