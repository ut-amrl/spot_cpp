#include <spot/spot.h>

#include <iostream>
#include <string>
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

	GetEstopSystemStatusResponse estopStatusResp = handler.estopClient().getStatus();
	std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;
	estop(handler);
	estopStatusResp = handler.estopClient().getStatus();
	std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;

	// do rpcs
	//print_robot_id(handler);
	//std::string token = print_auth_token(handler, username, password);

	// // Robot State Test
	// RobotStateResponse stateReply = handler.robotStateClient().getRobotState();
	// std::cout << "Robot State Information" << std::endl;
	// std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;

	// Estop Configuration
	// EstopConfig estopConfig;
	// // estopConfig.set_unique_id("fresh"); // TODO:: Determine what this needs to be
	// EstopEndpoint* endpoint = estopConfig.add_endpoints();
	// endpoint->set_role("PDB_rooted");
	// endpoint->set_name("base-PDB_rooted"); // TODO:: This may be wrong
	// endpoint->set_unique_id("enpoint-id"); // TODO:: Check if this is necessary
	// endpoint->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
	// SetEstopConfigResponse estopConfResp = handler.estopClient().setConfig(estopConfig, "5");
	// std::cout << estopConfResp.status() << std::endl;
	// std::cout << "Active config ID: " << estopConfResp.active_config().unique_id() << std::endl;
	// std::cout << estopConfResp.active_config().endpoints(0).unique_id() << std::endl;

	// // // Estop Registration
	// // // TODO:: Determine if necessary to fill out unique_id of endpoint
	// // // TODO:: Determine if neccessary to fill out target endpoint
	// EstopEndpoint newEndpoint;
	// newEndpoint.set_role("PDB_rooted");
	// newEndpoint.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
	// EstopEndpoint targetEndpoint;
	// targetEndpoint.set_role("PDB_rooted");
	// targetEndpoint.set_unique_id("3");
	// RegisterEstopEndpointResponse regEndResp = handler.estopClient().registerEndpoint("6", targetEndpoint, newEndpoint);
	// std::cout << regEndResp.status() << std::endl;
	// std::cout << "Active endpoint ID: " << regEndResp.new_endpoint().unique_id() << std::endl;
	// EstopEndpoint activeEndpoint = regEndResp.new_endpoint();

	// // // Estop Check-in
	// EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;
	// // TODO:: Determine if necessary to fill out unique_id of endpoint
	// // TODO:: Make it so that challenge doesn't need to be set
	// // TODO:: Make it so that response doesn't need to be set
	// EstopCheckInResponse checkInResp = handler.estopClient().checkIn(stopLevel, activeEndpoint, 0, 0, false);
	// std::cout << "Check In Status " << checkInResp.status() << std::endl;
	// std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
	// //handler.estopClient().checkIn(stopLevel, *endpoint, 0, 0, false);

	return 0;
}
