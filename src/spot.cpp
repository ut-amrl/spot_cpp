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

	// do rpcs
	//print_robot_id(handler);
	//std::string token = print_auth_token(handler, username, password);
	std::string token = "DUMMY_TOKEN";

	// Robot State Test
	RobotStateClient robStateClient2(token, cert, host);
	RobotStateResponse stateReply = robStateClient2.getRobotState(token);
	std::cout << "Robot State Information" << std::endl;
	std::cout << "Motor Power State: " << stateReply.robot_state().power_state().motor_power_state() << std::endl;

	// // Estop Configuration
	// EstopConfig estopConfig;
	// estopConfig.set_unique_id("base-estop-config"); // TODO:: Determine what this needs to be
	// EstopEndpoint* endpoint = estopConfig.add_endpoints();
	// endpoint->set_role("PDB_rooted");
	// endpoint->set_name("base-PDB_rooted"); // TODO:: This may be wrong
	// // endpoint.set_unique_id(""); // TODO:: Check if this is necessary
	// endpoint->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
	// handler.estopClient().setConfig(estopConfig, ""); // TODO:: Check if second argument should be filled in

	// // Estop Registration
	// // TODO:: Determine if necessary to fill out unique_id of endpoint
	// // TODO:: Determine if neccessary to fill out target endpoint
	// handler.estopClient().registerEndpoint("base-estop-config", *endpoint);

	// // Estop Check-in
	// EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;
	// // TODO:: Determine if necessary to fill out unique_id of endpoint
	// // TODO:: Make it so that challenge doesn't need to be set
	// // TODO:: Make it so that response doesn't need to be set
	// handler.estopClient().checkIn(stopLevel, *endpoint, 0, 0, false);

	return 0;
}
