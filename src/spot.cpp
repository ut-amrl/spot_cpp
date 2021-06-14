#include <spot/spot.h>

#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>
#include <assert.h>

// // print_robot_id(): print robot id information
// void print_robot_id(RobotIdClient &client) {
// 	RobotIdResponse reply = client.getId();
// 	std::cout << "Robot Id Information" << std::endl;
// 	std::cout << "Species: " << reply.robot_id().species() << std::endl;
// 	std::cout << "Version: " << reply.robot_id().version() << std::endl;
// 	std::cout << "Serial Number: " << reply.robot_id().computer_serial_number() << "\n" << std::endl;
// }

// // print_auth_token(): print auth token information
// std::string print_auth_token(AuthClient &client, std::string username, std::string password) {
// 	GetAuthTokenResponse reply = client.auth(username, password);
// 	std::cout << "Authentication Information" << std::endl;
// 	std::cout << "Status: " << reply.status() << std::endl;
// 	std::cout << "Token: " << reply.token() << "\n" << std::endl;
// 	return reply.token();
// }

// void estop(EstopClient &client){
// 	// Estop Configuration
// 	EstopConfig estopConfig;
// 	EstopEndpoint* endpoint = estopConfig.add_endpoints();
// 	endpoint->set_role("PDB_rooted");
// 	endpoint->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));

// 	SetEstopConfigResponse estopConfResp = client.setConfig(estopConfig);
// 	if(estopConfResp.status() == 2){
// 		estopConfResp = client.setConfig(estopConfig, estopConfResp.active_config().unique_id());
// 	}
// 	std::string activeConfigId = estopConfResp.active_config().unique_id();
// 	std::cout << "Config Status: " << estopConfResp.status() << std::endl;
// 	std::cout << "Active config ID: " << estopConfResp.active_config().unique_id() << std::endl;


// 	// // Estop Registration
// 	EstopEndpoint newEndpoint;
// 	newEndpoint.set_role("PDB_rooted");
// 	newEndpoint.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
// 	EstopEndpoint targetEndpoint;
// 	targetEndpoint.set_role("PDB_rooted");
// 	RegisterEstopEndpointResponse regEndResp = client.registerEndpoint(activeConfigId, targetEndpoint, newEndpoint);
// 	std::cout << "Registration Status: " << regEndResp.status() << std::endl;
// 	std::cout << "Active endpoint ID: " << regEndResp.new_endpoint().unique_id() << std::endl;
// 	EstopEndpoint activeEndpoint = regEndResp.new_endpoint();

// 	// // Estop Check-in
// 	EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;
// 	EstopCheckInResponse checkInResp = client.checkIn(stopLevel, activeEndpoint, 0, 0, false);
// 	std::cout << "Check In Status " << checkInResp.status() << std::endl;
// 	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
// 	checkInResp = client.checkIn(stopLevel, activeEndpoint, checkInResp.challenge(), ~checkInResp.challenge(), false);
// 	std::cout << "Check In Status " << checkInResp.status() << std::endl;
// 	std::cout << "Challenge: " << checkInResp.challenge() << std::endl;
// }

// main function for running Spot clients
int main(int argc, char *argv[]) {
	// // make sure username and password are supplied
	// assert(argc == 3);
	// std::string username = argv[1];
	// std::string password = argv[2];
	
	// // create client handler
	// ClientHandler handler;
	// handler.authorize(username, password);
	// handler.setup();

	// // do stuff
	// EstopClient estopClient = handler.getClient<EstopClient>(CLIENT_TYPES::ESTOP);
	// GetEstopSystemStatusResponse estopStatusResp = estopClient.getStatus();
	// std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;
	// estop(estopClient);
	// estopStatusResp = estopClient.getStatus();
	// std::cout << "Estop Level: " << estopStatusResp.status().stop_level() << std::endl;

	return 0;
}
