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
void print_auth_token(ClientHandler &handler, std::string username, std::string password) {
	GetAuthTokenResponse reply = handler.authClient().authAsync(username, password);
	std::cout << "Authentication Information" << std::endl;
	std::cout << "Status: " << reply.status() << std::endl;
	std::cout << "Token: " << reply.token() << "\n" << std::endl;
}

// main function for running Spot clients
int main(int argc, char *argv[]) {
	// make sure username and password are supplied
	assert(argc == 3);
	std::string username = argv[1];
	std::string password = argv[2];
	
	// read in the root certificate
	std::string cert = read_file("../src/resources/robot.pem");
	std::cout << "cert: " << cert << std::endl;

	// create client handler
	std::string host = DEFAULT_SERVER_ADDRESS + ":" + DEFAULT_SECURE_PORT;
	ClientHandler handler(host, cert);

	// do rpcs
	print_robot_id(handler);
	print_auth_token(handler, username, password);
	return 0;
}
