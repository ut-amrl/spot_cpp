#include <spot/spot.h>

#include <iostream>

int main() {
	std::string server = "192.168.80.3";
	RobotIdClient robot_id_client(server);

	RobotIdResponse reply = robot_id_client.getId();
	std::cout << "Species: " << reply.robot_id().species() << std::endl;
	std::cout << "Version: " << reply.robot_id().version() << std::endl;
	std::cout << "Serial Number: " << reply.robot_id().computer_serial_number() << std::endl;
	return 0;
}
