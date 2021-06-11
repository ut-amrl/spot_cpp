#include <spot/spot.h>

#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>

void read(const std::string& filename, std::string& data) {
  std::ifstream file(filename.c_str(), std::ios::in);
  if (file.is_open()) {
    std::stringstream ss;
    ss << file.rdbuf();
    file.close();
    data = ss.str();
  }
  return;
}

int main() {
	// get root
	std::string root;
	read("../include/robot.pem", root);
	std::ifstream pem("../include/robot.pem");
	std::string str(
		(std::istreambuf_iterator<char>(pem)),
		std::istreambuf_iterator<char>()
	);

	std::cout << root << std::endl;


	// get server and create client
	std::string server = "192.168.80.3:443";
	RobotIdClient robot_id_client(root, server);

	RobotIdResponse reply = robot_id_client.getId();
	std::cout << "Species: " << reply.robot_id().species() << std::endl;
	std::cout << "Version: " << reply.robot_id().version() << std::endl;
	std::cout << "Serial Number: " << reply.robot_id().computer_serial_number() << std::endl;
	return 0;
}

