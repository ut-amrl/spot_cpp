#include <spot/spot.h>

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <streambuf>
#include <assert.h>

int main(int argc, char *argv[]) {
	assert(argc == 3);
	std::string username = argv[1];
	std::string password = argv[2];
	Robot robot("spot");
	std::cout << robot.getId() << std::endl;
	robot.authenticate(username, password);
	robot.setup();
	robot.initBasicEstop();
	sleep(5);
	std::cout << robot.getEstopClientPtr()->getStatus().status().stop_level() << std::endl;
	return 0;
}