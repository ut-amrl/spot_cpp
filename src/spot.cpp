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
	Robot robot("spot", username, password);
	std::cout << robot.getId() << std::endl;
}