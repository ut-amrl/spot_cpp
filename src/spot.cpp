#include <spot/spot.h>

#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>

std::string read(const std::string& filename, std::string& data) {
  std::ifstream file(filename);
  std::string str((std::istreambuf_iterator<char>(file)),
  	std::istreambuf_iterator<char>());
//   if (file.is_open()) {
//     std::stringstream ss;
//     ss << file.rdbuf();
//     file.close();
//     data = ss.str();
//   }
//   else{
// 	  std::cout << "Problem opening file" << std::endl;
//   }
  return str;
}

int main(int argc, char** argv) {
	// get root
	std::string root;
	root = read("../src/resources/robot.pem", root);
	std::cout << root << std::endl;

	// get server and create client
	ClientHandler ch("192.168.80.3:443", root);
	assert(argc >= 3);
	ch.robotIdClient().getId();
	GetAuthTokenResponse authTokenResponse = ch.authClient().auth(argv[1], argv[2]);

	return 0;
}

