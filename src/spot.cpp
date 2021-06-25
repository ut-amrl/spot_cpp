#include <spot/spot.h>

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <streambuf>
#include <assert.h>
#include <map>
#include <set>

#include <functional>
#include <sys/ioctl.h> //For FIONREAD.
#include <termios.h>

//Call this at program start to setup for kbhit.
void initTerminalInput()
{
	//Disable internal buffering.
	std::wcout << std::unitbuf;

	//Turn off line buffering.
	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag &= ~ICANON;
	tcsetattr(0, TCSANOW, &term);
	setbuf(stdin, NULL);
}

//Returns 0 if there's no input character to read.
int kbhit()
{
	static int nbbytes;
	ioctl(0, FIONREAD, &nbbytes);
	return nbbytes;
}

static wchar_t getWCharClean()
{
  static wchar_t inputWChar;
  do
  {
    //Wait until there's an input character.
    while (!kbhit())
    {
    }
    inputWChar = getwchar();
	//Erase the valid character.
	std::wcout << L"\b \b";
    break;
  } while (true);
  return inputWChar;
}


// main function for running Spot clients
int main(int argc, char *argv[]) {
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	// spotbase testing code
	CoreLayer::SpotBase spotbase;
	spotbase.authenticate(username, password);
	spotbase.getRobotId();
	std::map<std::string, CoreLayer::ServiceEntry> services = spotbase.listAllServices();
	for (const auto &service : services) {
		std::cout << "name: " << service.first << std::endl;
		std::cout << "serv name: " << service.second.getName() << std::endl;
		std::cout << "auth: " << service.second.getAuthority() << std::endl;
	}
	spotbase.beginTimesync();
	spotbase.endTimesync();

	// spotcontrol testing code
	// RobotLayer::SpotControl spotcontrol(std::shared_ptr<CoreLayer::SpotBase>(&spotbase));
	// SpotEstopEndpoint endpoint(spotcontrol.getEstopClient(), "pdb_root", "PDB_rooted", "", "", 4, 3);
	// std::set<SpotEstopEndpoint> endpoints;
	// endpoints.insert(endpoint);
	// spotcontrol.setEstopConfiguration(endpoints, "");
	// spotcontrol.registerEstopEndpoint("pdb_root", "PDB_root", "", spotcontrol.getEstopConfigId(), 4, 3);
	
	// spotcontrol.beginEstopping();
	// spotcontrol.acquireLease("body");
	// spotcontrol.beginLeasing();
	
	// spotcontrol.powerOnMotors();
	// spotcontrol.powerOffMotors();
	
	// spotcontrol.endEstopping();
	// spotcontrol.endLeasing();



	return 0;
}

	// // move
	// initTerminalInput();
	// bool keepRunning = true;
	// while(keepRunning){
	// 	wchar_t wchar = getWCharClean();
		
	// 	// initialize velocities and angular velocity (rot)
	// 	double velX = 0;
	// 	double velY = 0;
	// 	double rot = 0;

	// 	// xy translation
	// 	if (wchar == L'w') {
	// 		velX += 1.0;
	// 	}
	// 	if (wchar == L'a') {
	// 		velY -= 1.0;
	// 	}
	// 	if (wchar == L's') {
	// 		velX -= 1.0;
	// 	}
	// 	if (wchar == L'd') {
	// 		velY += 1.0;
	// 	}

	// 	// orientation (once we figure out)
	// 	if (wchar == L'i') {

	// 	}
	// 	if (wchar == L'j') {
	// 		rot += 0.5;
	// 	}
	// 	if (wchar == L'k') {

	// 	}
	// 	if (wchar == L'l') {
	// 		rot -= 0.5;
	// 	}

	// 	// exit
	// 	if (wchar == L'e') {
	// 		keepRunning = false;
	// 		break;
	// 	}

	// 	// issue move
	// }
