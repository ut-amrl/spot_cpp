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
	std::cout << spotbase.getRobotId() << std::endl;
	std::map<std::string, CoreLayer::ServiceEntry> services = spotbase.listAllServices();
	for (const auto &service : services) {
		std::cout << "name: " << service.first << std::endl;
		std::cout << "serv name: " << service.second.getName() << std::endl;
		std::cout << "auth: " << service.second.getAuthority() << std::endl;
	}

	std::cout << "beginTimesync()" << std::endl;
	// spotbase.beginTimesync();
	std::cout << "endTimesync()" << std::endl;
	// spotbase.endTimesync();

	// spotcontrol testing code
	std::cout << "baseptr" << std::endl;
	std::shared_ptr<CoreLayer::SpotBase> baseptr = std::shared_ptr<CoreLayer::SpotBase>(&spotbase);
	
	std::cout << "spotcontrol" << std::endl;
	RobotLayer::SpotControl spotcontrol(baseptr);

	std::cout << "endpoint" << std::endl;
	std::shared_ptr<SpotEstopEndpoint> endpoint = std::shared_ptr<SpotEstopEndpoint>(new SpotEstopEndpoint(spotcontrol.getEstopClient(), "pdb_root", "PDB_rooted", "", "", 30, 3));
	
	std::cout << "spotestopendpoint" << std::endl;
	std::set<std::shared_ptr<SpotEstopEndpoint>> endpoints;
	
	std::cout << "endpoints.insert()" << std::endl;
	endpoints.insert(endpoint);
	
	std::cout << "setestopconfiguration()" << std::endl;
	spotcontrol.setEstopConfiguration(endpoints, "");

	std::cout << "configId: " << spotcontrol.getEstopConfigId() << std::endl;

	std::cout << "registerestopendpoint()" << std::endl;
	std::string unique_id = spotcontrol.registerEstopEndpoint("pdb_root", "PDB_rooted", spotcontrol.getEstopConfigId(), 30, 3);
	
	std::cout << "beignEstoppin()" << std::endl;
	spotcontrol.beginEstopping(unique_id); // needs unique id

	std::cout << "acquireLease()" << std::endl;
	spotcontrol.acquireLease("body");

	std::cout << "beginLeasing()" << std::endl;
	spotcontrol.beginLeasing();
	
	spotcontrol.powerOnMotors();
	spotcontrol.powerOffMotors();
	
	std::cout << "endEstopping()" << std::endl;
	spotcontrol.endEstopping();
	spotcontrol.endLeasing();



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
