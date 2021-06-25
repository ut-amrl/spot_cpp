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
	std::shared_ptr<CoreLayer::SpotBase> spotbase(new CoreLayer::SpotBase());
	spotbase->authenticate(username, password);
	std::cout << spotbase->getRobotId() << std::endl;
	spotbase->beginTimesync();

	// spotcontrol testing code	
	RobotLayer::SpotControl spotcontrol(spotbase);

	std::shared_ptr<SpotEstopEndpoint> endpoint = std::shared_ptr<SpotEstopEndpoint>(new SpotEstopEndpoint(spotcontrol.getEstopClient(), "pdb_root", "PDB_rooted", "", "", 30, 3));
	
	std::set<std::shared_ptr<SpotEstopEndpoint>> endpoints;
	
	endpoints.insert(endpoint);
	
	spotcontrol.setEstopConfiguration(endpoints, "");

	std::string unique_id = spotcontrol.registerEstopEndpoint("pdb_root", "PDB_rooted", spotcontrol.getEstopConfigId(), 30, 3);

	spotcontrol.beginEstopping(unique_id); // needs unique id

	spotcontrol.acquireLease("body");

	spotcontrol.beginLeasing();

	spotcontrol.powerOnMotors();	
	
	// spotcontrol.endEstopping(unique_id);
	
	// spotcontrol.endLeasing();

	// spotbase->endTimesync();

	// Trajectory2D trajTest;
	// trajTest.addPoint(0.5, 0, 0, 2);
	// //trajTest.addPoint(1, 0, 0, 2);
	// robot.trajectoryMove(trajTest, ODOM, 10);
	// sleep(10);
	spotcontrol.stand();

	// move
	initTerminalInput();
	bool keepRunning = true;
	
	double posX = 0;
	double posY = 0;
	double posZ = 0;
	double pitch = 0;
	double roll = 0;
	double yaw = 0;

	while(keepRunning){
		wchar_t wchar = getWCharClean();
		
		// initialize velocities and angular velocity (rot)
		double velX = 0;
		double velY = 0;
		double rot = 0;

		// xy translation

		if (wchar == L'w') {
			velX += 1.0;
		}
		if (wchar == L'a') {
			velY += 1.0;
		}
		if (wchar == L's') {
			velX -= 1.0;
		}
		if (wchar == L'd') {
			velY -= 1.0;
		}
		if (wchar == L'q') {
			rot += 0.5;
		}
		if (wchar == L'e') {
			rot -= 0.5;
		}

		// orientation (once we figure out)
		if (wchar == L'i') {
			pitch += 3.14/16;
		}
		if (wchar == L'j') {
			roll += 3.14/16;
		}
		if (wchar == L'k') {
			pitch -= 3.14/16;
		}
		if (wchar == L'l') {
			roll -= 3.14/16;
		}
		if (wchar == L'u') {
			yaw -= 3.14/16;
		}
		if (wchar == L'o') {
			yaw += 3.14/16;
		}

		// height / positions ?
		if (wchar == L'r') {
			posX += 0.2;
		}
		if (wchar == L'f') {
			posX -= 0.2;
		}
		if (wchar == L't') {
			posY += 0.2;
		}
		if (wchar == L'g') {
			posY -= 0.2;
		}
		if (wchar == L'y') {
			posZ += 0.2;
		}
		if (wchar == L'h') {
			posZ -= 0.2;
		}

		// exit
		if (wchar == L'x') {
			keepRunning = false;
			// robot.move(sit);
			spotcontrol.sit();
			break;
		}

		// Trajectory3D trajPose;
		// trajPose.addPointRPY(posX, posY, posZ, roll, pitch, yaw, 1);
		// robot.setBodyPose(trajPose, true);

		// issue move
		if (velY == 0 && velX == 0 && rot == 0){
			spotcontrol.stand();
		}
		else {
			spotcontrol.velocityMove(velX, velY, rot, 5, FLAT_BODY);
		}
	
	}
	return 0;
}