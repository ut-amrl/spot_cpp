#include <spot/spot.h>

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <streambuf>
#include <assert.h>
#include <map>

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

	// create robot
	Robot robot("spot");

	// print id information
	std::cout << robot.getId() << std::endl;

	// authenticate robot
	robot.authenticate(username, password);

	// setup robot (initialize clients)
	robot.setup();
	

	// create estop and lease threads
	robot.initBasicEstop();
	std::cout << "Estop initialized" << std::endl;
	robot.initBasicLease();
	std::cout << "Lease initialized" << std::endl;
	robot.initBasicTimesync();
	std::cout << "Timesync initialized" << std::endl;
	
	// power on
	robot.powerOn();
	std::cout << "Powered on" << std::endl;

	robot.stand();
	std::cout << "standing" << std::endl;
	sleep(3);

	// if(robot.trajectoryMove(1,0,0,5))
	// 	std::cout << "Command succeeded" << std::endl;
	// sleep(10);

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
			robot.sit();
			break;
		}

		//robot.setBodyPose(roll, pitch, yaw, posX, posY, posZ, true);

		// issue move
		if (velY == 0 && velX == 0 && rot == 0){
			// robot.stand(posX, posY, posZ, pitch, yaw, roll);
			robot.stand();
		}
		else {
			robot.velocityMove(velX, velY, rot, 0.5, FLAT_BODY);
			// robot.move(travelVelocity, velX, velY, rot, 0.5);
		}
	
	}
	return 0;
}
