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

// main function for running Spot clients
int main(int argc, char *argv[]){
	
	GtkApplication *app;
	int status;

	app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
	g_signal_connect (app, "activate", G_CALLBACK (Display::activate), NULL);
	status = g_application_run (G_APPLICATION (app), NULL, NULL);
	g_object_unref (app);

	return status;

	std::cout<<"in main"<< std::endl;
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	// create robot
	Robot robot("spot");

	// create a display
	Display display;

	// print id information
	std::cout << robot.getId() << std::endl;

	// authenticate robot
	robot.authenticate(username, password);

	// setup robot (initialize clients)
	robot.setup(); 

	int i = 0;
	while (i < 100){
		i += 1;
		// sleep(0.1);
		robot.getImages();
		// display.runDisplay(argc, argv);
	}

	return 0;
}