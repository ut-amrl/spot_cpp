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
#include <thread>

void Spot::startDisplay(){
	GtkApplication *app;
	int status;

	app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
	g_signal_connect (app, "activate", G_CALLBACK (Display::activate), NULL);
	status = g_application_run (G_APPLICATION (app), NULL, NULL);
	g_object_unref (app);
}

void Spot::mainThread(int argc, char *argv[]){
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

	// GtkApplication *app;
	// int status;

	// app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
	// g_signal_connect (app, "activate", G_CALLBACK (Display::activate), NULL);
	// status = g_application_run (G_APPLICATION (app), NULL, NULL);
	// g_object_unref (app);

	// robot.getImages();
	robot.getWorldObject();
	while (true){
		robot.getImages();
		display.refresh();
		// display.runDisplay(argc, argv);
	}
}

// main function for running Spot clients
int main(int argc, char *argv[]){
	std::thread first (Spot::startDisplay);     
  	std::thread second (Spot::mainThread, argc, argv);

	first.join();                
	second.join(); 

	return 0;
}