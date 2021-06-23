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

void Spot::activate (GtkApplication *app, gpointer user_data) {
	// _window = gtk_application_window_new (app);
    // gtk_window_set_title (GTK_WINDOW (_window), "SPOT");
    // gtk_window_set_default_size (GTK_WINDOW (_window), 400, 400);

    // ((Display *)user_data)->buildWidgets(_window);

    // g_signal_connect(_window, "destroy", G_CALLBACK(gtk_widget_destroy), NULL);
    // gtk_widget_show_all (_window);
}

// main function for running Spot clients
int main(int argc, char *argv[]){
	std::cout<<"in main"<< std::endl;
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	// create robot
	Robot robot("spot");

	// create a display

	// print id information
	std::cout << robot.getId() << std::endl;

	// authenticate robot
	robot.authenticate(username, password);

	// setup robot (initialize clients)
	robot.setup(); 

	int i = 0;
	Display ds();
	// KFRDisplay kd(mkc._rowsC, mkc._colsC, 4, aprilTags, &atp);
	GtkApplication _app = gtk_application_new ("SPOT", G_APPLICATION_FLAGS_NONE);
	
	static GtkWidget *_window = gtk_application_window_new (_app);
	
	gtk_window_set_title (GTK_WINDOW (_window), "SPOT");
    gtk_window_set_default_size (GTK_WINDOW (_window), 400, 400);

    ((Display *)this)->buildWidgets(_window);

    g_signal_connect(_window, "destroy", G_CALLBACK(gtk_widget_destroy), NULL);
    gtk_widget_show_all (_window);

	// g_signal_connect (_app, "activate", G_CALLBACK (Spot::activate), NULL);
	int status = g_application_run (G_APPLICATION (_app), argc, argv);
	g_object_unref (_app);

	while (i < 100){
		i += 1;
		robot.getImages();
		// std::cout << "sleeping" << std::endl;
		// sleep(1);
		// std::cout << "done sleeping" << std::endl;
		// {
		// 	// Display display;
		// 	Display::runDisplay(argc, argv);
		// }
		// std::cout << "start loop again" << std::endl;
	}

	return 0;
}