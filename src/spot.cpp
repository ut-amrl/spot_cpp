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

// ExampleWindow::ExampleWindow()
// {
//   set_title("DrawingArea");
//   set_default_size(300, 200);
//   set_child(m_area);
// }

// void activate (GtkWindow* win,
//           gpointer        user_data)
// {
//   GtkWidget *window;

//   window = gtk_window_new (win);
//   gtk_window_set_title (GTK_WINDOW (window), "Window");
//   gtk_window_set_default_size (GTK_WINDOW (window), 200, 200);
//   gtk_widget_show_all (window);
// }
struct {
	cairo_surface_t *image;  
} glob;

gboolean Spot::on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data)
{      
//   do_drawing(cr);
	cr = gdk_cairo_create (gtk_widget_get_window (widget));
	// do_drawing (cr, widget);

	gfloat screen_width;
	gfloat screen_height;
	gfloat image_width;
	gfloat image_height;
	gfloat x_scaling;
	gfloat y_scaling;

	/* Display screen dimension in console  */
	screen_width = gdk_screen_get_width (gdk_screen_get_default ());
	screen_height = gdk_screen_get_height (gdk_screen_get_default ());
	g_message ("Screen width %f", screen_width);
	g_message ("Screen height %f", screen_height);

	/* Scale the loaded image to occupy the entire screen  */
	image_width = cairo_image_surface_get_width (glob.image);
	image_height = cairo_image_surface_get_height (glob.image);

	g_message ("Image width %f", image_width);
	g_message ("Image height %f", image_height);

	x_scaling = screen_width / image_width;
	y_scaling = screen_height / image_height;

	g_message ("x_scaling %f", x_scaling);
	g_message ("y_scaling %f", y_scaling);

	cairo_scale (cr, x_scaling, y_scaling);

	cairo_set_source_surface (cr, glob.image, 0, 0);
	cairo_paint (cr);

	cairo_destroy (cr);

 	return FALSE;
}

// main function for running Spot clients
int main(int argc, char *argv[]){
	assert(argc == 3);

	// get username and password
	const std::string username = argv[1];
	const std::string password = argv[2];

	// create robot
	Robot robot("spot");

	// // create id 
	// Display display(100, 100, 1);

	// print id information
	std::cout << robot.getId() << std::endl;

	// authenticate robot
	robot.authenticate(username, password);

	// setup robot (initialize clients)
	// robot.setup();

	// robot.getImages();

	// GtkWidget *window;
	// gtk_init(&argc, &argv);

	// window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	// gtk_widget_show(window);
	// g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);
	// gtk_main();

	GtkWidget *window;
	GtkWidget *darea;
	cairo_t *cr;

	glob.image = cairo_image_surface_create_from_png ("data.png");

	gtk_init (&argc, &argv);


	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

	darea = gtk_drawing_area_new ();
	gtk_container_add (GTK_CONTAINER (window), darea);

	g_signal_connect (G_OBJECT (darea), "draw",
		G_CALLBACK (Spot::on_draw_event), NULL);
	g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

	gtk_window_set_position (GTK_WINDOW (window), GTK_WIN_POS_CENTER);
	gtk_window_set_title (GTK_WINDOW (window), "Cairo Test");
	gtk_window_set_decorated (GTK_WINDOW (window), FALSE);
	gtk_window_fullscreen (GTK_WINDOW (window));


	gtk_widget_show_all (window);

	gtk_main ();

	cairo_surface_destroy (glob.image);



	// GtkWidget *window;
	// GtkWidget *darea;
	
	// glob.image = cairo_image_surface_create_from_png("data.jpg");

	// gtk_init(&argc, &argv);

	// window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

	// darea = gtk_drawing_area_new();
	// gtk_container_add(GTK_CONTAINER (window), darea);

	// g_signal_connect(G_OBJECT(darea), "draw", 
	// 	G_CALLBACK(Spot::on_draw_event), NULL); 
	// g_signal_connect(window, "destroy",
	// 	G_CALLBACK (gtk_main_quit), NULL);

	// gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
	// gtk_window_set_default_size(GTK_WINDOW(window), 900, 900); 
	// gtk_window_set_title(GTK_WINDOW(window), "Image");

	// gtk_widget_show_all(window);

	// gtk_main();

	// cairo_surface_destroy(glob.image);
	
	// GtkWidget *win;

	// win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	// // win = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
	
	// std::cout << "reached here" << std::endl;
	// g_signal_connect (win, "activate", G_CALLBACK (activate), NULL);
	// int status = g_application_run (G_APPLICATION (win), argc, argv);
	// g_object_unref (app);

	// GtkApplication *app;
	// app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
	// g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);

	// std::cout << "reach" << std::endl;
	// GtkWidget *widget = gtk_widget_new(GTK_TYPE_LABEL, "label", "Hello World", "xalign", 0.0, NULL);
	// std::cout << "reach 2" << std::endl;
	// display.buildWidgets(widget);
	// std::cout << "reach 3" << std::endl;
	// display.receiveFrame();
	// std::cout << "reach 4" << std::endl;
	return 0;
}
