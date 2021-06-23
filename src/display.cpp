#include <spot/display.h>

struct {
    cairo_surface_t *image;  
} glob;

Display::Display() {

}

// boid Display::update_display(){

// 	gtk_widget_queue_draw(widget);
// }

gboolean Display::on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data)
{       
	cairo_set_source_surface(cr, glob.image, 10, 10);
	cairo_paint(cr);
	gtk_widget_queue_draw(widget);

  	return FALSE;
// //   do_drawing(cr);
// 	cr = gdk_cairo_create (gtk_widget_get_window (widget));
// 	// gtk_widget_queue_draw(widget);
// 	// do_drawing (cr, widget);

// 	// gfloat screen_width;
// 	// gfloat screen_height;
// 	// gfloat image_width;
// 	// gfloat image_height;
// 	// gfloat x_scaling;
// 	// gfloat y_scaling;

// 	// /* Display screen dimension in console  */
// 	// screen_width = gdk_screen_get_width (gdk_screen_get_default ());
// 	// screen_height = gdk_screen_get_height (gdk_screen_get_default ());
// 	// g_message ("Screen width %f", screen_width);
// 	// g_message ("Screen height %f", screen_height);

// 	// /* Scale the loaded image to occupy the entire screen  */
// 	// image_width = cairo_image_surface_get_width (glob.image);
// 	// image_height = cairo_image_surface_get_height (glob.image);

// 	// g_message ("Image width %f", image_width);
// 	// g_message ("Image height %f", image_height);

// 	// x_scaling = screen_width / image_width;
// 	// y_scaling = screen_height / image_height;

// 	// g_message ("x_scaling %f", x_scaling);
// 	// g_message ("y_scaling %f", y_scaling);

// 	// cairo_scale (cr, x_scaling, y_scaling);

// 	cairo_set_source_surface (cr, glob.image, 0, 0);
// 	cairo_paint (cr);

// 	cairo_destroy (cr);

// 	gtk_widget_destroy(widget);
 	return FALSE;
}

// void Display::modify_image_surface (cairo_surface_t *surface)
// {
//   unsigned char *data;
//   int width, height, stride;

//   // flush to ensure all writing to the image was done
//   cairo_surface_flush (surface);

//   // modify the image
//   data = cairo_image_surface_get_data (surface);
//   width = cairo_image_surface_get_width (surface);
//   height = cairo_image_surface_get_height (surface);
//   stride = cairo_image_surface_get_stride (surface);
//   modify_image_data (data, width, height, stride);

//   // mark the image dirty so Cairo clears its caches.
//   cairo_surface_mark_dirty (surface);
// }

void Display::runDisplay(int argc, char *argv[]){
	GtkWidget *window;
	GtkWidget *darea;
	
	glob.image = cairo_image_surface_create_from_png("data.png");

	gtk_init(&argc, &argv);

	window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

	darea = gtk_drawing_area_new();
	
	gtk_container_add(GTK_CONTAINER (window), darea);

	g_signal_connect(G_OBJECT(darea), "draw", 
		G_CALLBACK(on_draw_event), NULL); 

	g_signal_connect(window, "destroy",
		G_CALLBACK (gtk_main_quit), NULL);
	
	// g_signal_connect(window, "destroy", G_CALLBACK (gtk_widget_destroy), NULL);

	gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
	gtk_window_set_default_size(GTK_WINDOW(window), 500, 500); 
	gtk_window_set_title(GTK_WINDOW(window), "Image");

	gtk_widget_show_all(window);

	gtk_main();
	// gtk_widget_queue_draw();

	cairo_surface_destroy(glob.image);
	glob.image = NULL;

	// return 0;
	// std::cout << "in runDisplay" << std::endl;
	// glob.image = cairo_image_surface_create_from_png ("data.png");


	// gtk_init (&argc, &argv);


	// _window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

	// _darea = gtk_drawing_area_new ();
	// gtk_container_add (GTK_CONTAINER (_window), _darea);

	// g_signal_connect (G_OBJECT(_darea), "draw", G_CALLBACK (Display::on_draw_event), NULL);
	// g_signal_connect (GTK_CONTAINER (_window), "destroy", G_CALLBACK (gtk_main_quit), NULL);

	// gtk_window_set_position (GTK_WINDOW (_window), GTK_WIN_POS_CENTER);
	// gtk_window_set_title (GTK_WINDOW (_window), "Cairo Test");
	// gtk_window_set_decorated (GTK_WINDOW (_window), FALSE);
	// // gtk_window_fullscreen (GTK_WINDOW (_window));

	// gtk_widget_show_all (_window);

	// // gtk_main ();
	// std::cout << "bottom run display" << std::endl;


	// cairo_surface_destroy (glob.image);
	// // glob.image = NULL;
	// // remove("data.png");
	// // remove("data.jpg");
}

// void Display::continueDisplay(int argc, char *argv[]){

// 	glob.image = cairo_image_surface_create_from_png ("data.png");


// 	gtk_init (&argc, &argv);


// 	_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

// 	_darea = gtk_drawing_area_new ();
// 	gtk_container_add (GTK_CONTAINER (_window), _darea);

// 	g_signal_connect (G_OBJECT(_darea), "draw", G_CALLBACK (Display::on_draw_event), NULL);
// 	g_signal_connect (_window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

// 	gtk_window_set_position (GTK_WINDOW (_window), GTK_WIN_POS_CENTER);
// 	gtk_window_set_title (GTK_WINDOW (_window), "Cairo Test");
// 	gtk_window_set_decorated (GTK_WINDOW (_window), FALSE);
// 	// gtk_window_fullscreen (GTK_WINDOW (_window));

// 	gtk_widget_show_all (_window);

// 	gtk_main ();

// 	cairo_surface_destroy (glob.image);
// }