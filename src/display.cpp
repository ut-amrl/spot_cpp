#include <spot/display.h>

struct {
    cairo_surface_t *image;  
} glob;

Display::Display(int argc) : 
        _argc(argc){
}

gboolean Display::on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data)
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

void Display::runDisplay(int argc, char *argv[]){
    GtkWidget *window;
	GtkWidget *darea;
	cairo_t *cr;

	glob.image = cairo_image_surface_create_from_png ("test.png");

	gtk_init (&argc, &argv);


	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);

	darea = gtk_drawing_area_new ();
	gtk_container_add (GTK_CONTAINER (window), darea);

	g_signal_connect (G_OBJECT (darea), "draw",
		G_CALLBACK (Display::on_draw_event), NULL);
	g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

	gtk_window_set_position (GTK_WINDOW (window), GTK_WIN_POS_CENTER);
	gtk_window_set_title (GTK_WINDOW (window), "Cairo Test");
	gtk_window_set_decorated (GTK_WINDOW (window), FALSE);
	gtk_window_fullscreen (GTK_WINDOW (window));


	gtk_widget_show_all (window);

	gtk_main ();

	cairo_surface_destroy (glob.image);
}