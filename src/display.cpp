#include <spot/display.h>

struct {
    cairo_surface_t *image;  
} glob;

Display::Display() {

}

// boid Display::update_display(){

// 	gtk_widget_queue_draw(widget);
// }

gboolean Display::drawCallback(GtkWidget *widget, cairo_t *cr, gpointer user_data)
{     
	Display *display =(Display *)user_data;  
	// glob.image = cairo_image_surface_create_from_png("data.png"); 
	// cairo_set_source_surface(cr, glob.image, 10, 10);
	// cairo_paint(cr);
	// gtk_widget_queue_draw(widget);
	
	return display->doDraw(cr);
  	// return FALSE;
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

// void Display::runDisplay(int argc, char *argv[]){
void Display::buildWidgets(GtkWidget *container){
	_initialized = true;
	// GtkWidget *window;
	// GtkWidget *darea;
	_gtkGrid = (GtkGrid*)gtk_grid_new();
	gtk_container_add(GTK_CONTAINER (container), (GtkWidget * ) _gtkGrid );
	gtk_grid_set_row_homogeneous(_gtkGrid, true);
    gtk_grid_set_column_homogeneous(_gtkGrid, true);
	_darea = gtk_drawing_area_new();
	// glob.image = cairo_image_surface_create_from_png("data.png");

	gtk_grid_attach(_gtkGrid,_darea, 0, 0 , 1, 1);
	g_signal_connect (G_OBJECT (_darea), "draw", G_CALLBACK (drawCallback), this);


	// gtk_init(&argc, &argv);

	// window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

	// darea = gtk_drawing_area_new();
	
	// g_signal_connect (G_OBJECT (darea), "draw", G_CALLBACK (on_draw_event), this);

	// gtk_container_add(GTK_CONTAINER (window), darea);

	// g_signal_connect(G_OBJECT(darea), "draw", 
	// 	G_CALLBACK(on_draw_event), NULL); 

	// g_signal_connect(window, "destroy",
	// 	G_CALLBACK (gtk_main_quit), NULL);
	
	// // g_signal_connect(window, "destroy", G_CALLBACK (gtk_widget_destroy), NULL);

	// gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
	// gtk_window_set_default_size(GTK_WINDOW(window), 500, 500); 
	// gtk_window_set_title(GTK_WINDOW(window), "Image");

	// gtk_widget_show_all(window);

	// gtk_main();
	// // gtk_widget_queue_draw();
	// std::cout << "reached bottom" << std::endl;

	// cairo_surface_destroy(glob.image);
	// glob.image = NULL;
}

gboolean Display::doDraw(cairo_t *cr) {
	_rows = 480;
    _cols = 640;
	if(_buf != NULL) {
        GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(
            (guint8*)(_buf[0]),
            GDK_COLORSPACE_RGB,
            false,
            8,
            _cols,
            _rows,
            (int)3 * _cols, NULL, NULL);
        gdk_cairo_set_source_pixbuf(cr, pixbuf, 0, 0);
        cairo_paint(cr);
    }
}

void Display::receiveFrame(cv::Mat img) {
    //Add the frame to the buffer
    _rows = 480;
    _cols = 640;
    size_t bytes = _cols * _rows * 3;
    if(_buf == NULL) {
        _buf = new unsigned char*[8];
        for (int i = 0; i < 8; i++) {
            _buf[i] = (unsigned char*)calloc(1, bytes);
        }
    }

	memcpy(_buf[0], img.data, bytes);

    //Draw each area
    if(_initialized) {
		gtk_widget_queue_draw(_darea);
    }
}