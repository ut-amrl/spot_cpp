/*
    display.h: for displaying robot camera feed 
*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include <gtk/gtk.h>
#include <cairo.h>
#include <opencv2/opencv.hpp>
#include <gdk-pixbuf/gdk-pixbuf.h>

class Display{
public:
    Display();
    // void modify_image_surface (cairo_surface_t *surface);
    static gboolean drawCallback(GtkWidget *widget, cairo_t *cr, gpointer user_data);
    void do_drawing(cairo_t *cr);
    void buildWidgets(GtkWidget *container);
    gboolean doDraw(cairo_t *cr);
    void receiveFrame(cv::Mat img);
    // void runDisplay(GtkWidget *containerint argc, char *argv[]);
protected:
    int _rows, _cols;
    unsigned char **_buf;
    bool _initialized;
    GtkWidget *_window;
	GtkWidget *_darea;
    GtkGrid *_gtkGrid;
	cairo_t *_cr;
private:
    // GtkWidget *_window;
	// GtkWidget *_darea;
    // GtkGrid *_gtkGrid;
	// cairo_t *_cr;
};

#endif