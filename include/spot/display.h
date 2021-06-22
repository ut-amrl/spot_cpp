/*
    display.h: for displaying robot camera feed 
*/

#ifndef DISPLAY_H
#define DISPLAY_H

#include <gtk/gtk.h>
#include <cairo.h>
#include <opencv2/opencv.hpp>

class Display{
public:
    Display(int argc);
    static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data);
    void do_drawing(cairo_t *cr);
    void runDisplay(int argc, char *argv[]);
private:
    int _argc; 
    char *_argv[];
};

#endif