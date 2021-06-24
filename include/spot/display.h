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
    static void clear_surface(void);
    static gboolean configure_event_cb (GtkWidget         *widget,
                    GdkEventConfigure *event,
                    gpointer           data);
    static gboolean draw_cb (GtkWidget *widget,
         cairo_t   *cr,
         gpointer   data);
    static void draw_brush (GtkWidget *widget,
            gdouble    x,
            gdouble    y);
    static gboolean button_press_event_cb (GtkWidget      *widget,
                       GdkEventButton *event,
                       gpointer        data);
    static gboolean motion_notify_event_cb (GtkWidget      *widget,
                        GdkEventMotion *event,
                        gpointer        data);
    static void close_window(void);
    static void activate (GtkApplication *app, gpointer user_data);
    void refresh();
    // static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data);
    // void do_drawing(cairo_t *cr);
    // void runDisplay(int argc, char *argv[]);
private:
    // static GtkWidget *window;
    // static GtkWidget *frame;
    // static GtkWidget *drawing_area;
    // static bool _initialized;
};

#endif