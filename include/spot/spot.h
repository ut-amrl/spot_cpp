/*
    spot.h: header file for main
*/
#ifndef SPOT_H
#define SPOT_H

#include <spot/robot.h>

// #include "myarea.h"
// #include <gtkmm/application.h>
// #include <gtkmm/window.h>

class Spot {
public:
    static gboolean on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data);
    void do_drawing(cairo_t *cr);
    int main(int argc, char *argv[]);
private:
};

// class ExampleWindow : public Gtk::Window
// {
// public:
//   ExampleWindow();

// protected:
//   MyArea m_area;
// };

#endif
