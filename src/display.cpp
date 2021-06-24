#include <spot/display.h>

static cairo_surface_t *surface = NULL;

void Display::clear_surface (void)
{
  cairo_t *cr;

  cr = cairo_create (surface);

  cairo_set_source_rgb (cr, 1, 1, 1);
  cairo_paint (cr);

  cairo_destroy (cr);
}

gboolean Display::configure_event_cb (GtkWidget *widget, GdkEventConfigure *event, gpointer data) {
  if (surface)
    cairo_surface_destroy (surface);

  surface = gdk_window_create_similar_surface (gtk_widget_get_window (widget),
                                               CAIRO_CONTENT_COLOR,
                                               gtk_widget_get_allocated_width (widget),
                                               gtk_widget_get_allocated_height (widget));

  /* Initialize the surface to white */
  clear_surface ();

  /* We've handled the configure event, no need for further processing. */
  return TRUE;
}

gboolean Display::draw_cb (GtkWidget *widget, cairo_t   *cr, gpointer data) {
  cairo_set_source_surface (cr, surface, 0, 0);
  cairo_paint (cr);

  return FALSE;
}

void Display::draw_brush (GtkWidget *widget, gdouble x, gdouble y) {
  cairo_t *cr;

  /* Paint to the surface, where we store our state */
  cr = cairo_create (surface);

  cairo_rectangle (cr, x - 3, y - 3, 6, 6);
  cairo_fill (cr);

  cairo_destroy (cr);

  /* Now invalidate the affected region of the drawing area. */
  gtk_widget_queue_draw_area (widget, x - 3, y - 3, 6, 6);
}

gboolean Display::button_press_event_cb (GtkWidget *widget, GdkEventButton *event, gpointer data) {
  /* paranoia check, in case we haven't gotten a configure event */
  if (surface == NULL)
    return FALSE;

  if (event->button == GDK_BUTTON_PRIMARY)
    {
      draw_brush (widget, event->x, event->y);
    }
  else if (event->button == GDK_BUTTON_SECONDARY)
    {
      clear_surface ();
      gtk_widget_queue_draw (widget);
    }

  /* We've handled the event, stop processing */
  return TRUE;
}

gboolean Display::motion_notify_event_cb (GtkWidget *widget, GdkEventMotion *event, gpointer data) {
  /* paranoia check, in case we haven't gotten a configure event */
  if (surface == NULL)
    return FALSE;

  if (event->state & GDK_BUTTON1_MASK)
    draw_brush (widget, event->x, event->y);

  /* We've handled it, stop processing */
  return TRUE;
}

void Display::close_window (void) {
  if (surface)
    cairo_surface_destroy (surface);
}

void Display::activate (GtkApplication *app, gpointer user_data) {
  GtkWidget *window;
  GtkWidget *frame;
  GtkWidget *drawing_area;

  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "Drawing Area");

  g_signal_connect (window, "destroy", G_CALLBACK (close_window), NULL);

  gtk_container_set_border_width (GTK_CONTAINER (window), 8);

  frame = gtk_frame_new (NULL);
  gtk_frame_set_shadow_type (GTK_FRAME (frame), GTK_SHADOW_IN);
  gtk_container_add (GTK_CONTAINER (window), frame);

  drawing_area = gtk_drawing_area_new ();
  /* set a minimum size */
  gtk_widget_set_size_request (drawing_area, 100, 100);

  gtk_container_add (GTK_CONTAINER (frame), drawing_area);

  /* Signals used to handle the backing surface */
  g_signal_connect (drawing_area, "draw",
                    G_CALLBACK (draw_cb), NULL);
  g_signal_connect (drawing_area,"configure-event",
                    G_CALLBACK (configure_event_cb), NULL);

  /* Event signals */
  g_signal_connect (drawing_area, "motion-notify-event",
                    G_CALLBACK (motion_notify_event_cb), NULL);
  g_signal_connect (drawing_area, "button-press-event",
                    G_CALLBACK (button_press_event_cb), NULL);

  /* Ask to receive events the drawing area doesn't normally
   * subscribe to. In particular, we need to ask for the
   * button press and motion notify events that want to handle.
   */
  gtk_widget_set_events (drawing_area, gtk_widget_get_events (drawing_area)
                                     | GDK_BUTTON_PRESS_MASK
                                     | GDK_POINTER_MOTION_MASK);

  gtk_widget_show_all (window);
}

// struct {
//     cairo_surface_t *image;  
// } glob;

// gboolean Display::on_draw_event(GtkWidget *widget, cairo_t *cr, gpointer user_data)
// {      
// 	cr = gdk_cairo_create (gtk_widget_get_window (widget));
// 	cairo_set_source_surface (cr, glob.image, 0, 0);
// 	cairo_paint (cr);
// 	cairo_destroy (cr);
//  	return FALSE;
// }

// void Display::runDisplay(int argc, char *argv[]){
//     GtkWidget *window;
// 	GtkWidget *darea;
	
// 	cairo_t *cr;

// 	glob.image = cairo_image_surface_create_from_png ("data.png");

// 	gtk_init (&argc, &argv);

// 	window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
// 	darea = gtk_drawing_area_new ();
	
// 	gtk_container_add (GTK_CONTAINER (window), darea);
	
// 	g_signal_connect (G_OBJECT (darea), "draw", G_CALLBACK (Display::on_draw_event), NULL);
// 	g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);
	
// 	gtk_window_set_position (GTK_WINDOW (window), GTK_WIN_POS_CENTER);
// 	gtk_window_set_title (GTK_WINDOW (window), "Spot");
// 	gtk_window_set_decorated (GTK_WINDOW (window), FALSE);
// 	gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
// 	gtk_widget_show_all (window);
	
// 	// this.queue_draw();

// 	gtk_main ();
// 	// gtk_main_();
// 	std::cout << "reached here" << std::endl;
// 	cairo_surface_destroy (glob.image);
// }