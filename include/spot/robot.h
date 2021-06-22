
/*
    robot.h: main access point for user when communicating with Spot
*/
#ifndef ROBOT_H
#define ROBOT_H

#include <spot/clients/auth.h>
#include <spot/clients/directory.h>
#include <spot/clients/estop.h>
#include <spot/exception.h>
#include <spot/frame.h>
#include <spot/clients/image.h>
#include <spot/clients/lease.h>
#include <spot/clients/power.h>
#include <spot/clients/robot_command.h>
#include <spot/clients/robot_id.h>
#include <spot/clients/robot_state.h>
#include <spot/clients/spot_check.h>
#include <spot/clients/timesync.h>

#include <map>
#include <vector>
#include <list>

#include <iostream>
#include <fstream>

#include <gtk/gtk.h>
#include <opencv2/opencv.hpp>
// #include <gtkmm/drawingarea.h>
// #include <gdkmm/pixbuf.h>
// #include <cairomm/context.h>
// #include <giomm/resource.h>
// #include <gdkmm/general.h> // set_source_pixbuf()
// #include <glibmm/fileutils.h>
// #include <iostream>

enum movementType {sit, stand, travel}; // move to robot_command

// // class that encapsulates robot state
// class State {
// public:
//     State(const std::string &name, const std::string &address, const std::string &serial);
// private:
// };

class Robot {
public:
    Robot(const std::string &name);
    // robot id stuff
    std::string getId();

    // authentication
    void authenticate(const std::string &username, const std::string &password);
    void authenticateWithToken(const std::string &token);
    void updateToken(const std::string &token);

    // setup (register all clients)
    void setup();

    // Estop
    void initBasicEstop();

    // lease
    void initBasicLease();

    // timesync
    void initBasicTimesync();

    // startup
    void powerOn();
    void powerOff();

    // time sync
    void startTimeSync();
    void stopTimeSync();
    void getTime();

    // state stuff
    bool isPoweredOn();
    bool isEstopped();
    // State getState();

    // movement
    bool move(movementType mType);
    bool move(movementType, double, double, double, double);

    // image 
    bool getImages();

    std::shared_ptr<AuthClient> getAuthClientPtr() const { return _authClientPtr; }
    std::shared_ptr<DirectoryClient> getDirectoryClientPtr() const { return _directoryClientPtr; }
    std::shared_ptr<EstopClient> getEstopClientPtr() const { return _estopClientPtr; }
    std::shared_ptr<ImageClient> getImageClientPtr() const { return _imageClientPtr; }
    std::shared_ptr<LeaseClient> getLeaseClientPtr() const { return _leaseClientPtr; }
    std::shared_ptr<PowerClient> getPowerClientPtr() const { return _powerClientPtr; }
    std::shared_ptr<RobotCommandClient> getRobotCommandClientPtr() const { return _robotCommandClientPtr; }
    std::shared_ptr<RobotIdClient> getRobotIdClientPtr() const { return _robotIdClientPtr; }
    std::shared_ptr<RobotStateClient> getRobotStateClientPtr() const { return _robotStateClientPtr; }
    std::shared_ptr<SpotCheckClient> getSpotCheckClientPtr() const { return _spotCheckClientPtr; }
    std::shared_ptr<TimeSyncClient> getTimeSyncClientPtr() const { return _timeSyncClientPtr; }

private:
    // power
    bool _isOn = false; // false first
    bool _isEstopped = false; // false first
    
    // lease
    std::shared_ptr<Lease> _leasePtr = nullptr;

    // time sync
    std::string _timeSyncClockId;
    int64_t _clockSkew;

    // config data
    std::string _name;
    std::string _token;
    std::string _address;
    std::string _serialNumber;

    // Threads
    std::shared_ptr<EstopKeepAlive> _estopThread = nullptr;
    std::shared_ptr<LeaseKeepAlive> _leaseThread = nullptr;


    // clients (try to refactor into some client cache later)
    std::shared_ptr<AuthClient> _authClientPtr = nullptr;
    std::shared_ptr<DirectoryClient> _directoryClientPtr = nullptr;
    std::shared_ptr<EstopClient> _estopClientPtr = nullptr;
    std::shared_ptr<ImageClient> _imageClientPtr = nullptr;
    std::shared_ptr<LeaseClient> _leaseClientPtr = nullptr;
    std::shared_ptr<PowerClient> _powerClientPtr = nullptr;
    std::shared_ptr<RobotCommandClient> _robotCommandClientPtr = nullptr;
    std::shared_ptr<RobotIdClient> _robotIdClientPtr = nullptr;
    std::shared_ptr<RobotStateClient> _robotStateClientPtr = nullptr;
    std::shared_ptr<SpotCheckClient> _spotCheckClientPtr = nullptr;
    std::shared_ptr<TimeSyncClient> _timeSyncClientPtr = nullptr;
private:
    template <class client_T>
    std::shared_ptr<client_T> getPtr(CLIENT_TYPES type);
};

class Display {
public:
    Display(int rows, int cols, int nCameras);
    ~Display();
    bool getImages();
    // void receiveFrame(MultiKinectPacket &mkp);
    // void receiveAprilTag(AprilTagPacket atp);
    void buildWidgets(GtkWidget *container);
    void receiveFrame();
    static gboolean drawCallback (GtkWidget *widget, cairo_t *cr, gpointer data);
    gboolean doDraw(cairo_t *cr);

protected:
    int _rows, _cols, _cameras;
    bool _initialized;
    //cv::Mat _colorMat;
    GtkWidget *_darea;

    GtkWidget *drawingAreas[8];
    GtkGrid *_gtkGrid;
    //GdkPixbuf *_pixbuf;
    unsigned char **_buf;
};

// class MyArea : public Gtk::DrawingArea{
// public:
//     MyArea();
//     virtual ~MyArea();

// protected:
//     void on_draw(const Cairo::RefPtr<Cairo::Context>&cr, int width, int height);

//     Glib::RefPtr<Gdk::Pixbuf> m_image;
// };

#endif
