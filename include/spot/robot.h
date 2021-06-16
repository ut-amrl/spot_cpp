
/*
    robot.h: main access point for user when communicating with Spot
*/
#ifndef ROBOT_H
#define ROBOT_H

#include <spot/auth.h>
#include <spot/directory.h>
#include <spot/estop.h>
#include <spot/exception.h>
#include <spot/frame.h>
#include <spot/image.h>
#include <spot/lease.h>
#include <spot/power.h>
#include <spot/robot_command.h>
#include <spot/robot_id.h>
#include <spot/robot_state.h>
#include <spot/spot_check.h>
#include <spot/timesync.h>
#include <spot/common.h>

#include <map>
#include <list>

enum movementType {sit, stand, travel};

// class that encapsulates robot state
class State {
public:
    State(const std::string &name, const std::string &address, const std::string &serial);
private:
};

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
    State getState();

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
    std::shared_ptr<TimeSyncClient> getTimeSyncClientPtr() const { return _timesyncClientPtr; }

    // movement
    bool move(movementType, double, double, double, double, int64_t, std::__cxx11::string);

private:
    // power
    bool _isOn = false; // false first
    bool _isEstopped = false; // false first
    
    // lease
    std::shared_ptr<Lease> _leasePtr = nullptr;

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
    std::shared_ptr<TimeSyncClient> _timesyncClientPtr = nullptr;
private:
    template <class client_T>
    std::shared_ptr<client_T> getPtr(CLIENT_TYPES type);
};

#endif
