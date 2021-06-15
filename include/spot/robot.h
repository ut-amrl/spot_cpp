
/*
    robot.h: main access point for user when communicating with Spot
*/
#ifndef ROBOT_H
#define ROBOT_H

#include <spot/auth.h>
#include <spot/directory.h>
#include <spot/estop.h>
#include <spot/exception.h>
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

// class that encapsulates robot state
class State {
public:
    State(const std::string &name, const std::string &address, const std::string &serial);
private:
};

class Robot {
public:
    Robot(const std::string &name, const std::string &username, const std::string &password);
    // robot id stuff
    std::string getId();

    // authentication
    void authenticate(const std::string &username, const std::string &password);
    void authenticateWithToken(const std::string &token);
    void updateToken(const std::string &token);

    // ensureClient(): authorizes, caches client and creates channel
    template <class client_T>
    std::shared_ptr<client_T> ensureClient(CLIENT_TYPES type, const std::string &clientName);

    // getClient(): only gets a client (throws exception if client not cached)
    template <class client_T>
    std::shared_ptr<client_T> getClient(CLIENT_TYPES type);

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
    
private:
    // power
    bool _isOn = false; // false first
    bool _isEstopped = false; // false first

    // config data
    std::string _name;
    std::string _token;
    std::string _address;
    std::string _serialNumber;

    // clients (try to refactor into some client cache later)
    std::shared_ptr<AuthClient> _authClientPtr;
    std::shared_ptr<DirectoryClient> _directoryClientPtr;
    std::shared_ptr<EstopClient> _estopClientPtr;
    std::shared_ptr<ImageClient> _imageClientPtr;
    std::shared_ptr<LeaseClient> _leaseClientPtr;
    std::shared_ptr<PowerClient> _powerClientPtr;
    std::shared_ptr<RobotCommandClient> _robotCommandClientPtr;
    std::shared_ptr<RobotIdClient> _robotIdClientPtr;
    std::shared_ptr<SpotCheckClient> _spotCheckClientPtr;
    std::shared_ptr<TimeSyncClient> _timesyncClientPtr;
};

#endif