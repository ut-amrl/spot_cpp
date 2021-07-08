/*
    spot.h: header file for main
*/
#ifndef SPOT_H
#define SPOT_H

#include <spot/core_layer.h>
#include <spot/robot_layer.h>

/*
    class Spot: container class for sub-Spot classes
*/
class Spot
{
public:
    Spot();

    /* Initialization */
    
    /*
        initClients(): creates shared pointer for clients  
        Input: - 
        Output: - 
        Side effects: _spotcontrol and _spotstate are propogated
    */
    void initClients();
    
    /*
        basicInit(): runs basic processes required for spot to function
        Input: - robot username and password
        Output: - 
        Side effects: runs authenticate(), initClients(), initBasicEstop(), initBasicLease(), initBasicTimesync(), powerOnBlocking()
    */
    void basicInit(const std::string &username, const std::string &password);

    /* Authentication */
    
    /*
        authenticate(): authenticates the robot 
        Input: - robot username and password 
        Output: - 
        Side effects: -
    */
    void authenticate(const std::string &username, const std::string &password);
    
    /*
        authenticateWithToken(): NOT DONE
        Input: - 
        Output: - 
        Side effects: -
    */
    void authenticateWithToken(const std::string &token);

    /* Estop */
    
    /*
        initBasicEstop(): creates a basic estop
        Input: - 
        Output: - 
        Side effects: -
    */
    void initBasicEstop();

    /* Lease */
    
    /*
        initBasicLease(): acquires the robot's lease 
        Input: - 
        Output: - 
        Side effects: -
    */
    void initBasicLease();

    /* Timesync */
    
    /*
        initBasicTimeSync(): begins the robot's timesync 
        Input: - 
        Output: - 
        Side effects: -
    */
    void initBasicTimeSync();

    /* Power */

    /*
        powerOn(): powers on the motors  
        Input: - 
        Output: command id
        Side effects: -
    */
    uint32_t powerOn();
    
    /*
        powerOnBlocking(): powers on the motors, blocks until command is finished 
        Input: - 
        Output: - 
        Side effects: -
    */
    void powerOnBlocking();
    
    /*
        powerOff(): powers off the motors  
        Input: - 
        Output: command id
        Side effects: -
    */
    uint32_t powerOff();
    
    /*
        powerOffBlocking(): powers off the motors, blocks until command is finished 
        Input: - 
        Output: - 
        Side effects: -
    */
    void powerOffBlocking();
    
    /*
        blockUntilPowerComplete(): blocks until command is finished  
        Input: - command id
        Output: - 
        Side effects: -
    */
    void blockUntilPowerComplete(uint32_t powerCommandID);

    /* Movement */
    
    /*
        sit(): sits the robot down 
        Input: - 
        Output: command id
        Side effects: -
    */
    uint32_t sit();
    
    /*
        sitBlocking(): sits the robot down, blocks until command is finished 
        Input: - 
        Output: - 
        Side effects: -
    */
    void sitBlocking();
    
    /*
        stand(): stands the robot  
        Input: - 
        Output: command id 
        Side effects: -
    */
    uint32_t stand();
    
    /*
        standBlocking(): stands the robot, blocks until command is finished  
        Input: - 
        Output: - 
        Side effects: -
    */
    void standBlocking();
    
    /*
        velocityMove(): moves the robot a set velocity for a set time 
        Input: x linear speed, y linear speed, angular velocity, duration of command, frame name  
        Output: command id
        Side effects: -
    */
    uint32_t velocityMove(double x, double y, double angular, int64_t time, gravAlignedFrame frame);
    
    /*
        velocityMoveBlocking(): moves the robot a set velocity for a set time, blocks until command is finished 
        Input: x linear speed, y linear speed, angular velocity, duration of command, frame name
        Output: - 
        Side effects: -
    */
    void velocityMoveBlocking(double x, double y, double angular, int64_t time, gravAlignedFrame frame);
    
    /*
        trajectoryMove(): moves the robot along a determined trajectory  
        Input: the trajectory to move along, duration of command, frame name
        Output: command id
        Side effects: -
    */
    uint32_t trajectoryMove(Trajectory2D trajectory, int64_t time, gravAlignedFrame frame);
    
    /*
        trajectoryMoveBlocking(): moves the robot along a determined trajectory, blocks until command is finished 
        Input: the trajectory to move along, duration of command, frame name
        Output: - command id
        Side effects: -
    */
    void trajectoryMoveBlocking(Trajectory2D trajectory, int64_t time, gravAlignedFrame frame);

    /*
        setMobilityParams(): sets the mobility params 
        Input: mobility params
        Output: -
        Side effects: robot mobility params are changed 
    */
    void setMobilityParams(MobilityParams mParams);

    /*
        setBodyPose(): change the pose of the robot 
        Input: trajectory to follow, is frame gravity aligned
        Output: -
        Side effects: robot mobility params are changed 
    */
    void setBodyPose(Trajectory3D trajectory, bool gravityAlign);
    
    /*
        resetBodyPose(): resets the body pose to the default positions 
        Input: command duration
        Output: -
        Side effects: robot mobility params are changed 
    */
    void resetBodyPose(double time);

    /* Accessors */

    /* Client Group Accessors */
    const std::shared_ptr<CoreLayer::SpotBase> getSpotBase() const { return _spotbase; }
    const std::shared_ptr<CoreLayer::SpotPayloads> getSpotPayloads() const { return _spotpayloads; }
    const std::shared_ptr<RobotLayer::SpotControl> getSpotControl() const { return _spotcontrol; }
    const std::shared_ptr<RobotLayer::SpotData> getSpotData() const { return _spotdata; }
    const std::shared_ptr<RobotLayer::SpotState> getSpotState() const { return _spotstate; }

    /* Client Accessors */
    const std::shared_ptr<ClientLayer::AuthClient> getAuthClient() const { return _spotbase->getAuthClient(); }
    const std::shared_ptr<ClientLayer::DirectoryClient> getDirectoryClient() const { return _spotbase->getDirectoryClient(); }
    const std::shared_ptr<ClientLayer::RobotIdClient> getRobotIdClient() const { return _spotbase->getRobotIdClient(); }
    const std::shared_ptr<ClientLayer::TimeSyncClient> getTimeSyncClient() const { return _spotbase->getTimeSyncClient(); }

    const std::shared_ptr<ClientLayer::ImageClient> getImageClient() const { return _spotstate->getImageClient(); }
    const std::shared_ptr<ClientLayer::LocalGridClient> getLocalGridClient() const { return _spotstate->getLocalGridClient(); }
    const std::shared_ptr<ClientLayer::RobotStateClient> getRobotStateClient() const { return _spotstate->getRobotStateClient(); }
    const std::shared_ptr<ClientLayer::WorldObjectsClient> getWorldObjectsClient() const { return _spotstate->getWorldObjectsClient(); }

    const std::shared_ptr<ClientLayer::EstopClient> getEstopClient() const { return _spotcontrol->getEstopClient(); }
    const std::shared_ptr<ClientLayer::LeaseClient> getLeaseClient() const { return _spotcontrol->getLeaseClient(); }
    const std::shared_ptr<ClientLayer::PowerClient> getPowerClient() const { return _spotcontrol->getPowerClient(); }
    const std::shared_ptr<ClientLayer::RobotCommandClient> getRobotCommandClient() const { return _spotcontrol->getRobotCommandClient(); }
    const std::shared_ptr<ClientLayer::SpotCheckClient> getSpotCheckClient() const { return _spotcontrol->getSpotCheckClient(); }

private:
    std::shared_ptr<CoreLayer::SpotBase> _spotbase;
    std::shared_ptr<CoreLayer::SpotPayloads> _spotpayloads;
    std::shared_ptr<RobotLayer::SpotControl> _spotcontrol;
    std::shared_ptr<RobotLayer::SpotData> _spotdata;
    std::shared_ptr<RobotLayer::SpotState> _spotstate;
};

#endif
