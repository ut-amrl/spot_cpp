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
    void initClients();
    void basicInit(const std::string &username, const std::string &password);

    /* Authentication */
    void authenticate(const std::string &username, const std::string &password);
    void authenticateWithToken(const std::string &token);

    /* Estop */
    void initBasicEstop();

    /* Lease */
    void initBasicLease();

    /* Timesync */
    void initBasicTimeSync();

    /* Power */
    uint32_t powerOn();
    void powerOnBlocking();
    uint32_t powerOff();
    void powerOffBlocking();
    void blockUntilPowerComplete(uint32_t powerCommandID);

    /* Movement */
    uint32_t sit();
    void sitBlocking();
    uint32_t stand();
    void standBlocking();
    uint32_t velocityMove(double x, double y, double angular, int64_t time, gravAlignedFrame frame);
    void velocityMoveBlocking(double x, double y, double angular, int64_t time, gravAlignedFrame frame);
    uint32_t trajectoryMove(Trajectory2D trajectory, int64_t time, gravAlignedFrame frame);
    void trajectoryMoveBlocking(Trajectory2D trajectory, int64_t time, gravAlignedFrame frame);

    void setMobilityParams(MobilityParams mParams);
    void setBodyPose(Trajectory3D trajectory, bool gravityAlign);
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
