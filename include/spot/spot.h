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
class Spot {
public:
    Spot();

    /* Accessors */
    const std::shared_ptr<CoreLayer::SpotBase> getSpotBase() const { return _spotbase; }
    const std::shared_ptr<CoreLayer::SpotPayloads> getSpotPayloads() const { return _spotpayloads; }
    const std::shared_ptr<RobotLayer::SpotControl> getSpotControl() const { return _spotcontrol; }
    const std::shared_ptr<RobotLayer::SpotData> getSpotData() const { return _spotdata; }
    const std::shared_ptr<RobotLayer::SpotState> getSpotState() const { return _spotstate; }

private:
    std::shared_ptr<CoreLayer::SpotBase> _spotbase;
    std::shared_ptr<CoreLayer::SpotPayloads> _spotpayloads;
    std::shared_ptr<RobotLayer::SpotControl> _spotcontrol;
    std::shared_ptr<RobotLayer::SpotData> _spotdata;
    std::shared_ptr<RobotLayer::SpotState> _spotstate;
};

#endif
