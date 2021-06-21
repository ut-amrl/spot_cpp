#ifndef ROBOT_LAYER_H
#define ROBOT_LAYER_H

namespace RobotLayer {

    /*
        class SpotState: holds image, local-grid, robot-state, world-objects client functionality
    */
    class SpotState {
    public:
        SpotState();
    private:
        std::shared_ptr<ImageClient> _imageClient;
        std::shared_ptr<LocalGridClient> _localGridClient;
        std::shared_ptr<RobotStateClient> _robotStateClient;
        // todo: world objects client
    };

    /*
        class SpotControl: holds estop, lease, power, robot-command, spot-check functionality
    */
    class SpotControl {
    public:
        SpotControl();
    private:
        std::shared_ptr<EstopClient> _estopClient;
        std::shared_ptr<LeaseClient> _leaseClient;
        std::shared_ptr<PowerClient> _powerClient;
        std::shared_ptr<RobotCommandClient> _robotCommandClient;
        std::shared_ptr<SpotCheckClient> _spotCheckClient;
    };

    /*
        class SpotData: holds data-buffer, data-service, data-acquisition, and data-acquisition-store client functionality
    */
    class SpotData {
    public:
        SpotData();
    private:
        // todo: data clients
    };
};

#endif