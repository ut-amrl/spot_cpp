#include <spot/robot_layer.h>

namespace RobotLayer {
    SpotState::SpotState() {}

    SpotControl::SpotControl(std::shared_ptr<CoreLayer::SpotBase> spotBase) {}

    bool SpotControl::estopped() {

    }

    void SpotControl::setEstopConfiguration(size_t numEndpoints) {

    }

    void SpotControl::registerEstopEndpoint(const std::string &name, const std::string &role, const std::string &configId, 
            int64_t estopTimeout, int64_t estopCutPowerTimeout) {

    }

    void SpotControl::deregisterEstopEndpoint(const std::string &name) {

    }

    void SpotControl::beginEstopping(const std::string &name) {

    }

    void SpotControl::beginEstopping() {

    }

    void SpotControl::endEstopping(const std::string &name) {

    }

    void SpotControl::endEstopping() {

    }

    void SpotControl::acquireLease(const std::string &resource) {

    }

    bool SpotControl::poweredOn() {

    }

    void SpotControl::powerOnMotors() {

    }

    void SpotControl::powerOffMotors() {

    }

    void SpotControl::sit() {

    }

    void SpotControl::stand() {

    }

}