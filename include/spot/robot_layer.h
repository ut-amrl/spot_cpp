#ifndef ROBOT_LAYER_H
#define ROBOT_LAYER_H

#include <spot/core_layer.h>

#include <spot/clients/image.h>
#include <spot/clients/local_grid.h>
#include <spot/clients/robot_state.h>

#include <spot/clients/estop.h>
#include <spot/clients/lease.h>
#include <spot/clients/power.h>
#include <spot/clients/robot_command.h>
#include <spot/clients/spot_check.h>

#include <set>

namespace RobotLayer {

    /*
        class SpotState: holds image, local-grid, robot-state, world-objects client functionality
    */
    class SpotState {
    public:
        SpotState();
        /* Common SpotState functionality */
        /* Images */

        /* Local grid */

        /* Robot state */

        /* Client accessors */
        const std::shared_ptr<ClientLayer::ImageClient> getImageClient() const { return _imageClient; }
        const std::shared_ptr<ClientLayer::LocalGridClient> getLocalGridClient() const { return _localGridClient; }
        const std::shared_ptr<ClientLayer::RobotStateClient> getRobotStateClient() const { return _robotStateClient; }

        /* Accessors */
    private:
        std::shared_ptr<ClientLayer::ImageClient> _imageClient;
        std::shared_ptr<ClientLayer::LocalGridClient> _localGridClient;
        std::shared_ptr<ClientLayer::RobotStateClient> _robotStateClient;
        // todo: world objects client
    };

    /*
        class SpotControl: holds estop, lease, power, robot-command, spot-check functionality
    */
    class SpotControl {
    public:
        SpotControl(std::shared_ptr<CoreLayer::SpotBase> spotBase); 

        /* Common SpotControl functionality */
        /* Estop */
        /*
            estopped(): checks if the robot has been estopped
            Input: -
            Output: -
            Side effects: -
        */
        bool estopped();

        /*
            setEstopConfiguration(): set an estop configuration on Spot with the specified number of endpoints
            Input: endpoints to set in the configuration
            Output: -
            Side effects: initializes member variables, deletes all registered endpoints if called again
        */
        void setEstopConfiguration(const std::set<std::shared_ptr<ClientLayer::EstopEndpoint>> &endpoints, const std::string &targetConfigId);

        /*
            registerEstop(): registers in estop in the config with the given parameters
            Input: name, role, config id (instance config id if empty), timeout in seconds, cut power timeout in seconds
            Output: unique id of registered endpoint
            Side effects: adds to endpoints map 
        */
        std::string registerEstopEndpoint(const std::string &name, const std::string &role, const std::string &configId, 
                int64_t estopTimeout, int64_t estopCutPowerTimeout);

        /*
            deregisterEstopEndpoint(): deregisters and estop endpoint in the config with the given name
            Input: unique id of endpoint to deregister
            Output: -
            Side effects: removes kv pair from endpoints map
        */
        void deregisterEstopEndpoint(const std::string &uniqueId, const std::string &targetConfigId);

        /*
            beginEstopping(): kicks off estop threads
            Input: endpoint to kick thread off on, (no parameters kicks off all threads)
            Output: -
            Side effects: adds kv pairs to threads map
        */
        void beginEstopping(const std::string &uniqueId);
        void beginEstopping();

        /*
            endEstopping(): destroys estop threads
            Input: endpoint to stop check in threads on, (no parameters destroys all known threads)
            Output: -
            Side effects: removes kv pairs from threads map
        */
        void endEstopping(const std::string &uniqueId);
        void endEstopping();

        /* Lease */ // TODO: ADD METHODS LATER (lease wallet or map w/ [resource, lease])
        /*
            acquireLease(): acquires a lease for the given resource if possible
            Input: resource to acquire lease for
            Output: -
            Side effects: adds to LeaseWallet (once implemented)
        */
        void acquireLease(const std::string &resource);

        /*
            beginLeasing(): kicks off lease thread(s)
            Input: resource to kick off (if none, kicks off all threads)
            Output: -
            Side effects: -
        */
        void beginLeasing(const std::string &resource);
        void beginLeasing();

        /*
            endLeasing(): destroys lease thread(s)
            Input: resource to lease to stop (if none, kicks off all threads)
            Output: -
            Side effects: -
        */
       void endLeasing(const std::string &resource);
       void endLeasing();

        /* Power */ // todo: payload power
        /*
            poweredOn(): returns if the motors are currently powered on
            Input: -
            Output: -
            Side effects: -
        */
        bool poweredOn();
        
        /*
            powerOnMotors(): turns on motor power for the robot
            Input: -
            Output: -
            Side effects: -
        */
        void powerOnMotors();

        /*
            powerOffMotors(): turns off motor power for robot
            Input: -
            Output: -
            Side effects: -
        */
       void powerOffMotors();

        /* Commands */
        /*
            sit(): Spot, sit!
            Input: -
            Output: -
            Side effects: -
        */
        void sit();

        /*
            stand(): makes Spot stand
            Input: -
            Output: -
            Side effects: -
        */
        void stand();

        /*
            velocityMove(): Issues a velocitycommand to the robot
            Input: - x, y, rot, duration (in milliseconds), frame
            Output: -
            Side effects: -
        */
        void velocityMove(double, double, double, int64_t, gravAlignedFrame);

        /*
            trajectoryMove(): Issues a trajectorycommand to the robot
            Input: - traj2d, frame, duration (in milliseconds)
            Output: -
            Side effects: -
        */
        void trajectoryMove(Trajectory2D, gravAlignedFrame, int64_t);

        
        /* Spot check */

        /* Accessors */
        const std::string getEstopConfigId() const { return _estopConfigId; }
        const std::map<std::string, std::shared_ptr<ClientLayer::EstopEndpoint>> getEndpoints() const { return _endpoints; }

        /* Client accessor methods */
        const std::shared_ptr<ClientLayer::EstopClient> getEstopClient() const { return _estopClient; }
        const std::shared_ptr<ClientLayer::LeaseClient> getLeaseClient() const { return _leaseClient; }
        const std::shared_ptr<ClientLayer::PowerClient> getPowerClient() const { return _powerClient; }
        const std::shared_ptr<ClientLayer::RobotCommandClient> getRobotCommandClient() const { return _robotCommandClient; }
        const std::shared_ptr<ClientLayer::SpotCheckClient> getSpotCheckClient() const { return _spotCheckClient; }
    private:
    std::string getClockIdentifier();
    google::protobuf::Duration getClockSkew();

    private:
    /* services: map of [service name, service entry] */
    std::map<std::string, ClientLayer::ServiceEntry> _services;

    /* endpoints: map of [endpoint unique id, endpoint] */
    std::map<std::string, std::shared_ptr<ClientLayer::EstopEndpoint>> _endpoints;

    /* estopThreads: map of [endpoint unique id, endpoint thread] */
    std::map<std::string, std::shared_ptr<ClientLayer::EstopThread>> _estopThreads;
    
    /* estopConfigId: current estop config id of which endpoints are registered against */
    std::string _estopConfigId; 

    /* leases: map of [resource, lease obj] (temp for now) */
    std::map<std::string, Lease> _leases;

    /* leaseThreads: map of [resource, lease thread] */
    std::map<std::string, std::shared_ptr<ClientLayer::LeaseThread>> _leaseThreads;
    
    private:
        std::shared_ptr<CoreLayer::SpotBase> _spotBase;
        std::shared_ptr<ClientLayer::EstopClient> _estopClient;
        std::shared_ptr<ClientLayer::LeaseClient> _leaseClient;
        std::shared_ptr<ClientLayer::PowerClient> _powerClient;
        std::shared_ptr<ClientLayer::RobotCommandClient> _robotCommandClient;
        std::shared_ptr<ClientLayer::SpotCheckClient> _spotCheckClient;
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