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
        const std::shared_ptr<ImageClient> getImageClient() const { return _imageClient; }
        const std::shared_ptr<LocalGridClient> getLocalGridClient() const { return _localGridClient; }
        const std::shared_ptr<RobotStateClient> getRobotStateClient() const { return _robotStateClient; }

        /* Accessors */
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
        void setEstopConfiguration(const std::set<std::shared_ptr<SpotEstopEndpoint>> &endpoints, const std::string &targetConfigId);

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
            velocityMove(): 
            Input: -
            Output: -
            Side effects: -
        */
        void velocityMove(double, double, double ,double, gravAlignedFrame);

        /*
            trajectoryMove(): 
            Input: -
            Output: -
            Side effects: -
        */
        void trajectoryMove(Trajectory2D, gravAlignedFrame, double);

        
        /* Spot check */

        /* Accessors */
        const std::string getEstopConfigId() const { return _estopConfigId; }
        const std::map<std::string, std::shared_ptr<SpotEstopEndpoint>> getEndpoints() const { return _endpoints; }

        /* Client accessor methods */
        const std::shared_ptr<EstopClient> getEstopClient() const { return _estopClient; }
        const std::shared_ptr<LeaseClient> getLeaseClient() const { return _leaseClient; }
        const std::shared_ptr<PowerClient> getPowerClient() const { return _powerClient; }
        const std::shared_ptr<RobotCommandClient> getRobotCommandClient() const { return _robotCommandClient; }
        const std::shared_ptr<SpotCheckClient> getSpotCheckClient() const { return _spotCheckClient; }
    private:
    std::string getClockIdentifier();
    int64_t getClockSkew();

    private:
    /* services: map of [service name, service entry] */
    std::map<std::string, CoreLayer::ServiceEntry> _services;

    /* endpoints: map of [endpoint unique id, endpoint] */
    std::map<std::string, std::shared_ptr<SpotEstopEndpoint>> _endpoints;

    /* estopThreads: map of [endpoint unique id, endpoint thread] */
    std::map<std::string, std::shared_ptr<EstopThread>> _estopThreads;
    
    /* estopConfigId: current estop config id of which endpoints are registered against */
    std::string _estopConfigId; 

    /* leases: map of [resource, lease obj] (temp for now) */
    std::map<std::string, Lease> _leases;

    /* leaseThreads: map of [resource, lease thread] */
    std::map<std::string, std::shared_ptr<LeaseThread>> _leaseThreads;
    
    private:
        std::shared_ptr<CoreLayer::SpotBase> _spotBase;
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