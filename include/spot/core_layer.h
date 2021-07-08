#ifndef CORE_LAYER_H
#define CORE_LAYER_H

#include <spot/clients/auth.h>
#include <spot/clients/directory.h>
#include <spot/clients/robot_id.h>
#include <spot/clients/timesync.h>
#include <spot/clients/directory_registration.h>

#include <list>
#include <map>

namespace CoreLayer {

    /*
        class SpotBase: Holds auth, directory, robot-id, timesync client functionality
    */
    class SpotBase {
    public:
        static int DEFAULT_TIME_SYNC_INTERVAL_SECS;
        static int DEFAULT_TIME_SYNC_NOT_READY_INTERVAL_SECS;
        
        SpotBase();

        /* Common SpotBase functionality */
        /* Authentication */
        /* authenticate(): initializes the _authToken() private member in this instance.
           Input: Spot username and password
           Output: -
           Side effects: initializes the authToken member in the instance
        */
        void authenticate(const std::string &username, const std::string &password);

        /* Authentication check */
        /* finish me
           notAuthenticated(): checks if the _authToken() private member in this instance 
           has been authenticated or not.
           Input: -
           Output: -
           Side effects: -
        */
        void notAuthenticated();


        /* Robot Id */
        /* getRobotId(): Gets the robot id information from Spot
           Input: -
           Output: -
           Side effects: -
        */
        const std::string getRobotId() const; 

        /* Directory */
        /* listService(): returns a ServiceEntry for a specific service
           Input: service name of the service to get
           Output: ServiceEntry object representing the info from the response
           Side effects: -
        */
        bosdyn::api::ServiceEntry listService(const std::string &serviceName) const;
        
        /* listAllServices(): lists all services that are registered on Spot
           Input: -
           Output: Map of [service name, ServiceEntry]
           Side effects: -
        */
        std::map<std::string, bosdyn::api::ServiceEntry> listAllServices() const;

        /* Timesync */
        /* beginTimesync(): starts a timesync thread for periodic checkins with Spot
           Input: -
           Output: -
           Side effects: Creates a thread using the TimesyncKeepAlive class
        */
        void beginTimesync();

        /* endTimesync(): ends timesync with robot
           Input: -
           Output: -
           Side effects: Destroys timesync thread through the TimesyncKeepAlive class
        */
        void endTimesync();

        /* Client accessor methods (if user needs direct access to client) */
        const std::shared_ptr<ClientLayer::AuthClient> getAuthClient() const { return _authClient; }
        const std::shared_ptr<ClientLayer::DirectoryClient> getDirectoryClient() const { return _directoryClient; }
        const std::shared_ptr<ClientLayer::RobotIdClient> getRobotIdClient() const { return _robotIdClient; }
        const std::shared_ptr<ClientLayer::TimeSyncClient> getTimeSyncClient() const { return _timeSyncClient; }

        /* Other accessor methods */
        const std::string getAuthToken() const { return _authToken; }
        const std::shared_ptr<ClientLayer::TimeSyncThread> getTimeSyncThread() const { return _timeSyncThread; }
    private:
        /* authToken: stored authorization token */
        std::string _authToken;

        /* timeSyncThread: stored pointer to time sync thread */
        std::shared_ptr<ClientLayer::TimeSyncThread> _timeSyncThread;
    private:
        std::shared_ptr<ClientLayer::AuthClient> _authClient;
        std::shared_ptr<ClientLayer::DirectoryClient> _directoryClient;
        std::shared_ptr<ClientLayer::RobotIdClient> _robotIdClient;
        std::shared_ptr<ClientLayer::TimeSyncClient> _timeSyncClient;
    };

    /*
        class SpotPayloads: Holds directory-registration, payload functionality
    */
    class SpotPayloads {
    public:
        SpotPayloads();
    private:
        std::shared_ptr<ClientLayer::DirectoryRegistrationClient> _directoryRegistrationClient; 
    };
};

#endif