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
        class ServiceEntry: holds service entries returned from the directory service
    */
    class ServiceEntry {
    public:
        ServiceEntry(const std::string &name, const std::string &type, const std::string &authority) :
                _name(name),
                _type(type),
                _authority(authority) {}

        /* Accessors */
        const std::string getName() const { return _name; }
        const std::string getType() const { return _type; }
        const std::string getAuthority() const { return _authority; }
    private:
        const std::string _name;
        const std::string _type;
        const std::string _authority;
    };

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
        CoreLayer::ServiceEntry listService(const std::string &serviceName) const;
        
        /* listAllServices(): lists all services that are registered on Spot
           Input: -
           Output: Map of [service name, ServiceEntry]
           Side effects: -
        */
        std::map<std::string, CoreLayer::ServiceEntry> listAllServices() const;

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
        const std::shared_ptr<AuthClient> getAuthClient() const { return _authClient; }
        const std::shared_ptr<DirectoryClient> getDirectoryClient() const { return _directoryClient; }
        const std::shared_ptr<RobotIdClient> getRobotIdClient() const { return _robotIdClient; }
        const std::shared_ptr<TimeSyncClient> getTimeSyncClient() const { return _timeSyncClient; }

        /* Other accessor methods */
        const std::string getAuthToken() const { return _authToken; }
        const std::shared_ptr<TimeSyncThread> getTimeSyncThread() const { return _timeSyncThread; }
    private:
        /* authToken: stored authorization token */
        std::string _authToken;

        /* timeSyncThread: stored pointer to time sync thread */
        std::shared_ptr<TimeSyncThread> _timeSyncThread;
    private:
        std::shared_ptr<AuthClient> _authClient;
        std::shared_ptr<DirectoryClient> _directoryClient;
        std::shared_ptr<RobotIdClient> _robotIdClient;
        std::shared_ptr<TimeSyncClient> _timeSyncClient;
    };

    /*
        class SpotPayloads: Holds directory-registration, payload functionality
    */
    class SpotPayloads {
    public:
        SpotPayloads();
    private:
        std::shared_ptr<DirectoryRegistrationClient> _directoryRegistrationClient; 
    };
};

#endif