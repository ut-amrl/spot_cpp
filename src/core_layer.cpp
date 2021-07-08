#include <spot/core_layer.h>

namespace CoreLayer {

    int SpotBase::DEFAULT_TIME_SYNC_INTERVAL_SECS = 60;
    int SpotBase::DEFAULT_TIME_SYNC_NOT_READY_INTERVAL_SECS = 3;

    SpotBase::SpotBase() {
        // initialize pointers
        _authClient = std::shared_ptr<ClientLayer::AuthClient>(new ClientLayer::AuthClient());
        _robotIdClient = std::shared_ptr<ClientLayer::RobotIdClient>(new ClientLayer::RobotIdClient());
    }

    /* Common SpotBase functionality implementations */
    /* Authentication */
    void SpotBase::authenticate(const std::string &username, const std::string &password) {
        try {
            _authToken = _authClient->auth(username, password).token();
        } catch (Error &e){
            std::cout << e.what() << std::endl;
            return;
        }

        _directoryClient = std::shared_ptr<ClientLayer::DirectoryClient>(new ClientLayer::DirectoryClient(_authToken));
    }

    /* SpotBase Auth Exception Handling */
    void SpotBase::notAuthenticated() {
        if(_authToken.empty()) {
            throw "empty auth token?";
        } else {
            std::cout << "generated authToken successfully" << std::endl;
        }
    }

    /* Robot Id */
    const std::string SpotBase::getRobotId() const {
        RobotIdResponse reply;
        try {
            reply = _robotIdClient->getId();
        } catch (Error &error) {
            std::cout << error.what() << std::endl;
            return "";
        }

        RobotId id = reply.robot_id();
        std::string ret = "[SERIAL NUMBER]: " + id.serial_number();
        ret += "\n[SPECIES]: " + id.species();
        ret += "\n[VERSION]: " + id.version();
        ret += "\n[NAME]: " + id.software_release().name();
        ret += "\n[NICKNAME]: " + id.nickname() + "\n";
        return ret;
    }

    /* Directory */
    bosdyn::api::ServiceEntry SpotBase::listService(const std::string &serviceName) const {
        GetServiceEntryResponse reply;
        try {
            reply = _directoryClient->getEntry(serviceName);
        } catch (Error &e) {
            std::cout << e.what() << std::endl;
            bosdyn::api::ServiceEntry badEntry;
            return badEntry;
        }

        return reply.service_entry();
    }

    std::map<std::string, bosdyn::api::ServiceEntry> SpotBase::listAllServices() const {
        std::map<std::string, bosdyn::api::ServiceEntry> ret;
        ListServiceEntriesResponse reply;
        
        // do rpc
        try {
            reply = _directoryClient->list();
        } catch (Error &error) {
            std::cout << error.what() << std::endl;
            return ret; // return empty map
        }

        // populate map from rpc
        for (int i = 0; i < reply.service_entries_size(); i++) {
            ret.insert(std::pair<std::string, bosdyn::api::ServiceEntry>(reply.service_entries(i).name(), reply.service_entries(i)));
        }

        return ret;
    }


    void SpotBase::beginTimesync() {
        // create time sync client
        _timeSyncClient = std::shared_ptr<ClientLayer::TimeSyncClient>(new ClientLayer::TimeSyncClient(_directoryClient->getEntry(TIMESYNC_CLIENT_NAME).service_entry().authority(), _authToken));

        // create time sync thread object and kick off thread
        _timeSyncThread = std::shared_ptr<ClientLayer::TimeSyncThread>(new ClientLayer::TimeSyncThread(_timeSyncClient));
        _timeSyncThread->start();
    }

    void SpotBase::endTimesync() {
        // todo: other stuff, for now just kill thread
        _timeSyncThread->stop();
    }
};