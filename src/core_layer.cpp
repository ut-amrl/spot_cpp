#include <spot/core_layer.h>

namespace CoreLayer {

    int SpotBase::DEFAULT_TIME_SYNC_INTERVAL_SECS = 60;
    int SpotBase::DEFAULT_TIME_SYNC_NOT_READY_INTERVAL_SECS = 5;

    SpotBase::SpotBase() {
        // initialize pointers
        _authClient = std::shared_ptr<AuthClient>(new AuthClient());
        _robotIdClient = std::shared_ptr<RobotIdClient>(new RobotIdClient());
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
    CoreLayer::ServiceEntry SpotBase::listService(const std::string &serviceName) const {
        GetServiceEntryResponse reply;
        try {
            reply = _directoryClient->getEntry(serviceName);
        } catch (Error &e) {
            std::cout << e.what() << std::endl;
            CoreLayer::ServiceEntry badEntry("", "", "");
            return badEntry;
        }

        CoreLayer::ServiceEntry entry(reply.service_entry().name(), reply.service_entry().type(), reply.service_entry().authority());
        return entry;
    }

    std::map<std::string, CoreLayer::ServiceEntry> SpotBase::listAllServices() const {
        std::map<std::string, CoreLayer::ServiceEntry> ret;
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
            const bosdyn::api::ServiceEntry replEntry = reply.service_entries(i);
            CoreLayer::ServiceEntry newEntry(replEntry.name(), replEntry.type(), replEntry.authority());
            ret.insert(std::pair<std::string, CoreLayer::ServiceEntry>(replEntry.name(), newEntry));
        }

        return ret;
    }


    void SpotBase::beginTimesync() {
        // do initial rpc
        bosdyn::api::TimeSyncUpdateResponse reply;
        try {
            reply = _timeSyncClient->getTimeSyncUpdate();
        } catch (Error &error) {
            std::cout << error.what() << std::endl;
            return;
        }

        std::string clockIdentifier = reply.clock_identifier();
        int64_t clockSkew;

        // send rpcs until synchronized
        while (reply.state().status() == 2 || reply.state().status() == 3) {
            // send new rpc and set clockskew
            reply = _timeSyncClient->getTimeSyncUpdate(createTrip(reply), clockIdentifier);
            clockSkew  = TimeUtil::DurationToSeconds(reply.state().best_estimate().clock_skew());
            std::this_thread::sleep_for(std::chrono::seconds(DEFAULT_TIME_SYNC_NOT_READY_INTERVAL_SECS));
        }

        // create time sync thread object and kick off thread
        _timeSyncThread = std::shared_ptr<TimeSyncThread>(new TimeSyncThread(_timeSyncClient, clockIdentifier, clockSkew));
        _timeSyncThread->beginTimeSync();
    }

    void SpotBase::endTimesync() {
        // todo: other stuff, for now just kill thread
        _timeSyncThread->endTimeSync();
    }
};