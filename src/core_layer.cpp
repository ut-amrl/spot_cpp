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
    ClientLayer::ServiceEntry SpotBase::listService(const std::string &serviceName) const {
        GetServiceEntryResponse reply;
        try {
            reply = _directoryClient->getEntry(serviceName);
        } catch (Error &e) {
            std::cout << e.what() << std::endl;
            ClientLayer::ServiceEntry badEntry("", "", "");
            return badEntry;
        }

        ClientLayer::ServiceEntry entry(reply.service_entry().name(), reply.service_entry().type(), reply.service_entry().authority());
        return entry;
    }

    std::map<std::string, ClientLayer::ServiceEntry> SpotBase::listAllServices() const {
        std::map<std::string, ClientLayer::ServiceEntry> ret;
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
            ClientLayer::ServiceEntry newEntry(replEntry.name(), replEntry.type(), replEntry.authority());
            ret.insert(std::pair<std::string, ClientLayer::ServiceEntry>(replEntry.name(), newEntry));
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