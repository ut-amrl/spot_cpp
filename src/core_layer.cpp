#include <spot/core_layer.h>

namespace CoreLayer {
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

    const std::string SpotBase::getAuthToken() const {
        if (_authToken.empty()) {
            std::cout << "SpotBase.authenticate() must be called before getting the auth token." << std::endl;
            return "";
        }
        return _authToken;
    }

    /* Robot Id */
    const std::string SpotBase::getRobotId() const {
        RobotIdResponse reply;
        try {
            reply = _robotIdClientPtr->getId();
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
    }

    /* Directory */
    ServiceEntry SpotBase::listService(const std::string &serviceName) const {
        GetServiceEntryResponse reply;
        try {
            reply = _directoryClient->getEntry(serviceName);
        } catch (Error &e) {
            std::cout << e.what() << std::endl;
            ServiceEntry badEntry("", "", "");
            return badEntry;
        }

        ServiceEntry entry(reply.service_entry().name(), reply.service_entry().type(), reply.service_entry().authority());
        return entry;
    }

    std::map<std::string, ServiceEntry> SpotBase::listAllServices() const {
        std::map<std::string, ServiceEntry> ret;
        ListServiceEntriesResponse reply;
        
        // do rpc
        try {
            reply = _directoryClient->list();
        } catch (Error &error) {
            std::cout << e.what() << std::endl;
            return ret; // return empty map
        }

        // populate map from rpc
        for (int i = 0; i < reply.service_entries_size(); i++) {
            const bosdyn::api::ServiceEntry replEntry = reply.service_entries(i);
            ServiceEntry newEntry(replEntry.name(), replEntry.type(), replEntry.authority());
            ret.insert(std::pair<std::string, ServiceEntry>(replEntry.name(), newEntry));
        }

        return ret;
    }
};

void SpotBase::beginTimesync() {

}

void SpotBase::endTimesync() {

}