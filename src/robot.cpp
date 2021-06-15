#include <spot/robot.h>

Robot::Robot(const std::string name) : _name(name) {}

template<class client_T>
void Robot::cacheClient(CLIENT_TYPES type) {
    client_T client;
    std::shared_ptr<client_T> ptr(&client);
    _clients.insert(std::pair<CLIENT_TYPES, CLIENT_CONTAINER>(type, ptr));
}

void Robot::cacheDirectoryClient(const std::string &token) {
    // create client
    DirectoryClient client(token);
    std::shared_ptr<DirectoryClient> ptr(&client);
    _clients.insert(std::pair<CLIENT_TYPES, CLIENT_CONTAINER>(CLIENT_TYPES::DIRECTORY, ptr));
}

template <class client_T>
void Robot::cacheClient(CLIENT_TYPES type, const std::string &clientName, const std::string &token) {
    // get directory client
    std::shared_ptr<DirectoryClient> ptr = std::get<DirectoryClient>(_clients.at(CLIENT_TYPES::DIRECTORY));
    DirectoryClient &dClient = *ptr;
    
    client_T client(dClient.getEntry(clientName).service_entry().authority(), token);
    std::shared_ptr<client_T> nptr(&client);
    _clients.insert(std::pair<CLIENT_TYPES, CLIENT_CONTAINER>(type, nptr));
}

std::string Robot::getId() {
    // check if robotid client already cached
    if (_clients.find(CLIENT_TYPES::ROBOT_ID) == _clients.end()) {
       cacheClient<RobotIdClient>(CLIENT_TYPES::ROBOT_ID);
    }
    // get information and populate string
    std::shared_ptr<RobotIdClient> ptr = boost::any_cast<std::shared_ptr<RobotIdClient>>(_clients.at(CLIENT_TYPES::ROBOT_ID));
    RobotIdClient &client = *ptr;
    RobotIdResponse reply = client.getId();
    RobotId id = reply.robot_id();

    // populate ret string (disgusting concat)
    std::string ret = "[SERIAL NUMBER]: " + id.serial_number();
    ret += "\n[SPECIES]: " + id.species();
    ret += "\n[VERSION]: " + id.version();
    ret += "\n[NAME]: " + id.software_release().name();
    ret += "\n[NICKNAME]: " + id.nickname() + "\n";
    return ret;
}

void Robot::authenticate(const std::string &username, const std::string &password) {
    // check if authclient already cached
    if (_clients.find(CLIENT_TYPES::AUTH) == _clients.end()) {
        cacheClient<AuthClient>(CLIENT_TYPES::AUTH);
    }

    // get back token
    std::shared_ptr<AuthClient> ptr = boost::any_cast<std::shared_ptr<AuthClient>>(_clients.at(CLIENT_TYPES::AUTH));
    AuthClient &client = *ptr;
    GetAuthTokenResponse reply = client.auth(username, password);
    _token = reply.token();
}

void Robot::authenticateWithToken(const std::string &token) {
    // check if authclient already cached
    if (_clients.find(CLIENT_TYPES::AUTH) == _clients.end()) {
        cacheClient<AuthClient>(CLIENT_TYPES::AUTH);
    }

    // get back token
    std::shared_ptr<AuthClient> ptr = boost::any_cast<std::shared_ptr<AuthClient>>(_clients.at(CLIENT_TYPES::AUTH));
    AuthClient &client = *ptr;
    GetAuthTokenResponse reply = client.authWithToken(token);
    _token = reply.token();
}

void Robot::updateToken(const std::string &token) {
    _token = token;
}

template <class client_T>
std::shared_ptr<client_T> Robot::ensureClient(CLIENT_TYPES type, const std::string &clientName) {
    // check that robot is authenticated
    if (_token.empty()) {
        throw 1; // change later
    }
    
    // check that the directory client exists
    if (_clients.find(CLIENT_TYPES::DIRECTORY) == _clients.end()) {
        cacheDirectoryClient(_token);
    }
    
    // check that we are not ensuring auth, robotid, or directory
    if (type == CLIENT_TYPES::AUTH || type == CLIENT_TYPES::DIRECTORY || type == CLIENT_TYPES::ROBOT_ID) {
        std::shared_ptr<client_T> ptr = boost::any_cast<std::shared_ptr<client_T>>(_clients.at(type));
        return ptr;
    }
    
    // check that client doesn't exist in cache
    auto it = _clients.find(type);
    if (it == _clients.end()) {
        // create and cache if doesn't exist
        cacheClient<client_T>(type, clientName, _token);
    }
    
    // return client
    std::shared_ptr<client_T> ptr = boost::any_cast<std::shared_ptr<client_T>>(it->second);
    return ptr;
}

template <class client_T>
std::shared_ptr<client_T> Robot::getClient(CLIENT_TYPES type) {
    auto it = _clients.find(type);
    if (it == _clients.end()) {
        throw 1; // change later
    } else {
        // return client
        std::shared_ptr<client_T> ptr = boost::any_cast<std::shared_ptr<client_T>>(it->second);
        return ptr;
    }
}

void Robot::powerOn() {
    // check that the robot is off
    if (_isOn) {
        throw 1; // change later
    }

    // check that the robot is authenticated
    if (_token.empty()) {
        throw 1; // change later
    }
    
    // check that the robot is setup
    if (_clients.find(CLIENT_TYPES::DIRECTORY) == _clients.end()) {
        cacheDirectoryClient(_token);
    }
    
    // ensure estop, lease, and power clients
    std::shared_ptr<EstopClient> estopPtr = ensureClient<EstopClient>(CLIENT_TYPES::ESTOP, ESTOP_CLIENT_NAME);
    std::shared_ptr<LeaseClient> leasePtr = ensureClient<LeaseClient>(CLIENT_TYPES::LEASE, LEASE_CLIENT_NAME);
    std::shared_ptr<PowerClient> powerPtr = ensureClient<PowerClient>(CLIENT_TYPES::POWER, POWER_CLIENT_NAME);
    std::shared_ptr<TimeSyncClient> timePtr = ensureClient<TimeSyncClient>(CLIENT_TYPES::TIMESYNC, TIMESYNC_CLIENT_NAME);

    // acquire lease
    AcquireLeaseResponse leaseReply = leasePtr->acquire("body");
    std::shared_ptr<Lease> lease(new Lease(leaseReply.lease()));

    // estop config and endpoint
    EstopConfig estopConfig;
    std::shared_ptr<EstopEndpoint> endpointPtr(estopConfig.add_endpoints());
    endpointPtr->set_role("PDB_rooted");
    endpointPtr->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
    SetEstopConfigResponse estopConfigReply = estopPtr->setConfig(estopConfig);
    if(estopConfigReply.status() == 2){
		estopConfigReply = estopPtr->setConfig(estopConfig, estopConfigReply.active_config().unique_id());
	}
    std::string activeConfigId = estopConfigReply.active_config().unique_id();

    EstopEndpoint newEndpoint;
    newEndpoint.set_role("PDB_rooted");
    newEndpoint.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
    EstopEndpoint targetEndpoint;
	targetEndpoint.set_role("PDB_rooted");
    RegisterEstopEndpointResponse regEndResp = estopPtr->registerEndpoint(activeConfigId, targetEndpoint, newEndpoint);
    EstopEndpoint activeEndpoint = regEndResp.new_endpoint();  
    std::shared_ptr<EstopEndpoint> activeEndpointPtr(&activeEndpoint);

    // estop stop level
    EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;
    std::shared_ptr<EstopStopLevel> stopPtr(&stopLevel);
    
    // create keep alive threads for estop, lease, timesync
    EstopKeepAlive estopThread(estopPtr, activeEndpointPtr, stopPtr, 0, 0, 0); // challenge is 0
    LeaseKeepAlive leaseThread(leasePtr, lease, 0); // set rpcinterval to 0 for now

    // power
	PowerCommandRequest_Request pcr_r;
	pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_ON; // PowerCommandRequest_Request_REQUEST_OFF to turn off, change to _ON to turn on
	PowerCommandResponse powerCommResp = powerPtr->PowerCommand(leaseReply.lease(), pcr_r); 

    _isOn = true;
}

void Robot::powerOff() {
    // check that robot is on
    if (!_isOn) {
        throw 1; // change later
    }

    // get power client
    std::shared_ptr<PowerClient> powerPtr = getClient<PowerClient>(CLIENT_TYPES::POWER);

    _isOn = false;
}

void Robot::startTimeSync() {
    // create thread w/ keepalive object
}

void Robot::stopTimeSync() {
    // kill thread (do other stuff probably)
}

void Robot::getTime() {
    // query timesync service
}

bool Robot::isPoweredOn() {
    return _isOn;
}

bool Robot::isEstopped() {
    return _isEstopped;
}

State::State(const std::string &name, const std::string &address, const std::string &serial) {

}

State Robot::getState() {

}

void Robot::print_cache() {
    for (auto const& [key, val] : _clients) {
        std::cout << key
                  << ':'
                //   << val->getClientName();
                  << std::endl;
    }
}

