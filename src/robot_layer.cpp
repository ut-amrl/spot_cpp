#include <spot/robot_layer.h>

namespace RobotLayer {
SpotState::SpotState(std::shared_ptr<CoreLayer::SpotBase> spotBase) :
        _spotBase(spotBase) {
    // initialize pointers
    std::string authToken = spotBase->getAuthToken();

    try {
        spotBase->notAuthenticated();
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        // probably need to do something here?
        return;
    }

    // initalize services map
    _services = spotBase->listAllServices();

    // initialize clients
    _imageClient = std::shared_ptr<ClientLayer::ImageClient>(new ClientLayer::ImageClient(_services.find(IMAGE_CLIENT_NAME)->second.authority(), authToken));  
    //_localGridClient = std::shared_ptr<ClientLayer::LocalGridClient>(new ClientLayer::LocalGridClient(_services.find(LOCAL_GRID_CLIENT_NAME)->second.authority(), authToken));        
    _robotStateClient = std::shared_ptr<ClientLayer::RobotStateClient>(new ClientLayer::RobotStateClient(_services.find(ROBOT_STATE_CLIENT_NAME)->second.authority(), authToken));  
    _worldObjectsClient = std::shared_ptr<ClientLayer::WorldObjectsClient>(new ClientLayer::WorldObjectsClient(_services.find(WORLD_OBJECTS_CLIENT_NAME)->second.authority(), authToken)); 
}

bosdyn::api::ImageResponse SpotState::image(const std::string &sourceName, double qualityPercent, bosdyn::api::Image_Format format) {
    // build image request
    ImageRequest request;
    request.set_image_source_name(sourceName);
    request.set_quality_percent(qualityPercent);
    request.set_image_format(format);

    // send image request
    GetImageResponse reply;
    try {
        std::vector<ImageRequest> oneRequest;
        oneRequest.push_back(request);
        reply = _imageClient->getImage(oneRequest);
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        bosdyn::api::ImageResponse empty;
        return empty;
    }

    // return
    return reply.image_responses(0);
}

std::list<bosdyn::api::ImageSource> SpotState::imageSources() {
    // send req
    ListImageSourcesResponse reply;
    try {
        reply = _imageClient->listImageSources();
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        std::list<bosdyn::api::ImageSource> empty;
        return empty;
    }

    // create list
    std::list<bosdyn::api::ImageSource> ret;
    for (int i = 0; i < reply.image_sources_size(); i++) {
        ret.push_back(reply.image_sources(i));
    }

    // return
    return ret;
}

bosdyn::api::RobotState SpotState::robotState() {
    // send req
    RobotStateResponse reply;
    try {
        reply = _robotStateClient->getRobotState();
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        bosdyn::api::RobotState empty;
        return empty;
    }

    return reply.robot_state();
}

bosdyn::api::RobotMetrics SpotState::robotMetrics() {
    RobotMetricsResponse reply;
    try {
        reply = _robotStateClient->getRobotMetrics();
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        bosdyn::api::RobotMetrics empty;
        return empty;
    }

    return reply.robot_metrics();
}

bosdyn::api::HardwareConfiguration SpotState::robotHardwareConfiguration() {
    RobotHardwareConfigurationResponse reply;
    try {

    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        bosdyn::api::HardwareConfiguration empty;
        return empty;
    }

    return reply.hardware_configuration();
}

std::list<bosdyn::api::WorldObject> SpotState::worldObjects() {
    ListWorldObjectResponse reply;
    try {
        reply = _worldObjectsClient->listWorldObjects();
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        std::list<bosdyn::api::WorldObject> empty;
        return empty;
    }

    // ret
    std::list<bosdyn::api::WorldObject> ret;
    for (int i = 0; i < reply.world_objects_size(); i++) {
        ret.push_back(reply.world_objects(i));
    }

    return ret;
}

bool SpotState::mutateWorldObject(bosdyn::api::WorldObject object, bosdyn::api::MutateWorldObjectRequest_Action action) {
    // create mutation
    MutateWorldObjectRequest_Mutation mutation;
    mutation.set_action(action);
    mutation.mutable_object()->CopyFrom(object);

    // send rpc
    MutateWorldObjectResponse reply;
    try {   
        reply = _worldObjectsClient->mutateWorldObjects(mutation);
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        return false;
    }

    return reply.status() == 1;
}

SpotControl::SpotControl(std::shared_ptr<CoreLayer::SpotBase> spotBase) :
        _services(),
        _endpoints(),
        _estopThreads(),
        _leases(),
        _leaseThreads(),
        _spotBase(spotBase),
        _standing(false),
        _moving(false) {
    // initialize pointers
    std::string authToken = spotBase->getAuthToken();
    
    try {
        spotBase->notAuthenticated();
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        // probably need to do something here?
        return;
    }

    // initalize services map
    _services = spotBase->listAllServices();

    // initialize clients
    _estopClient = std::shared_ptr<ClientLayer::EstopClient>(new ClientLayer::EstopClient(_services.find(ESTOP_CLIENT_NAME)->second.authority(), authToken));  
    _leaseClient = std::shared_ptr<ClientLayer::LeaseClient>(new ClientLayer::LeaseClient(_services.find(LEASE_CLIENT_NAME)->second.authority(), authToken));        
    _powerClient = std::shared_ptr<ClientLayer::PowerClient>(new ClientLayer::PowerClient(_services.find(POWER_CLIENT_NAME)->second.authority(), authToken));  
    _robotCommandClient = std::shared_ptr<ClientLayer::RobotCommandClient>(new ClientLayer::RobotCommandClient(_services.find(ROBOT_COMMAND_CLIENT_NAME)->second.authority(), authToken)); 
    _spotCheckClient = std::shared_ptr<ClientLayer::SpotCheckClient>(new ClientLayer::SpotCheckClient(_services.find(SPOT_CHECK_CLIENT_NAME)->second.authority(), authToken));  
}

bool SpotControl::estopped() {
    GetEstopSystemStatusResponse reply;
    try {
        reply = _estopClient->getStatus();
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        throw; // may change later depending on if this is the last stop for errors
    }

    // check if we are not at stop level none
    return reply.status().stop_level() != bosdyn::api::ESTOP_LEVEL_NONE;
}

void SpotControl::setEstopConfiguration(const std::set<std::shared_ptr<ClientLayer::EstopEndpoint>> &endpoints) {
    // create config
    EstopConfig config; 

    // iterate through set of endpoints
    for (auto estopEndpoint : endpoints) {
        // add endpoint
        EstopEndpoint *endpoint = config.add_endpoints();

        // only set role and name (will get replaced by actual endpoint)
        endpoint->set_role(estopEndpoint->getRole());
        // endpoint->set_name(estopEndpoint->getName());
        endpoint->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(estopEndpoint->getEstopTimeout()));
    }

    // set config
    SetEstopConfigResponse reply;
    try {
        reply = _estopClient->setConfig(config); 

        if (reply.status() == 2) {
            reply = _estopClient->setConfig(config, reply.active_config().unique_id());
        }
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        return;
    }

    // get config id
    _estopConfigId = reply.active_config().unique_id();
}

std::string SpotControl::registerEstopEndpoint(const std::string &name, const std::string &role, const std::string &configId, 
        int64_t estopTimeout, int64_t estopCutPowerTimeout) {
    // check if we are registering to instance config or another config
    std::string temp = configId.empty() ? _estopConfigId : configId;
    
    EstopEndpoint fromEndpoint;
    // create new endpoint obj w/ blank unique id (assigned by server)
    try {
        // create new endpoint
        std::shared_ptr<ClientLayer::EstopEndpoint> ptr(new ClientLayer::EstopEndpoint(_estopClient, name, role, temp, "", estopTimeout, estopCutPowerTimeout));

        // convert to proto representation
        bosdyn::api::EstopEndpoint fromEndpoint = ptr->toProto();
        
        // create temp endpoint to replace
        bosdyn::api::EstopEndpoint endpointToReplace;
        endpointToReplace.set_role(role);

        // rpc and set new unique id
        RegisterEstopEndpointResponse reply = _estopClient->replaceEndpoint(temp, endpointToReplace, fromEndpoint);
        ptr->setUniqueId(reply.new_endpoint().unique_id());
        
        // insert into map
        _endpoints.insert(std::pair<std::string, std::shared_ptr<ClientLayer::EstopEndpoint>>(ptr->getUniqueId(), ptr));
        return ptr->getUniqueId();
    } catch (Error &e) {
        std::cout << e.what() << std::endl;
        return "";
    }
}

void SpotControl::deregisterEstopEndpoint(const std::string &uniqueId, const std::string &targetConfigId) {
    // check config id
    std::string temp = targetConfigId.empty() ? _estopConfigId : targetConfigId;

    // find in map
    auto it = _endpoints.find(uniqueId);
    try {
        if (it == _endpoints.end()) {
            // todo: change to exception
            throw "Endpoint has not been registered or has failed to register. ";
        }
    } catch(Error &e) {
        std::cout << e.what() << std::endl;
        return;
    }
    // if (it == _endpoints.end()) {
    //     // todo: change to exception
    //     std::cout << "Endpoint has not been registered or has failed to register. " << std::endl;
    // }

    // deregister from spot
    EstopEndpoint endpointToDeregister = it->second->toProto();
    DeregisterEstopEndpointResponse reply;
    try {
        reply = _estopClient->deregister(temp, endpointToDeregister);
    } catch (Error &error) {
        std::cout << error.what() << std::endl;
        return;
    }

    // remove from endpoints
    _endpoints.erase(it);
}

void SpotControl::beginEstopping(const std::string &uniqueId) {
    auto it = _endpoints.find(uniqueId);
    // if (it == _endpoints.end()) {
    //     // todo: exception classes 
    //     std::cout << "it.end()" << std::endl;
    // }
    try {
        if(it == _endpoints.end()) {
            throw "Endpoint has not been registered or has failed to register.";
        }
    } catch(Error e) {
        std::cout << e.what() << std::endl;
        return;
    }

    // create estop thread
    std::shared_ptr<ClientLayer::EstopThread> ptr = std::shared_ptr<ClientLayer::EstopThread>(new ClientLayer::EstopThread(_estopClient, it->second));

    // add thread to map
    _estopThreads.insert(std::pair<std::string, std::shared_ptr<ClientLayer::EstopThread>>(it->second->getUniqueId(), ptr));

    // kick off estop
    ptr->beginEstop();
}

// todo: fix
void SpotControl::beginEstopping() {
    for (const auto &endpoint : _endpoints) {
        // create estop thread
        std::shared_ptr<ClientLayer::EstopThread> ptr = std::shared_ptr<ClientLayer::EstopThread>(new ClientLayer::EstopThread(_estopClient, endpoint.second));

        // add thread to map
        _estopThreads.insert(std::pair<std::string, std::shared_ptr<ClientLayer::EstopThread>>(endpoint.second->getName(), ptr));

        // kick off estop
        ptr->beginEstop();
    }
}

void SpotControl::endEstopping(const std::string &uniqueId) {
    auto it = _estopThreads.find(uniqueId);
    // if (it == _estopThreads.end()) {
    //     // todo: exception hanoding
    // }
    try {
        if(it == _estopThreads.end()) {
            throw "Endpoint has not been registered or has failed to register.";
        }
    } catch(Error e) {
        std::cout << e.what() << std::endl;
        return;
    }

    // end estop
    it->second->endEstop();

    // delete from thread map
    _estopThreads.erase(it);
}

void SpotControl::endEstopping() {
    // end estopping for all threads
    for (const auto &thread : _estopThreads) {
        thread.second->endEstop();            
    }

    // erase all entries from map
    _estopThreads.clear();
}

void SpotControl::acquireLease(const std::string &resource) {
    AcquireLeaseResponse reply;

    // acquire lease for given resource
    try {
        reply = _leaseClient->acquire(resource);
    } catch (Error &error) {
        std::cout << error.what() << std::endl;
        return;
    }

    // check if already claimed
    Lease acquiredLease;
    switch (reply.status()) {
        case 0: 
            break;
        case 1:
            acquiredLease = reply.lease();
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        default:
            ;
    }

    // add to lease map
    _leases.insert(std::pair<std::string, Lease>(resource, acquiredLease));
}

void SpotControl::beginLeasing(const std::string &resource) {
    auto it = _leases.find(resource);
    if (it == _leases.end()) {

    }

    // create thread
    std::shared_ptr<ClientLayer::LeaseThread> thread = std::shared_ptr<ClientLayer::LeaseThread>(new ClientLayer::LeaseThread(_leaseClient, it->second));

    // add thread to thread map
    _leaseThreads.insert(std::pair<std::string, std::shared_ptr<ClientLayer::LeaseThread>>(resource, thread));

    // kick off thread
    thread->beginLease();
}

// todo: fix
void SpotControl::beginLeasing() {
    for (const auto &lease : _leases) {
        // create thread
        std::shared_ptr<ClientLayer::LeaseThread> thread = std::shared_ptr<ClientLayer::LeaseThread>(new ClientLayer::LeaseThread(_leaseClient, lease.second));

        // add to thread map
        _leaseThreads.insert(std::pair<std::string, std::shared_ptr<ClientLayer::LeaseThread>>(lease.first, thread));

        // kick off thread
        thread->beginLease();
    }
}

void SpotControl::endLeasing(const std::string &resource) {
    auto it = _leaseThreads.find(resource);
    // if (it == _leaseThreads.end()) {
    //     // todo: exception hanoding
    // }
    try {
        if(it == _leaseThreads.end()) {
            throw "Endpoint has not been registered or has failed to register.";
        }
    } catch(Error e) {
        std::cout << e.what() << std::endl;
        return;
    }

    // end lease
    it->second->endLease();

    // delete from thread map
    _leaseThreads.erase(it);
}

void SpotControl::endLeasing() {
    // end leasing for all threads
    for (const auto &thread : _leaseThreads) {
        thread.second->endLease();            
    }

    // erase all entries from map
    _leaseThreads.clear();
}

bool SpotControl::poweredOn() {
    return false; 
}

uint32_t SpotControl::powerOnMotors() {
    // todo: exception handling 
    // power
    PowerCommandRequest_Request pcr_r;
    pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_ON; // PowerCommandRequest_Request_REQUEST_OFF to turn off, change to _ON to turn on

    // get lease
    bosdyn::api::Lease bodyLease = _leases.find("body")->second;
    if (bodyLease.resource().find("motor") == std::string::npos) {
        throw "not leasing to motors?";
    }
    PowerCommandResponse powerCommResp = _powerClient->PowerCommand(bodyLease, pcr_r);
    if(powerCommResp >= 3 && powerCommResp <= 9) {
        throw "Error" + powerCommResp;
    }
    uint32_t pcID = powerCommResp.power_command_id();

    return pcID;
}

uint32_t SpotControl::powerOffMotors() {
    // todo: exception handling
    // turn off
    PowerCommandRequest_Request pcr_r;
    pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_OFF;

    // get lease
    bosdyn::api::Lease bodyLease = _leases.find("body")->second;
    // look at protos, check to see lease is present
    if (bodyLease.resource().find("motor") == std::string::npos) {
        throw "not leasing to motors?";
    }
    PowerCommandResponse powerCommResp = _powerClient->PowerCommand(bodyLease, pcr_r);
    // no need to err here
    uint32_t pcID = powerCommResp.power_command_id();

    return pcID;
}

std::string SpotControl::getClockIdentifier(){
    return _spotBase->getTimeSyncThread()->getEndpoint()->getClockIdentifier();
}

google::protobuf::Duration SpotControl::getClockSkew(){
    return _spotBase->getTimeSyncThread()->getEndpoint()->clockSkew();
} 

RobotCommandResponse SpotControl::stand() {
    _standing = false;
    RobotCommand command;
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();  
    bosdyn::api::Lease bodyLease = _leases.find("body")->second;    
    try{  
        std::string clockIdentifier = getClockIdentifier();
        RobotCommandResponse robCommResp = _robotCommandClient->robotCommand(bodyLease, command, clockIdentifier);
    } catch (Error &e){
        std::cout << e.what() << std::endl;
        return;
    }
}

RobotCommandResponse SpotControl::stand() {
    _standing = true;
    RobotCommand command;
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
    Any any;
    any.PackFrom(_mobilityParams);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->CopyFrom(any);
    bosdyn::api::Lease bodyLease = _leases.find("body")->second;
    try{  
        std::string clockIdentifier = getClockIdentifier();
        RobotCommandResponse robCommResp = _robotCommandClient->robotCommand(bodyLease, command, clockIdentifier);
    } catch (Error &e){
        std::cout << e.what() << std::endl;
        return;
    }
}

RobotCommandResponse SpotControl::velocityMove(double x, double y, double rot, int64_t time, gravAlignedFrame frame){
    RobotCommand command;
    SE2VelocityCommand_Request se2VelocityCommand_Request;

    // get robot clock from local timestamp
    google::protobuf::Timestamp endTime = _spotBase->getTimeSyncThread()->getEndpoint()->robotTimestampFromLocalTimestamp(TimeUtil::GetCurrentTime());
    
    // add the user-specified time
    endTime = endTime += TimeUtil::MillisecondsToDuration(time);

    se2VelocityCommand_Request.mutable_end_time()->CopyFrom(endTime);
    se2VelocityCommand_Request.set_se2_frame_name(frameNameGravAligned(frame));
    se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_x(x);
    se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_y(y);
    se2VelocityCommand_Request.mutable_velocity()->set_angular(rot);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);
    
    Any any;
    any.PackFrom(_mobilityParams);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->CopyFrom(any);

    bosdyn::api::Lease bodyLease = _leases.find("body")->second;
    try{  
        std::string clockIdentifier = getClockIdentifier();
        RobotCommandResponse robCommResp = _robotCommandClient->robotCommand(bodyLease, command, clockIdentifier);
        std::cout << "robot command status: " << robCommResp.status() << std::endl;
    } catch (Error &e){
        std::cout << e.what() << std::endl;
        return;
    }        
}

RobotCommandResponse SpotControl::trajectoryMove(Trajectory2D trajectory, gravAlignedFrame frame, int64_t time){
    std::string frameName;
    // TODO: Make it so that flat body works as a frame 
    if (frame == FLAT_BODY) {
        frameName = frameNameGravAligned(ODOM);
    }
    else {
        frameName = frameNameGravAligned(frame);
    }
    
    // get robot clock from local timestamp
    google::protobuf::Timestamp endTime = _spotBase->getTimeSyncThread()->getEndpoint()->robotTimestampFromLocalTimestamp(TimeUtil::GetCurrentTime());
    endTime = endTime += TimeUtil::MillisecondsToDuration(time);
    
    bosdyn::api::SE2TrajectoryCommand_Request trajectoryCommandReq;
    trajectoryCommandReq.mutable_end_time()->CopyFrom(endTime);
    trajectoryCommandReq.set_se2_frame_name(frameName);
    trajectoryCommandReq.mutable_trajectory()->CopyFrom(trajectory.getTrajectory());
    
    RobotCommand command;
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_trajectory_request()->CopyFrom(trajectoryCommandReq);
    
    Any any;
    any.PackFrom(_mobilityParams);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->CopyFrom(any);

    bosdyn::api::Lease bodyLease = _leases.find("body")->second;
    try{  
        std::string clockIdentifier = getClockIdentifier();
        RobotCommandResponse robCommResp = _robotCommandClient->robotCommand(bodyLease, command, clockIdentifier);
    } catch (Error &e){
        std::cout << e.what() << std::endl;
        return;
    }  
}

void SpotControl::setMobilityParams(MobilityParams mParams){
    _mobilityParams = mParams;
}
}