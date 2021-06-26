#include <spot/robot_layer.h>

namespace RobotLayer {
    SpotState::SpotState() {}

    SpotControl::SpotControl(std::shared_ptr<CoreLayer::SpotBase> spotBase) :
            _services(),
            _endpoints(),
            _estopThreads(),
            _leases(),
            _leaseThreads(),
            _spotBase(spotBase) {
        // initialize pointers
        std::string authToken = spotBase->getAuthToken();
        if (authToken.empty()) {
            // todo: exception handling
        }

        // initalize services map
        _services = spotBase->listAllServices();

        // initialize clients
        _estopClient = std::shared_ptr<ClientLayer::EstopClient>(new ClientLayer::EstopClient(_services.find(ESTOP_CLIENT_NAME)->second.getAuthority(), authToken));  
        _leaseClient = std::shared_ptr<ClientLayer::LeaseClient>(new ClientLayer::LeaseClient(_services.find(LEASE_CLIENT_NAME)->second.getAuthority(), authToken));        
        _powerClient = std::shared_ptr<ClientLayer::PowerClient>(new ClientLayer::PowerClient(_services.find(POWER_CLIENT_NAME)->second.getAuthority(), authToken));  
        _robotCommandClient = std::shared_ptr<ClientLayer::RobotCommandClient>(new ClientLayer::RobotCommandClient(_services.find(ROBOT_COMMAND_CLIENT_NAME)->second.getAuthority(), authToken)); 
        _spotCheckClient = std::shared_ptr<ClientLayer::SpotCheckClient>(new ClientLayer::SpotCheckClient(_services.find(SPOT_CHECK_CLIENT_NAME)->second.getAuthority(), authToken));  
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

    void SpotControl::setEstopConfiguration(const std::set<std::shared_ptr<ClientLayer::EstopEndpoint>> &endpoints, const std::string &targetConfigId) {
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
        if (it == _endpoints.end()) {
            // todo: change to exception
            std::cout << "Endpoint has not been registered or has failed to register. " << std::endl;
        }

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
        if (it == _endpoints.end()) {
           // todo: exception classes 
           std::cout << "it.end()" << std::endl;
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
        if (it == _estopThreads.end()) {
            // todo: exception hanoding
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
        if (it == _leaseThreads.end()) {
            // todo: exception hanoding
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

    void SpotControl::powerOnMotors() {
        // todo: exception handling
        // power
        PowerCommandRequest_Request pcr_r;
        pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_ON; // PowerCommandRequest_Request_REQUEST_OFF to turn off, change to _ON to turn on

        // get lease
        bosdyn::api::Lease bodyLease = _leases.find("body")->second;
        PowerCommandResponse powerCommResp = _powerClient->PowerCommand(bodyLease, pcr_r); 
        uint32_t pcID = powerCommResp.power_command_id();

        PowerCommandFeedbackResponse pcfr = _powerClient->PowerCommandFeedback(pcID);
	    while(pcfr.status() != 2){
		    pcfr = _powerClient->PowerCommandFeedback(pcID);
		    sleep(1);
	    }
    }

    void SpotControl::powerOffMotors() {
        // todo: exception handling
        // turn off
        PowerCommandRequest_Request pcr_r;
        pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_OFF;

        // get lease
        bosdyn::api::Lease bodyLease = _leases.find("body")->second;
        PowerCommandResponse powerCommResp = _powerClient->PowerCommand(bodyLease, pcr_r);
        uint32_t pcID = powerCommResp.power_command_id();

        PowerCommandFeedbackResponse pcfr = _powerClient->PowerCommandFeedback(pcID);
	    while(pcfr.status() != 2){
		    pcfr = _powerClient->PowerCommandFeedback(pcID);
		    sleep(1);
	    }
    }

    std::string SpotControl::getClockIdentifier(){
        return _spotBase->getTimeSyncThread()->getClockIdentifier();
    }

    int64_t SpotControl::getClockSkew(){
        return _spotBase->getTimeSyncThread()->getClockSkew();
    } 

    void SpotControl::sit() {
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

    void SpotControl::stand() {
        RobotCommand command;
        command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
        bosdyn::api::Lease bodyLease = _leases.find("body")->second;
        try{  
            std::string clockIdentifier = getClockIdentifier();
            RobotCommandResponse robCommResp = _robotCommandClient->robotCommand(bodyLease, command, clockIdentifier);
        } catch (Error &e){
            std::cout << e.what() << std::endl;
            return;
        }
    }

    void SpotControl::velocityMove(double x, double y, double rot, double time, gravAlignedFrame frame){
        RobotCommand command;
        SE2VelocityCommand_Request se2VelocityCommand_Request;
        std::cout << TimeUtil::GetCurrentTime() << std::endl;
        std::cout << TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + getClockSkew()) + time*75000000000)) << std::endl;
        se2VelocityCommand_Request.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + getClockSkew()) + time*75000000000)));
        se2VelocityCommand_Request.set_se2_frame_name(frameNameGravAligned(frame));
        se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_x(x);
        se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_y(y);
        se2VelocityCommand_Request.mutable_velocity()->set_angular(rot);
        command.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);
       
        bosdyn::api::Lease bodyLease = _leases.find("body")->second;
        try{  
            std::string clockIdentifier = getClockIdentifier();
            RobotCommandResponse robCommResp = _robotCommandClient->robotCommand(bodyLease, command, clockIdentifier);
            std::cout << robCommResp.status() << std::endl;
        } catch (Error &e){
            std::cout << e.what() << std::endl;
            return;
        }        
    }

    void SpotControl::trajectoryMove(Trajectory2D trajectory, gravAlignedFrame frame, double time){
        std::string frameName;
        // TODO: Make it so that flat body works as a frame 
        if(frame == FLAT_BODY){
            frameName = frameNameGravAligned(ODOM);
        }
        else{
            frameName = frameNameGravAligned(frame);
        }
        
        bosdyn::api::SE2TrajectoryCommand_Request trajectoryCommandReq;
        trajectoryCommandReq.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + getClockSkew()) + (time)*75000000000)));
        trajectoryCommandReq.set_se2_frame_name(frameName);
        trajectoryCommandReq.mutable_trajectory()->CopyFrom(trajectory.getTrajectory());
        
        RobotCommand command;
        command.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_trajectory_request()->CopyFrom(trajectoryCommandReq);
        
        bosdyn::api::Lease bodyLease = _leases.find("body")->second;
        try{  
            std::string clockIdentifier = getClockIdentifier();
            RobotCommandResponse robCommResp = _robotCommandClient->robotCommand(bodyLease, command, clockIdentifier);
        } catch (Error &e){
            std::cout << e.what() << std::endl;
            return;
        }  
    }

}