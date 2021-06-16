#include <spot/robot.h>

Robot::Robot(const std::string &name) : 
        _name(name),
        _authClientPtr(std::shared_ptr<AuthClient>(new AuthClient)),
        _robotIdClientPtr(std::shared_ptr<RobotIdClient>(new RobotIdClient)) {
}

void Robot::setup() {
    if (_token.empty()) {
        std::cout << "token not set (did you forget to authenticate?)" << std::endl;
        throw 1; // change later
    }
    _directoryClientPtr = std::shared_ptr<DirectoryClient>(new DirectoryClient(_token));
    _estopClientPtr = std::shared_ptr<EstopClient>(new EstopClient(_directoryClientPtr->getEntry(ESTOP_CLIENT_NAME).service_entry().authority(), _token));
    _imageClientPtr =std::shared_ptr<ImageClient>(new ImageClient(_directoryClientPtr->getEntry(IMAGE_CLIENT_NAME).service_entry().authority(), _token));
    _leaseClientPtr = std::shared_ptr<LeaseClient>(new LeaseClient(_directoryClientPtr->getEntry(LEASE_CLIENT_NAME).service_entry().authority(), _token));
    _powerClientPtr = std::shared_ptr<PowerClient>(new PowerClient(_directoryClientPtr->getEntry(POWER_CLIENT_NAME).service_entry().authority(), _token));
    _robotCommandClientPtr = std::shared_ptr<RobotCommandClient>(new RobotCommandClient(_directoryClientPtr->getEntry(ROBOT_COMMAND_CLIENT_NAME).service_entry().authority(), _token));
    _spotCheckClientPtr = std::shared_ptr<SpotCheckClient>(new SpotCheckClient(_directoryClientPtr->getEntry(SPOT_CHECK_CLIENT_NAME).service_entry().authority(), _token));
    _timesyncClientPtr = std::shared_ptr<TimeSyncClient>(new TimeSyncClient(_directoryClientPtr->getEntry(TIMESYNC_CLIENT_NAME).service_entry().authority(), _token));
}

void Robot::authenticate(const std::string &username, const std::string &password) {
    _token = _authClientPtr->auth(username, password).token();
    if (_token.empty()) {
        std::cout << "incorrect credentials" << std::endl;
        throw 1;
    }
}

std::string Robot::getId() {
    // check if robotid client already cached
    RobotIdResponse reply = _robotIdClientPtr->getId();
    RobotId id = reply.robot_id();

    // populate ret string (disgusting concat)
    std::string ret = "[SERIAL NUMBER]: " + id.serial_number();
    ret += "\n[SPECIES]: " + id.species();
    ret += "\n[VERSION]: " + id.version();
    ret += "\n[NAME]: " + id.software_release().name();
    ret += "\n[NICKNAME]: " + id.nickname() + "\n";
    return ret;
}

void Robot::initBasicEstop(){
    // estop config and endpoint
    EstopConfig estopConfig;
    EstopEndpoint* endpointPtr = estopConfig.add_endpoints();
    endpointPtr->set_role("PDB_rooted");
    endpointPtr->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
    SetEstopConfigResponse estopConfigReply = _estopClientPtr->setConfig(estopConfig);
    if(estopConfigReply.status() == 2){
		estopConfigReply = _estopClientPtr->setConfig(estopConfig, estopConfigReply.active_config().unique_id());
	}
    std::string activeConfigId = estopConfigReply.active_config().unique_id();

    EstopEndpoint newEndpoint;
    newEndpoint.set_role("PDB_rooted");
    newEndpoint.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
    EstopEndpoint targetEndpoint;
	targetEndpoint.set_role("PDB_rooted");
    RegisterEstopEndpointResponse regEndResp = _estopClientPtr->registerEndpoint(activeConfigId, targetEndpoint, newEndpoint);
    EstopEndpoint activeEndpoint = regEndResp.new_endpoint();  

    // estop stop level
    EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;

    _estopThread = std::shared_ptr<EstopKeepAlive>(new EstopKeepAlive(_estopClientPtr, activeEndpoint, stopLevel, 0, 0, 0));
}

void Robot::initBasicLease() {
    // acquire lease
    AcquireLeaseResponse leaseReply = _leaseClientPtr->acquire("body");
    std::shared_ptr<Lease> lease(new Lease(leaseReply.lease()));

    // create thread
    _leaseThread = std::shared_ptr<LeaseKeepAlive>(new LeaseKeepAlive(_leaseClientPtr, *lease, 0));
}

void Robot::powerOn() {
    // // check that the robot is off
    // if (_isOn) {
    //     throw 1; // change later
    // }
    
    // // acquire lease
    // AcquireLeaseResponse leaseReply = _leaseClientPtr->acquire("body");
    // std::shared_ptr<Lease> lease(new Lease(leaseReply.lease()));

    // // estop config and endpoint
    // EstopConfig estopConfig;
    // std::shared_ptr<EstopEndpoint> endpointPtr = estopConfig.add_endpoints();
    // endpointPtr->set_role("PDB_rooted");
    // endpointPtr->mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
    // SetEstopConfigResponse estopConfigReply = _estopClientPtr->setConfig(estopConfig);
    // if(estopConfigReply.status() == 2){
	// 	estopConfigReply = estopPtr->setConfig(estopConfig, estopConfigReply.active_config().unique_id());
	// }
    // std::string activeConfigId = estopConfigReply.active_config().unique_id();

    // EstopEndpoint newEndpoint;
    // newEndpoint.set_role("PDB_rooted");
    // newEndpoint.mutable_timeout()->CopyFrom(TimeUtil::SecondsToDuration(30));
    // EstopEndpoint targetEndpoint;
	// targetEndpoint.set_role("PDB_rooted");
    // RegisterEstopEndpointResponse regEndResp = _estopClientPtr->registerEndpoint(activeConfigId, targetEndpoint, newEndpoint);
    // EstopEndpoint activeEndpoint = regEndResp.new_endpoint();  
    // std::shared_ptr<EstopEndpoint> activeEndpointPtr(&activeEndpoint);

    // // estop stop level
    // EstopStopLevel stopLevel = EstopStopLevel::ESTOP_LEVEL_NONE;
    // std::shared_ptr<EstopStopLevel> stopPtr(&stopLevel);
    
    // // create keep alive threads for estop, lease, timesync
    // EstopKeepAlive estopThread(_estopClientPtr, activeEndpointPtr, stopPtr, 0, 0, 0); // challenge is 0
    // LeaseKeepAlive leaseThread(_leaseClientPtr, lease, 0); // set rpcinterval to 0 for now

    // // power
	// PowerCommandRequest_Request pcr_r;
	// pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_ON; // PowerCommandRequest_Request_REQUEST_OFF to turn off, change to _ON to turn on
	// PowerCommandResponse powerCommResp = _powerClientPtr->PowerCommand(leaseReply.lease(), pcr_r); 

    // _isOn = true;
}

boolean move(movementType mType, double x, double y, double rot, double time, int64_t clockSkew, std::string timeSyncClockId){
	if (_robotCommandClientPtr == NULL){
        std::cout << "Need to setup" << std::endl; 
        throw 1;
    } // TODO: change later 

    RobotCommand command;
	Lease *lease = new Lease(leaseResp.lease());
	AcquireLeaseResponse retLeaseResp = _leaseClientPtr.retainLease(lease);

	switch (mType){
		case sit:
			command.mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();
			break;
		case stand:
			command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
			break;
		case travel:
			SE2VelocityCommand_Request se2VelocityCommand_Request;
			se2VelocityCommand_Request.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + clockSkew) + time*1000000000)));
			se2VelocityCommand_Request.set_se2_frame_name(BODY_FRAME_NAME);
			se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_x(x);
			se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_y(y);
			se2VelocityCommand_Request.mutable_velocity()->set_angular(rot);
/*
			SE2Velocity velocity;
			velocity.mutable_linear()->set_x(x);
			velocity.mutable_linear()->set_y(y);
			velocity.set_angular(rot);
*/
//			se2VelocityCommand_Request->mutable_velocity()->copyFrom(velocity);

			RobotCommand command2;
			command2.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);
			break;
	}

	RobotCommandResponse robCommResp = robotCommandClient.robotCommand(leaseResp.lease(), command, timeSyncClockId);
    return (robCommResp.Status == 1);
}