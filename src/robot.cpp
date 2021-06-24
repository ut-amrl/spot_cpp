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
    _timeSyncClientPtr = std::shared_ptr<TimeSyncClient>(new TimeSyncClient(_directoryClientPtr->getEntry(TIMESYNC_CLIENT_NAME).service_entry().authority(), _token));
    _robotStateClientPtr = std::shared_ptr<RobotStateClient>(new RobotStateClient(_directoryClientPtr->getEntry(ROBOT_STATE_CLIENT_NAME).service_entry().authority(), _token));
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
    _leasePtr = lease;

    // create thread
    _leaseThread = std::shared_ptr<LeaseKeepAlive>(new LeaseKeepAlive(_leaseClientPtr, *lease, 0));
}

Lease Robot::getLease(){
    return *_leasePtr;
}

void Robot::performTimesync() {
    TimeSyncUpdateResponse timeSyncResp;
    if(_timeSyncClockId.empty()){
        timeSyncResp = _timeSyncClientPtr->getTimeSyncUpdate();
        _timeSyncClockId = timeSyncResp.clock_identifier();
    }
    else{
        timeSyncResp = _timeSyncClientPtr->getTimeSyncUpdate(_prevRoundTripTimeSync, _timeSyncClockId);
    }
    while(timeSyncResp.state().status() != 1){
		TimeSyncRoundTrip prevRoundTrip;
		prevRoundTrip.mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
		prevRoundTrip.mutable_client_tx()->CopyFrom(timeSyncResp.header().request_header().request_timestamp());
		prevRoundTrip.mutable_server_tx()->CopyFrom(timeSyncResp.header().response_timestamp());
		prevRoundTrip.mutable_server_rx()->CopyFrom(timeSyncResp.header().request_received_timestamp());
		timeSyncResp = _timeSyncClientPtr->getTimeSyncUpdate(prevRoundTrip, _timeSyncClockId);
        _clockSkew = TimeUtil::DurationToNanoseconds(timeSyncResp.state().best_estimate().clock_skew());
        _prevRoundTripTimeSync = prevRoundTrip;
    }
    
}

int64_t Robot::getClockSkew(){
    return _clockSkew;
}

void Robot::powerOn() {
    // check that the robot is off
    if (_isOn) {
        std::cout << "robot already on" << std::endl;
        return;
    }
    
    // power
	PowerCommandRequest_Request pcr_r;
	pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_ON; // PowerCommandRequest_Request_REQUEST_OFF to turn off, change to _ON to turn on
	PowerCommandResponse powerCommResp = _powerClientPtr->PowerCommand(*_leasePtr, pcr_r); 
    uint32_t pcID = powerCommResp.power_command_id();

    PowerCommandFeedbackResponse pcfr = _powerClientPtr->PowerCommandFeedback(pcID);
	while(pcfr.status() != 2){
		pcfr = _powerClientPtr->PowerCommandFeedback(pcID);
		sleep(1);
	}
    _isOn = true;
}

void Robot::powerOff() {
    if (!_isOn) {
        std::cout << "robot already off" << std::endl;
        return;
    }

    // turn off
    PowerCommandRequest_Request pcr_r;
    pcr_r = bosdyn::api::PowerCommandRequest_Request_REQUEST_OFF;
    PowerCommandResponse powerCommResp = _powerClientPtr->PowerCommand(*_leasePtr, pcr_r);
    _isOn = false;
}

// teleop stuff 

bool Robot::sit() {
    if (_robotCommandClientPtr == NULL){
	        std::cout << "Need to setup" << std::endl; 
	        throw 1;
	} // TODO: change later 

    RobotCommand command;
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();
    Any any;
    any.PackFrom(_mobilityParams);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->CopyFrom(any);
    
    RobotCommandResponse robCommResp = _robotCommandClientPtr->robotCommand(*_leasePtr, command, _timeSyncClockId);
    return (robCommResp.status() == 1);
} // sits the robot down

bool Robot::stand(){
    if (_robotCommandClientPtr == NULL){
	        std::cout << "Need to setup" << std::endl; 
	        throw 1;
	} // TODO: change later

    RobotCommand command;
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
    Any any;
    any.PackFrom(_mobilityParams);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->CopyFrom(any);

    RobotCommandResponse robCommResp = _robotCommandClientPtr->robotCommand(*_leasePtr, command, _timeSyncClockId);
    return (robCommResp.status() == 1);
} // stands the robot up

bool Robot::velocityMove(double x, double y, double rot, double time, gravAlignedFrame frame){
    if (_robotCommandClientPtr == NULL){
        std::cout << "Need to setup" << std::endl; 
        throw 1;
	} // TODO: change later 
	
    RobotCommand command;
    SE2VelocityCommand_Request se2VelocityCommand_Request;
    se2VelocityCommand_Request.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew) + time*1000000000)));
    se2VelocityCommand_Request.set_se2_frame_name(frameNameGravAligned(frame));
    se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_x(x);
    se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_y(y);
    se2VelocityCommand_Request.mutable_velocity()->set_angular(rot);

    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);
    Any any;
    any.PackFrom(_mobilityParams);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->CopyFrom(any);
    
    RobotCommandResponse robCommResp = _robotCommandClientPtr->robotCommand(*_leasePtr, command, _timeSyncClockId);
    std::cout << "Velocity Move Status: " << robCommResp.status() << std::endl;
    return (robCommResp.status() == 1);
} // allows the robot to translate left/right and/or forward/backward and/or rotate left/right 

// move method for travelling (trajectory based)
bool Robot::trajectoryMove(Trajectory2D trajectory, gravAlignedFrame frame, double time){
	if (_robotCommandClientPtr == NULL){
	        std::cout << "Need to setup" << std::endl; 
	        throw 1;
	 } // TODO: change later 

    // TODO: Make it so that flat body works as a frame
    std::string frameName; 
    if(frame == FLAT_BODY){
        frameName = frameNameGravAligned(ODOM);
    }
    else{
        frameName = frameNameGravAligned(frame);
    }
	
    bosdyn::api::SE2TrajectoryCommand_Request trajectoryCommandReq;
    trajectoryCommandReq.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew) + (time)*1000000000)));
    trajectoryCommandReq.set_se2_frame_name(frameName);
    trajectoryCommandReq.mutable_trajectory()->CopyFrom(trajectory.getTrajectory());
    
    RobotCommand command;
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_trajectory_request()->CopyFrom(trajectoryCommandReq);

    Any any;
    any.PackFrom(_mobilityParams);
    command.mutable_synchronized_command()->mutable_mobility_command()->mutable_params()->CopyFrom(any);

	RobotCommandResponse robCommResp = _robotCommandClientPtr->robotCommand(*_leasePtr, command, _timeSyncClockId);
    std::cout << "traj move status: " << robCommResp.status() << std::endl;
    return (robCommResp.status() == 1);
}

void Robot::setMobilityParams(MobilityParams params){
    _mobilityParams = params;
}

void Robot::setBodyPose(Trajectory3D trajectory, bool gravityAlign){
    BodyControlParams bodyParams;

    if(gravityAlign)
        bodyParams.set_rotation_setting(bosdyn::api::spot::BodyControlParams_RotationSetting_ROTATION_SETTING_ABSOLUTE);
    else
        bodyParams.set_rotation_setting(bosdyn::api::spot::BodyControlParams_RotationSetting_ROTATION_SETTING_OFFSET);
    
    bodyParams.mutable_base_offset_rt_footprint()->CopyFrom(trajectory.getTrajectory());
    _mobilityParams.mutable_body_control()->CopyFrom(bodyParams);
}

void Robot::resetBodyPose(double time){
    Trajectory3D traj;
    traj.addPointRPY(0,0,0,0,0,0, time);
    setBodyPose(traj, false);
}