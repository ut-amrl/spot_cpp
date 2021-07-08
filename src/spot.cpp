#include <spot/spot.h>

Spot::Spot() :
		_spotbase(new CoreLayer::SpotBase) 
		/*_spotpayloads(new CoreLayer::SpotPayloads), */
		//_spotcontrol(new RobotLayer::SpotControl(_spotbase)),
		//_spotstate(new RobotLayer::SpotState(_spotbase)) //,
		/* _spotdata(new RobotLayer::SpotData) */ {}

void Spot::initClients(){
	_spotcontrol = std::shared_ptr<RobotLayer::SpotControl>(new RobotLayer::SpotControl(_spotbase));
	_spotstate = std::shared_ptr<RobotLayer::SpotState>(new RobotLayer::SpotState(_spotbase));
}

void Spot::basicInit(const std::string &username, const std::string &password){
	authenticate(username, password);
	initClients();
	initBasicEstop();
	initBasicLease();
	initBasicTimeSync();
	powerOnBlocking();
}

void Spot::authenticate(const std::string &username, const std::string &password){
	_spotbase->authenticate(username, password);
}

void Spot::authenticateWithToken(const std::string &token){
	// TODO:: This method is incomplete
}

void Spot::initBasicEstop(){
	std::shared_ptr<ClientLayer::EstopEndpoint> endpoint = std::shared_ptr<ClientLayer::EstopEndpoint>(new ClientLayer::EstopEndpoint(_spotcontrol->getEstopClient(), "pdb_root", "PDB_rooted", "", "", 30, 3));
	std::set<std::shared_ptr<ClientLayer::EstopEndpoint>> endpoints;
	endpoints.insert(endpoint);
	_spotcontrol->setEstopConfiguration(endpoints);
	std::string unique_id = _spotcontrol->registerEstopEndpoint("pdb_root", "PDB_rooted", _spotcontrol->getEstopConfigId(), 30, 3);
	_spotcontrol->beginEstopping(unique_id);
}

void Spot::initBasicLease(){
	_spotcontrol->acquireLease("body");
	_spotcontrol->beginLeasing();
}

void Spot::initBasicTimeSync(){
	_spotbase->beginTimesync();
}

uint32_t Spot::powerOn(){
	return _spotcontrol->powerOnMotors();
}

void Spot::powerOnBlocking(){
	blockUntilPowerComplete(powerOn());
}

uint32_t Spot::powerOff(){
	return _spotcontrol->powerOffMotors();
}

void Spot::powerOffBlocking(){
	blockUntilPowerComplete(powerOff());
}

void Spot::blockUntilPowerComplete(uint32_t powerCommandID){
	PowerCommandFeedbackResponse pcfr = getPowerClient()->PowerCommandFeedback(powerCommandID);
    while(pcfr.status() != 2){
        pcfr = getPowerClient()->PowerCommandFeedback(powerCommandID);
        sleep(1);
    }
}

Spot::RobotCmdRespData Spot::sit(){
	RobotCommandResponse resp = _spotcontrol->sit();
	Spot::RobotCmdRespData respData(resp.robot_command_id(), 
		enumConvertRobotCommandStatus(resp.status()), 
		Spot::RobotCmdRespData::CommandType::SIT);
	return respData;
}

Spot::RobotCmdRespData Spot::stand(){
	RobotCommandResponse resp = _spotcontrol->stand();
	Spot::RobotCmdRespData respData(resp.robot_command_id(), 
		enumConvertRobotCommandStatus(resp.status()), 
		Spot::RobotCmdRespData::CommandType::STAND);
	return respData;
}

Spot::RobotCmdRespData Spot::velocityMove(double x, double y, double angular, int64_t time, gravAlignedFrame frame){
	RobotCommandResponse resp = _spotcontrol->velocityMove(x, y, angular, time, frame);
	Spot::RobotCmdRespData respData(resp.robot_command_id(),
		enumConvertRobotCommandStatus(resp.status()), 
		Spot::RobotCmdRespData::CommandType::VELOCITY_MOVE);
	return respData;
}

Spot::RobotCmdRespData Spot::trajectoryMove(Trajectory2D trajectory, int64_t time, gravAlignedFrame frame){
	RobotCommandResponse resp = _spotcontrol->trajectoryMove(trajectory, frame, time);
	Spot::RobotCmdRespData respData(resp.robot_command_id(), 
		enumConvertRobotCommandStatus(resp.status()), 
		Spot::RobotCmdRespData::CommandType::TRAJECTORY_MOVE);
	return respData;
}

bool Spot::checkCommandComplete(RobotCmdRespData respData){
	RobotCommandFeedbackResponse rcfr = getRobotCommandClient()->robotCommandFeedback(respData.id());
	switch(respData.type()){
		case Spot::RobotCmdRespData::CommandType::SIT:
			return rcfr.feedback().synchronized_feedback().mobility_command_feedback().sit_feedback().status() == bosdyn::api::SitCommand_Feedback_Status_STATUS_IS_SITTING;
		case Spot::RobotCmdRespData::CommandType::STAND:
			return rcfr.feedback().synchronized_feedback().mobility_command_feedback().stand_feedback().status() == bosdyn::api::StandCommand_Feedback_Status_STATUS_IS_STANDING;
		case Spot::RobotCmdRespData::CommandType::VELOCITY_MOVE:
			std::cout << "Velocity Commands are time based and cannot have their completion status checked" << std::endl;
			return false;
		case Spot::RobotCmdRespData::CommandType::TRAJECTORY_MOVE:
			return rcfr.feedback().synchronized_feedback().mobility_command_feedback().se2_trajectory_feedback().status() == bosdyn::api::SE2TrajectoryCommand_Feedback_Status_STATUS_AT_GOAL;
		default:
			std::cout << "Provided command not supported for completion status checking" << std::endl;
			return false;
	}
}

void Spot::setMobilityParams(MobilityParams mParams){
	_spotcontrol->setMobilityParams(mParams);
}

void Spot::setBodyPose(Trajectory3D trajectory, bool gravityAlign){
	BodyControlParams bodyParams;

    if(gravityAlign)
        bodyParams.set_rotation_setting(bosdyn::api::spot::BodyControlParams_RotationSetting_ROTATION_SETTING_ABSOLUTE);
    else
        bodyParams.set_rotation_setting(bosdyn::api::spot::BodyControlParams_RotationSetting_ROTATION_SETTING_OFFSET);
    
    bodyParams.mutable_base_offset_rt_footprint()->CopyFrom(trajectory.getTrajectory());

	MobilityParams mParams = _spotcontrol->getMobilityParams();
    mParams.mutable_body_control()->CopyFrom(bodyParams);
	_spotcontrol->setMobilityParams(mParams);
}

void Spot::resetBodyPose(double time){
	Trajectory3D traj;
    traj.addPointRPY(0,0,0,0,0,0, time);
    setBodyPose(traj, false);
}

Spot::RobotCmdRespData::CommandStatus Spot::enumConvertRobotCommandStatus(bosdyn::api::RobotCommandResponse_Status status){
	switch(status){
		case bosdyn::api::RobotCommandResponse_Status_STATUS_UNKNOWN:
			return RobotCmdRespData::CommandStatus::STATUS_UNKNOWN;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_OK:
			return RobotCmdRespData::CommandStatus::STATUS_OK;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_INVALID_REQUEST:
			return RobotCmdRespData::CommandStatus::STATUS_INVALID_REQUEST;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_UNSUPPORTED:
			return RobotCmdRespData::CommandStatus::STATUS_UNSUPPORTED;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_NO_TIMESYNC:
			return RobotCmdRespData::CommandStatus::STATUS_NO_TIMESYNC;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_EXPIRED:
			return RobotCmdRespData::CommandStatus::STATUS_EXPIRED;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_TOO_DISTANT:
			return RobotCmdRespData::CommandStatus::STATUS_TOO_DISTANT;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_NOT_POWERED_ON:
			return RobotCmdRespData::CommandStatus::STATUS_NOT_POWERED_ON;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_BEHAVIOR_FAULT:
			return RobotCmdRespData::CommandStatus::STATUS_BEHAVIOR_FAULT;
		case bosdyn::api::RobotCommandResponse_Status_STATUS_UNKNOWN_FRAME:
			return RobotCmdRespData::CommandStatus::STATUS_UNKNOWN_FRAME;
	}
}
