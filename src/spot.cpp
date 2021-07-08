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

uint32_t Spot::sit(){
	return _spotcontrol->sit();
}

void Spot::sitBlocking(){
	uint32_t movementCommandID = sit();
	RobotCommandFeedbackResponse mcfr = getRobotCommandClient()->robotCommandFeedback(movementCommandID);

	while(mcfr.feedback().synchronized_feedback().mobility_command_feedback().sit_feedback().status() != 1){
		mcfr = getRobotCommandClient()->robotCommandFeedback(movementCommandID);
		sleep(0.2);
	}
}

uint32_t Spot::stand(){
	return _spotcontrol->stand();
}

void Spot::standBlocking(){
	uint32_t movementCommandID = stand();
	RobotCommandFeedbackResponse mcfr = getRobotCommandClient()->robotCommandFeedback(movementCommandID);
	while(mcfr.feedback().synchronized_feedback().mobility_command_feedback().stand_feedback().status() != 2){
		mcfr = getRobotCommandClient()->robotCommandFeedback(movementCommandID);
		sleep(0.2);
	}
}

uint32_t Spot::velocityMove(double x, double y, double angular, int64_t time, gravAlignedFrame frame){
	return _spotcontrol->velocityMove(x, y, angular, time, frame);
}

// velocityMoveBlocking is done differently in that it blocks by time instead of from feedback as there is no feedback provided
// by the command itself
void Spot::velocityMoveBlocking(double x, double y, double angular, int64_t time, gravAlignedFrame frame) {
	velocityMove(x, y, angular, time, frame);
	int64_t startTime = TimeUtil::TimestampToSeconds(TimeUtil::GetCurrentTime());
	while(TimeUtil::TimestampToSeconds(TimeUtil::GetCurrentTime()) - startTime < time){
		sleep(0.2);
	}
}

uint32_t Spot::trajectoryMove(Trajectory2D trajectory, int64_t time, gravAlignedFrame frame){
	return _spotcontrol->trajectoryMove(trajectory, frame, time);
}

void Spot::trajectoryMoveBlocking(Trajectory2D trajectory, int64_t time, gravAlignedFrame frame){
	uint32_t movementCommandID = trajectoryMove(trajectory, time, frame);
	RobotCommandFeedbackResponse mcfr = getRobotCommandClient()->robotCommandFeedback(movementCommandID);
	while(mcfr.feedback().synchronized_feedback().mobility_command_feedback().se2_trajectory_feedback().status() != 1 &&
			mcfr.feedback().synchronized_feedback().mobility_command_feedback().se2_trajectory_feedback().body_movement_status() != 2){
		mcfr = getRobotCommandClient()->robotCommandFeedback(movementCommandID);
		sleep(0.2);
	}
}

void Spot::setMobilityParams(MobilityParams mParams){
	return _spotcontrol->setMobilityParams(mParams);
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
