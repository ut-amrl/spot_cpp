#include <spot/robot.h>

Robot::Robot(const std::string &name) : 
        _name(name),
        _authClientPtr(std::shared_ptr<AuthClient>(new AuthClient)),
        _robotIdClientPtr(std::shared_ptr<RobotIdClient>(new RobotIdClient)) {
}

void Robot::setup() {
    _directoryClientPtr = std::shared_ptr<DirectoryClient>(new DirectoryClient(_token));
    _estopClientPtr = std::shared_ptr<EstopClient>(new EstopClient(_directoryClientPtr->getEntry(ESTOP_CLIENT_NAME).service_entry().authority(), _token));
    _imageClientPtr =std::shared_ptr<ImageClient>(new ImageClient(_directoryClientPtr->getEntry(IMAGE_CLIENT_NAME).service_entry().authority(), _token));
    _leaseClientPtr = std::shared_ptr<LeaseClient>(new LeaseClient(_directoryClientPtr->getEntry(LEASE_CLIENT_NAME).service_entry().authority(), _token));
    _powerClientPtr = std::shared_ptr<PowerClient>(new PowerClient(_directoryClientPtr->getEntry(POWER_CLIENT_NAME).service_entry().authority(), _token));
    _robotCommandClientPtr = std::shared_ptr<RobotCommandClient>(new RobotCommandClient(_directoryClientPtr->getEntry(ROBOT_COMMAND_CLIENT_NAME).service_entry().authority(), _token));
    _spotCheckClientPtr = std::shared_ptr<SpotCheckClient>(new SpotCheckClient(_directoryClientPtr->getEntry(SPOT_CHECK_CLIENT_NAME).service_entry().authority(), _token));
    _timeSyncClientPtr = std::shared_ptr<TimeSyncClient>(new TimeSyncClient(_directoryClientPtr->getEntry(TIMESYNC_CLIENT_NAME).service_entry().authority(), _token));
    _worldObjectsClientPtr = std::shared_ptr<WorldObjectsClient>(new WorldObjectsClient(_directoryClientPtr->getEntry(WORLD_OBJECTS_CLIENT_NAME).service_entry().authority(), _token));
}

void Robot::authenticate(const std::string &username, const std::string &password) {
    GetAuthTokenResponse reply;
    try {
        reply = _authClientPtr->auth(username, password);
    } catch (Error &error) {
        std::cout << error.what() << std::endl;
    }
    // check if token is empty
    if (reply.token().empty()) {
        throw InvalidCredentialsError(reply, "Username and/or password incorrect.");
    } else {
        _token = reply.token();
    }
}

std::string Robot::getId() {
    // check if robotid client already cached
    RobotIdResponse reply;
    try {
        reply = _robotIdClientPtr->getId();
    } catch (Error &error) {
        std::cout << error.what() << std::endl;
    }

    RobotId id = reply.robot_id();
    // populate ret string (disgusting concat)
    std::string ret = "[SERIAL NUMBER]: " + id.serial_number();
    ret += "\n[SPECIES]: " + id.species();
    ret += "\n[VERSION]: " + id.version();
    ret += "\n[NAME]: " + id.software_release().name();
    ret += "\n[NICKNAME]: " + id.nickname() + "\n";
    return ret;
}

// todo : error handling
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

// todo: error handling
void Robot::initBasicLease() {
    // acquire lease
    AcquireLeaseResponse leaseReply = _leaseClientPtr->acquire("body");
    std::shared_ptr<Lease> lease(new Lease(leaseReply.lease()));
    _leasePtr = lease;

    // create thread
    _leaseThread = std::shared_ptr<LeaseKeepAlive>(new LeaseKeepAlive(_leaseClientPtr, *lease, 0));
}

// todo: error handling
void Robot::initBasicTimesync() {
    TimeSyncUpdateResponse timeSyncResp = _timeSyncClientPtr->getTimeSyncUpdate();
    _timeSyncClockId = timeSyncResp.clock_identifier();
    while(timeSyncResp.state().status() != 1){
		TimeSyncRoundTrip prevRoundTrip;
		prevRoundTrip.mutable_client_rx()->CopyFrom(TimeUtil::GetCurrentTime());
		prevRoundTrip.mutable_client_tx()->CopyFrom(timeSyncResp.header().request_header().request_timestamp());
		prevRoundTrip.mutable_server_tx()->CopyFrom(timeSyncResp.header().response_timestamp());
		prevRoundTrip.mutable_server_rx()->CopyFrom(timeSyncResp.header().request_received_timestamp());
		timeSyncResp = _timeSyncClientPtr->getTimeSyncUpdate(prevRoundTrip, _timeSyncClockId);
        _clockSkew = TimeUtil::DurationToNanoseconds(timeSyncResp.state().best_estimate().clock_skew());
	}
}

// todo: error handling
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

// todo: error handling
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

// todo: error handling
bool Robot::move(movementType mType){
	if (_robotCommandClientPtr == NULL){
	        std::cout << "Need to setup" << std::endl; 
	        throw 1;
	 } // TODO: change later 
	
	RobotCommand command;

	if (mType == sit) {
        command.mutable_synchronized_command()->mutable_mobility_command()->mutable_sit_request();
    } else if (mType == stand) {
        command.mutable_synchronized_command()->mutable_mobility_command()->mutable_stand_request();
    } else {
        std::cout << "Use other move() command to issue velocity requests to robot." << std::endl;
        return false;
    }

    try {
	    RobotCommandResponse robCommResp = _robotCommandClientPtr->robotCommand(*_leasePtr, command, _timeSyncClockId);
        return (robCommResp.status() == 1);
    } catch (Error &error) {
        std::cout << error.what() << std::endl;
        return false;
    }
}

// move method for travelling
bool Robot::move(movementType mType, double x, double y, double rot, double time){
	if (_robotCommandClientPtr == NULL){
	        std::cout << "Robot class needs call to setup method to setup clients." << std::endl; 
	        return false;
	 }
	
	RobotCommand command;

    if (mType == travel) {
        SE2VelocityCommand_Request se2VelocityCommand_Request;
        se2VelocityCommand_Request.mutable_end_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew) + time*1000000000)));
        se2VelocityCommand_Request.set_se2_frame_name(BODY_FRAME_NAME);
        se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_x(x);
        se2VelocityCommand_Request.mutable_velocity()->mutable_linear()->set_y(y);
        se2VelocityCommand_Request.mutable_velocity()->set_angular(rot);
        command.mutable_synchronized_command()->mutable_mobility_command()->mutable_se2_velocity_request()->CopyFrom(se2VelocityCommand_Request);
    } else {
        std::cout << "Use other move command to sit and stand." << std::endl;
        return false;
    }

    try {
        RobotCommandResponse robCommResp = _robotCommandClientPtr->robotCommand(*_leasePtr, command, _timeSyncClockId);
        return (robCommResp.status() == 1);
    } catch (Error &error) {
        std::cout << error.what() << std::endl;
        return false;
    }
}

bool Robot::getImages(){
    if (_imageClientPtr == NULL){
	        std::cout << "Need to setup" << std::endl; 
	        throw 1;
	} // TODO: change later 

    ImageRequest request;
    ListImageSourcesResponse sources = _imageClientPtr->listImageSources();

    // // list out image sources 
    // for (ImageSource source : sources.image_sources()){
    //     std::cout << source.name() << std::endl;
    // }

    request.set_image_source_name("frontright_fisheye_image");
    request.set_quality_percent(50.0);
    request.set_image_format(bosdyn::api::Image_Format_FORMAT_JPEG);
	std::vector<bosdyn::api::ImageRequest> vector;
    vector.push_back(request);

	GetImageResponse imageResp = _imageClientPtr->getImage(vector);

    // std::cout << "status: " << imageResp.image_responses(0).status() << std::endl;
    // std::cout << "pixel format: " << imageResp.image_responses(0).shot().image().pixel_format() << std::endl;

    std::ofstream myfile ("data.jpg");
    myfile << imageResp.image_responses(0).shot().image().data();
    // GBytes *g_bytes_new (*imageResp.image_responses(0).shot().image().data(), );
    // GBytes temp = imageResp.image_responses(0).shot().image().data();
    myfile.close();

    cv::Mat img = cv::imread("data.jpg", cv::IMREAD_ANYCOLOR);
    cv::imwrite("data.png", img);

    return true;
}

void Robot::getWorldObject(){
    if (_worldObjectsClientPtr == NULL){
        std::cout << "Need to setup" << std::endl;
        throw 1;
    } // TODO: change later
    
    MutateWorldObjectRequest_Mutation mutation;
    
    WorldObject worldObject;
    worldObject.set_name("bottle");
    worldObject.mutable_acquisition_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew))));
    
    mutation.set_action(MutateWorldObjectRequest_Action_ACTION_ADD);
    mutation.mutable_object()->CopyFrom(worldObject);
    std::cout << "time world object creation: " << TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew))) << std::endl;
        
    Frame frame;
    mutation.mutable_object()->mutable_transforms_snapshot()->CopyFrom(frame.getFrameTreeSnapshot(1,0,0,0,0,0)); // FrameTreeSnapshot
    mutation.mutable_object()->mutable_image_properties()->set_camera_source("frontright");
    bosdyn::api::Polygon coordinates;
    int l_x = 128;
    int r_x = 252;
    int t_y = 244;
    int b_y = 303;

    Vec2 tl; //169 204
    tl.set_x(l_x);
    tl.set_y(t_y);
    // tl.set_x(77);
    // tl.set_y(136);
    Vec2 tr;
    tr.set_x(r_x);
    tr.set_y(t_y);
    Vec2 br;
    br.set_x(r_x);
    br.set_y(b_y);
    Vec2 bl;
    bl.set_x(l_x);
    bl.set_y(b_y);
    coordinates.add_vertexes()->CopyFrom(tl);
    coordinates.add_vertexes()->CopyFrom(tr);
    coordinates.add_vertexes()->CopyFrom(br);
    coordinates.add_vertexes()->CopyFrom(bl);
    mutation.mutable_object()->mutable_image_properties()->mutable_coordinates()->CopyFrom(coordinates);
    mutation.mutable_object()->mutable_image_properties()->set_frame_name_image_coordinates("clockwise");
    
    MutateWorldObjectResponse response = _worldObjectsClientPtr->mutateWorldObjects(mutation);
    
    sleep (10);
    
    std::cout << "status: " << response.status() << std::endl;
    std::cout << "int id of new object:  " << response.mutated_object_id() << std::endl;
    
    // ListWorldObjectResponse objects = _worldObjectsClientPtr->listWorldObjects
    ListWorldObjectResponse objects = _worldObjectsClientPtr->listWorldObjects(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew) - 100000000000))); //_clockSkew
    // std::cout << "time world object read: " << TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew) - 15 * 1000000000)) << std::endl;
    
    std::cout << "number of world objects: " << objects.world_objects().size() << std::endl;
    
    std::cout << "\ndelete object" << std:: endl;
    
    // MutateWorldObjectRequest_Mutation mutation2;
    // mutation2.set_action(MutateWorldObjectRequest_Action_ACTION_DELETE);
    // mutation2.mutable_object()->set_id(169);
    // mutation2.mutable_object()->mutable_acquisition_time()->CopyFrom(TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew))));
    // std::cout << "time world object creation: " << TimeUtil::NanosecondsToTimestamp(((TimeUtil::TimestampToNanoseconds(TimeUtil::GetCurrentTime()) + _clockSkew))) << std::endl;
    // // request.mutable_mutation()->mutable_object()->mutable_transforms_snapshot(); // FrameTreeSnapshot
    // mutation2.mutable_object()->mutable_image_properties()->set_camera_source("frontright");
    // bosdyn::api::Polygon coordinates;
    // Vec2 tl; //169 204
    // tl.set_x(169);
    // tl.set_y(204);
    // // tl.set_x(77);
    // // tl.set_y(136);
    // Vec2 tr;
    // tr.set_x(267);
    // tr.set_y(136);
    // Vec2 br;
    // br.set_x(267);
    // br.set_y(264);
    // Vec2 bl;
    // bl.set_x(77);
    // bl.set_y(264);
    // coordinates.add_vertexes()->CopyFrom(tl);
    // // coordinates.add_vertexes()->CopyFrom(tr);
    // // coordinates.add_vertexes()->CopyFrom(br);
    // // coordinates.add_vertexes()->CopyFrom(bl);
    // mutation2.mutable_object()->mutable_image_properties()->mutable_coordinates()->CopyFrom(coordinates);
    // mutation2.mutable_object()->mutable_image_properties()->set_frame_name_image_coordinates("clockwise");
    
    // MutateWorldObjectResponse response2 = _worldObjectsClientPtr->mutateWorldObjects(mutation2);
    
    // std::cout << "status: " << response2.status() << std::endl;
    
    // list world objects
    // for (WorldObject object: objects.world_objects()){
    //     std::cout << "in world object 2" << std::endl;
    //     std::cout << object.name() << std::endl;
    // }
}