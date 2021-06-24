/*
    methods and constants for use in relation to robot positioning 
*/

#include <spot/frame.h>

Vector3::Vector3(double x, double y, double z) :
	x(x), y(y), z(z) {
}

Quaternion::Quaternion(double x, double y, double z, double w) :
	_x(x), _y(y), _z(z), _w(w) {
}

Quaternion Quaternion::inverse(){
	return Quaternion(-_x, -_y, -_z, _w);
}

Quaternion Quaternion::mult(Quaternion otherQuat){
	return Quaternion( 
		_w * otherQuat._x + _x * otherQuat._w + _y * otherQuat._z - _z * otherQuat._y,
		_w * otherQuat._y - _x * otherQuat._z + _y * otherQuat._w + _z * otherQuat._x, 
		_w * otherQuat._z + _x * otherQuat._y - _y * otherQuat._x + _z * otherQuat._w,
		_w * otherQuat._w - _x * otherQuat._x - _y * otherQuat._y - _z * otherQuat._z);
}

Vector3 Quaternion::transformPoint(double x, double y, double z){
	Quaternion inv = this->inverse();
	Quaternion q = Quaternion(x, y, z, 0);
	q = q.mult(inv);
	q = this->mult(q);
	return Vector3(q._x, q._y, q._z);
}

double Quaternion::w(){
	return _w;
}
double Quaternion::x(){
	return _x;
}
double Quaternion::y(){
	return _y;
}
double Quaternion::z(){
	return _z;
}


Pose3::Pose3(double x, double y, double z, Quaternion quat): 
	_x(x), _y(y), _z(z), _q(quat) {
}

Pose3::Pose3(SE3Pose pose) :
	_q(pose.rotation().x(), pose.rotation().y(), pose.rotation().z(), pose.rotation().w()){
	_x = pose.position().x();
	_y = pose.position().y();
	_z = pose.position().z();
}

Pose3 Pose3::mult(Pose3 otherPose){
	Vector3 vec = _q.transformPoint(otherPose._x, otherPose._y, otherPose._z);
	return Pose3(_x + vec.x, _y + vec.y, _z + vec.z, _q.mult(otherPose._q));
}

Pose3 Pose3::inverse(){
	Quaternion invQuat = _q.inverse();
	Vector3 vec = invQuat.transformPoint(_x, _y, _z);
	return Pose3(-vec.x, -vec.y, -vec.z, invQuat);
}

double Pose3::x(){
	return _x;
}
double Pose3::y(){
	return _y;
}
double Pose3::z(){
	return _z;
}
Quaternion Pose3::quat(){
	return _q;
}

FrameTree::FrameTree(bosdyn::api::FrameTreeSnapshot snapshot){
	google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge> frameMap = snapshot.child_to_parent_edge_map();

	for (google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge>::const_iterator it=frameMap.begin(); it!=frameMap.end(); ++it){
		Parent parent(Pose3(it->second.parent_tform_child()), it->second.parent_frame_name());
		_childToParentEdges.insert(std::pair<std::string, Parent>(it->first, parent));
	}
}

std::map<std::string, FrameTree::Parent> FrameTree::childToParentEdges(){
	return _childToParentEdges;
}

void FrameTree::addEdge(Pose3 parent_tf_child, std::string parentFrame, std::string childFrame){
	// Ensure child not already in frame tree
	if(_childToParentEdges.find(childFrame) != _childToParentEdges.end()){
		std::cout << "Error: Frame already present in frame tree" << std::endl;
	}

	Parent parent(parent_tf_child, parentFrame);
	_childToParentEdges.insert(std::pair<std::string, Parent>(childFrame, parent));
}

Pose3 FrameTree::a_tf_b(std::string frameA, std::string frameB){
	if(_childToParentEdges.find(frameB) == _childToParentEdges.end()){
		std::cout << "Error: Frame A not in frame tree" << std::endl;
		return Pose3(0,0,0, Quaternion(0,0,0,0));
	}
	if(_childToParentEdges.find(frameB) == _childToParentEdges.end()){
		std::cout << "Error: Frame B not in frame tree" << std::endl;
		return Pose3(0,0,0, Quaternion(0,0,0,0));
	}

	std::vector<FrameTree::Parent> inverseEdges = listParentEdges(frameA);
	std::vector<FrameTree::Parent> forwardEdges = listParentEdges(frameB);

	Pose3 frameA_tf_root = accumulateTransforms(inverseEdges).inverse();
	Pose3 root_tf_frameB = accumulateTransforms(forwardEdges);
	return frameA_tf_root.mult(root_tf_frameB);
}

std::vector<FrameTree::Parent> FrameTree::listParentEdges(std::string leafFrame){
	std::vector<FrameTree::Parent> parentEdges;
	std::string cur = leafFrame;
	while(true){
		FrameTree::Parent parent = _childToParentEdges.at(cur);
		if(parent.parentFrameName().empty())
			break;
		parentEdges.push_back(parent);
		cur = parent.parentFrameName();
	}
	return parentEdges;
}

Pose3 FrameTree::accumulateTransforms(std::vector<Parent> parents){
	Pose3 pose(0,0,0, Quaternion(0,0,0,1));
	for(Parent parent : parents){
		pose = parent.parent_tf_child().mult(pose);
	}
	return pose;
}


FrameTree::Parent::Parent(Pose3 parent_tf_child, std::string parentFrameName) :
	_parent_tf_child(parent_tf_child), _parentFrameName(parentFrameName){
}

Pose3 FrameTree::Parent::parent_tf_child(){
	return _parent_tf_child;
}
std::string FrameTree::Parent::parentFrameName(){
	return _parentFrameName;
}


std::string frameNameGravAligned(gravAlignedFrame frame){
	switch (frame){
        case ODOM: return ODOM_FRAME_NAME;  
		case VISION: return VISION_FRAME_NAME;
		case FLAT_BODY: return FLAT_BODY_FRAME_NAME;  
    }
}

/*
map<string, Vec2> getFootPositionMap(double fl_x, double fl_y, double fr_x, double fr_y, double bl_x, double bl_y, double br_x, double br_y){
    Vec2 fl;
	fl.x(fl_x);
	fl.y(fl_y);
	Vec2 fr;
	fr.x(fr_x);
	fr.y(fr_y);
	Vec2 bl;
	bl.x(bl_x);
	bl.y(bl_y);
	Vec2 br;
	br.x(br_x);
	br.y(br_y);
	map<string, Vec2> footPositions;
	footPositions.insert(pair<string, Vec2>("fl", fl));
	footPositions.insert(pair<string, Vec2>("fr", fr));
	footPositions.insert(pair<string, Vec2>("bl", bl));
	footPositions.insert(pair<string, Vec2>("br", br));
    return footPositions;
} // create map of foot positions

Vec2 makeVec2(double x, double y){
    Vec2 ret;
    ret.x(x);
    ret.y(y);
    return Vec2;
} // create Vec2

SE2Pose makeSE2Pose(double x, double y, double angle){
    SE2Pose ret;
    ret.position(makeVec2(x,y));
    ret.angle(angle);
    return ret;
} // create SE2Pose

// x and y are linear components of velocity, angular is angular component of the velocity
SE2Velocity makeSE2Velocity(double x, double y, double angular){
    SE2Velocity ret;
    ret.linear(makeVec2(x,y));
    ret.angular(angular);
    return ret;
} // create SE2Velocity 
*/
