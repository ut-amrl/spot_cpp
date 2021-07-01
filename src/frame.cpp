/*
    methods and constants for use in relation to robot positioning 
*/

#include <spot/frame.h>

FrameTree::FrameTree(bosdyn::api::FrameTreeSnapshot snapshot){
	google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge> frameMap = snapshot.child_to_parent_edge_map();

	for (google::protobuf::Map<std::string, bosdyn::api::FrameTreeSnapshot::ParentEdge>::const_iterator it=frameMap.begin(); it!=frameMap.end(); ++it){
		Parent parent(Math::SE3Pose::from_proto((it->second.parent_tform_child())), it->second.parent_frame_name());
		_childToParentEdges.insert(std::pair<std::string, Parent>(it->first, parent));
	}
}

std::map<std::string, FrameTree::Parent> FrameTree::childToParentEdges(){
	return _childToParentEdges;
}

void FrameTree::addEdge(Math::SE3Pose parent_tf_child, std::string parentFrame, std::string childFrame){
	// Ensure child not already in frame tree
	if(_childToParentEdges.find(childFrame) != _childToParentEdges.end()){
		std::cout << "Error: Frame already present in frame tree" << std::endl;
	}

	Parent parent(parent_tf_child, parentFrame);
	_childToParentEdges.insert(std::pair<std::string, Parent>(childFrame, parent));
}

Math::SE3Pose FrameTree::a_tf_b(std::string frameA, std::string frameB){
	if(_childToParentEdges.find(frameB) == _childToParentEdges.end()){
		std::cout << "Error: Frame A not in frame tree" << std::endl;
		return Math::SE3Pose(0,0,0, Math::Quaternion(0,0,0,0));
	}
	if(_childToParentEdges.find(frameB) == _childToParentEdges.end()){
		std::cout << "Error: Frame B not in frame tree" << std::endl;
		return Math::SE3Pose(0,0,0, Math::Quaternion(0,0,0,0));
	}

	std::vector<FrameTree::Parent> inverseEdges = listParentEdges(frameA);
	std::vector<FrameTree::Parent> forwardEdges = listParentEdges(frameB);

	Math::SE3Pose frameA_tf_root = accumulateTransforms(inverseEdges).inverse();
	Math::SE3Pose root_tf_frameB = accumulateTransforms(forwardEdges);
	return frameA_tf_root * root_tf_frameB;
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

Math::SE3Pose FrameTree::accumulateTransforms(std::vector<Parent> parents){
	Math::SE3Pose pose(0,0,0, Math::Quaternion(0,0,0,1));
	for(Parent parent : parents){
		pose = parent.parent_tf_child() * pose;
	}
	return pose;
}


FrameTree::Parent::Parent(Math::SE3Pose parent_tf_child, std::string parentFrameName) :
	_parent_tf_child(parent_tf_child), _parentFrameName(parentFrameName){
}

Math::SE3Pose FrameTree::Parent::parent_tf_child(){
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