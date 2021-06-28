/*
   methods and constants for use in relation to robot positioning
*/
 
#include <spot/frame.h>

// Frame::Frame(){}
 
SE3Pose Frame::getSE3Pose(double x, double y, double z, double pitch, double yaw, double roll){
   SE3Pose pose;
   
   Vec3 position;
   position.set_x(x);
   position.set_y(y);
   position.set_z(z);
   
   Eigen::AngleAxisd rotX(roll, Eigen::Vector3d::UnitX());
   Eigen::AngleAxisd rotY(pitch, Eigen::Vector3d::UnitY());
   Eigen::AngleAxisd rotZ(yaw, Eigen::Vector3d::UnitZ());
   Eigen::Quaternion<double> q = rotX * rotZ * rotY;
   
   pose.mutable_rotation()->set_x(q.x()); // quaternion
   pose.mutable_rotation()->set_y(q.y());
   pose.mutable_rotation()->set_z(q.z());
   pose.mutable_rotation()->set_w(q.w());
   
   return pose; 
}
 
FrameTreeSnapshot Frame::getFrameTreeSnapshot(double xPos, double yPos, double zPos, double pitch, double yaw, double roll){
   FrameTreeSnapshot ftSnapshot;
   std::map<std::string, FrameTreeSnapshot_ParentEdge> childParentEdges;
 
   FrameTreeSnapshot_ParentEdge parent;
   parent.set_parent_frame_name("vision");
   SE3Pose transform = getSE3Pose(xPos, yPos, zPos, pitch, yaw, roll);
   parent.set_parent_tform_child(transform);
 
   childParentEdges.insert(std::make_pair("object", parent));
 
   ftSnapshot.mutable_child_to_parent_edge_map()->CopyFrom(childParentEdges); //->CopyFrom(childParentEdges);
   return ftSnapshot;
}