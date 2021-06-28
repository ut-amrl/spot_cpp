/*
   methods and constants for use in relation to robot positioning
*/
 
#include <spot/frame.h>
 
// SE3Pose getSE3Pose(double x, double y, double z, double pitch, double yaw, double roll){
//  SE3Pose pose;
 
//  Vec3 position;
//  position.set_x(x);
//  position.set_y(y);
//  position.set_z(z);
 
//     Eigen::AngleAxisd rotX(roll, Eigen::Vector3d::UnitX());
//     Eigen::AngleAxisd rotY(pitch, Eigen::Vector3d::UnitY());
//     Eigen::AngleAxisd rotZ(yaw, Eigen::Vector3d::UnitZ());
//     Eigen::Quaternion<double> q = rotX * rotZ * rotY;
 
//  trajectoryPoint.mutable_pose()->mutable_rotation()->set_x(q.x()); // quaternion
//     trajectoryPoint.mutable_pose()->mutable_rotation()->set_y(q.y());
//     trajectoryPoint.mutable_pose()->mutable_rotation()->set_z(q.z());
//     trajectoryPoint.mutable_pose()->mutable_rotation()->set_w(q.w());
// }
 
FrameTreeSnapshot Frame::getFrameTreeSnapshot(double xPos, double yPos, double zPos, double pitch, double yaw, double roll){
   FrameTreeSnapshot ftSnapshot;
   typedef pair<std::string, ParentEdge> childParentEdges;
 
   FrameTreeSnapshot_ParentEdge parent;
   parent.set_parent_frame_name("vision");
   SE3Pose transform;
   parent.mutable_parent_tform_child()->CopyFrom(transform);
 
   childParentEdges.insert(std::make_pair("object", parent));
 
   ftSnapshot.mutable_child_to_parent_edge_map->CopyFrom(childParentEdges);
}