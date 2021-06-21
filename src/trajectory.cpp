#include "spot/trajectory.h"

Trajectory2D::Trajectory2D(){
    _trajectory.set_interpolation(bosdyn::api::POS_INTERP_LINEAR);
}

void Trajectory2D::addPoint(double x, double y, double rot, double time){
    SE2TrajectoryPoint point;
    point.mutable_pose()->mutable_position()->set_x(x);
    point.mutable_pose()->mutable_position()->set_y(y);
    point.mutable_pose()->set_angle(rot);
    point.mutable_time_since_reference()->CopyFrom(TimeUtil::SecondsToDuration(time));

    _trajectory.add_points()->CopyFrom(point);
}

SE2Trajectory Trajectory2D::getTrajectory(){
    return _trajectory;
}

void Trajectory2D::setInterp(bool cubic){
    if(cubic){
        _trajectory.set_interpolation(bosdyn::api::POS_INTERP_CUBIC);
    }
    else{
        _trajectory.set_interpolation(bosdyn::api::POS_INTERP_LINEAR);
    }
}




Trajectory3D::Trajectory3D(){
	_trajectory.set_pos_interpolation(bosdyn::api::POS_INTERP_LINEAR);
	_trajectory.set_ang_interpolation(bosdyn::api::ANG_INTERP_LINEAR);
}

void Trajectory3D::addPointRPY(double x, double y, double z, double roll, double pitch, double yaw, double time){
	Eigen::AngleAxisd rotX(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotY(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotZ(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rotX * rotZ * rotY;

	addPointQuat(x, y, z, q.x(), q.y(), q.z(), q.w(), time);
}

void Trajectory3D::addPointRPYVel(double x, double y, double z, double roll, double pitch, double yaw, double time,
		double velX, double velY, double velZ, double angVelX, double angVelY, double angVelZ){
	Eigen::AngleAxisd rotX(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotY(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rotZ(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rotX * rotZ * rotY;
	
	addPointQuatVel(x, y, z, q.x(), q.y(), q.z(), q.w(), time, velX, velY, velZ, angVelX, angVelY, angVelZ);
}

void Trajectory3D::addPointQuat(double x, double y, double z, double qx, double qy, double qz, double qw, double time){	
	SE3TrajectoryPoint point;
	point.mutable_pose()->mutable_position()->set_x(x);
    point.mutable_pose()->mutable_position()->set_y(y);
	point.mutable_pose()->mutable_position()->set_z(z);
	point.mutable_pose()->mutable_rotation()->set_x(qx);
    point.mutable_pose()->mutable_rotation()->set_y(qy);
    point.mutable_pose()->mutable_rotation()->set_z(qz);
    point.mutable_pose()->mutable_rotation()->set_w(qw);
	point.mutable_time_since_reference()->CopyFrom(TimeUtil::SecondsToDuration(time));

	_trajectory.add_points()->CopyFrom(point);
}

void Trajectory3D::addPointQuatVel(double x, double y, double z, double qx, double qy, double qz, double qw, double time,
		double velX, double velY, double velZ, double angVelX, double angVelY, double angVelZ){	
	SE3TrajectoryPoint point;
	point.mutable_pose()->mutable_position()->set_x(x);
    point.mutable_pose()->mutable_position()->set_y(y);
	point.mutable_pose()->mutable_position()->set_z(z);
	point.mutable_pose()->mutable_rotation()->set_x(qx);
    point.mutable_pose()->mutable_rotation()->set_y(qy);
    point.mutable_pose()->mutable_rotation()->set_z(qz);
    point.mutable_pose()->mutable_rotation()->set_w(qw);
	point.mutable_velocity()->mutable_linear()->set_x(velX);
	point.mutable_velocity()->mutable_linear()->set_y(velY);
	point.mutable_velocity()->mutable_linear()->set_z(velZ);
	point.mutable_velocity()->mutable_angular()->set_x(velX);
	point.mutable_velocity()->mutable_angular()->set_y(velY);
	point.mutable_velocity()->mutable_angular()->set_z(velZ);
	point.mutable_time_since_reference()->CopyFrom(TimeUtil::SecondsToDuration(time));

	_trajectory.add_points()->CopyFrom(point);
}

void Trajectory3D::setPosInterp(bool cubic){
	if(cubic)
		_trajectory.set_pos_interpolation(bosdyn::api::POS_INTERP_CUBIC);
	else
		_trajectory.set_pos_interpolation(bosdyn::api::POS_INTERP_LINEAR);
	
}

void Trajectory3D::setAngInterp(bool cubic){
	if(cubic)
		_trajectory.set_ang_interpolation(bosdyn::api::ANG_INTERP_CUBIC_EULER);
	else
		_trajectory.set_ang_interpolation(bosdyn::api::ANG_INTERP_LINEAR);
}

SE3Trajectory Trajectory3D::getTrajectory(){
	return _trajectory;
}