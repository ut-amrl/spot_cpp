#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "bosdyn/api/spot/robot_command.grpc.pb.h"
#include <google/protobuf/util/time_util.h>
#include <Eigen/Geometry>

using bosdyn::api::SE2Trajectory;
using bosdyn::api::SE2TrajectoryPoint;	
using bosdyn::api::SE3Trajectory;
using bosdyn::api::SE3TrajectoryPoint;	
using google::protobuf::util::TimeUtil;



class Trajectory2D{
public:
    Trajectory2D();

    void addPoint(double x, double y, double rot, double time);
    void setInterp(bool cubic);
    SE2Trajectory getTrajectory();

private:
    SE2Trajectory _trajectory;
};



class Trajectory3D{
public:
    Trajectory3D();

    void addPointRPY(double x, double y, double z, double roll, double pitch, double yaw, double time);
	void addPointRPYVel(double x, double y, double z, double roll, double pitch, double yaw, double time,
		    double velX, double velY, double velZ, double angVelX, double angVelY, double angVelZ);
	void addPointQuat(double x, double y, double z, double qx, double qy, double qz, double qw, double time);
	void addPointQuatVel(double x, double y, double z, double qx, double qy, double qz, double qw, double time,
		    double velX, double velY, double velZ, double angVelX, double angVelY, double angVelZ);
    void setPosInterp(bool cubic);
	void setAngInterp(bool cubic);

    SE3Trajectory getTrajectory();

private:
    SE3Trajectory _trajectory;
};

#endif