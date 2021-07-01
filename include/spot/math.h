/*
    math.h: basic math and geometry functions and values 
*/

#ifndef MATH_H
#define MATH_H

#include <common.h>

using bosdyn::api::Vec2;
using bosdyn::api::SE2Pose;
using bosdyn::api::Vec3;
using bosdyn::api::SE3Pose;

namespace Math {

    /*
        class Vector3: class that represents a vector with 3 dimenions 
    */
    class Vector3{
    public:
        Vector3(double x, double y, double z);
        Vector3(bosdyn::api::Vec3 vec);

        /* Accessors */
        double x() const { return _x; }
        double y() const { return _y; }
        double z() const { return _z; }

    private:
        double _x;
        double _y;
        double _z;   
    };

    /*
        class SE2Pose: represents SE2Pose with position and angle
    */
    class SE2Pose {
    public:
        SE2Pose();
    private:
    };

    /* 
        class SE2Velocity: represents SE2Velocity with linear and angular velocity
    */
    class SE2Velocity {
    public:
        SE2Velocity();
    private:
    };

    /*
        class Quaternion: represents a Quarternion, essentially a wrapper around the Eigen quaternion class
    */
    class Quaternion{
    public:
        Quaternion(double x, double y, double z, double w);

        Quaternion inverse();
        Quaternion operator*(const Quaternion &other);
        Vector3 transformPoint(double x, double y, double z);

        /* Static methods */
        static Quaternion from_proto(bosdyn::api::Quaternion);

        /* Accessors */
        double w() const { return _w; }
        double x() const { return _x; }
        double y() const { return _y; }
        double z() const { return _z; }

    private:
        double _w; 
        double _x;
        double _y;
        double _z;
};

    /*
        class SE3Pose: class representing an SE3Pose with position and rotation (quaternion)
    */
    class SE3Pose {
    public:
        SE3Pose(double x, double y, double z, Quaternion quat);

        SE3Pose operator*(const SE3Pose &other)
        SE3Pose inverse();

        /* Static methods */
        static SE3Pose from_identity();
        static SE3Pose from_proto(bosdyn::api::SE3Pose pose);

        /* Accessors */
        double x() const { return _x; }
        double y() const { return _y; }
        double z() const { return _z; }
        Quaternion quat() const { return _q; }

    private:
        double _x;
        double _y;
        double _z;
        Quaternion _q;
    };

    /*
        class SE3Velocity: class representing SE3Velocity with linear and angular velocity
    */
    class SE3Velocity {
    public:
        SE3Velocity();
    private:
    };

};

#endif