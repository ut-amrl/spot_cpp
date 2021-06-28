/*
    math.h: basic math and geometry functions and values 
*/

#ifndef MATH_H
#define MATH_H

namespace Math {
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
    class Quaternion {
    public:
        Quaternion();
    private:
    };

    /*
        class SE3Pose: class representing an SE3Pose with position and rotation (quaternion)
    */
    class SE3Pose {
    public:
        SE3Pose();
    private:
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