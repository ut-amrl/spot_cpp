#include <spot/math.h>

namespace Math {
    Vector3::Vector3(double x, double y, double z) :
            _x(x),
            _y(y),
            _z(z) {}

    Vector3::Vector3(bosdyn::api::Vec3 vec) {

    }

    Quaternion::Quaternion(double x, double y, double z, double w) :
        _x(x),
        _y(y),
        _z(z),
        _w(w) {}

    Quaternion Quaternion::inverse() {
        return Quaternion(-_x, -_y, -_z, _w);
    }

    Quaternion Quaternion::operator*(const Quaternion &otherQuat) {
        return Quaternion( 
            _w * otherQuat._x + _x * otherQuat._w + _y * otherQuat._z - _z * otherQuat._y,
            _w * otherQuat._y - _x * otherQuat._z + _y * otherQuat._w + _z * otherQuat._x, 
            _w * otherQuat._z + _x * otherQuat._y - _y * otherQuat._x + _z * otherQuat._w,
            _w * otherQuat._w - _x * otherQuat._x - _y * otherQuat._y - _z * otherQuat._z);
    }

    Vector3 Quaternion::transformPoint(double x, double y, double z) {
        Quaternion inv = this->inverse();
        Quaternion q = Quaternion(x, y, z, 0);
        q = q.mult(inv);
        q = this->mult(q);
        return Vector3(q._x, q._y, q._z);
    }

    SE3Pose::SE3Pose(double x, double y, double z, Quaternion quat) :
            _x(x),
            _y(y),
            _z(z),
            _q(quat) {}

    SE3Pose SE3Pose::operator*(const SE3Pose &other) {
        Vector3 vec = _q.transformPoint(other._x, other._y, other._z);
        return SE3Pose(_x + vec.x, _y + vec.y, _z + vec.z, _q * other._q);
    }

    SE3Pose SE3Pose::inverse() {
        Quaternion invQuat = _q.inverse();
	    Vector3 vec = invQuat.transformPoint(_x, _y, _z);
	    return SE3Pose(-vec.x, -vec.y, -vec.z, invQuat);
    }

    SE3Pose SE3Pose::from_identity() {
        return SE3Pose(0, 0, 0, Quaternion(0, 0, 0, 1));
    }

    SE3Pose SE3Pose::from_proto(bosdyn::api::SE3Pose pose) {
        return SE3Pose(
            pose.position().x(),
            pose.position().y(),
            pose.position().z(),
            Quaternion(pose.rotation().x(), pose.rotation().y(), pose.rotation().z(), pose.rotation().w())
        );
    }
};