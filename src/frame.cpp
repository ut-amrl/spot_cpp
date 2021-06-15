/*
    methods and constants for use in relation to robot positioning 
*/

#include <spot/frame.h>

const VISION_FRAME_NAME = "vision";
const BODY_FRAME_NAME = "body";
const GRAV_ALIGNED_BODY_FRAME_NAME = "gpe";
const ODOM_FRAME_NAME = "odom";
const GROUND_PLANE_FRAME_NAME = "gpe";
const HAND_FRAME_NAME = "hand";
const UNKNOWN_FRAME_NAME = "unknown";
const RAYCAST_FRAME_NAME = "walkto_raycast_intersection";

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