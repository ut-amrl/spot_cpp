#include <spot/math.h>

class SE2Pose(){
    double x;
    double y;
    double angular;

    void SE2Pose(double xPos, double yPos, double angularPos){
        x = xPos;
        y = yPos;
        angular = angularPos;
    } // constructor 

    double getX(){
        return x;
    } // get x

    double getY(){
        return y;
    } // get y

    double getAngular(){
        return angular;
    } // get angular 
} // SE2 Pose 