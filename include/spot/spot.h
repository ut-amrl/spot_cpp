/*
    spot.h: header file for main
*/
#ifndef SPOT_H
#define SPOT_H

#include <spot/robot.h>

class Spot {
public:
    static void displayThread();
    static void mainThread(int argc, char *argv[]);
private:
};

#endif
