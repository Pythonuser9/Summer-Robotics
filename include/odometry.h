#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"

#ifndef ODOH
#define ODOH
//foward offset = distance between center encoder and center of robot
#define FOWARD_OFFSET 0
#define IMU_THRESHOLD 1
#define HEADING_CUTOFF 100

extern void Odometry();
extern void boomerang(double xTarget, double yTarget);
extern void driveToPoint(double xTarget, double yTarget, double preferredHeading);
extern void setPosition(float xcoord, float ycoord, float heading);

#endif