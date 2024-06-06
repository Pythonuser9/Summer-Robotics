#include "api.h"
#include "main.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"

#ifndef PIDH
#define PIDH
#define STRAIGHT_KP 1.05
#define STRAIGHT_KI 0.002
#define STRAIGHT_KD 6.75
#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5
#define ARC_KP 1.05
#define ARC_KI 0.002
#define ARC_KD 6.75

#define TURN_KP 4.5
#define TURN_KI 0.01
#define TURN_KD 30

#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define FOLLOW_INTEGRAL_KI 4.0
#define FOLLOW_MAX_INTEGRAL 1.4

#define FOLLOW_KP 0.4
#define FOLLOW_KI 0
#define FOLLOW_KD 0

extern void resetEncoders();
extern void driveStraight(int target);
extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveStraight2(int target);
extern int time2;
extern int time22;
extern float error;
extern void driveArcL(double theta, double radius, int timeout);
extern void driveArcLF(double theta, double radius, int timeout);
extern void driveArcR(double theta, double radius, int timeout);
extern void driveArcRF(double theta, double radius, int timeout);
extern void driveStraightC(int target);
extern void driveFollow();

#endif