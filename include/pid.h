#include "api.h"
#include "main.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"

#ifndef PIDH
#define PIDH
#define STRAIGHT_KP 2
#define STRAIGHT_KI 0.002
#define STRAIGHT_KD 10
#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5
#define ARC_KP 1
#define ARC_KI 0
#define ARC_KD 0

#define TURN_KP 4
#define TURN_KI 0.01
#define TURN_KD 30

#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define FOLLOW_INTEGRAL_KI 4.0
#define FOLLOW_MAX_INTEGRAL 1.4

#define FOLLOW_KP 0.4
#define FOLLOW_KI 0
#define FOLLOW_KD 0

#define HEADING_KP 4
#define HEADING_KI 0
#define HEADING_KD 30

#define HEADING_INTEGRAL_KI 0
#define HEADING_MAX_INTEGRAL 0

#define ARC_HEADING_KP 8.25 
#define ARC_HEADING_KI 0
#define ARC_HEADING_KD 0

#define ARC_HEADING_INTEGRAL_KI 0
#define ARC_HEADING_MAX_INTEGRAL 0

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
extern double calcPID(double target, double input, int integralKi, int maxIntegral);
extern double calcPID2(double target, double input, int integralKi, int maxIntegral);
extern void setConstants(double kp, double ki, double kd);
extern void chasMove(int voltageLF, int voltageLB, int voltageLM, int voltageRF, int voltageRB, int voltageRM);
extern void driveClamp(int target, int percentage);
extern void driveClampS(int target, int percentage, int speed);

#endif