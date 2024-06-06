#include "robot.h"
#include "pros/motors.h"
#include "api.h"
#include "main.h"

#define LF_PORT 4
#define RF_PORT 1
#define LB_PORT 2
#define RB_PORT 13
#define LIFT_PORT 8
#define INTAKE_PORT 10
#define IMU2_PORT 20
#define VISION_PORT 3


pros::Controller con (pros::E_CONTROLLER_MASTER);
pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LIFT(LIFT_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor INTAKE(INTAKE_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::ADIDigitalOut piston1 ('A', false);
pros::Imu imu2 (IMU2_PORT);
pros::ADIDigitalIn selec ('H');
pros::Vision VIS(VISION_PORT);