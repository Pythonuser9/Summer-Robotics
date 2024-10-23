#include "robot.h"
#include "pros/motors.h"
#include "api.h"
#include "main.h"

#define LF_PORT 16
#define RF_PORT 4
#define LB_PORT 13
#define RB_PORT 4
#define LM_PORT 11
#define RM_PORT 10
#define LIFT_PORT 5
#define INTAKE_PORT 6
#define IMU2_PORT 17
#define VISION_PORT 8
#define ROTOX_PORT 9
#define ROTORIGHT_PORT 10
#define ROTOLEFT_PORT 11
#define OPTICAL_PORT 21

pros::Rotation rotox(ROTOX_PORT);
pros::Rotation rotoleft(ROTOLEFT_PORT);
pros::Rotation rotoright(ROTORIGHT_PORT);
pros::Optical optical(OPTICAL_PORT, 10);

pros::Controller con (pros::E_CONTROLLER_MASTER);
pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true);

pros::Motor LIFT(LIFT_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor INTAKE(INTAKE_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::ADIDigitalOut piston1 ('A', false);
pros::Imu imu2 (IMU2_PORT);
pros::ADIDigitalIn selec ('H');
pros::Vision VIS(VISION_PORT);