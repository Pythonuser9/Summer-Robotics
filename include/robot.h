#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

#ifndef ROBOTH
#define ROBOTH

extern pros::Motor LF;
extern pros::Motor RF;
extern pros::Motor LB;
extern pros::Motor RB;
extern pros::Controller con;
extern pros::Motor LIFT;
extern pros::Motor INTAKE;
extern pros::ADIDigitalOut piston1;
extern pros::Imu imu2;
extern pros::ADIDigitalIn selec;
extern pros:: Vision VIS;


#endif