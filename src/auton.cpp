#include "api.h"
#include "pid.h"
#include "robot.h"
#include "main.h"
#include "auton.h"


void autonomous() {



if (atn == 0) {
} else if (atn == 1) {
driveStraight2(1818);
driveTurn2(-90);
driveStraight2(1818);
driveTurn2(90);
driveStraight2(1818);
driveTurn2(180);
driveStraight(1818);
} else if (atn == 2) {
} else if (atn == 3) {
} else if (atn == 4) {
} else if (atn == 5) {
} else if (atn == 6) {
} 


}