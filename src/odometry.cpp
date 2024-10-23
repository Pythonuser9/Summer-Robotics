#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "odometry.h"
#include "pid.h"

using namespace pros;
using namespace std;

int turnv = 0;
double position = 0;
double hypot2 = 0;
double absoulouteAngleToTarget = 0;
        int btime = 0;

float deltax;
float deltay;

float startingX;
float startingY;
float startingHeading;

float r0;
float r1;

float delta_left_encoder_pos;
float delta_right_encoder_pos;
float delta_center_encoder_pos;

float prev_left_encoder_pos;
float prev_right_encoder_pos;
float prev_center_encoder_pos;

float left_encoder_pos;
float right_encoder_pos;
float center_encoder_pos;

float localx;
float localy;

float phi;

float prev_imu_pos;
float imu_pos;

float x_pos;
float y_pos;

float pi = 3.14159265359;

int odo_time = 0;

void setPosition(float xcoord, float ycoord, float heading) {
    startingX = xcoord;
    startingY = ycoord;
    startingHeading = heading;
    x_pos = startingX;
    y_pos = startingY;
}

void Odometry () {
    
    
    // x_pos = startingX;
    // y_pos = startingY;
    
    // while(true){

    	int power = con.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int RX = con.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

		int turn = int(RX);
		int left = power + turn;
		int right = power - turn;

        LF.move(left);
		LB.move(left);
		LM.move(left);
		RF.move(right);
		RB.move(right);
		RM.move(right);

    prev_imu_pos = imu_pos;
    imu_pos = imu2.get_rotation() + startingHeading;

    prev_left_encoder_pos = left_encoder_pos;
    prev_right_encoder_pos = right_encoder_pos;
    prev_center_encoder_pos = center_encoder_pos;

    left_encoder_pos = LF.get_position();
    right_encoder_pos = RF.get_position();
    center_encoder_pos = 0;
    /*/////////
    left_encoder_pos = rotoleft.get_position();
    right_encoder_pos = rotoright.get_position();
    center_encoder_pos = rotox.get_position();
    *//////////

    delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
    delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
    delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

    phi = imu_pos - prev_imu_pos;

    r0 = ((delta_left_encoder_pos + delta_right_encoder_pos) / 2) / phi;
    r1 = delta_center_encoder_pos/phi;

    if (abs(phi) < IMU_THRESHOLD) {
     localx = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
     localy = delta_center_encoder_pos - FOWARD_OFFSET * ((pi * phi) / 180);
    } else {
    localx = r0 * sin((pi * phi)/180) - r1*(1 - cos((pi * phi) / 180));
    localy = r1 * sin((pi * phi)/180) + r0*(1 - cos((pi * phi) / 180));
    }


    
    // localx = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
    // localy = delta_center_encoder_pos - FOWARD_OFFSET * phi;

    deltay = localx * cos((pi * imu_pos) / 180) - localy * sin((pi * imu_pos) / 180);
    deltax = localx * sin((pi * imu_pos) / 180) + localy * cos((pi * imu_pos) / 180);

    x_pos += deltax;
    y_pos += deltay;

    // delay(1);
    odo_time+= 10;
    if (odo_time % 50 == 0 && odo_time % 100 != 0 && odo_time % 150 != 0){
        con.print(0,0, "hypot: %f         " ,float(hypot2));
    } else if (odo_time % 100 == 0 && odo_time % 150 !=0){
        con.print(1,0, "angle: %f         ", float(absoulouteAngleToTarget));
    } else if (odo_time % 150 == 0){
        con.print(2,0, "btime %f        ", float(btime));
    }
    }


void driveToPoint (double xTarget, double yTarget, double preferredHeading) {
    while(true) {
       double turnv= 0;
        Odometry();
            double distanceToTarget = sqrt(pow((x_pos - xTarget), 2) + pow((y_pos - yTarget), 2));
            
            double absoulouteAngleToTarget = atan2(((x_pos - xTarget), 2), ((y_pos - yTarget), 2));
    double angleToTarget = absoulouteAngleToTarget - (imu_pos - 90);

    while(angleToTarget >= 360) {
    angleToTarget = angleToTarget - 360;
    }

    while(angleToTarget <= 360) {
    angleToTarget = angleToTarget + 360;
    }

    double relativeXToPoint = cos((pi*angleToTarget)/180) * distanceToTarget;
    double relativeYToPoint = sin((pi*angleToTarget)/180) * distanceToTarget;
    double relativeTurnAngle = angleToTarget - 180 + preferredHeading;

    double movementXPower = relativeXToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));
    double movementYPower = relativeYToPoint / (abs(relativeXToPoint) + abs(relativeYToPoint));

    double movementTurn = clamp(((angleToTarget)/30), 1.0, -1.0);




    }
}
    void boomerang(double xTarget, double yTarget) {
        double voltage = 0;
        double heading_correction = 0;

        int timeout = 30000;
        int count = 0;
        double hypot = 0;

        while(true) {
            Odometry();
            hypot = sqrt(pow((x_pos - xTarget), 2) + pow((y_pos - yTarget), 2));
            absoulouteAngleToTarget = (atan2(((xTarget - x_pos)),((yTarget - y_pos)))) *(180/pi);
            if (absoulouteAngleToTarget > 180) {
                absoulouteAngleToTarget = absoulouteAngleToTarget - 360;
            }
            position = imu2.get_heading();
        if (position > 180) {
            position = (position - 360);
        }

        if((absoulouteAngleToTarget > 0) && (position < 0)){
            turnv = absoulouteAngleToTarget - position;
        }   else if ((absoulouteAngleToTarget < 0) && (position > 0)) {
            turnv = position - absoulouteAngleToTarget;
        } else {
            turnv = abs(abs(absoulouteAngleToTarget) - abs(position));
        }

        // if((absoulouteAngleToTarget < 0) && (position > 0)) {
        //     if((position - absoulouteAngleToTarget) >= 180) {
        //         absoulouteAngleToTarget = absoulouteAngleToTarget + 360;
        //         position = imu2.get_heading();
        //         turnv = (absoulouteAngleToTarget - position);
        //     } else {
        //         turnv = (abs(position) + abs(absoulouteAngleToTarget));
        //     }
        // } else if ((absoulouteAngleToTarget > 0) && (position < 0)) {
        //     if((absoulouteAngleToTarget - position) >= 180) {
        //         position = imu2.get_heading();
        //         turnv = abs(abs(position) - abs(absoulouteAngleToTarget));
        //     } else {
        //         turnv = (abs(position) + absoulouteAngleToTarget);
        //     }
        // }   else {
        //         turnv = abs(abs(position) - abs(absoulouteAngleToTarget));
        // }

        if(abs(turnv) > 90) {
            absoulouteAngleToTarget = absoulouteAngleToTarget + 180;
            hypot = -hypot;
        }
        hypot2 = hypot;

    
        while (absoulouteAngleToTarget >= 359) {
            absoulouteAngleToTarget = absoulouteAngleToTarget - 360;
        }

        // if((absoulouteAngleToTarget < 0) && (position > 0)) {
        //     if((position - absoulouteAngleToTarget) >= 180) {
        //         absoulouteAngleToTarget = absoulouteAngleToTarget + 360;
        //         position = imu2.get_heading();
        //     }
        // } else if ((absoulouteAngleToTarget > 0) && (position < 0)) {
        //     if((absoulouteAngleToTarget - position) >= 180) {
        //         position = imu2.get_heading();
        //     }
        // }
        setConstants(TURN_KP, TURN_KI, TURN_KD);
        if (abs(hypot) > HEADING_CUTOFF){
                heading_correction = calcPID(absoulouteAngleToTarget, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);
        } else {
            heading_correction = 0;
        }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        voltage = -calcPID2(0, hypot, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
        if (voltage > 127) {
            voltage = 127;
        } else if (voltage < -127) {
            voltage = -127;
        }

        // heading_correction = 0;
       chasMove((voltage + heading_correction), (voltage + heading_correction), (voltage + heading_correction), (voltage - heading_correction), (voltage - heading_correction), (voltage - heading_correction));
        if(abs(hypot) < 15) count++;
        if((count > 20) || (btime > timeout)) {
             break;
        }




        btime += 10;
        pros::delay(10);

        }
    }