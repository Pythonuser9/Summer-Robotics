#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "odometry.h"


using namespace pros;
using namespace c;
using namespace std;

double vKp;
double vKi;
double vKd;
double power;
double derivative;
double prevError;
float error;
int time2;
int time22 = 10;

double vKp2;
double vKi2;
double vKd2;
double power2;
double derivative2;
double prevError2;
float error2;
int integral2;

double vKp3;
double vKi3;
double vKd3;
double power3;
double derivative3;
double prevError3;
float error3;
int integral3;

void setConstants(double kp, double ki, double kd) {
    vKp = kp;
    vKi = ki;
    vKd = kd;
}

void resetEncoders() {
    LF.tare_position();
    LM.tare_position();
    LB.tare_position();
    RF.tare_position();
    RM.tare_position();
    RB.tare_position();
}

void chasMove(int voltageLF, int voltageLB, int voltageLM, int voltageRF, int voltageRB, int voltageRM){
    LF.move(voltageLF);
    LB.move(voltageLB);
    LM.move(voltageLM);
    RF.move(voltageRF);
    RB.move(voltageRB);
    RM.move(voltageRM);
}

double calcPID(double target, double input, int integralKi, int maxIntegral) {
int integral;
prevError = error;
error = target - input;

if(std::abs(error) < integralKi ) {
    integral += error;
} else {
    integral = 0;
}

if(integral >= 0) {
    integral = std::min(integral, maxIntegral);
} else {

    integral = std::max(integral, -maxIntegral);
}

    derivative = error - prevError;

    power = (vKp * error) + (vKi * integral) + (vKd * derivative);

    return power;

}

double calcPID2(double target, double input, int integralKi, int maxIntegral) {
int integral2;
prevError2 = error2;
error2 = target - input;

if(std::abs(error2) < integralKi ) {
    integral2 += error2;
} else {
    integral2 = 0;
}

if(integral2 >= 0) {
    integral2 = std::min(integral2, maxIntegral);
} else {

    integral2 = std::max(integral2, -maxIntegral);
}

    derivative2 = error2 - prevError2;

    power2 = (vKp * error2) + (vKi * integral2) + (vKd * derivative2);

    return power2;
}

double calcPID3(double target, double input, int integralKi, int maxIntegral) {
int integral3;
prevError3 = error3;
error3 = target - input;

if(std::abs(error3) < integralKi ) {
    integral3 += error3;
} else {
    integral3 = 0;
}

if(integral3 >= 0) {
    integral3 = std::min(integral3, maxIntegral);
} else {

    integral3 = std::max(integral3, -maxIntegral);
}

    derivative3 = error3 - prevError3;

    power3 = (vKp * error3) + (vKi * integral3) + (vKd * derivative3);

    return power3;
}

void driveStraight(int target) {

    int timeout = 50000;

 double x = 0; 
 x = double(abs(target));
 //timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;

double voltage;
double encoderAvg;
int count = 0;
int cycle = 0;
int time2 = 0;


setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
resetEncoders();

 imu2.tare_heading();

double init_heading = imu2.get_heading();
if (init_heading > 180){
    init_heading = init_heading - 360;
}
double heading_error = 0;
while(true) {

    double position = imu2.get_heading();

if (position > 180) {
    position = position - 360;
}

if((init_heading < 0) && (position > 0)) {
    if((position - init_heading) >= 180) {
        init_heading = init_heading + 360;
        position = imu2.get_heading();
    }
} else if ((init_heading > 0) && (position < 0)) {
    if((init_heading - position) >= 180) {
        position = imu2.get_heading();
    }
}

setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

setConstants(STRAIGHT_KP,STRAIGHT_KI,STRAIGHT_KD);
encoderAvg = (LF.get_position() + RF.get_position()) / 2;
voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

chasMove (voltage + heading_error, voltage + heading_error, voltage + heading_error, voltage - heading_error, voltage - heading_error, voltage - heading_error);
if (abs(target - encoderAvg) <= 4) count++;
if (count >= 20 || time2 > timeout) {
  break;
}

if (time2 % 50 == 0) {
con.print(0,0,"TIME: %f          ", float(time2) );

}
delay(10);
time2 += 10;
time22 = time2;
}

LF.brake();
LB.brake();
RF.brake();
RB.brake();
}
 

void driveStraight2(int target) {

    int timeout = 5000;
double x = 0; 
 x = double(abs(target));
 //timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;

double voltage;
double encoderAvg;
int count = 0;
int cycle = 0;
int time2 = 0;

setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
resetEncoders();


double init_heading = imu2.get_heading();
if (init_heading > 180) {
    init_heading = (360 - init_heading);
}
double heading_error = 0;
while(true) {

Odometry();

double position = imu2.get_heading();
if (position > 180) {
    position = position - 360;
}

if((init_heading < 0) && (position > 0)) {
    if((position - init_heading) >= 180) {
        init_heading = init_heading + 360;
        position = imu2.get_heading();
    }
} else if ((init_heading > 0) && (position < 0)) {
    if((init_heading - position) >= 180) {
        position = imu2.get_heading();
    }
}

setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

encoderAvg = (LF.get_position() + RF.get_position()) / 2;
setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

chasMove (voltage + heading_error, voltage + heading_error, voltage + heading_error, voltage - heading_error, voltage - heading_error, voltage - heading_error);
if (abs(target - encoderAvg) <= 4) count++;
if (count >= 20 || time2 > timeout) {
   break;
}

if (time2 % 50 == 0) {
con.print(0,0,"ERROR: %f          ", float(heading_error) );

}
delay(10);
time2 += 10;
}

LF.brake();
LB.brake();
RF.brake();
RB.brake();
}

void driveSlow(int target, int speed) {

    int timeout = 5000;
double x = 0; 
 x = double(abs(target));
 //timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;

double voltage;
double encoderAvg;
int count = 0;
int cycle = 0;
int time2 = 0;

setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
resetEncoders();


double init_heading = imu2.get_heading();
if (init_heading > 180) {
    init_heading = (360 - init_heading);
}
double heading_error = 0;
while(true) {

Odometry();

double position = imu2.get_heading();
if (position > 180) {
    position = position - 360;
}

if((init_heading < 0) && (position > 0)) {
    if((position - init_heading) >= 180) {
        init_heading = init_heading + 360;
        position = imu2.get_heading();
    }
} else if ((init_heading > 0) && (position < 0)) {
    if((init_heading - position) >= 180) {
        position = imu2.get_heading();
    }
}

setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

encoderAvg = (LF.get_position() + RF.get_position()) / 2;
setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

if(voltage > 127 * double(speed)/100.0) {
    voltage = 127 * double(speed)/100.0;
} else if (voltage < -127 * double(speed)/100.0){
    voltage = -127 * double(speed)/100.0;
}


chasMove (voltage + heading_error, voltage + heading_error, voltage + heading_error, voltage - heading_error, voltage - heading_error, voltage - heading_error);
if (abs(target - encoderAvg) <= 4) count++;
if (count >= 20 || time2 > timeout) {
   break;
}

if (time2 % 50 == 0) {
con.print(0,0,"ERROR: %f          ", float(heading_error) );

}
delay(10);
time2 += 10;
}

LF.brake();
LB.brake();
RF.brake();
RB.brake();
}


void driveClamp(int target, int percentage) {

    int timeout = 5000;
double x = 0; 
 x = double(abs(target));
 //timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;

double voltage;
double encoderAvg;
int count = 0;
int cycle = 0;
int time2 = 0;

setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
resetEncoders();


double init_heading = imu2.get_heading();
if (init_heading > 180) {
    init_heading = (360 - init_heading);
}
double heading_error = 0;
while(true) {

Odometry();

double position = imu2.get_heading();
if (position > 180) {
    position = position - 360;
}

if((init_heading < 0) && (position > 0)) {
    if((position - init_heading) >= 180) {
        init_heading = init_heading + 360;
        position = imu2.get_heading();
    }
} else if ((init_heading > 0) && (position < 0)) {
    if((init_heading - position) >= 180) {
        position = imu2.get_heading();
    }
}

setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

encoderAvg = (LF.get_position() + RF.get_position()) / 2;
setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

if(abs(encoderAvg) > target * (double(percentage)/100.0)){
    piston1.set_value(true);
}


chasMove (voltage + heading_error, voltage + heading_error, voltage + heading_error, voltage - heading_error, voltage - heading_error, voltage - heading_error);
if (abs(target - encoderAvg) <= 4) count++;
if (count >= 20 || time2 > timeout) {
   break;
}

if (time2 % 50 == 0) {
con.print(0,0,"ERROR: %f          ", float(heading_error) );

}
delay(10);
time2 += 10;
}

LF.brake();
LB.brake();
LM.brake();
RF.brake();
RB.brake();
RM.brake();
}
void driveClampS(int target, int percentage, int speed) {

    int timeout = 5000;
double x = 0; 
 x = double(abs(target));
 //timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;

double voltage;
double encoderAvg;
int count = 0;
int cycle = 0;
int time2 = 0;

setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
resetEncoders();


double init_heading = imu2.get_heading();
if (init_heading > 180) {
    init_heading = (360 - init_heading);
}
double heading_error = 0;
while(true) {

Odometry();

double position = imu2.get_heading();
if (position > 180) {
    position = position - 360;
}

if((init_heading < 0) && (position > 0)) {
    if((position - init_heading) >= 180) {
        init_heading = init_heading + 360;
        position = imu2.get_heading();
    }
} else if ((init_heading > 0) && (position < 0)) {
    if((init_heading - position) >= 180) {
        position = imu2.get_heading();
    }
}

setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

encoderAvg = (LF.get_position() + RF.get_position()) / 2;
setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

if(abs(encoderAvg) > target * (double(percentage)/100.0)){
    piston1.set_value(true);
}
if(voltage > 127 * double(speed)/100.0) {
    voltage = 127 * double(speed)/100.0;
} else if (voltage < -127 * double(speed)/100.0){
    voltage = -127 * double(speed)/100.0;
}

chasMove (voltage + heading_error, voltage + heading_error, voltage + heading_error, voltage - heading_error, voltage - heading_error, voltage - heading_error);
if (abs(target - encoderAvg) <= 4) count++;
if (count >= 20 || time2 > timeout) {
   break;
}

if (time2 % 50 == 0) {
con.print(0,0,"ERROR: %f          ", float(heading_error) );

}
delay(10);
time2 += 10;
}

LF.brake();
LB.brake();
LM.brake();
RF.brake();
RB.brake();
RM.brake();
}

void driveStraightC(int target) {

bool over = false;
    int timeout = 5000;

double x = 0; 
 x = double(abs(target));
//  timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;

if(target > 0){
target = target + 500;
} else {
    target = target - 500;
}

double voltage;
double encoderAvg;
int count = 0;
int cycle = 0;
int time2 = 0;

setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
resetEncoders();


double init_heading = imu2.get_heading();
if (init_heading > 180) {
    init_heading = (360 - init_heading);
}

double heading_error = 0;
while(true) {

    Odometry();

    double position = imu2.get_heading();    

    if (position > 180) {
    position = position - 360;
}

if((init_heading < 0) && (position > 0)) {
    if((position - init_heading) >= 180) {
        init_heading = init_heading + 360;
        position = imu2.get_heading();
    }
} else if ((init_heading > 0) && (position < 0)) {
    if((init_heading - position) >= 180) {
        position = imu2.get_heading();
    }
}
    setConstants(HEADING_KP, HEADING_KI, HEADING_KD);
    heading_error = calcPID2(init_heading, position, HEADING_INTEGRAL_KI, HEADING_MAX_INTEGRAL);

    encoderAvg = (LF.get_position() + RF.get_position()) / 2;
    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

    chasMove (voltage + heading_error, voltage + heading_error, voltage + heading_error, voltage - heading_error, voltage - heading_error, voltage - heading_error);
    if (target > 0 ){
        if ((encoderAvg - (target - 500)) > 0){
            over = true;
        }
    } else {
        if (((target+500) - encoderAvg) > 0){
            over = true;
        }
    }
    if (over || time2 > timeout) {
    break;
    }



    if (time2 % 50 == 0) {
    con.print(0,0,"ERROR: %f          ", float(heading_error) );

    }
    delay(10);
    time2 += 10;
}

LF.brake();
LB.brake();
RF.brake();
RB.brake();
}




void driveTurn(int target) {
double position;
double voltage;
int count = 0;
int timeout = 50000;
int time2 = 0;
setConstants(TURN_KP, TURN_KI, TURN_KD);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    double x = 0;

    double variKP = 0;
    x = double(abs(target));
    variKP = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;
    
    double variKD = 0;
    x = double(abs(target));
    variKD = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;
    
    setConstants(variKP, TURN_KI, variKD);

    double x = 0; 
 x = double(abs(target));
 timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

imu2.tare_heading();

while(true) {

position = imu2.get_heading();

if(position > 180) {
    position = (position - 360);
}
voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

if (time2 % 50 == 0) {
con.print(0,0,"ERROR: %f          ", float(error) );

}

if(abs(target - position) <= 1) count++; 
if(count >= 20 || time2 > timeout) {

   break;
}
time2 += 10;
delay(10);
}
LF.brake();
LB.brake();
RF.brake();
RB.brake();
}

void driveTurn2(int target) {
    double voltage;
    double position;
    int count = 0;
    int time2 = 0;
    int turnv = 0; 
    setConstants(TURN_KP, TURN_KI, TURN_KD); 

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    double x = 0;

    double variKP = 0;
    x = double(abs(target));
    variKP = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;
    
    double variKD = 0;
    x = double(abs(target));
    variKD = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;
    
    setConstants(variKP, TURN_KI, variKD);

    double x = 0; 
 x = double(abs(target));
 timeout = (-0.000000000000038674 * pow(x,5)) +  (0.00000000022566 * pow(x,4)) +  (-0.00000046518 * pow(x,3)) +  (0.00036292 * pow(x,2)) +  (0.318831 * (x)) + 465.386;
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    position = imu2.get_heading();
    if (position > 180) {
        position = (position - 360);
    }

    if((target < 0) && (position > 0)) {
        if((position - target) >= 180) {
            target = target + 360;
            position = imu2.get_heading();
            turnv = (target - position);
        } else {
            turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)) {
        if((target - position) >= 180) {
            position = imu2.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }
    }   else {
            turnv = abs(abs(position) - abs(target));
    }
        int timeout = 5000;

    while(true) {

        Odometry();

        position = imu2.get_heading();
        if(position > 180) {
            position = ((360 - position) * -1);
        } 
 if((target < 0) && (position > 0)) {
        if((position - target) >= 180) {
            target = target + 360;
            position = imu2.get_heading();
            turnv = (target - position);
        } else {
            turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)) {
        if((target - position) >= 180) {
            position = imu2.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);  
        }
    }   else {
            turnv = abs(abs(position) - abs(target));
    }        
        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);

        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);

        if(abs(target -position) <= 1) count++;
        if (count >=20 || time2 > timeout ) {
            break;
        }
        time2 += 10;
        delay(10);
        if (time2 % 50 == 0) {
con.print(0,0,"ERROR: %f          ", float(error) );

}
    }
    LF.brake();
    LB.brake();
    LM.brake();
    RF.brake();
    RB.brake();
    RM.brake();
}

void driveArcL(double theta, double radius, int timeout) {
double ltarget = 0;
double rtarget = 0;
double pi = 3.14159265359;
double init_heading = imu2.get_heading();
int count = 0;
int time = 0;
resetEncoders();

if(init_heading > 180){
    init_heading = init_heading - 360;
}

con.clear();
ltarget = double((theta / 360) * 2 * pi * radius);
rtarget = double((theta / 360) * 2 * pi * (radius + 820));
while (true) {

    Odometry();

    double position = imu2.get_heading();
    if (position > 180) {
        position = position - 360;
    }

    if((init_heading < 0) && (position > 0)){
        if((position - init_heading ) >= 180){
            init_heading = init_heading + 360;
            position = imu2.get_heading();
        }
    }   else if ((init_heading > 0) && (position < 0)){
        if((init_heading - position) >= 180){
            position = imu2.get_heading();
        }
    }


double encoderAvgL = (LB.get_position() + LF.get_position()) /2;
double encoderAvgr = (RB.get_position() + RF.get_position()) /2;

setConstants(ARC_KP, ARC_KI, ARC_KD);
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageL > 70){
    voltageL = 70;
} else if (voltageL < -70){
    voltageL = -70;
}
int voltageR = calcPID2(rtarget, encoderAvgr, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageR > 100){
    voltageR = 100;
} else if (voltageR < -100){
    voltageR = -100;
}
double leftcorrect = (encoderAvgL * 360) / (2 * pi * radius);
setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
int fix = calcPID3((init_heading - leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);

chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgr) >= 4)) count++;
if (count >= 20 || time > timeout){
    break;
}

time+= 10;
delay(10);
con.print(0,0, "Target: %f         ", float(voltageR));
}
}

void driveArcLF(double theta, double radius, int timeout) {
    bool over = false;
double ltarget = 0;
double rtarget = 0;
double ltargetFinal = 0;
double rtargetFinal = 0;
double pi = 3.14159265359;
double init_heading = imu2.get_heading();
int count = 0;
int time = 0;
resetEncoders();
    if (init_heading > 180) {
        init_heading = init_heading - 360;
}

con.clear();

ltargetFinal = double((theta / 360) * 2 * pi * radius);
rtargetFinal = double((theta / 360) * 2 * pi * (radius + 820));

theta = theta + 45;

ltarget = double((theta / 360) * 2 * pi * radius);
rtarget = double((theta / 360) * 2 * pi * (radius + 820));
while (true) {

    Odometry();

double position = imu2.get_heading();
    if (position > 180) {
        position = position - 360;
    }

    if((init_heading < 0) && (position > 0)){
        if((position - init_heading ) >= 180){
            init_heading = init_heading + 360;
            position = imu2.get_heading();
        }
    }   else if ((init_heading > 0) && (position < 0)){
        if((init_heading - position) >= 180){
            position = imu2.get_heading();
        }
    }


double encoderAvgL = (LB.get_position() + LF.get_position()) /2;
double encoderAvgr = (RB.get_position() + RF.get_position()) /2;

setConstants(ARC_KP, ARC_KI, ARC_KD);
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageL > 70){
    voltageL = 70;
} else if (voltageL < -70){
    voltageL = -70;
}
int voltageR = calcPID2(rtarget, encoderAvgr, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageR > 100){
    voltageR = 100;
} else if (voltageR < -100){
    voltageR = -100;
}
double leftcorrect = (encoderAvgL * 360) / (2 * pi * radius);
setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
int fix = calcPID3((init_heading - leftcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);


chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
if (radius > 0){
    if ((encoderAvgL - ltargetFinal) > 0){
        over = true;
    }
} else {
    if ((ltargetFinal - encoderAvgL) > 0){
        over = true;
    }
}
if (over || time > timeout){
    break;
}

time+= 10;
delay(10);
con.print(0,0, "Target: %f         ", float(voltageR));
}

}
void driveArcR(double theta, double radius, int timeout) {
double ltarget = 0;
double rtarget = 0;
double pi = 3.14159265359;
double init_heading = imu2.get_heading();
int count = 0;
int time = 0;
resetEncoders();
if (init_heading > 180) {
        init_heading = init_heading - 360;
}

con.clear();
rtarget = double((theta / 360) * 2 * pi * radius);
ltarget = double((theta / 360) * 2 * pi * (radius + 820));
while (true) {

    Odometry();

double position = imu2.get_heading();
    if (position > 180) {
        position = position - 360;
    }

    if((init_heading < 0) && (position > 0)){
        if((position - init_heading ) >= 180){
            init_heading = init_heading + 360;
            position = imu2.get_heading();
        }
    }   else if ((init_heading > 0) && (position < 0)){
        if((init_heading - position) >= 180){
            position = imu2.get_heading();
        }
    }

double encoderAvgL = (LB.get_position() + LF.get_position()) /2;
double encoderAvgr = (RB.get_position() + RF.get_position()) /2;

setConstants(ARC_KP, ARC_KI, ARC_KD);
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageL > 70){
    voltageL = 70;
} else if (voltageL < -70){
    voltageL = -70;
}
int voltageR = calcPID2(rtarget, encoderAvgr, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageR > 100){
    voltageR = 100;
} else if (voltageR < -100){
    voltageR = -100;
}
double rightcorrect = (encoderAvgr * 360) / (2 * pi * radius);
setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
int fix = calcPID3((init_heading + rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);


chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgr) >= 4)) count++;
if (count >= 20 || time > timeout){
    break;
}

time+= 10;
delay(10);
con.print(0,0, "Target: %f         ", float(voltageR));
}


}

void driveArcRF(double theta, double radius, int timeout) {
    bool over = false;
double ltarget = 0;
double rtarget = 0;
double ltargetFinal = 0;
double rtargetFinal = 0;
double pi = 3.14159265359;
double init_heading = imu2.get_heading();
if(init_heading > 180){
    init_heading = init_heading - 360;
}
int count = 0;
int time = 0;
resetEncoders();
con.clear();
rtargetFinal = double((theta / 360) * 2 * pi * radius);
ltargetFinal = double((theta / 360) * 2 * pi * (radius + 820));

theta = theta + 45;

rtarget = double((theta / 360) * 2 * pi * radius);
ltarget = double((theta / 360) * 2 * pi * (radius + 820));
while (true) {

    Odometry();

double position = imu2.get_heading();
    if (position > 180) {
        position = position - 360;
    }

    if((init_heading < 0) && (position > 0)){
        if((position - init_heading ) >= 180){
            init_heading = init_heading + 360;
            position = imu2.get_heading();
        }
    }   else if ((init_heading > 0) && (position < 0)){
        if((init_heading - position) >= 180){
            position = imu2.get_heading();
        }
    }


double encoderAvgL = (LB.get_position() + LF.get_position()) /2;
double encoderAvgr = (RB.get_position() + RF.get_position()) /2;

setConstants(ARC_KP, ARC_KI, ARC_KD);
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageL > 70){
    voltageL = 70;
} else if (voltageL < -70){
    voltageL = -70;
}
int voltageR = calcPID2(rtarget, encoderAvgr, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
if(voltageR > 100){
    voltageR = 100;
} else if (voltageR < -100){
    voltageR = -100;
}
double rightcorrect = (encoderAvgr * 360) / (2 * pi * radius);
setConstants(ARC_HEADING_KP, ARC_HEADING_KI, ARC_HEADING_KD);
int fix = calcPID3((init_heading + rightcorrect), position, ARC_HEADING_INTEGRAL_KI, ARC_HEADING_MAX_INTEGRAL);


chasMove((voltageL + fix), (voltageL + fix), (voltageL + fix), (voltageR - fix), (voltageR - fix), (voltageR - fix));
if (radius > 0){
        if ((encoderAvgr - (rtargetFinal)) > 0){
            over = true;
        }
} else {
    if (((rtargetFinal) - encoderAvgr) > 0){
        over = true;
    }
}
if (over || time > timeout){
    break;
}

time+= 10;
delay(10);
con.print(0,0, "Target: %f         ", float(voltageR));
}


}

void driveFollow (){
   pros::vision_signature_s_t RED_SIG =
	pros::Vision::signature_from_utility(1, 6111, 9383, 7746, -789, 331, -230, 3.300, 0);
	VIS.set_signature(1, &RED_SIG);
    setConstants (FOLLOW_KP, FOLLOW_KI, FOLLOW_KD);
    while (true) {
        pros::vision_object_s_t driver = VIS.get_by_sig(0, 1);
		int XCOORD = driver.x_middle_coord - 150;
        int voltage = calcPID(0, XCOORD, FOLLOW_INTEGRAL_KI, FOLLOW_MAX_INTEGRAL);
        chasMove(-voltage, -voltage, -voltage, voltage, voltage, voltage);

    }
}