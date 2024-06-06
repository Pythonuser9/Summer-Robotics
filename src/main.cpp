#include "main.h"
#include "robot.h"
#include "api.h"
#include "pid.h"
//#include "api.hpp"
#include "auton.h"

using namespace pros;
using namespace std;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

int atn = 0;
string autstr;

void competition_initialize(){
while(true) {
	if(selec.get_value() == true) {
		atn ++;
		delay(350);
	}
	if(atn == 0) {
		autstr = "SKILLS";
		con.print(0, 0, "Aut 0: %s        ", autstr);
	}
	else if (atn == 1) {
		autstr = "AUTON 1";
		con.print(0, 0, "Aut 1: %s        ", autstr);
	} 
	else if (atn == 2) {
		autstr = "AUTON 2";
		con.print(0, 0, "Aut 2: %s        ", autstr);
	}
	else if (atn == 3) {
		autstr = "AUTON 3";
		con.print(0, 0, "Aut 3: %s        ", autstr);
	}
	else if (atn == 4) {
		autstr = "AUTON 4";
		con.print(0, 0, "Aut 4: %s        ", autstr);
	}
	else if (atn == 5) {
		autstr = "AUTON 5";
		con.print(0, 0, "Aut 5: %s        ", autstr);
	}
	else if (atn == 6) {
		autstr = "AUTON 6";
		con.print(0, 0, "Aut 6: %s        ", autstr);
	}
	else if (atn == 7) {
		atn = 0;
	}
} 
}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::vision_signature_s_t RED_SIG =
	pros::Vision::signature_from_utility(1, 6111, 9383, 7746, -789, 331, -230, 3.300, 0);
	VIS.set_signature(1, &RED_SIG);
	imu2.reset();
imu2.tare_heading();
int time = 0;
	bool arcToggle = true;
	bool tankToggle = false;
	bool pistonToggle = false;
	// pros::Controller master(pros::E_CONTROLLER_MASTER);
	// pros::Motor left_mtr(1);
	// pros::Motor right_mtr(2);
 bool NEWR1;
 bool NEWR2;
	while (true) {
		pros::vision_object_s_t driver = VIS.get_by_sig(0, 1);
		int XCOORD = driver.x_middle_coord;

		int power = con.get_analog(ANALOG_LEFT_Y);
		int RX = con.get_analog(ANALOG_RIGHT_X);

		int turn = int(RX);
		int left = power + turn;
		int right = power - turn;

		if (con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))  {

		arcToggle = !arcToggle;
		tankToggle = !tankToggle;
		}
		if (con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {

		pistonToggle = !pistonToggle;
		}	

		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
			NEWR1 = true;
		} else {
			NEWR1 = false;
		} 

		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
			NEWR2 = true;
		} else {
			NEWR2 = false;
		} 
		if(((con.get_digital(E_CONTROLLER_DIGITAL_R1) && NEWR2) || (NEWR1 && con.get_digital(E_CONTROLLER_DIGITAL_R2))) || ((NEWR1 && NEWR2) || (con.get_digital(E_CONTROLLER_DIGITAL_R1) && con.get_digital(E_CONTROLLER_DIGITAL_R2)))){
		}
		// pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		//                  (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		//                  (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		// int left = master.get_analog(ANALOG_LEFT_Y);
		// int right = master.get_analog(ANALOG_RIGHT_Y);

		// left_mtr = left;
		// right_mtr = right;
	    if (tankToggle) {
		LF.move(con.get_analog(ANALOG_LEFT_Y));
		LB.move(con.get_analog(ANALOG_LEFT_Y));
		RF.move(con.get_analog(ANALOG_RIGHT_Y));
		RB.move(con.get_analog(ANALOG_RIGHT_Y));
		}
		// LF.move(127);
		// LB.move(127);
		// RF.move(127);
		// RB.move(127);

		if (arcToggle) {
		LF.move(left);
		LB.move(left);
		RF.move(right);
		RB.move(right);
		}
		if (pistonToggle) {
		piston1.set_value(true);
		} else {
		piston1.set_value(false);
		}
		if (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
		INTAKE.move(127);
		} else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
		INTAKE.move(-127);
		} else {
		INTAKE.move(0);
		}

		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
		
		 //driveStraight(1115);
		driveFollow();
		// driveArcRF(90,500,3500);
		//  driveStraight2(700);
		}

		if (time % 50 == 0 && time % 100 != 0 && time % 150 == 0){
			con.print(0, 0, "AUTON: %s      ", autstr );
		} else if (time % 50 == 0 && time % 100 != 0) {
			con.print(1, 0, "Imu: %f    ", float(imu2.get_heading()));			
		} else if (time % 50 == 0) {
			con.print(2, 0, "COORD: %f      ", float (XCOORD));
		}





delay(10);
time += 10;
		}

}
