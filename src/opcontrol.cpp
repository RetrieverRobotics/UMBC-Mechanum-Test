/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "umbc.h"
#include "umbc/robot.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

using namespace pros;
using namespace umbc;
using namespace std;

#define GEARBOX_COLOR                'g'
#define MOTOR_RED_GEAR_MULTIPLIER    100
#define MOTOR_GREEN_GEAR_MULTIPLIER  200
#define MOTOR_BLUE_GEAR_MULTIPLIER   600
#define MOTOR_REVERSE                true
#define JOYSTICK_MAX                 16129

//motor ports
#define LEFT_FRONT_MOTOR_PORT        1
#define LEFT_BACK_MOTOR_PORT         6
#define RIGHT_FRONT_MOTOR_PORT       3
#define RIGHT_BACK_MOTOR_PORT        7

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize motors and groups
    std::vector<int8_t> leftFront, leftBack, rightFront, rightBack;
    pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT, MOTOR_REVERSE);
    pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT,MOTOR_REVERSE);
    pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT);
    pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT);

    leftFront.assign ({LEFT_FRONT_MOTOR_PORT });
    leftBack.assign  ({LEFT_BACK_MOTOR_PORT  });
    rightFront.assign({RIGHT_FRONT_MOTOR_PORT});
    rightBack.assign ({RIGHT_BACK_MOTOR_PORT });

    Motor_Group frontLeftGroup (leftFront);
    Motor_Group backLeftGroup  (leftBack);
    Motor_Group frontRightGroup(rightFront);
    Motor_Group backRightGroup (rightBack);

    // assigns the relevant gearing to all of the motor groups
    int32_t gearMult = 1; 
    pros::motor_gearset_e gearColor;
    switch (GEARBOX_COLOR) {
        case 'R':
        case 'r':
            gearColor = E_MOTOR_GEAR_RED;
            gearMult = MOTOR_RED_GEAR_MULTIPLIER;
            break;
        case 'G':
        case 'g':
            gearColor = pros::E_MOTOR_GEAR_GREEN;
            gearMult = MOTOR_GREEN_GEAR_MULTIPLIER;
            break;
        case 'B':
        case 'b':
        default:
            gearColor = pros::E_MOTOR_GEAR_BLUE;
            gearMult = MOTOR_BLUE_GEAR_MULTIPLIER;
    }

    frontLeftGroup.set_gearing(gearColor);
    backLeftGroup.set_gearing(gearColor);
    frontRightGroup.set_gearing(gearColor);
    backRightGroup.set_gearing(gearColor);

    double x, y, turn, lf, rf, lb, rb; // used in vector calculations

    while(1) {

        // drive train

        //gets inputs from controller and assigns the values to x, y, and turn
        x = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        y = -controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        turn = (-1 * controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        
        //translates the x, y, and turn values into an integer value from -1 to 1
        x = (x < 0) ? x * x * -1 : x * x;
        x /= JOYSTICK_MAX;
        y = (y < 0) ? y * y * -1 : y * y;
        y /= JOYSTICK_MAX;
        turn = (turn < 0) ? turn * turn * -1 : turn * turn;
        turn /= JOYSTICK_MAX;
        
        //calculates the required power to each motor as a value from -1 to 1
        lf = (y+x+turn);
        rf = (y-x-turn);
        lb = (y-x+turn);
        rb = (y+x-turn);
        pros::lcd::print(0, "inputLF %d\n", lf);
        pros::lcd::print(1, "inputRF %d\n", rf);
        pros::lcd::print(2, "inputLB %d\n", lb);
        pros::lcd::print(3, "inputRB %d\n", rb);
        
        //sends the velocity to the motor
        drive_left_front_motor.move_velocity(lf*gearMult);
        drive_right_front_motor.move_velocity(rf*gearMult);
        drive_left_back_motor.move_velocity(lb*gearMult);
        drive_right_back_motor.move_velocity(rb*gearMult);
        
        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }   
}