/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "pros/misc.h"
#include "umbc.h"

#include <cstdint>

using namespace pros;
using namespace umbc;
using namespace std;

#define MOTOR_RED_GEAR_MULTIPLIER    100
#define MOTOR_GREEN_GEAR_MULTIPLIER  200
#define MOTOR_BLUE_GEAR_MULTIPLIER   600
#define MOTOR_REVERSE                true

//motor ports
#define LEFT_FRONT_MOTOR_PORT        1
#define LEFT_BACK_MOTOR_PORT         2
#define RIGHT_FRONT_MOTOR_PORT       3
#define RIGHT_BACK_MOTOR_PORT        4


pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT, MOTOR_REVERSE);

pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT);

pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT, MOTOR_REVERSE);

pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT);


void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize motors and sensors



    while(1) {
        drive_left_front_motor.move_velocity(controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 + 
        controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 + controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        drive_left_back_motor.move_velocity(controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 - 
        controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 + controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        drive_right_front_motor.move_velocity(-controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 + 
        controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 + controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        drive_right_back_motor.move_velocity(controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 - 
        controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 + controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        
        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}