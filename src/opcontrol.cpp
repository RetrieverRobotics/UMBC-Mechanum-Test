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

#include <cmath>
#include <cstdint>
#include <cstdlib>

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
#define RIGHT_BACK_MOTOR_PORT        5


pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT);

pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT,MOTOR_REVERSE);

pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT);

pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT, MOTOR_REVERSE);

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    float x;
    float y;
    float turn;
    float power;
    float theta;
    float sin;
    float cos;
    float max;

    

    // initialize motors and sensors



    while(1) {
        x = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        y = -controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        turn = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        theta = std::atan2(y, x);
        power = std::hypot(x, y);

        sin = std::sin(theta - (M_PI/4));
        cos = std::cos(theta - (M_PI/4));
        max = std::max(std::abs(sin),std::abs(cos));

        drive_left_front_motor.move(std::int32_t (127 * power * cos/max + turn));
        drive_right_front_motor.move(std::int32_t (127 * power * sin/max - turn));
        drive_left_back_motor.move(std::int32_t (127 * power * sin/max + turn));
        drive_right_back_motor.move(std::int32_t (127 * power * cos/max - turn));




        
        
        
        // drive_left_front_motor.move_velocity(controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 + 
        // controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 - controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // drive_left_back_motor.move_velocity(-controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 - 
        // controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 - controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // drive_right_front_motor.move_velocity(-controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 + 
        // controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 - controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // drive_right_back_motor.move_velocity(controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*1.58 - 
        // controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*1.58 - controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        
        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}