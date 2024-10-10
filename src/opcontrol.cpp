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
#include "umbc/robot.hpp"

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
#define LEFT_BACK_MOTOR_PORT         6
#define RIGHT_FRONT_MOTOR_PORT       3
#define RIGHT_BACK_MOTOR_PORT        7


pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT, MOTOR_REVERSE);

pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT,MOTOR_REVERSE);

pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT);

pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT);

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    umbc::util* util;

    int32_t x;
    int32_t y;
    int32_t turn;
    // int32_t power;
    // int32_t theta;
    // int32_t sin;
    // int32_t cos;
    // int32_t max;
    int32_t lf;
    int32_t rf;
    int32_t lb;
    int32_t rb;

    

    // initialize motors and sensors



    while(1) {

        //DriveBase 
         x = (controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X));
         y =    (-1 * controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
         turn = (-1 * controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

         lf = y+x+turn;        
         rf = y-x-turn;
         lb = y-x+turn;
         rb = y+x-turn;
        
        // theta = std::atan2(y, x);
        // power = std::hypot(x, y);

        // sin = std::sin(theta - (M_PI/4));
        // cos = std::cos(theta - (M_PI/4));
        // max = std::max(std::abs(sin),std::abs(cos));

        // drive_left_front_motor.move(std::int32_t (58 * (power * cos/max + turn)));
        // drive_right_front_motor.move(std::int32_t (58 * (power * sin/max - turn)));
        // drive_left_back_motor.move(std::int32_t (58 * (power * sin/max + turn)));
        // drive_right_back_motor.move(std::int32_t (58 * (power * cos/max - turn)));
        
        // drive_left_front_motor.move(std::int32_t (y*127));
        // drive_right_front_motor.move(std::int32_t (y*127));
        // drive_left_back_motor.move(std::int32_t (y*127));
        // drive_right_back_motor.move(std::int32_t (y*127));
        //  drive_left_front_motor.move(std::int32_t (x*127));
        //  drive_right_front_motor.move(std::int32_t (-x*127));
        //  drive_left_back_motor.move(std::int32_t (-x*127));
        //  drive_right_back_motor.move(std::int32_t (x*127));
        
        drive_left_front_motor.move_velocity(util->bound(lf));
        pros::lcd::print(0, "inputLF %i\n", util->bound(lf));
        drive_right_front_motor.move_velocity(util->bound(rf));
        pros::lcd::print(1, "inputRF %i\n", util->bound(rf));
        drive_left_back_motor.move_velocity(util->bound(lb));
        pros::lcd::print(2, "inputLB %i\n", util->bound(lb));
        drive_right_back_motor.move_velocity(util->bound(rb));
        pros::lcd::print(3, "inputRB %i\n", util->bound(rb));

       
        
        





        
        
        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }


    
}



