/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "pros/adi.hpp"
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
#define RIGHT_FRONT_MOTOR_PORT_1    -5
#define RIGHT_FRONT_MOTOR_PORT_2    6
#define RIGHT_BACK_MOTOR_PORT_1     -7
#define RIGHT_BACK_MOTOR_PORT_2     8
#define LEFT_FRONT_MOTOR_PORT_1 1
#define LEFT_FRONT_MOTOR_PORT_2 -2
#define LEFT_BACK_MOTOR_PORT_1 -16
#define LEFT_BACK_MOTOR_PORT_2 4
#define INTAKE_MOTOR 12
#define SCORING_MOTOR -11


//sensors
#define PISTON_DIO 1
#define PISTON_LIMIT_SWITCH_DIO 2


void umbc::Robot::opcontrol() {

    pros::ADIDigitalOut piston (PISTON_DIO);
    pros::ADIDigitalIn  pistonLimitSwitch (PISTON_LIMIT_SWITCH_DIO);

    // nice names for controllers (do not edit)12121212
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize motors and groups
    std::vector<int8_t> leftFront, leftBack, rightFront, rightBack, scoring, intake;

    rightFront.assign({RIGHT_FRONT_MOTOR_PORT_1, RIGHT_FRONT_MOTOR_PORT_2});
    rightBack.assign({RIGHT_BACK_MOTOR_PORT_1, RIGHT_BACK_MOTOR_PORT_2});
    leftFront.assign({LEFT_FRONT_MOTOR_PORT_1, LEFT_FRONT_MOTOR_PORT_2});
    leftBack.assign({LEFT_BACK_MOTOR_PORT_1, LEFT_BACK_MOTOR_PORT_2});
    scoring.assign({SCORING_MOTOR});
    intake.assign({INTAKE_MOTOR});

    Motor_Group frontLeftGroup (leftFront);
    Motor_Group backLeftGroup  (leftBack);
    Motor_Group frontRightGroup(rightFront);
    Motor_Group backRightGroup (rightBack);
    Motor_Group scoringGroup   (scoring);
    Motor_Group intakeGroup    (intake);

    // assigns the relevant gearing to all of the motor groups
    int32_t gearMult = 1; 
    pros::motor_gearset_e gearColor;
    gearColor = pros::E_MOTOR_GEAR_GREEN;
    gearMult = MOTOR_GREEN_GEAR_MULTIPLIER;

    frontLeftGroup.set_gearing(gearColor);
    backLeftGroup.set_gearing(gearColor);
    frontRightGroup.set_gearing(gearColor);
    backRightGroup.set_gearing(gearColor);
    scoringGroup.set_gearing(gearColor);
    intakeGroup.set_gearing(gearColor);

    intakeGroup.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    
    //drivetrain variables
    double x, y, turn, lf, rf, lb, rb; // used in vector calculations

    //scoring variables
    int isScoring; //used to trigger scoring system

    //variables to change top speed
    double speedPercent;
    double speedlist[] = {0.25, 0.50, 0.75, 1};
    int speedIndex = 3;
    //locking mechanism varibales
    bool isLocked = false;
    //intake bool
    bool isIntaking = false;
    bool isOuttaking = false;
    

    while(1) {

        // drive train

            //sets the speed
            if((controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_UP))&&(!(speedIndex >= 3))){
                speedIndex += 1;
            }
            if((controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))&&(!(speedIndex <= 0))) {
                speedIndex -= 1;
            }
            

            //gets inputs from controller and assigns the values to x, y, and turn
            x = controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            y = -controller_master->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            turn = (-1 * controller_master->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
            
            //translates the x, y, and turn values into an integer value from -1 to 1
            x = -1*((x < 0) ? x * x * -1 : x * x);
            x /= JOYSTICK_MAX;
            y = -1*((y < 0) ? y * y * -1 : y * y);
            y /= JOYSTICK_MAX;
            turn = -1*((turn < 0) ? turn * turn * -1 : turn * turn);
            turn /= JOYSTICK_MAX;
            
            //calculates the required power to each motor as a value from -1 to 1
            lf = (y-x+turn);
            rf = (y-x-turn);
            lb = (y+x+turn);
            rb = (y+x-turn);
            
            //sends the velocity to the motor
            speedPercent = speedlist[speedIndex];
            frontRightGroup.move_velocity(rf*gearMult);
            backRightGroup.move_velocity(rb*gearMult);
            frontLeftGroup.move_velocity(lf*gearMult);
            backLeftGroup.move_velocity(lb*gearMult);

        //scoring
            if(controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
                if(isOuttaking){
                    isOuttaking = false;
                    intakeGroup.move_velocity(0);
                    scoringGroup.move_velocity(0);
                }
                else{
                    isOuttaking = true;
                    intakeGroup.move_velocity(1*MOTOR_BLUE_GEAR_MULTIPLIER);
                    scoringGroup.move_velocity(0.8*gearMult);
                    controller_master->rumble("------");
                }
            }
            if(controller_master->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
                if(isIntaking){
                    isIntaking = false;
                    intakeGroup.move_velocity(0);
                    scoringGroup.move_velocity(0);
                }
                else{
                    isIntaking = true;
                    intakeGroup.move_velocity(-1.1*gearMult);
                    scoringGroup.move_velocity(-1.2*gearMult);
                }
            }
        

        //locking
            
                if((pistonLimitSwitch.get_value() == 1) && !(controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_A))){
                    piston.set_value(true);
                    isLocked = true;
                    controller_master->set_text(0, 0, "");
                    controller_master->set_text(0, 0, "goal locked");
                }
                else if(isLocked && controller_master->get_digital(pros::E_CONTROLLER_DIGITAL_A)){
                    piston.set_value(false);
                    isLocked = false;
                    controller_master->set_text(0, 0, "");
                    controller_master->set_text(0, 0, "goal unlocked");
                }
                else if(!pistonLimitSwitch.get_value()){
                    piston.set_value(false);
                    isLocked = false;
                    controller_master->set_text(0, 0, "");
                    controller_master->set_text(0, 0, "goal unlocked");
                }
                
            
            


        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }   
}