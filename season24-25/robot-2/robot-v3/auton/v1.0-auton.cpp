/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       rogerc                                                    */
/*    Created:      12/27/2024, 11:17:29 PM                                   */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vex.h"
using namespace vex;

brain Brain;
competition Competition;

// Define all needed devices: (Ports are  currently placeholders)
controller Controller = controller(primary);
motor leftMotor1 = motor(PORT20, ratio6_1, true);
motor leftMotor2 = motor(PORT9, ratio6_1, true);
motor leftMotor3 = motor(PORT10, ratio6_1, false);
motor_group leftDriveSmart = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor rightMotor1 = motor(PORT11, ratio6_1, false);
motor rightMotor2 = motor(PORT1, ratio6_1, false);
motor rightMotor3 = motor(PORT2, ratio6_1, true);
motor_group rightDriveSmart = motor_group(rightMotor1, rightMotor2, rightMotor3);

// Defining the inertial sensor
inertial Inertial = inertial(PORT4); 

// left motor, right motor, wheelTravel (circumfrence of wheels), drivetrainInertial, trackWidth, wheelBase, units, gearRatio
smartdrive Drivetrain = smartdrive(leftDriveSmart, rightDriveSmart, Inertial, 259.338473554, 323.85, 254.0, mm, 0.6666667);

// Defining the optical sensor
optical Optical = optical(PORT21);

// Defining other motors: converyor and intake
motor conveyor = motor(PORT18, ratio6_1, false);
motor intake = motor(PORT19, ratio18_1, true); 

motor ladyBrown = motor(PORT8, ratio18_1, false);

// Defining the pistons
digital_out pistonClamp = digital_out(Brain.ThreeWirePort.F);
digital_out pistonSweeper = digital_out(Brain.ThreeWirePort.H);

// Defining speed constants
const int DEFAULT_DRIVE_VELOCITY = 85;
const int DEFAULT_TURN_VELOCITY = 60;

double speedAdder = 0.0;

bool isRingDetected = false;

// Definitng drive and turn functions
void resetDrive() {
    Drivetrain.setDriveVelocity(DEFAULT_DRIVE_VELOCITY, percent);
    Drivetrain.setTurnVelocity(DEFAULT_TURN_VELOCITY, percent);
    Drivetrain.setTimeout(0, seconds);
}

void driveFor(double distance, int timeout = 0, double speed = DEFAULT_DRIVE_VELOCITY, bool waitUntilDone = true) {
    Drivetrain.setDriveVelocity(speed, percent);
    Drivetrain.setTimeout(timeout, msec);
    Drivetrain.driveFor(distance, inches, waitUntilDone);
    resetDrive();
}

void turnTo(double angle, int timeout = 0, double speed = DEFAULT_TURN_VELOCITY, bool waitUntilDone = true) {
    Drivetrain.setTurnVelocity(speed, percent);
    Drivetrain.setTimeout(timeout, msec);
    Drivetrain.turnToHeading(angle, degrees, waitUntilDone);
    resetDrive();
}

// void scoreWallStake() {
//     ladyBrown.spinToPosition(0, degrees);
//     ladyBrown.setPosition(0, degrees);

//     ladyBrown.spinToPosition(130, degrees);
//     conveyor.setVelocity(60, percent);
//     conveyor.spin(forward);
//     waitUntil(isRingDetected);
//     wait(500, msec);
//     conveyor.spin(reverse);
//     wait(200, msec);
//     conveyor.spin(forward);
//     wait(200, msec);
//     conveyor.spin(reverse);
//     wait(350, msec);
//     conveyor.stop();
//     wait(200, msec);

//     ladyBrown.spin(forward);
//     wait(750, msec);
//     driveFor(3, 250, 20.0);
//     driveFor(-2, 250, 20.0);
//     driveFor(-14, 2000, 20.0, false);
//     wait(100, msec);
//     ladyBrown.spinTo(400, degrees);
//     ladyBrown.stop();
// }

// Start of autonomous program
void autonomous(void) {
    Drivetrain.setHeading(0, degrees);
    ladyBrown.setPosition(0, degrees);

    conveyor.spin(forward); // Score preload on alliance stkae
    wait(500, msec);
    conveyor.stop();
    driveFor(14, 3000, 20.0 + speedAdder);
    turnTo(-90, 3000, 5.0);
    driveFor(-24, 3000, 20.0 + speedAdder);

    pistonClamp.set(true); // 1st mobile goal in possession
    turnTo(0, 3000, 5.0);
    intake.spin(forward);
    conveyor.spin(forward);
    driveFor(24, 3000, 30.0 + speedAdder);
    wait(300, msec);
    turnTo(35, 3000, 5.0);
    driveFor(30, 3000, 20.0 + speedAdder);
    turnTo(90, 3000, 5.0);
    driveFor(14.0, 3000, 10.0 + speedAdder);
    wait(500, msec);
    driveFor(-7, 3000, 20.0 + speedAdder);

    turnTo(180, 3000, 5.0);
    intake.spin(forward);
    conveyor.spin(forward);
    driveFor(36, 3000, 30.0 + speedAdder);
    driveFor(26, 5000, 7.5 + speedAdder);
    driveFor(-14, 3000, 20.0 + speedAdder);
    turnTo(90, 3000, 5.0);
    driveFor(10, 3000, 20.0 + speedAdder);
    turnTo(-30, 3000, 5.0);
    Drivetrain.drive(reverse);
    wait(750, msec);
    Drivetrain.stop();
    intake.stop();
    conveyor.stop();
    pistonClamp.set(false);
    wait(250, msec);
    driveFor(12, 1000, 20.0 + speedAdder);
    turnTo(90, 3000, 5.0);
    driveFor(13, 3000, 10.0 + speedAdder);
    wait(300, msec);
    Drivetrain.setHeading(90, degrees);
    driveFor(-76, 4000, 30.0 + speedAdder);
    driveFor(-12, 3000, 15.0 + speedAdder);

    pistonClamp.set(true); // 2nd mobile goal in possession
    Drivetrain.setHeading(90, degrees);
    turnTo(0, 3000, 5.0);
    intake.spin(forward);
    conveyor.spin(forward);
    driveFor(24, 3000, 30.0 + speedAdder);
    wait(300, msec);
    turnTo(-35, 3000, 5.0);
    driveFor(30, 3000, 20.0 + speedAdder);
    turnTo(-90, 3000, 5.0);

    driveFor(14.0, 3000, 10.0 + speedAdder);
    wait(500, msec);
    driveFor(-7, 3000, 20.0 + speedAdder);

    turnTo(180, 3000, 5.0);
    intake.spin(forward);
    conveyor.spin(forward);
    driveFor(36, 3000, 30.0 + speedAdder);
    driveFor(26, 5000, 7.5 + speedAdder);
    driveFor(-14, 3000, 20.0 + speedAdder);
    turnTo(-90, 3000, 5.0);
    driveFor(10, 3000, 20.0 + speedAdder);
    turnTo(30, 3000, 5.0);
    Drivetrain.drive(reverse);
    wait(750, msec);
    Drivetrain.stop();
    intake.stop();
    conveyor.stop();
    pistonClamp.set(false);
    wait(250, msec);
    driveFor(12, 1000, 20.0 + speedAdder);
    turnTo(-90, 3000, 5.0);
    driveFor(13, 3000, 10.0 + speedAdder);
    wait(200, msec);
    Drivetrain.setHeading(90, degrees);   
}

int main() {
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);

    // Initalizing
    Drivetrain.setDriveVelocity(DEFAULT_DRIVE_VELOCITY, percent);
    Drivetrain.setTurnVelocity(DEFAULT_TURN_VELOCITY, percent);
    Drivetrain.setStopping(brake);
    Drivetrain.setTurnThreshold(5);

    conveyor.setVelocity(100, percent);
    conveyor.setMaxTorque(100, percent);
    conveyor.setStopping(coast);

    intake.setVelocity(75, percent);
    intake.setMaxTorque(100, percent);
    intake.setStopping(coast);

    ladyBrown.setVelocity(100, percent);
    ladyBrown.setMaxTorque(100, percent);
    ladyBrown.setStopping(hold);
    ladyBrown.setPosition(0, degrees);

    Optical.setLight(vex::ledState::on);
    Optical.objectDetectThreshold(50);
    
    Inertial.calibrate(3);
    while (Inertial.isCalibrating()) {
        wait(100, msec);
    }

    // Prevent main from exiting with an infinite loop.
    while (true) {
        wait(100, msec);
    }
}
