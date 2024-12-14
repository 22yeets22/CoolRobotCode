/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       lekan                                                     */
/*    Created:      11/2/2024, 3:02:29 PM                                     */
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

// Define all needed devices:
controller Controller = controller(primary);
motor leftMotorFront = motor(PORT1, ratio18_1, true);
motor leftMotorBack = motor(PORT2, ratio18_1, true);
motor_group leftDriveSmart = motor_group(leftMotorFront, leftMotorBack);
motor rightMotorFront = motor(PORT3, ratio18_1, false);
motor rightMotorBack = motor(PORT4, ratio18_1, false);
motor_group rightDriveSmart = motor_group(rightMotorFront, rightMotorBack);
motor wallStakeArm = motor(PORT12, ratio18_1, false);
motor sweeperArm = motor(PORT13, ratio18_1, false);

inertial Inertial = inertial(PORT16);

// left motor, right motor, wheelTravel (circumfrence of wheels), drivetrainInertial, trackWidth, wheelBase, units, gearRatio
smartdrive Drivetrain = smartdrive(leftDriveSmart, rightDriveSmart, Inertial, 259.34, 320, 230, mm, 2);

motor conveyor = motor(PORT6, ratio18_1, true);
motor intake = motor(PORT11, ratio18_1, true);

digital_out piston = digital_out(Brain.ThreeWirePort.A);

const int DEFAULT_DRIVE_VELOCITY = 85;
const int DEFAULT_TURN_VELOCITY = 60;

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

void autonomous(void) {
    wallStakeArm.spinToPosition(260, degrees);

    // Goal 1
    piston.set(false);
    driveFor(-8);
    piston.set(true);

    intake.spin(forward);
    conveyor.spin(forward);
    turnTo(-180);

    // Reset the heading and rotation
    // Drivetrain.setHeading(0, degrees);
    // Drivetrain.setRotation(0, degrees);

    driveFor(30);
    wait(500, msec);
    turnTo(-90);
    driveFor(26);
    turnTo(0);
    driveFor(32, 0, 30);  // Drive slower to allow intaking
    wait(750, msec);
    // Go up to the wall to intake that one red
    turnTo(-120);
    driveFor(20, 1500);  // Drive with timeout 1500
    wait(750, msec);
    // Place in positive corner
    turnTo(162);
    driveFor(-30, 1500);
    piston.set(false);

    // Goal 2
    turnTo(169, 1750);
    driveFor(82);
    conveyor.stop();
    turnTo(-72);
    intake.stop();
    driveFor(-48);
    piston.set(true);  // Grabbed the next goal
    conveyor.spin(forward);
    intake.spin(forward);
    wait(500, msec);
    turnTo(45);
    driveFor(32);
    wait(500, msec);
    turnTo(90);
    driveFor(26);
    wait(500, msec);

    turnTo(0);
    driveFor(78);
    wait(1, seconds);

    // Goal 3
    turnTo(90);
    driveFor(-20);
    piston.set(true);

    // Todo: shove in corner

    // Touch stake
    turnTo(-135);
    driveFor(48);

    return;
    Drivetrain.turnFor(135, degrees);
    Drivetrain.driveFor(-14, inches);
    piston.set(false);
    Drivetrain.driveFor(56, inches);
}

int main() {
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);

    // Initalize some motor stuff
    Drivetrain.setDriveVelocity(DEFAULT_DRIVE_VELOCITY, percent);
    Drivetrain.setTurnVelocity(DEFAULT_TURN_VELOCITY, percent);
    Drivetrain.setStopping(brake);
    conveyor.setVelocity(80, percent);
    conveyor.setMaxTorque(100, percent);
    conveyor.setStopping(coast);
    intake.setVelocity(100, percent);
    intake.setMaxTorque(100, percent);
    intake.setStopping(coast);
    sweeperArm.setVelocity(50, percent);
    sweeperArm.setMaxTorque(100, percent);
    sweeperArm.setStopping(hold);
    wallStakeArm.setVelocity(30, percent);
    wallStakeArm.setMaxTorque(100, percent);
    wallStakeArm.setStopping(hold);

    Inertial.calibrate(3);
    while (Inertial.isCalibrating()) {
        wait(100, msec);
    }

    // driveStraight(leftDriveSmart, rightDriveSmart, -24);

    // Prevent main from exiting with an infinite loop.
    while (true) {
        wait(100, msec);
    }
}
