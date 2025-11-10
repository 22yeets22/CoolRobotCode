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

// Define all needed devices:
controller Controller = controller(primary);
motor leftMotorFront = motor(PORT20, ratio18_1, true);
motor leftMotorBack = motor(PORT19, ratio18_1, true);
motor_group leftDriveSmart = motor_group(leftMotorFront, leftMotorBack);
motor rightMotorFront = motor(PORT11, ratio18_1, false);
motor rightMotorBack = motor(PORT1, ratio18_1, false);
motor_group rightDriveSmart = motor_group(rightMotorFront, rightMotorBack);
motor wallStakeMotorLeft = motor(PORT10, ratio18_1, false);
motor wallStakeMotorRight = motor(PORT4, ratio18_1, false);
motor_group wallStakeArm = motor_group(wallStakeMotorLeft, wallStakeMotorRight);

inertial Inertial = inertial(PORT9);

// left motor, right motor, wheelTravel (circumfrence of wheels), drivetrainInertial, trackWidth, wheelBase, units, gearRatio
smartdrive Drivetrain = smartdrive(leftDriveSmart, rightDriveSmart, Inertial, 239.3893602, 317.5, 228.6, mm, 2);

motor conveyor = motor(PORT17, ratio18_1, true);
motor intake = motor(PORT3, ratio18_1, true);

digital_out pistonClamp = digital_out(Brain.ThreeWirePort.A);
digital_out pistonSweeper = digital_out(Brain.ThreeWirePort.B);

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
    Drivetrain.setHeading(40, degrees);

    // Goal 1
    pistonClamp.set(false);
    driveFor(-14,2000);
    pistonClamp.set(true);

    intake.spin(forward);
    conveyor.spin(forward);
    turnTo(180);

    // Reset the heading and rotation
    // Drivetrain.setHeading(0, degrees);
    // Drivetrain.setRotation(0, degrees);

    driveFor(20,3000,30.0);
    wait(500, msec);
    turnTo(-85);
    driveFor(22,3000,30.0);
    turnTo(0);
    driveFor(32, 5000, 30.0);  // Drive slower to allow intaking
    wait(750, msec);
    // Go up to the wall to intake that one red
    turnTo(-130, 2500);
    driveFor(17, 1500);  // Drive with timeout 1500
    wait(750, msec);
    // Place in positive corner
    turnTo(162, 3000);
    driveFor(-30, 1500);
    pistonClamp.set(false);

    // Goal 2
    driveFor(2, 200);
    turnTo(169, 2000);
    driveFor(88);
    conveyor.stop();
    turnTo(-72, 2000);
    intake.stop();
    driveFor(-48, 5000);
    pistonClamp.set(true);  // Grabbed the next goal
    conveyor.spin(forward);
    intake.spin(forward);
    wait(500, msec);
    turnTo(0, 2000);
    driveFor(16, 3000, 30.0);
    wait(500, msec);
    turnTo(90, 2000);
    driveFor(24, 3000, 30.0);
    turnTo(10, 2000);
    driveFor(90, 7000, 60.0);
    wait(500, msec);
    driveFor(-5);
    turnTo(-90, 2000);
    driveFor(-20, 2000);
    pistonClamp.set(false);
    driveFor(20, 2000);
    return;

    // Goal 3
    turnTo(90, 2500);
    driveFor(-20, 2500);
    pistonClamp.set(true);

    // Todo: shove in corner

    // Touch stake
    turnTo(-135, 3500);
    driveFor(48, 4000);

    return;
    Drivetrain.turnFor(135, degrees);
    Drivetrain.driveFor(-14, inches);
    pistonClamp.set(false);
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
