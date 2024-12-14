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

#include "drivetrain_controller.h"

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

DrivetrainController DriveController(Drivetrain, DEFAULT_DRIVE_VELOCITY, DEFAULT_TURN_VELOCITY);


void autonomous(void) {
    wallStakeArm.spinToPosition(260, degrees);

    // Goal 1
    piston.set(false);
    DriveController.driveFor(-8);
    piston.set(true);

    intake.spin(forward);
    conveyor.spin(forward);
    DriveController.turnTo(-180);

    // Reset the heading and rotation
    // Drivetrain.setHeading(0, degrees);
    // Drivetrain.setRotation(0, degrees);

    DriveController.driveFor(30);
    wait(500, msec);
    DriveController.turnTo(-90);
    DriveController.driveFor(26);
    DriveController.turnTo(0);
    DriveController.driveFor(32, 0, 30);  // Drive slower to allow intaking
    wait(750, msec);
    // Go up to the wall to intake that one red
    DriveController.turnTo(-120);
    DriveController.driveFor(20, 1500);  // Drive with timeout 1500
    wait(750, msec);
    // Place in positive corner
    DriveController.turnTo(162);
    DriveController.driveFor(-30, 1500);
    piston.set(false);

    // Goal 2
    DriveController.turnTo(169, 1750);
    DriveController.driveFor(82);
    conveyor.stop();
    DriveController.turnTo(-72);
    intake.stop();
    DriveController.driveFor(-48);
    piston.set(true);  // Grabbed the next goal
    conveyor.spin(forward);
    intake.spin(forward);
    wait(500, msec);
    DriveController.turnTo(45);
    DriveController.driveFor(32);
    wait(500, msec);
    DriveController.turnTo(90);
    DriveController.driveFor(26);
    wait(500, msec);

    DriveController.turnTo(0);
    DriveController.driveFor(78);
    wait(1, seconds);

    // Goal 3
    DriveController.turnTo(90);
    DriveController.driveFor(-20);
    piston.set(true);

    // Todo: shove in corner

    // Touch stake
    DriveController.turnTo(-135);
    DriveController.driveFor(48);

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
