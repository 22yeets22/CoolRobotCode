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

void resetDrive() {
    Drivetrain.setDriveVelocity(85, percent);
    Drivetrain.setTurnVelocity(60, percent);
    Drivetrain.setTimeout(0, seconds);
}

void autonomous(void) {
    // Goal 1
    piston.set(false);
    Drivetrain.driveFor(-8, inches);
    piston.set(true);

    intake.spin(forward);
    conveyor.spin(forward);
    Drivetrain.turnToHeading(-180, degrees);

    // Reset the heading and rotation
    // Drivetrain.setHeading(0, degrees);
    // Drivetrain.setRotation(0, degrees);

    Drivetrain.driveFor(30, inches);
    Drivetrain.turnToHeading(-90, degrees);
    Drivetrain.driveFor(26, inches);
    Drivetrain.turnToHeading(0, degrees);
    Drivetrain.setDriveVelocity(40, percent);  // slow
    Drivetrain.driveFor(32, inches);
    wait(500, msec);
    resetDrive();
    Drivetrain.turnToHeading(-120, degrees);
    Drivetrain.setTimeout(1500, msec);
    Drivetrain.driveFor(20, inches);
    Drivetrain.setTimeout(0, seconds);
    wait(750, msec);
    // Place in positive corner
    Drivetrain.turnToHeading(162, degrees);
    Drivetrain.setTimeout(1500, msec);
    Drivetrain.driveFor(-30, inches);
    piston.set(false);

    // Goal 2
    Drivetrain.turnToHeading(169, degrees);
    Drivetrain.setTimeout(0, seconds);
    Drivetrain.driveFor(82, inches);
    conveyor.stop();
    Drivetrain.turnToHeading(-72, degrees);
    intake.stop();
    Drivetrain.driveFor(-48, inches);
    piston.set(true);  // Grabbed the next goal
    conveyor.spin(forward);
    intake.spin(forward);
    wait(500, msec);
    Drivetrain.turnToHeading(45, degrees);
    Drivetrain.driveFor(32, inches);
    wait(500, msec);
    Drivetrain.turnToHeading(90, degrees);
    Drivetrain.driveFor(26, inches);
    wait(500, msec);
    
    Drivetrain.turnToHeading(0, degrees);
    Drivetrain.driveFor(78, inches);
    wait(1, seconds);

    // Goal 3
    Drivetrain.turnToHeading(90, degrees);
    Drivetrain.driveFor(-20, inches);

    piston.set(false);
    Drivetrain.turnFor(120, degrees);
    piston.set(true);

    // Todo: shove in corner

    Drivetrain.turnToHeading(-135, degrees);
    Drivetrain.driveFor(48, inches);

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
    Drivetrain.setDriveVelocity(85, percent);
    Drivetrain.setTurnVelocity(60, percent);
    Drivetrain.setStopping(brake);
    conveyor.setVelocity(100, percent);
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
