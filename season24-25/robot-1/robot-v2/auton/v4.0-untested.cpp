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

const double defaultDriveVelocity = 75;  // 80
const double defaultTurnVelocity = 50;   // 60

bool conveyorOperating = true;
bool intakeOperating = true;

const bool stallLoopEnabled = true;

const double defaultConveyorVelocity = 80;
const double defaultIntakeVelocity = 80;

directionType conveyorDirection = directionType::undefined;
double conveyorVelocity = defaultConveyorVelocity;
directionType intakeDirection = directionType::undefined;
double intakeVelocity = defaultIntakeVelocity;

const unsigned short conveyorReverseTime = 1000;  // The time the conveyor will reverse for when stuck in msec
const unsigned short conveyorStuckVelocity = 5;   // The velocity below this will trigger the counter (in rpm)
const float conveyorStuckCurrent = 2;             // The amount of current that will trigger the counter (in amps)
const unsigned short conveyorStuckLimit = 15;     // The counter limit before reversing (in loops)

const unsigned short intakeReverseTime = 300;  // The time the conveyor will reverse for when stuck in msec
const unsigned short intakeStuckVelocity = 5;  // The velocity below this will trigger the counter (in rpm)
const float intakeStuckCurrent = 2;            // The amount of current that will trigger the counter (in amps)
const unsigned short intakeStuckLimit = 15;    // The counter limit before reversing (in loops)

void resetDrive() {
    Drivetrain.setDriveVelocity(defaultDriveVelocity, percent);
    Drivetrain.setTurnVelocity(defaultTurnVelocity, percent);
    Drivetrain.setTimeout(0, seconds);
}

void driveFor(double distance, int timeout = 0, double speed = defaultDriveVelocity, bool waitUntilDone = true) {
    printf("\033[30mDriving for %.2f inches\n", distance);
    Drivetrain.setDriveVelocity(speed, percent);
    Drivetrain.setTimeout(timeout, msec);
    Drivetrain.driveFor(distance, inches, waitUntilDone);
    resetDrive();
}

void turnTo(double angle, int timeout = 0, double speed = defaultTurnVelocity, bool waitUntilDone = true) {
    printf("\033[30mTurning to %.2f degrees\n", angle);
    Drivetrain.setTurnVelocity(speed, percent);
    Drivetrain.setTimeout(timeout, msec);
    Drivetrain.turnToHeading(angle, degrees, waitUntilDone);
    resetDrive();
}

void spinIntake(directionType direction) {
    while (!intakeOperating)
        wait(20, msec);
    intake.spin(forward);
    intakeDirection = direction;
    intakeVelocity = defaultIntakeVelocity;
}
void spinIntake(directionType direction, double speed) {
    while (!intakeOperating)
        wait(20, msec);
    intake.spin(forward, speed, percent);
    intakeDirection = direction;
    intakeVelocity = speed;
}
void stopIntake() {
    while (!intakeOperating)
        wait(20, msec);
    intake.stop();
    intakeDirection = directionType::undefined;
    intakeVelocity = 0;
}

void spinConveyor(directionType direction) {
    while (!conveyorOperating)
        wait(20, msec);
    conveyor.spin(forward);
    conveyorDirection = direction;
    conveyorVelocity = defaultConveyorVelocity;
}
void spinConveyor(directionType direction, double speed) {
    while (!conveyorOperating)
        wait(20, msec);
    conveyor.spin(forward, speed, percent);
    conveyorDirection = direction;
    conveyorVelocity = speed;
}
void stopConveyor() {
    while (!conveyorOperating)
        wait(20, msec);
    conveyor.stop();
    conveyorDirection = directionType::undefined;
    conveyorVelocity = 0;
}

void autonomous(void) {
    printf("\033[31m====Autonomous started====\n");
    wallStakeArm.spinFor(260, degrees, false);
    Drivetrain.setHeading(33, degrees);

    // Goal 1
    piston.set(false);
    driveFor(-14, 2000);
    piston.set(true);

    spinIntake(forward);
    spinConveyor(forward);
    turnTo(180);

    driveFor(26);
    wait(500, msec);
    turnTo(-90, 3500);
    driveFor(26);
    wait(500, msec);
    turnTo(0, 3000, 30);     // Turn slower for accuracy
    driveFor(36, 5000, 30);  // Drive slower to allow intaking
    wait(750, msec);
    printf("\033[31m====about to go to the wall====\n");
    // Go up to the wall to intake that one red
    turnTo(-125, 2500);
    driveFor(20, 1500);
    wait(750, msec);

    printf("\033[31m====Placing goal 1 in corner====\n");
    // Place in corner
    turnTo(162, 3000);
    driveFor(-30, 1500);
    piston.set(false);  // Drop goal in corner
    printf("\033[31m====Goal 1 placed====\n");

    // Goal 2
    driveFor(-2, 200);  // Back up to give clearance
    driveFor(6, 1000);  // Get a little further from the goal
    wait(100, msec);
    turnTo(169, 2500, 30);  // Angle to correct place to pick up that ring
    printf("\033[31m====So far away====\n");
    driveFor(84, 10000, 80);  // Drive to the ring (90)
    stopConveyor();           // Once picking it up, stop conveyor to not lose the ring
    printf("\033[31m====Turn to pick up goal====\n");
    turnTo(-68, 9000, 30);     // Turn to goal angle
    stopIntake();              // Ok, now we can stop the intake
    driveFor(-56, 5000, 100);  // Go to the next goal
    printf("\033[31m====Grabbing goal 🙏 pls work====\n");
    piston.set(true);  // Grabbed the next goal

    // Now we can start spinning
    spinIntake(forward);
    spinConveyor(forward);

    wait(250, msec);
    turnTo(30, 2000);      // Turn to face first ring
    driveFor(28, 3500);    // Drive to first ring
    wait(500, msec);       // Wait for intake
    turnTo(90, 2000, 35);  // Turn to next ring
    driveFor(24, 3500);    // Drive to next ring
    wait(500, msec);       // Wait for intake

    // Drive all the way down the field
    printf("\033[31m====Driving down the field====\n");
    turnTo(0, 4500, 25);
    driveFor(84, 9000, 50);
    wait(1500, msec);
    driveFor(-12, 1500);
    // Time to shove goal in corner
    printf("\033[31m====Shoving goal 2 in corner====\n");
    turnTo(-135, 5000);
    driveFor(-18, 3000);
    piston.set(false);
    wait(500, msec);

    stopConveyor();

    // Goal 3
    printf("\033[31m====Going for goal 3====\n");
    driveFor(4, 500, 100);
    turnTo(-140, 1000);
    spinIntake(forward);
    wait(250, msec);
    driveFor(42, 2000);
    wait(500, msec);
    turnTo(180, 2000);
    driveFor(-24, 2000);
    spinConveyor(forward);
    piston.set(true);  // With the new goal, we can start spinning again
    wait(2, seconds);
    piston.set(false);
    printf("\033[31m====Goal 3 placed====\n");
}

/**
 * Continuously checks the intake and conveyor's velocity and current, reverses them if stuck.
 *
 * @return 0 if stallLoop is false
 */
int stallLoop() {
    if (!stallLoopEnabled)
        return 0;

    unsigned short conveyorCounter = 0;  // Stall counter for conveyor
    unsigned short intakeCounter = 0;    // Stall counter for intake
    while (true) {
        if (conveyor.velocity(rpm) <= conveyorStuckVelocity && conveyor.current() >= conveyorStuckCurrent)
            conveyorCounter += 1;
        else
            conveyorCounter = 0;
        if (intake.velocity(rpm) <= intakeStuckVelocity && intake.current() >= intakeStuckCurrent)
            intakeCounter += 1;
        else
            intakeCounter = 0;

        if (conveyorCounter >= conveyorStuckLimit) {
            printf("\033[31mConveyor is stuck!\n");
            conveyorCounter = 0;

            printf("\033[31mDirection: %d, Velocity: %.2f\n", conveyorDirection, conveyorVelocity);

            conveyorOperating = false;
            conveyor.spinFor(reverse, conveyorReverseTime, msec);
            conveyor.spin(conveyorDirection, conveyorVelocity, percent);
            conveyorOperating = true;
        }
        if (intakeCounter >= intakeStuckLimit) {
            printf("\033[31mIntake is stuck!\n");
            intakeCounter = 0;

            printf("\033[31mDirection: %d, Velocity: %.2f\n", conveyorDirection, conveyorVelocity);

            intakeOperating = false;
            intake.spinFor(reverse, intakeReverseTime, msec);
            intake.spin(intakeDirection, intakeVelocity, percent);
            intakeOperating = true;
        }

        wait(20, msec);
    }
    return 0;
}

int main() {
    // Initalize some motor stuff
    Drivetrain.setDriveVelocity(defaultDriveVelocity, percent);
    Drivetrain.setTurnVelocity(defaultTurnVelocity, percent);
    Drivetrain.setStopping(brake);
    conveyor.setVelocity(defaultConveyorVelocity, percent);
    conveyor.setMaxTorque(100, percent);
    conveyor.setStopping(coast);
    intake.setVelocity(defaultIntakeVelocity, percent);
    intake.setMaxTorque(100, percent);
    intake.setStopping(coast);
    sweeperArm.setVelocity(50, percent);
    sweeperArm.setMaxTorque(100, percent);
    sweeperArm.setStopping(hold);
    wallStakeArm.setVelocity(30, percent);
    wallStakeArm.setMaxTorque(100, percent);
    wallStakeArm.setStopping(hold);

    task stallTask(stallLoop);

    Brain.Screen.printAt(0, 0, "Calibrating...");
    Inertial.calibrate();
    while (Inertial.isCalibrating()) {
        wait(100, msec);
    }
    Brain.Screen.clearScreen();

    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);

    // driveStraight(leftDriveSmart, rightDriveSmart, -24);

    // Prevent main from exiting with an infinite loop.
    while (true) {
        wait(1, seconds);
    }
}
