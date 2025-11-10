/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       lekan                                                     */
/*    Created:      9/14/2024, 2:50:26 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

using namespace vex;
using vexController = drivetrain;
using vexDrivetrain = drivetrain;
// using signature = vision::signature;
// using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// A global instance of competition
competition Competition;

// Define all needed devices:
vexController controller = vexController(primary);
motor leftMotorA = motor(PORT1, ratio18_1, true);
motor leftMotorB = motor(PORT2, ratio18_1, true);
motor_group leftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT3, ratio18_1, false);
motor rightMotorB = motor(PORT4, ratio18_1, false);
motor_group rightDriveSmart = motor_group(rightMotorA, rightMotorB);
vexDrivetrain drivetrain = vexDrivetrain(leftDriveSmart, rightDriveSmart, 319.19, 295, 40, mm, 1);

motor conveyorA = motor(PORT5, ratio18_1, true);
motor conveyorB = motor(PORT6, ratio18_1, false);
motor_group conveyor = motor_group(conveyorA, conveyorB);

digital_out piston = digital_out(Brain.ThreeWirePort.A);

// Define variables that enable/disable sections of code
bool remoteControlCodeEnabled = true;
bool stuckHelperCodeEnabled = false;
bool infoCodeEnabled = false;

// Define variables used for controlling motors
bool conveyorMotorsStopped = true;
// bool intakeMotorsStopped = true;
bool drivetrainLNeedsToBeStopped = true;
bool drivetrainRNeedsToBeStopped = true;

// Robot components
bool pistonState = false;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void initalize(void) {
    // All activities that occur before the competition starts.
    drivetrain.setDriveVelocity(100, percent);
    drivetrain.setTurnVelocity(100, percent);
    drivetrain.setStopping(coast);
    conveyor.setVelocity(100, percent);
    conveyor.setMaxTorque(100, percent);
    conveyor.setStopping(coast);

    Brain.Screen.setFont(mono30);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
    // All the autonomous code
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void controlDrivetrain() {
    // Function to handle drivetrain motor control
    int drivetrainLeftSideSpeed = controller.Axis3.position();
    int drivetrainRightSideSpeed = controller.Axis2.position();

    auto handleMotorControl = [](motor_group & motorGroup, int speed, bool & motorNeedsToBeStopped) {
        if (abs(speed) < 5) {
            if (motorNeedsToBeStopped) {
                motorGroup.stop();
                motorNeedsToBeStopped = false;
            }
        } else {
            motorNeedsToBeStopped = true;
            motorGroup.setVelocity(speed, percent);
            motorGroup.spin(forward);
        }
    };

    handleMotorControl(leftDriveSmart, drivetrainLeftSideSpeed, drivetrainLNeedsToBeStopped);
    handleMotorControl(rightDriveSmart, drivetrainRightSideSpeed, drivetrainRNeedsToBeStopped);
}

void controlPiston() {
    static bool lastButtonState = false;  // Track the previous state of the button
    bool currentButtonState = controller.ButtonA.pressing();

    if (currentButtonState && !lastButtonState) {  // Detect button press
        pistonState = !pistonState;  // Toggle the piston state
        piston.set(pistonState);
    }

    lastButtonState = currentButtonState;  // Update the previous state
}

void controlConveyor() {
    // Function to handle conveyor control
    if (controller.ButtonL1.pressing()) {
        conveyor.spin(forward);
        conveyorMotorsStopped = false;
    } else if (controller.ButtonL2.pressing()) {
        conveyor.spin(reverse);
        conveyorMotorsStopped = false;
    } else if (!conveyorMotorsStopped) {
        conveyor.stop();
        conveyorMotorsStopped = true;
    }
}

/*void controlIntake() {
    // Function to handle intake control
    if (controller.ButtonR1.pressing()) {
        Intake.spin(forward);
        intakeMotorsStopped = false;
    } else if (controller.ButtonR2.pressing()) {
        Intake.spin(reverse);
        intakeMotorsStopped = false;
    } else if (!intakeMotorsStopped) {
        Intake.stop();
        intakeMotorsStopped = true;
    }
}*/

void stuckHelperLoop() {
    if (!stuckHelperCodeEnabled) {
        return;
    }

    while (true) {
        // if too high current and too low velocity, stop
        wait(20, msec);
    }
}

void controllerLoop() {
    // Handle controller input
    while (true) {
        if (remoteControlCodeEnabled) {
            controlDrivetrain();
            controlConveyor();
            controlPiston();
        }
        wait(20, msec);
    }
}

void infoLoop() {
    if (!infoCodeEnabled) {
        return;
    }

    // Prints info on brain & controller
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);

        double infoToPrint[] = {
            static_cast<double>(Brain.Battery.capacity()),
            static_cast<double>(Brain.Battery.current()),
            static_cast<double>(Brain.Battery.temperature()),
            static_cast<double>(drivetrain.efficiency()),
            static_cast<double>(drivetrain.torque())
        };

        // Iterate over the array and print each value
        for (double info: infoToPrint) {
            Brain.Screen.print(info);
            Brain.Screen.newLine();
        }

        wait(200, msec); // Give some delay
    }
}


void userControl(void) {
    controller.rumble(". . . -");

    task controllerTask(controllerLoop);
    task stuckHelperTask(stuckHelperLoop);
    task infoTask(infoLoop);
}

// Set up callbacks and stuff
int main() {
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(userControl);

    // Run the pre-autonomous function.
    initalize();

    // Prevent main from exiting with an infinite loop.
    while (true) {
        wait(1000, msec);
    }
}
