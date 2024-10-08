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
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// A global instance of competition
competition Competition;

// Define all needed devices:
controller Controller = controller(primary);
motor leftMotorA = motor(PORT10, ratio18_1, true);
motor leftMotorB = motor(PORT11, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT1, ratio18_1, false);
motor rightMotorB = motor(PORT20, ratio18_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor Conveyor = motor(PORT14, ratio18_1, true);
motor Intake = motor(PORT2, ratio18_1, false);
digital_out Piston = digital_out(Brain.ThreeWirePort.A);

// Define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
bool StuckHelperCodeEnabled = true;
// Define variables used for controlling motors based on controller inputs
bool ControllerLeftShoulderControlMotorsStopped = true;
bool ControllerRightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller = true;
bool DrivetrainRNeedsToBeStopped_Controller = true;
// Piston
bool PistonState = false;

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
    Drivetrain.setDriveVelocity(100, percent);
    Drivetrain.setTurnVelocity(100, percent);
    Drivetrain.setStopping(coast);
    Conveyor.setVelocity(100, percent);
    Conveyor.setMaxTorque(100, percent);
    Conveyor.setStopping(coast);
    Intake.setVelocity(100, percent);
    Intake.setMaxTorque(100, percent);
    Intake.setStopping(coast);

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
    int drivetrainLeftSideSpeed = Controller.Axis3.position();
    int drivetrainRightSideSpeed = Controller.Axis2.position();

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

    handleMotorControl(LeftDriveSmart, drivetrainLeftSideSpeed, DrivetrainLNeedsToBeStopped_Controller);
    handleMotorControl(RightDriveSmart, drivetrainRightSideSpeed, DrivetrainRNeedsToBeStopped_Controller);
}

void controlPiston() {
    static bool lastButtonState = false;  // Track the previous state of the button
    bool currentButtonState = Controller.ButtonA.pressing();

    if (currentButtonState && !lastButtonState) {  // Detect button press
        PistonState = !PistonState;  // Toggle the piston state
        Piston.set(PistonState);
    }

    lastButtonState = currentButtonState;  // Update the previous state
}

void controlConveyor() {
    // Function to handle conveyor control
    if (Controller.ButtonL1.pressing()) {
        Conveyor.spin(forward);
        ControllerLeftShoulderControlMotorsStopped = false;
    } else if (Controller.ButtonL2.pressing()) {
        Conveyor.spin(reverse);
        ControllerLeftShoulderControlMotorsStopped = false;
    } else if (!ControllerLeftShoulderControlMotorsStopped) {
        Conveyor.stop();
        ControllerLeftShoulderControlMotorsStopped = true;
    }
}

void controlIntake() {
    // Function to handle intake control
    if (Controller.ButtonR1.pressing()) {
        Intake.spin(forward);
        ControllerRightShoulderControlMotorsStopped = false;
    } else if (Controller.ButtonR2.pressing()) {
        Intake.spin(reverse);
        ControllerRightShoulderControlMotorsStopped = false;
    } else if (!ControllerRightShoulderControlMotorsStopped) {
        Intake.stop();
        ControllerRightShoulderControlMotorsStopped = true;
    }
}

int stuckHelperLoop() {
    if (!StuckHelperCodeEnabled) {
        return;
    }

    while (true) {
        // if too high current and too low velocity, stop
        wait(20, msec);
    }
    return 0;
}

int controllerLoop() {
    // Handle controller input
    while (true) {
        if (RemoteControlCodeEnabled) {
            controlDrivetrain();
            controlConveyor();
            controlIntake();
            controlPiston();
        }
        wait(20, msec);
    }
}

int infoLoop() {
    // Prints info on brain & controller
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);

        double infoToPrint[] = {
            static_cast<double>(Brain.Battery.capacity()),
            static_cast<double>(Brain.Battery.current()),
            static_cast<double>(Brain.Battery.temperature()),
            static_cast<double>(Drivetrain.efficiency()),
            static_cast<double>(Drivetrain.torque())
        };

        // Iterate over the array and print each value
        for (double info: infoToPrint) {
            Brain.Screen.print(info);
            Brain.Screen.newLine();
        }

        wait(200, msec); // Give some delay
    }
    return 0;
}


void userControl(void) {
    Controller.rumble(". . . -");

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
