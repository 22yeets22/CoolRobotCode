/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       lekan                                                     */
/*    Created:      9/14/2024, 2:50:26 PM                                     */
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
// using signature = vision::signature;
// using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// A global instance of competition
competition Competition;

// Define all needed devices:
controller Controller = controller(primary);
motor leftMotorA = motor(PORT1, ratio18_1, true);
motor leftMotorB = motor(PORT2, ratio18_1, true);
motor_group leftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT9, ratio18_1, false);
motor rightMotorB = motor(PORT10, ratio18_1, false);
motor_group rightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(leftDriveSmart, rightDriveSmart, 319.19, 295, 40, mm, 1);

motor conveyor = motor(PORT3, ratio18_1, true);
motor intake = motor(PORT11, ratio18_1, true);

digital_out piston = digital_out(Brain.ThreeWirePort.A);

// Define variables that enable/disable sections of code
bool remoteControlCodeEnabled = true;
bool stuckHelperCodeEnabled = false;
bool infoCodeEnabled = false;

// Define variables used for controlling motors
bool conveyorMotorsStopped = true;
bool intakeMotorsStopped = true;
bool drivetrainLNeedsToBeStopped = true;
bool drivetrainRNeedsToBeStopped = true;

// Robot components
bool pistonState = false;

/**
 * Initializes the robot's systems before the competition starts.
 * This function is only called once after the V5 has been powered on and not every time that the robot is disabled.
 *
 * @return None
 */
void initalize(void) {
    // All activities that occur before the competition starts.
    Drivetrain.setDriveVelocity(100, percent);
    Drivetrain.setTurnVelocity(100, percent);
    Drivetrain.setStopping(coast);
    conveyor.setVelocity(100, percent);
    conveyor.setMaxTorque(100, percent);
    conveyor.setStopping(coast);
    intake.setVelocity(100, percent);
    intake.setMaxTorque(100, percent);
    intake.setStopping(coast);

    Brain.Screen.setFont(mono30);
}

/**
 * This function is used to control the robot during the autonomous phase of a VEX Competition.
 *
 * @return None
 */
void autonomous(void) {
    // All activities that occur during the autonomous phase
}

void controlDrivetrain() {
    // Function to handle drivetrain motor control
    int drivetrainLeftSideSpeed = Controller.Axis3.position();
    int drivetrainRightSideSpeed = Controller.Axis2.position();

    auto handleMotorControl = [](motor_group& motorGroup, int speed, bool& motorNeedsToBeStopped) {
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
    bool currentButtonState = Controller.ButtonA.pressing();

    if (currentButtonState && !lastButtonState) {  // Detect button press
        pistonState = !pistonState;  // Toggle the piston state
        piston.set(pistonState);
    }

    lastButtonState = currentButtonState;  // Update the previous state
}

void controlConveyor() {
    // Function to handle conveyor control
    if (Controller.ButtonL1.pressing()) {
        conveyor.spin(forward);
        conveyorMotorsStopped = false;
    } else if (Controller.ButtonL2.pressing()) {
        conveyor.spin(reverse);
        conveyorMotorsStopped = false;
    } else if (!conveyorMotorsStopped) {
        conveyor.stop();
        conveyorMotorsStopped = true;
    }
}

void controlIntake() {
    // Function to handle intake control
    if (Controller.ButtonR1.pressing()) {
        intake.spin(forward);
        intakeMotorsStopped = false;
    } else if (Controller.ButtonR2.pressing()) {
        intake.spin(reverse);
        intakeMotorsStopped = false;
    } else if (!intakeMotorsStopped) {
        intake.stop();
        intakeMotorsStopped = true;
    }
}

/**
 * Continuously checks the conveyor's velocity and current, 
 * and triggers the controller to rumble if the conveyor is stuck.
 *
 * @return 0 if stuckHelperCodeEnabled is false
 */
int stuckHelperLoop() {
    bool alreadyRumbled = false;

    if (!stuckHelperCodeEnabled) {
        return 0;
    }

    while (true) {
        if (conveyor.velocity(rpm) < 20 && conveyor.current() > 5 && !alreadyRumbled) {
            Controller.rumble(".");
            alreadyRumbled = true;
        } else {
            alreadyRumbled = false;
        }

        // if too high current and too low velocity, stop
        wait(20, msec);
    }
    return 0;
}

/**
 * Continuously handles controller input and updates the robot's systems accordingly.
 *
 * @return None
 */
int controllerLoop() {
    // Handle controller input
    while (true) {
        if (remoteControlCodeEnabled) {
            controlDrivetrain();
            controlConveyor();
            controlIntake();
            controlPiston();
        }
        wait(20, msec);
    }
    return 0;
}

/**
 * Continuously prints information about the robot's brain and drivetrain to the screen.
 *
 * @return 0 if infoCodeEnabled is false
 */
int infoLoop() {
    if (!infoCodeEnabled) {
        return 0;
    }

    // Prints info on brain & controller
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);

        double infoToPrint[] = {
            static_cast<double>(Brain.Battery.capacity()),
            static_cast<double>(Brain.Battery.current()),
            static_cast<double>(Brain.Battery.temperature()),
            static_cast<double>(Drivetrain.efficiency()),
            static_cast<double>(Drivetrain.torque())};

        // Iterate over the array and print each value
        for (double info : infoToPrint) {
            Brain.Screen.print(info);
            Brain.Screen.newLine();
        }

        wait(200, msec);  // Give some delay
    }
    return 0;
}

/**
 * Handles all user control of the robot during the driver control phase of a VEX Competition.
 *
 * @return None
 */
void userControl(void) {
    Controller.rumble(". . . -");

    task controllerTask(controllerLoop);
    task stuckHelperTask(stuckHelperLoop);
    task infoTask(infoLoop);
}

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
