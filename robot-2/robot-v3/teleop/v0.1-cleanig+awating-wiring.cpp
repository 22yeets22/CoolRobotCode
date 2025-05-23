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
motor leftMotor1 = motor(PORT1, ratio6_1, false);
motor leftMotor2 = motor(PORT1, ratio6_1, true);
motor leftMotor3 = motor(PORT1, ratio6_1, false);
motor_group leftDriveSmart = motor_group(leftMotor1, leftMotor2, leftMotor3);
motor rightMotor1 = motor(PORT1, ratio6_1, true);
motor rightMotor2 = motor(PORT1, ratio6_1, false);
motor rightMotor3 = motor(PORT1, ratio6_1, true);
motor_group rightDriveSmart = motor_group(rightMotor1, rightMotor2, rightMotor3);

// Defining the inertial sensor
inertial Inertial = inertial(PORT1); 

// left motor, right motor, wheelTravel (circumfrence of wheels), drivetrainInertial, trackWidth, wheelBase, units, gearRatio
smartdrive Drivetrain = smartdrive(leftDriveSmart, rightDriveSmart, Inertial, 259.338473554, 0.0, 254.0, mm, 2/3);

// Defining the optical sensor
optical Optical = optical(PORT1);

// Defining other motors: converyor and intake
motor conveyor = motor(PORT1, ratio6_1, true);
motor intake = motor(PORT1, ratio18_1, true); 

motor wallStakeArm = motor(PORT1, ratio18_1, false);

// Defining the pistons
digital_out pistonClamp = digital_out(Brain.ThreeWirePort.A);
digital_out pistonSweeper = digital_out(Brain.ThreeWirePort.B);

// Variables that enable/disable sections of code
bool teleopCodeEnabled = true;
bool stuckHelperCodeEnabled = true;
bool infoCodeEnabled = true;
bool drivetrainSwitchEnabled = false;
bool autonomousCodeEnabled = true;
bool conveyorManual = true;

// Color Sort Stuff
double opticalColor;
int colorSortTime = 400;
bool colorSortEnabled = true;

// Variables for autonomous
unsigned short autonomousRoute = 1;  // 0 = testing, 1 = positive, 2 = negative (complex), 3 = do nothing
short allianceColor = 1;          // Let 1 be red and -1 be blue
bool autonWinPoint = true;


// Drivetrain variables
enum DrivetrainMode {
    TANK,
    ARCADE
};
DrivetrainMode drivetrainMode = TANK;  // Tank by default

// Control variables
bool conveyorActive = false;
bool intakeActive = false;
bool drivetrainLNeedsToBeStopped = true;
bool drivetrainRNeedsToBeStopped = true;
bool pistonClampState = false;
bool pistonSweeperState = false;


// Stuck helper variables
const unsigned short conveyorStuckLimit = 10;
const float conveyorStuckCurrent = 2;
const unsigned short conveyorStuckVelocity = 5;
const unsigned short intakeStuckLimit = 10;
const float intakeStuckCurrent = 2;
const unsigned short intakeStuckVelocity = 5;

// Defining speed constants
const int DEFAULT_DRIVE_VELOCITY = 85;
const int DEFAULT_TURN_VELOCITY = 70;

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

void turnFor(double angle, int timeout = 0, double speed = DEFAULT_TURN_VELOCITY, bool waitUntilDone = true) {
    Drivetrain.setTurnVelocity(speed, percent);
    Drivetrain.setTimeout(timeout, msec);
    Drivetrain.turnFor(angle, degrees, waitUntilDone);
    resetDrive();
}

void turnTo(double angle, int timeout = 0, double speed = DEFAULT_TURN_VELOCITY, bool waitUntilDone = true) {
    Drivetrain.setTurnVelocity(speed, percent);
    Drivetrain.setTimeout(timeout, msec);
    Drivetrain.turnToHeading(angle, degrees, waitUntilDone);
    resetDrive();
}


/**
 * Initializes the robot's systems before the competition starts.
 * This function is only called once after the V5 has been powered on and not every time that the robot is disabled.
 *
 * @return None
 */
void initalize(bool onlyMotors = false) {
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

    wallStakeArm.setVelocity(50, percent);
    wallStakeArm.setMaxTorque(100, percent);
    wallStakeArm.setStopping(hold);

    Optical.objectDetectThreshold(100);
    
    if (onlyMotors) return;

    Brain.Screen.setFont(mono12);

    Inertial.calibrate();
    while (Inertial.isCalibrating()) {
        wait(100, msec);
    }
}

/**
 * This function is used to control the robot during the autonomous phase of a VEX Competition.
 *
 * @return None
 */
void autonomous(void) {
    if (!autonomousCodeEnabled)
        return;

    // Autonomous
    if (autonomousRoute == 0) {
        // Route 0: Testing purposes only
        Drivetrain.setHeading(-90 * allianceColor, degrees);
        wait(15, sec);
    } else if (autonomousRoute == 1) {
        // Route 1: The positive/simple side (WIP)

    } else if (autonomousRoute == 2) {
        // Route 2: The negative/coomplex side (WIP)
        
    } else if (autonomousRoute == 3)  {
        // Does nothing
        wait(15, seconds);
    } else {
        // Default case if not in range
        Brain.Screen.print("No autonomous route found");
    }

    intake.stop();
    conveyor.stop();
}

/**
 * Handles the control of the drivetrain based on the controller's input.
 *
 * If the drivetrain is in tank mode, the left side of the drivetrain will be controlled by the left joystick's Y-axis,
 * and the right side of the drivetrain will be controlled by the right joystick's Y-axis.
 * If the drivetrain is in arcade mode, the left joystick's Y-axis will control the speed of both sides of the drivetrain,
 * and the left joystick's X-axis will control the difference between the left and right side.
 *
 * Otherwise, the drivetrain will stop if it is not already stopped.
 */
void controlDrivetrain() {
    // Function to handle drivetrain control type
    int drivetrainLeftSideSpeed = 0, drivetrainRightSideSpeed = 0;

    if (drivetrainMode == TANK) {
        drivetrainLeftSideSpeed = Controller.Axis3.position();
        drivetrainRightSideSpeed = Controller.Axis2.position();
    } else if (drivetrainMode == ARCADE) {
        drivetrainLeftSideSpeed = Controller.Axis3.position() + Controller.Axis1.position();
        drivetrainRightSideSpeed = Controller.Axis3.position() - Controller.Axis1.position();
    }

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

/**
 * Toggles the piston state each time the A button is pressed.
 *
 * @note Uses a static variable to track the previous state of the button.
 */
void controlPistonClamp() {
    static bool lastButtonAState = false;  // Track the previous state of the button
    bool currentButtonAState = Controller.ButtonA.pressing();

    if (currentButtonAState && !lastButtonAState) {  // Detect button press
        pistonClampState = !pistonClampState;                // Toggle the piston state
        pistonClamp.set(pistonClampState);
    }

    lastButtonAState = currentButtonAState;  // Update the previous state
}

/**
 * Toggles the piston state each time the B button is pressed.
 *
 * @note Uses a static variable to track the previous state of the button.
 */
void controlPistonSweeper() {
    static bool lastButtonBState = false;  // Track the previous state of the button
    bool currentButtonBState = Controller.ButtonB.pressing();

    if (currentButtonBState && !lastButtonBState) {  // Detect button press
        pistonSweeperState = !pistonSweeperState;                // Toggle the piston state
        pistonSweeper.set(pistonSweeperState);
    }

    lastButtonBState = currentButtonBState;  // Update the previous state
}


/**
 * Handles the control of the intake and conveyor motors based on the controller's input.
 *
 * If R1 is pressed, the intake and conveyor motors spin forward.
 * If R2 is pressed, the intake and conveyor motors spin reverse.
 *
 * Otherwise, the motors will stop if they are not already stopped.
 */

void controlIntakeAndConveyor() {
    if (conveyorManual) {
        // Handle intake and conveyor control
        if (Controller.ButtonR1.pressing()) {
            conveyor.spin(forward);
            intake.spin(forward);
            conveyorActive = true;
            intakeActive = true;
        } else if (Controller.ButtonR2.pressing()) {
            conveyor.spin(reverse);
            intake.spin(reverse);
            conveyorActive = true;
            intakeActive = true;
        } else {
            intakeActive = false;
            conveyorActive = false;
        }

        // Stop conveyor and intake if no active command
        if (!conveyorActive)
            conveyor.stop();
        if (!intakeActive)
            intake.stop();
    } else {
        wait(20, msec);
    }
}

/**
 * Handles the control of the wall stake arm based on the controller's input.
 *
 * If the L1 button is pressed, the wall stake arm spins forward.
 * If the L2 button is pressed, the wall stake arm spins reverse.
 *
 * Otherwise, the wall stake arm will stop if it is not already stopped.
 */


void controlWallStakeArm() {
    if (Controller.ButtonL1.pressing())
        wallStakeArm.spin(forward);
    else if (Controller.ButtonL2.pressing())
        wallStakeArm.spin(reverse);
    else
        wallStakeArm.stop();
}

/**
 * Sorts the rigns based on their color using the optical sensor.
 *
 * If the color of the detected ring is different from the alliance color, the conveyor will spin for a certain amount of time,
 * trying to launch it.
 *
 * Otherwise, the conveyor will continue manual control.
 */
void colorSort() {
    if (colorSortEnabled) {
        Optical.setLight(ledState::on);
 
        if (Optical.color() == color::red && Optical.isNearObject()) {
            opticalColor = 1;
        } else if (Optical.color() == color::blue && Optical.isNearObject()) {
            opticalColor = -1;
        } else {
            opticalColor = 0;
        }

        if (opticalColor == 1 && allianceColor != 1) {
            conveyorManual = false;
            conveyor.spinFor(forward, colorSortTime, msec);
            conveyor.stop();
            wait(500, msec);
            conveyorManual = true;
        } else if (opticalColor == -1 && allianceColor != -1) {
            conveyorManual = false;
            conveyor.spinFor(forward, colorSortTime, msec);
            conveyor.stop();
            wait(500, msec);
            conveyorManual = true;
        } else {
            wait(20, msec);
        }
    } else {
        wait(20, msec);
    }
}

/**
 * Toggles color sort each time the left arrow button is pressed.
 *
 * @note Uses a static variable to track the previous state of the button.
 */

void enableDisableColorSort() {
    static bool lastButtonLeftArrowState = true;  // Track the previous state of the button
    bool currentButtonLeftArrowState = Controller.ButtonLeft.pressing();

    if (currentButtonLeftArrowState && !lastButtonLeftArrowState) {  // Detect button press
        colorSortEnabled = !colorSortEnabled;                
        Controller.rumble(".");
    }

    lastButtonLeftArrowState = currentButtonLeftArrowState;  // Update the previous state
}



/**
 * Continuously handles controller input and updates the robot's systems accordingly.
 *
 * @return None
 */
int teleopLoop() {
    if (!teleopCodeEnabled)
        return 0;

    // Handle controller input
    while (true) {
        controlDrivetrain();
        controlIntakeAndConveyor();
        controlPistonClamp();
        controlPistonSweeper();
        controlWallStakeArm();
        colorSort();
        enableDisableColorSort();
        wait(20, msec);
    }
}

/**
 * Continuously checks the conveyor's velocity and current,
 * and triggers the controller to rumble if the conveyor is stuck.
 *
 * @return 0 if stuckHelperCodeEnabled is false
 */
int stuckHelperLoop() {
    if (!stuckHelperCodeEnabled)
        return 0;

    unsigned short conveyorCounter = 0;
    unsigned short intakeCounter = 0;
    while (true) {
        if (conveyor.velocity(rpm) < conveyorStuckVelocity && conveyor.current() >= conveyorStuckCurrent)
            conveyorCounter += 1;
        else
            conveyorCounter = 0;
        if (intake.velocity(rpm) < intakeStuckVelocity && intake.current() >= intakeStuckCurrent)
            intakeCounter += 1;
        else
            intakeCounter = 0;

        if (conveyorCounter >= conveyorStuckLimit) {
            // conveyor.stop();
            Controller.rumble(".");
            conveyorCounter = 0;
        }
        if (intakeCounter >= intakeStuckLimit) {
            // intake.stop();
            Controller.rumble(".");
            intakeCounter = 0;
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
    if (!infoCodeEnabled)
        return 0;

    // Prints info on brain & controller
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);

        Brain.Screen.print("Yaw:");
        Brain.Screen.print(Inertial.yaw());
        Brain.Screen.newLine();
        Brain.Screen.print("Optical Color:");
        if (opticalColor == 1) {
            Brain.Screen.print("Red");
        } else if (opticalColor == -1) {
            Brain.Screen.print("Blue");
        } else {
            Brain.Screen.print("Not blue nor red");
        }
        Brain.Screen.newLine();

        Brain.Screen.render();

        wait(200, msec);  // Give some delay
    }
    return 0;
}

/**
 * Handles all user control of the robot during the driver control phase of a VEX Competition.
 *
 * @return None
 */
void driverControl(void) {
    // Setup for motors (in case autonomous does not finish in time)
    Drivetrain.setTimeout(0, seconds);
    Drivetrain.setStopping(coast);
    initalize(true);  // Only reset motors

    // Controller.rumble(". . . -");

    task teleopTask(teleopLoop);
    task stuckHelperTask(stuckHelperLoop);
    task infoTask(infoLoop);
}

int main() {
    // Run the pre-autonomous function.
    initalize();

    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(driverControl);

    // Prevent main from exiting with an infinite loop
    while (true) {
        wait(1000, msec);
    }
}
