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
// #include <unordered_map.h>
// #include <vector.tcc>
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
// left motor, right motor, wheelTravel (circumfrence of wheels), trackWith, wheelBase, units, gearRatio
drivetrain Drivetrain = drivetrain(leftDriveSmart, rightDriveSmart, 259.34, 320, 230, mm, 2);

motor conveyor = motor(PORT6, ratio18_1, true);
motor intake = motor(PORT11, ratio18_1, true);

digital_out piston = digital_out(Brain.ThreeWirePort.A);

inertial Inertial = inertial(PORT16);

// Variables that enable/disable sections of code
bool teleopCodeEnabled = true;
bool stuckHelperCodeEnabled = false;
bool infoCodeEnabled = false;
bool autonomousCodeEnabled = true;

// Variables for autonomous
unsigned short autonomousRoute = 2;  // 0 = testing, 1 = simple, 2 = complex, 3 = do nothing
short autonomousColor = -1;          // Let 1 be red and -1 be blue

// Variables used for controlling motors and components
unsigned short drivetrainMode = 0;   // 0 = tank, 1 = arcade
unsigned short wallStakeArmPos = 0;  // 0 = resting state, 1 =  enabled state
unsigned short previousWallStakeArmPos;
bool conveyorActive = false;
bool intakeActive = false;
bool drivetrainLNeedsToBeStopped = true;
bool drivetrainRNeedsToBeStopped = true;
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
    wallStakeArm.setVelocity(50, percent);
    wallStakeArm.setMaxTorque(100, percent);
    wallStakeArm.setStopping(brake);
    sweeperArm.setVelocity(25, percent);
    sweeperArm.setMaxTorque(100, percent);
    sweeperArm.setStopping(brake);

    Inertial.calibrate();
    while (Inertial.isCalibrating()) {
        wait(100, msec);
    }


    Brain.Screen.setFont(mono30);
}

/**
 * This function is used to control the robot during the autonomous phase of a VEX Competition.
 *
 * @return None
 */
void autonomous(void) {
    if (!autonomousCodeEnabled)
        return;

    // All activities that occur during the autonomous phase

    // Setup motors
    Drivetrain.setStopping(brake);
    Drivetrain.setDriveVelocity(80, percent);
    Drivetrain.setTurnVelocity(40, percent);

    // Create a map with route numbers as keys and commands as values (vector of strings)
    // std::unordered_map<int, std::vector<std::string>> robotCommands;

    // Adding commands to the map for different routes
    // robotCommands[1] = { "Drivetrain.driveFor", "10" }; // command and a distance
    // robotCommands[2] = { "Drivetrain.turnFor", "90" };
    // robotCommands[3] = { "Drivetrain.turnFor", "3 seconds" };

    // Autonomous
    if (autonomousRoute == 0) {
        // Route 0: Testing purposes only
        Drivetrain.driveFor(24, inches);
    } else if (autonomousRoute == 1) {
        // Route 1: The right side (simple route)
        piston.set(false);
        Drivetrain.driveFor(-30, inches);
        piston.set(true);
        wait(250, msec);
        Drivetrain.setTimeout(1500, msec);
        Drivetrain.turnFor(-68 * autonomousColor, degrees);
        wait(100, msec);
        intake.spin(forward);
        conveyor.spin(forward);
        Drivetrain.setTimeout(1500, msec);
        Drivetrain.driveFor(26, inches);
        wait(300, msec);
        Drivetrain.turnFor(-118 * autonomousColor, degrees);
        wait(100, msec);
        intake.stop();
        Drivetrain.setTimeout(2800, msec);
        Drivetrain.driveFor(42, inches);
    } else if (autonomousRoute == 2) {
        // Route 1: The left side (complex route)
        piston.set(false);
        Drivetrain.driveFor(-30, inches);
        piston.set(true);
        wait(250, msec);
        Drivetrain.turnFor(68 * autonomousColor, degrees);
        wait(100, msec);
        intake.spin(forward);
        conveyor.spin(forward);
        Drivetrain.driveFor(26, inches);
        wait(250, msec);
        Drivetrain.turnFor(48 * autonomousColor, degrees);
        wait(100, msec);
        Drivetrain.driveFor(12, inches);
        wait(800, msec);
        Drivetrain.driveFor(-8, inches);
        wait(100, msec);
        Drivetrain.turnFor(18 * autonomousColor, degrees);
        wait(100, msec);
        Drivetrain.driveFor(10, inches);
        wait(1000, msec);
        Drivetrain.setTimeout(1, seconds);
        Drivetrain.turnFor(-45 * autonomousColor, degrees);
        wait(100, msec);
        Drivetrain.setTimeout(2500, msec);
        Drivetrain.driveFor(-45, inches);
        Drivetrain.turnFor(75 * autonomousColor, degrees);
        Drivetrain.setTimeout(1, seconds);
        Drivetrain.driveFor(14, inches);
    } else if (autonomousRoute == 3) {
        wait(15, seconds);
    } else {
        // Default case if not in range
        Brain.Screen.print("No autonomous route found");
    }

    intake.stop();
    conveyor.stop();
}

void controlDrivetrain() {
    // Function to handle drivetrain control type
    static bool lastButtonStateDrive = true;  // Track the previous state of the button
    bool currentButtonStateDrive = Controller.ButtonY.pressing();
    int drivetrainLeftSideSpeed = 0, drivetrainRightSideSpeed = 0;
    if (currentButtonStateDrive && !lastButtonStateDrive)  // Detect button press
        drivetrainMode = (drivetrainMode + 1) % 2;
    if (drivetrainMode == 0) {
        drivetrainLeftSideSpeed = Controller.Axis3.position();
        drivetrainRightSideSpeed = Controller.Axis2.position();
    } else if (drivetrainMode == 1) {
        drivetrainLeftSideSpeed = Controller.Axis3.position() + Controller.Axis1.position();
        drivetrainRightSideSpeed = Controller.Axis3.position() - Controller.Axis1.position();
    }
    lastButtonStateDrive = currentButtonStateDrive;  // Update the previous state

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
 * Toggles the piston state each time the L1 button is pressed.
 *
 * @note Uses a static variable to track the previous state of the button.
 */
void controlPiston() {
    static bool lastButtonState = false;  // Track the previous state of the button
    bool currentButtonState = Controller.ButtonL1.pressing();

    if (currentButtonState && !lastButtonState) {  // Detect button press
        pistonState = !pistonState;                // Toggle the piston state
        piston.set(pistonState);
    }

    lastButtonState = currentButtonState;  // Update the previous state
}

/**
 * Handles the control of the intake and conveyor motors based on the controller's input.
 *
 * If R1 is pressed, the intake and conveyor motors spin forward.
 * If R2 is pressed, the intake and conveyor motors spin reverse.
 * If the Up button is pressed, the conveyor motor spins forward.
 * If the Down button is pressed, the conveyor motor spins reverse.
 * If the X button is pressed, the intake motor spins forward.
 * If the B button is pressed, the intake motor spins reverse.
 *
 * Otherwise, the motors will stop if they are not already stopped.
 */
void controlIntakeAndConveyor() {
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
        // Handle individual conveyor control
        if (Controller.ButtonUp.pressing()) {
            conveyor.spin(forward);
            conveyorActive = true;
        } else if (Controller.ButtonDown.pressing()) {
            conveyor.spin(reverse);
            conveyorActive = true;
        } else
            conveyorActive = false;

        // Handle individual intake control
        if (Controller.ButtonX.pressing()) {
            intake.spin(forward);
            intakeActive = true;
        } else if (Controller.ButtonB.pressing()) {
            intake.spin(reverse);
            intakeActive = true;
        } else
            intakeActive = false;
    }

    // Stop conveyor and intake if no active command
    if (!conveyorActive)
        conveyor.stop();
    if (!intakeActive)
        intake.stop();
}

/**
 * Handles the control of the wall stake arm based on the controller's input.
 *
 * If the A button is pressed, the wall stake arm spins forward.
 * If the Y button is pressed, the wall stake arm spins reverse.
 *
 * Otherwise, the wall stake arm will stop if it is not already stopped.
 */
void controlWallStakeArmManually() {
    if (Controller.ButtonA.pressing())
        wallStakeArm.spin(forward);
    else if (Controller.ButtonY.pressing())
        wallStakeArm.spin(reverse);
    else
        wallStakeArm.stop();
}

/**
 * Handles the control of the sweeper arm based on the controller's input.
 *
 * If the Right button is pressed, the sweeper arm spins forward.
 * If the Left button is pressed, the sweeper arm spins reverse.
 *
 * Otherwise, the sweeper arm will stop if it is not already stopped.
 */
void controlSweeperArm() {
    if (Controller.ButtonRight.pressing())
        sweeperArm.spin(forward);
    else if (Controller.ButtonLeft.pressing())
        sweeperArm.spin(reverse);
    else
        sweeperArm.stop();
}

/**
 * Continuously handles controller input and updates the robot's systems accordingly.
 *
 * @return None
 */
int controllerLoop() {
    if (!teleopCodeEnabled)
        return 0;

    // Handle controller input
    while (true) {
        controlDrivetrain();
        controlIntakeAndConveyor();
        controlPiston();
        controlSweeperArm();
        controlWallStakeArmManually();
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

    bool alreadyRumbled = false;
    while (true) {
        if ((conveyor.velocity(rpm) < 20 && conveyor.current() > 5) && !alreadyRumbled) {
            Controller.rumble(".");
            alreadyRumbled = true;
        } else
            alreadyRumbled = false;

        // if too high current and too low velocity, stop
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
void driverControl(void) {
    // Setup for motors (in case autonomous does not finish in time)
    Drivetrain.setTimeout(0, seconds);
    Drivetrain.setStopping(coast);

    // Controller.rumble(". . . -");

    task controllerTask(controllerLoop);
    task stuckHelperTask(stuckHelperLoop);
    task infoTask(infoLoop);
}

int main() {
    // Run the pre-autonomous function.
    initalize();

    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(driverControl);

    // Prevent main from exiting with an infinite loop + gyro stuff.
    Brain.Screen.setFont(mono12);
    while (true) {
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(0, 0);

        double angle = Inertial.angle();
        Brain.Screen.print("Angle: ");
        Brain.Screen.print(angle);
        Brain.Screen.newLine();

        double roll = Inertial.roll();
        Brain.Screen.print("Roll: ");
        Brain.Screen.print(roll);
        Brain.Screen.newLine();

        double pitch = Inertial.pitch();
        Brain.Screen.print("Pitch: ");
        Brain.Screen.print(pitch);
        Brain.Screen.newLine();

        double yaw = Inertial.yaw();
        Brain.Screen.print("Yaw: ");
        Brain.Screen.print(yaw);
        Brain.Screen.newLine();

        double gyroRateX = Inertial.gyroRate(xaxis, rpm);
        Brain.Screen.print("Gyro Rate X: ");
        Brain.Screen.print(gyroRateX);
        Brain.Screen.newLine();

        double gyroRateY = Inertial.gyroRate(yaxis, rpm);
        Brain.Screen.print("Gyro Rate Y: ");
        Brain.Screen.print(gyroRateY);
        Brain.Screen.newLine();

        double gyroRateZ = Inertial.gyroRate(zaxis, rpm);
        Brain.Screen.print("Gyro Rate Z: ");
        Brain.Screen.print(gyroRateZ);
        Brain.Screen.newLine();

        double accelX = Inertial.acceleration(xaxis);
        Brain.Screen.print("Acceleration X: ");
        Brain.Screen.print(accelX);
        Brain.Screen.newLine();

        double accelY = Inertial.acceleration(yaxis);
        Brain.Screen.print("Acceleration Y: ");
        Brain.Screen.print(accelY);
        Brain.Screen.newLine();

        double accelZ = Inertial.acceleration(zaxis);
        Brain.Screen.print("Acceleration Z: ");
        Brain.Screen.print(accelZ);
        Brain.Screen.newLine();

        wait(50, msec);
    }
}
