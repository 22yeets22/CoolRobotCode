#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
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


// Define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
bool StuckHelperCodeEnabled = true;
// Define variables used for controlling motors based on controller inputs
bool ControllerLeftShoulderControlMotorsStopped = true;
bool ControllerRightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller = true;
bool DrivetrainRNeedsToBeStopped_Controller = true;
// Piston Thing
bool PistonState = false;


void controlDrivetrain() {
  // Function to handle drivetrain motor control
  int drivetrainLeftSideSpeed = Controller.Axis3.position();
  int drivetrainRightSideSpeed = Controller.Axis2.position();

  auto handleMotorControl = [](motor_group &motorGroup, int speed, bool &motorNeedsToBeStopped) {
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

void pistonLoop() {
  while (true) {
    if (Controller.ButtonA.pressing()) {
      PistonState = !PistonState;
      Piston.set(PistonState);
      while (Controller.ButtonA.pressing()) {
        wait(20, msec);
      }
    }
    wait(20, msec);
  }
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

int controllerLoop() {
  // Handle inputs from controller
  while (true) {
    if (RemoteControlCodeEnabled) {
      controlDrivetrain();
      controlConveyor();
      controlIntake();
    }
    wait(20, msec);
  }
  return 0;
}

int stuckHelperLoop() {
  while (true) {
    if (StuckHelperCodeEnabled) {
      
    }
    wait(20, msec);
  }
  return 0;
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
    for (double info : infoToPrint) {
        Brain.Screen.print(info);
        Brain.Screen.newLine();
    }

    wait(200, msec);  // Give some delay
  }
  return 0;
}

int main() {
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

  Controller.rumble(". . . -");
  task controllerTask(controllerLoop);
  task stuckHelperTask(stuckHelperLoop);
  task infoTask(infoLoop);
  task pistonTask(pistonLoop);
}