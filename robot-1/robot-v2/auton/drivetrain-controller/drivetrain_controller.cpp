// drivetrain_controller.cpp

#include "drivetrain_controller.h"

DrivetrainController::DrivetrainController(vex::smartdrive &drivetrain,
                                           int driveVelocity,
                                           int turnVelocity)
    : Drivetrain(drivetrain),
      DEFAULT_DRIVE_VELOCITY(driveVelocity),
      DEFAULT_TURN_VELOCITY(turnVelocity) {}

void DrivetrainController::resetDrive() {
    Drivetrain.setDriveVelocity(DEFAULT_DRIVE_VELOCITY, vex::percent);
    Drivetrain.setTurnVelocity(DEFAULT_TURN_VELOCITY, vex::percent);
    Drivetrain.setTimeout(0, vex::seconds);
}

void DrivetrainController::driveFor(double distance, int timeout, double speed, bool waitUntilDone) {
    if (speed < 0) {  // Use instance default if no speed is provided
        speed = DEFAULT_DRIVE_VELOCITY;
    }
    Drivetrain.setDriveVelocity(speed, vex::percent);
    Drivetrain.setTimeout(timeout, vex::msec);
    Drivetrain.driveFor(distance, vex::inches, waitUntilDone);
    resetDrive();
}

void DrivetrainController::turnTo(double angle, int timeout, double speed, bool waitUntilDone) {
    if (speed < 0) {  // Use instance default if no speed is provided
        speed = DEFAULT_TURN_VELOCITY;  
    }
    Drivetrain.setTurnVelocity(speed, vex::percent);
    Drivetrain.setTimeout(timeout, vex::msec);
    Drivetrain.turnToHeading(angle, vex::degrees, waitUntilDone);
    resetDrive();
}

void DrivetrainController::turnFor(double angle, int timeout, double speed, bool waitUntilDone) {
    if (speed < 0) {  // Use instance default if no speed is provided
        speed = DEFAULT_TURN_VELOCITY;
    }
    Drivetrain.setTurnVelocity(speed, vex::percent);
    Drivetrain.setTimeout(timeout, vex::msec);
    Drivetrain.turnFor(angle, vex::degrees, waitUntilDone);
    resetDrive();
}
