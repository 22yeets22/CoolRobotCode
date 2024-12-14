// drivetrain_controller.h

#ifndef DRIVETRAIN_CONTROLLER_H
#define DRIVETRAIN_CONTROLLER_H

#include "vex.h"

class DrivetrainController {
private:
    vex::smartdrive &Drivetrain;
    const int DEFAULT_DRIVE_VELOCITY; // Instance-specific default
    const int DEFAULT_TURN_VELOCITY; // Instance-specific default

    void resetDrive();

public:
    DrivetrainController(vex::smartdrive &drivetrain, 
                         int driveVelocity = 85, 
                         int turnVelocity = 60);

    void driveFor(double distance, int timeout = 0, double speed = -1, bool waitUntilDone = true);
    void turnTo(double angle, int timeout = 0, double speed = -1, bool waitUntilDone = true);
    void turnFor(double angle, int timeout = 0, double speed = -1, bool waitUntilDone = true);
};


#endif // DRIVETRAIN_CONTROLLER_H
