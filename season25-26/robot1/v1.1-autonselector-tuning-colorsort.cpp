#include "main.h"

#include "lemlib/api.hpp"  // https://lemlib.readthedocs.io/en/stable/index.html

// ### ENUMS ###
enum class DriveMode {
    TANK,
    ARCADE
};

enum class AllianceColor {
    RED,
    BLUE
};

// ### HARDWARE ###
// Pneumatics
pros::adi::DigitalOut hoodSolenoid('A');  // hood solenoid
pros::adi::DigitalOut tubeSolenoid('B');  // tube solenoid

pros::Imu inertial(4);      // inertial sensor
pros::Optical optical(18);  // optical sensor for intake detection

// Drivetrain motor groups
pros::MotorGroup leftMotors({-1, -2, 3}, pros::MotorGears::blue);
pros::MotorGroup rightMotors({-8, 9, 10}, pros::MotorGears::blue);

// Intake motors
pros::Motor frontRoller(19, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor topRoller(20, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor backRoller(12, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor indexRoller(11, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                 // left motor group
                              &rightMotors,                // right motor group
                              12.5,                        // track width (dist between center of wheels width)
                              lemlib::Omniwheel::NEW_275,  // using 2.75 omnis
                              450,                         // drivetrain rpm
                              2                            // horizontal drift is 2 (for now)
);

// lateral motion controller
lemlib::ControllerSettings lateralController(10,   // proportional gain (kP) (10)
                                             0,    // integral gain (kI)
                                             3,    // derivative gain (kD)
                                             3,    // anti windup
                                             1,    // small error range, in inches
                                             100,  // small error range timeout, in milliseconds
                                             3,    // large error range, in inches
                                             500,  // large error range timeout, in milliseconds
                                             20    // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2,    // proportional gain (kP) (2)
                                             0,    // integral gain (kI)
                                             10,   // derivative gain (kD)
                                             3,    // anti windup
                                             1,    // small error range, in degrees
                                             100,  // small error range timeout, in milliseconds
                                             3,    // large error range, in degrees
                                             500,  // large error range timeout, in milliseconds
                                             0     // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors odomSensors(nullptr,   // apparently if two nullptrs for the vertical tracking wheels, it will use the drivetrain
                                nullptr,   // vertical tracking wheel 2
                                nullptr,   // horizontal tracking wheel
                                nullptr,   // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                                &inertial  // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(10,   // joystick deadband out of 127
                                     15,   // minimum output where drivetrain will move out of 127
                                     1.02  // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(10,   // joystick deadband out of 127
                                  15,   // minimum output where drivetrain will move out of 127
                                  1.02  // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, odomSensors, &throttleCurve, &steerCurve);

pros::Controller controller(pros::E_CONTROLLER_MASTER);  // define controller

// ### GLOBAL VARIABLES ###
const int LOOP_DELAY = 20;                         // loop delay in milliseconds
const bool COLOR_SORT_ENABLED = true;              // whether color sorting is enabled
AllianceColor allianceColor = AllianceColor::RED;  // default alliance color (initialized)

// ### AUTON VARIABLES ###
bool autonLocked = false;  // whether the auton selection is confirmed yet
int autonSelection = 0;
const char* autonNames[] = {
    "Red Nothing",
    "Red Left",
    "Red Right",
    "Blue Nothing",
    "Blue Left",
    "Blue Right"};
const int NUM_AUTONS = 6;  // total number of auton routines

// ASSET(test_txt);

DriveMode driveMode = DriveMode::TANK;  // default drive mode

// Toggle states for intake control
bool intakeForward = false;
bool intakeReverse = false;
bool cycleForward = false;
bool cycleReverse = false;
bool topGoalMode = false;
bool tubeMechDeployed = false;

// Button press tracking to detect new presses
bool r1Pressed = false;
bool r2Pressed = false;
bool l1Pressed = false;
bool aPressed = false;
bool bPressed = false;
bool xPressed = false;  // Added missing X button tracking
bool yPressed = false;

// Global task pointers for proper task management
pros::Task* debugPrintTask = nullptr;
pros::Task* autonSelectorTask = nullptr;


void debugPrint(void* param) {
    // Separate task for printing robot position and status to screens
    while (true) {
        lemlib::Pose pose = chassis.getPose();

        // Print to brain LCD
        pros::lcd::print(3, "X: %.1f", pose.x);
        pros::lcd::print(4, "Y: %.1f", pose.y);
        pros::lcd::print(5, "T: %.1f", pose.theta);

        // Print to controller screen
        controller.print(0, 1, "X:%.1f Y:%.1f T:%.1f", pose.x, pose.y, pose.theta);

        pros::delay(200);
    }
}

void autonSelector(void* param) {
    while (true) {
        int buttons = pros::lcd::read_buttons();

        // Only allow changes if not locked
        if (!autonLocked) {
            if (buttons & LCD_BTN_LEFT) {
                // Move left
                autonSelection = (autonSelection - 1 + NUM_AUTONS) % NUM_AUTONS;
                pros::lcd::clear_line(0);  // only clear the line with auton name
                pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonNames[autonSelection]);

                // Wait until button is released
                while (pros::lcd::read_buttons() & LCD_BTN_LEFT) {
                    pros::delay(LOOP_DELAY);
                }
            }
            if (buttons & LCD_BTN_RIGHT) {
                // Move right
                autonSelection = (autonSelection + 1) % NUM_AUTONS;
                pros::lcd::clear_line(0);
                pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonNames[autonSelection]);

                // Wait until button is released
                while (pros::lcd::read_buttons() & LCD_BTN_RIGHT) {
                    pros::delay(LOOP_DELAY);
                }
            }
        }

        // Center button locks the selection
        if (buttons & LCD_BTN_CENTER) {
            autonLocked = !autonLocked;  // toggle lock state

            // todo: not hardcode it
            allianceColor = (autonSelection == 0 || autonSelection == 1 || autonSelection == 2) ? AllianceColor::RED : AllianceColor::BLUE;

            pros::lcd::clear_line(2);
            if (autonLocked) {
                pros::lcd::print(2, "Status: LOCKED");
            } else {
                pros::lcd::print(2, "Status: UNLOCKED");
            }

            // Wait until button is released
            while (pros::lcd::read_buttons() & LCD_BTN_CENTER) {
                pros::delay(LOOP_DELAY);
            }
        }

        pros::delay(LOOP_DELAY);  // small loop delay
    }
}

void initialize() {
    // Display initial selection and instructions
    printf("Now initalizing motors.\n");

    // fix up the 5.5w
    leftMotors.set_gearing(pros::MotorGears::green, 2);   // set gearing for 5.5w motor
    rightMotors.set_gearing(pros::MotorGears::green, 0);  // set gearing for 5.5w motor

    // set all rollers to coast except indexer
    frontRoller.set_reversed(true);
    frontRoller.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    backRoller.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    topRoller.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    indexRoller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // configure optical sensor
    optical.set_integration_time(60);  // 3-712 ms limits
    optical.set_led_pwm(80);           // 0-100

    printf("Now initializing LCD...\n");
    pros::lcd::initialize();

    printf("Now calibrating inertial + chassis...\n");
    // calibrate chassis + initialize lcd
    inertial.reset();
    chassis.calibrate();

    pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonNames[autonSelection]);
    pros::lcd::print(1, "Left/Right change, center to confirm");
    pros::lcd::print(2, "Status: UNLOCKED");

    // Create debug print task that runs separately
    debugPrintTask = new pros::Task(debugPrint, nullptr, "Debug Print");
    debugPrintTask->set_priority(4);  // prio ranges from 1-16, default 8

    // Display auton selection interface
    autonSelectorTask = new pros::Task(autonSelector, nullptr, "Auton Selector");
    autonSelectorTask->set_priority(4);  // prio ranges from 1-16, default 8
}

void competition_initialize() {
    // This function runs after initialize() and before autonomous or opcontrol
}

void autonomous() {
    chassis.setPose(0, 0, 0);

    switch (autonSelection) {
        case 0:
            // Red Nothing - do nothing
            break;
        case 1:
            // Red Left
            chassis.moveToPoint(0, 12, 4000);
            chassis.waitUntilDone();
            // chassis.turnToHeading(90, 4000);
            // chassis.waitUntilDone();
            //  chassis.follow(test_txt, 15, 4000);
            //  chassis.follow(red_left, 15, 2000);  // lookahead,timeout
            break;
        case 2:
            // Red Right
            // chassis.follow(red_right, 15, 2000);  // lookahead,timeout
            break;
        case 3:
            // Blue Nothing - do nothing
            break;
        case 4:
            // Blue Left
            // chassis.follow(blue_left, 15, 2000);  // lookahead,timeout
            break;
        case 5:
            // Blue Right
            // chassis.follow(blue_right, 15, 2000);  // lookahead,timeout
            break;
        default:
            // Do nothing (basically case 0)
            break;
    }
}

void controlDrivetrain() {
    // Toggle drive mode with L1 button
    bool l1CurrentState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    if (l1CurrentState && !l1Pressed) {
        // Toggle between modes
        driveMode = (driveMode == DriveMode::TANK) ? DriveMode::ARCADE : DriveMode::TANK;
    }
    l1Pressed = l1CurrentState;

    if (driveMode == DriveMode::TANK) {
        // Tank drive
        int left = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(left, right);
    } else {
        // Arcade drive
        int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int steer = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, steer);
    }
}

bool isOtherAllianceBlockDetected() {
    if (COLOR_SORT_ENABLED) {
        // Sort by color
        int proximity = optical.get_proximity();
        if (proximity > 50) {
            // object detected
            double hue = optical.get_hue();
            if ((hue >= 0 && hue <= 20) || (hue >= 340 && hue <= 360)) {  // detected red
                return (allianceColor == AllianceColor::BLUE);
            } else if (hue >= 170 && hue <= 260) {  // detected blue
                return (allianceColor == AllianceColor::RED);
            }
        }
    }
    return false;
}

void controlIntake() {
    // Button R1 - Intake forward into bin (back, front, top rollers forward, indexer stopped, hood low)
    // ACTION: Collecting blocks from the field into the robot's bin
    bool r1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    if (r1Current && !r1Pressed) {
        intakeForward = !intakeForward;
        if (intakeForward) {
            intakeReverse = false;
            cycleForward = false;
            cycleReverse = false;
            topGoalMode = false;
            // Set hood low only when intaking into bin
            hoodSolenoid.set_value(false);
        }
    }
    r1Pressed = r1Current;

    // Button R2 - Outtake/reverse (all rollers backward, indexer stopped, hood unchanged)
    // ACTION: Emergency outtake - clear jams without cycling, (reversing balls)
    bool r2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    if (r2Current && !r2Pressed) {
        intakeReverse = !intakeReverse;
        if (intakeReverse) {
            intakeForward = false;
            cycleForward = false;
            cycleReverse = false;
            topGoalMode = false;
        }
    }
    r2Pressed = r2Current;

    // Button A - Cycle into middle goal (front fwd, top reverse, back fwd, indexer stopped)
    // ACTION: Cycling blocks into middle goal scoring position
    bool aCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    if (aCurrent && !aPressed) {
        cycleForward = !cycleForward;
        if (cycleForward) {
            intakeForward = false;
            intakeReverse = false;
            cycleReverse = false;
            topGoalMode = false;
        }
    }
    aPressed = aCurrent;

    // Button B - Cycle into bottom goal (all backward including indexer, hood unchanged)
    // ACTION: Cycling blocks into bottom goal scoring position
    bool bCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (bCurrent && !bPressed) {
        cycleReverse = !cycleReverse;
        if (cycleReverse) {
            intakeForward = false;
            intakeReverse = false;
            cycleForward = false;
            topGoalMode = false;
        }
    }
    bPressed = bCurrent;

    // Button X - Cycle into top goal (intake + indexer forward with hood high)
    // ACTION: Cycling blocks into top goal scoring position
    bool xCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (xCurrent && !xPressed) {
        topGoalMode = !topGoalMode;
        if (topGoalMode) {
            intakeForward = false;
            intakeReverse = false;
            cycleForward = false;
            cycleReverse = false;
            // Raise hood for top goal scoring
            hoodSolenoid.set_value(true);

            if (tubeMechDeployed) {  // retract tube mech if deployed - to not exceed size
                tubeSolenoid.set_value(false);
                tubeMechDeployed = false;
            }
        }
    }
    xPressed = xCurrent;

    // Execute the active mode based on which button is toggled on
    if (intakeForward) {
        // EXECUTING: Intake mode - actively collecting blocks into bin
        backRoller.move_velocity(200);
        frontRoller.move_velocity(200);
        topRoller.move_velocity(isOtherAllianceBlockDetected() ? -200 : 200);
        indexRoller.move_velocity(0);  // Indexer off to allow blocks to settle in bin
        // Hood is already set to low when this mode was activated
    } else if (intakeReverse) {
        // EXECUTING: Emergency outtake - ejecting blocks from robot
        backRoller.move_velocity(-200);
        frontRoller.move_velocity(-200);
        topRoller.move_velocity(-200);
        indexRoller.move_velocity(0);  // Indexer off for simple reverse operation
        // Hood state unchanged
    } else if (cycleForward) {
        // EXECUTING: Middle goal cycling - positioning blocks for mid-height scoring
        backRoller.move_velocity(200);   // Continue feeding blocks
        frontRoller.move_velocity(200);  // Pull blocks forward
        topRoller.move_velocity(-200);   // Counter-rotate to control block flow
        indexRoller.move_velocity(200);  // Indexer off for manual control
        // Hood state unchanged
    } else if (cycleReverse) {
        // EXECUTING: Bottom goal cycling - positioning blocks for low-height scoring
        backRoller.move_velocity(200);  // Reverse all main rollers
        frontRoller.move_velocity(-200);
        topRoller.move_velocity(-200);
        indexRoller.move_velocity(200);  // Indexer forward to assist with positioning
        // Hood state unchanged
    } else if (topGoalMode) {
        // EXECUTING: Top goal cycling - positioning blocks for high-height scoring
        backRoller.move_velocity(200);  // All rollers forward including indexer
        frontRoller.move_velocity(200);
        topRoller.move_velocity(200);
        indexRoller.move_velocity(200);
        // Hood is already set to high when this mode was activated
    } else {
        // IDLE: All systems stopped when no mode is active
        backRoller.move_velocity(0);
        frontRoller.move_velocity(0);
        topRoller.move_velocity(0);
        indexRoller.move_velocity(0);
        // Hood state unchanged - remains at last position
    }
}

void controlTubeMech() {
    // Toggles the tube mech piston with the Y button
    bool yCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    if (yCurrent && !yPressed) {
        tubeMechDeployed = !tubeMechDeployed;
        tubeSolenoid.set_value(tubeMechDeployed);
    }
    yPressed = yCurrent;
}

void opcontrol() {
    // Stop auton selector task during driver control
    if (autonSelectorTask != nullptr) {
        autonSelectorTask->remove();
        delete autonSelectorTask;
        autonSelectorTask = nullptr;
    }

    while (true) {
        controlDrivetrain();
        controlIntake();
        controlTubeMech();
        pros::delay(LOOP_DELAY);
    }
}
