#include "main.h"

#include <algorithm>

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

enum class IntakeMode {
    OFF,
    FORWARD,
    REVERSE,
    LOW_GOAL,
    MIDDLE_GOAL,
    TOP_GOAL
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
                              12.45,                       // track width (dist between center of wheels width)
                              lemlib::Omniwheel::NEW_275,  // using 2.75 omnis
                              450,                         // drivetrain rpm
                              3                            // horizontal drift is 3 (for now)
);

// lateral motion controller
lemlib::ControllerSettings lateralController(14.5,  // proportional gain (kP) (10) - 14
                                             0,     // integral gain (kI)
                                             4.5,   // derivative gain (kD) (2) - 2.75
                                             0,     // anti windup
                                             0,     // small error range, in inches
                                             0,     // small error range timeout, in milliseconds
                                             0,     // large error range, in inches
                                             0,     // large error range timeout, in milliseconds
                                             0      // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2.4,  // proportional gain (kP) (2) - 2.4
                                             0,    // integral gain (kI)
                                             20,   // derivative gain (kD) (10) - 20
                                             0,    // anti windup
                                             0,    // small error range, in degrees
                                             0,    // small error range timeout, in milliseconds
                                             0,    // large error range, in degrees
                                             0,    // large error range timeout, in milliseconds
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

// ### GLOBAL CONSTANTS ###
const int LOOP_DELAY = 20;  // loop delay in milliseconds

const int OPTICAL_INTEGRATION_TIME = LOOP_DELAY * 3;  // 60 ms
const int COLORSORT_TICKER_LIMIT = 10;
const int OPTICAL_LED_PWM = 100;

const bool COLOR_SORT_ENABLED = true;    // whether color sorting is enabled
const bool DEBUG_PRINT_ENABLED = false;  // whether debug printing task is enabled

// ### AUTON VARIABLES ###
struct AutonRoute {
    const char* name;
    AllianceColor alliance;
};

const AutonRoute autonRoutes[] = {
    {"Red Nothing", AllianceColor::RED},    // 0
    {"Red Left", AllianceColor::RED},       // 1
    {"Red Right", AllianceColor::RED},      // 2
    {"Blue Nothing", AllianceColor::BLUE},  // 3
    {"Blue Left", AllianceColor::BLUE},     // 4
    {"Blue Right", AllianceColor::BLUE}};   // 5

const int NUM_AUTONS = 6;

bool autonLocked = false;  // whether the auton selection is confirmed yet
int autonSelection = 2;
AllianceColor allianceColor = autonRoutes[autonSelection].alliance;  // default alliance color follows first route

ASSET(redright_auton);

// ### GLOBAL VARIABLES ###
int ticks = 0;            // loop tick counter
int colorSortTicker = 0;  // tick count for colorsorting

DriveMode driveMode = DriveMode::TANK;  // default drive mode

// Intake control state
IntakeMode intakeMode = IntakeMode::OFF;
bool tubeMechDeployed = false;

// Button press tracking to detect new presses
bool r1Pressed = false;
bool r2Pressed = false;
bool l1Pressed = false;
bool aPressed = false;
bool bPressed = false;
bool xPressed = false;
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
                pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonRoutes[autonSelection].name);

                // Wait until button is released
                while (pros::lcd::read_buttons() & LCD_BTN_LEFT) {
                    pros::delay(LOOP_DELAY);
                }
            }
            if (buttons & LCD_BTN_RIGHT) {
                // Move right
                autonSelection = (autonSelection + 1) % NUM_AUTONS;
                pros::lcd::clear_line(0);
                pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonRoutes[autonSelection].name);

                // Wait until button is released
                while (pros::lcd::read_buttons() & LCD_BTN_RIGHT) {
                    pros::delay(LOOP_DELAY);
                }
            }
        }

        // Center button locks the selection
        if (buttons & LCD_BTN_CENTER) {
            autonLocked = !autonLocked;  // toggle lock state

            allianceColor = autonRoutes[autonSelection].alliance;

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
    optical.set_integration_time(OPTICAL_INTEGRATION_TIME);  // 3-712 ms limits
    optical.set_led_pwm(OPTICAL_LED_PWM);                    // 0-100

    printf("Now initializing LCD...\n");
    pros::lcd::initialize();

    printf("Now calibrating inertial + chassis...\n");
    // calibrate chassis + initialize lcd
    inertial.reset();
    chassis.calibrate();

    pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonRoutes[autonSelection].name);
    pros::lcd::print(1, "Left/Right change, center to confirm");
    pros::lcd::print(2, "Status: UNLOCKED");

    // Create debug print task that runs separately
    if (DEBUG_PRINT_ENABLED) {
        debugPrintTask = new pros::Task(debugPrint, nullptr, "Debug Print");
        debugPrintTask->set_priority(4);  // prio ranges from 1-16, default 8
    }

    // Display auton selection interface
    autonSelectorTask = new pros::Task(autonSelector, nullptr, "Auton Selector");
    autonSelectorTask->set_priority(4);  // prio ranges from 1-16, default 8
}

void competition_initialize() {
    // This function runs after initialize() and before autonomous or opcontrol
}

void autonomous() {
    allianceColor = autonRoutes[autonSelection].alliance;  // sync alliance color with current autonomous choice
    chassis.setPose(0, 0, 0);

    switch (autonSelection) {
        case 0:
            // Red Nothing - do nothing
            break;
        case 1:
            // Red Left
            //  chassis.follow(red_left, 15, 2000);  // lookahead,timeout
            break;
        case 2: {
            // Red Right
            topRoller.move_velocity(200);
            backRoller.move_velocity(200);
            frontRoller.move_velocity(200);

            chassis.follow(redright_auton, 15, 10000);  // lookahead,timeout
            chassis.waitUntil(48);                      // wait 36 inches
            tubeSolenoid.set_value(true);
            chassis.waitUntilDone();

            lemlib::Pose pose = chassis.getPose();
            chassis.moveToPose(pose.x - 2, pose.y, pose.theta, 1000);  // nudge forward a bit

            pros::delay(2000);
            pose = chassis.getPose();
            chassis.moveToPose(pose.x + 27, pose.y, -pose.theta, 2000);  // back up a bit
            break;
        }
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
        controller.print(1, 0, "Drive Mode: %s", (driveMode == DriveMode::TANK) ? "TANK  " : "ARCADE");
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
        if (proximity > 40) {
            // object detected
            double hue = optical.get_hue();
            if (hue >= 300 && hue <= 360) {  // detected red
                return (allianceColor == AllianceColor::BLUE);
            } else if (hue >= 215 && hue <= 250) {  // detected blue
                return (allianceColor == AllianceColor::RED);
            }
        }
    }
    return false;
}

void controlIntake() {
    // Button R1 - Intake forward into bin
    bool r1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    if (r1Current && !r1Pressed) {
        intakeMode = (intakeMode == IntakeMode::FORWARD) ? IntakeMode::OFF : IntakeMode::FORWARD;
        if (intakeMode == IntakeMode::FORWARD) {
            hoodSolenoid.set_value(false);
        }
    }
    r1Pressed = r1Current;

    // Button R2 - Outtake/reverse
    bool r2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    if (r2Current && !r2Pressed) {
        intakeMode = (intakeMode == IntakeMode::REVERSE) ? IntakeMode::OFF : IntakeMode::REVERSE;
    }
    r2Pressed = r2Current;

    // Button A - Cycle into middle goal
    bool aCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    if (aCurrent && !aPressed) {
        intakeMode = (intakeMode == IntakeMode::MIDDLE_GOAL) ? IntakeMode::OFF : IntakeMode::MIDDLE_GOAL;
    }
    aPressed = aCurrent;

    // Button B - Cycle into low goal
    bool bCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (bCurrent && !bPressed) {
        intakeMode = (intakeMode == IntakeMode::LOW_GOAL) ? IntakeMode::OFF : IntakeMode::LOW_GOAL;
    }
    bPressed = bCurrent;

    // Button X - Cycle into top goal
    bool xCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (xCurrent && !xPressed) {
        intakeMode = (intakeMode == IntakeMode::TOP_GOAL) ? IntakeMode::OFF : IntakeMode::TOP_GOAL;
        if (intakeMode == IntakeMode::TOP_GOAL) {
            hoodSolenoid.set_value(true);
            // Ensure tube mech is retracted when going to top goal to make sure not out of size limit
            if (tubeMechDeployed) {
                tubeSolenoid.set_value(false);
                tubeMechDeployed = false;
            }
        }
    }
    xPressed = xCurrent;

    // Execute motor commands based on active mode
    if (intakeMode != IntakeMode::FORWARD) {
        colorSortTicker = 0;
    }

    switch (intakeMode) {
        case IntakeMode::FORWARD: {
            if (ticks % (OPTICAL_INTEGRATION_TIME / LOOP_DELAY) == 0) {  // update every few ticks - eq to integration time
                if (isOtherAllianceBlockDetected()) {
                    colorSortTicker = std::min(colorSortTicker + 5, COLORSORT_TICKER_LIMIT);
                } else {
                    colorSortTicker = std::max(colorSortTicker - 3, 0);
                }
            }

            // TODO: maybe make colorsort threshold > 5, then make decrease slower to compensate
            // This way the sensor can double check to make sure the ball really is the right color
            int topRollerSpeed = colorSortTicker >= 3 ? -200 : 200;
            backRoller.move_velocity(200);
            frontRoller.move_velocity(200);
            topRoller.move_velocity(topRollerSpeed);
            indexRoller.move_velocity(0);
            break;
        }
        case IntakeMode::REVERSE:
            backRoller.move_velocity(-200);
            frontRoller.move_velocity(-200);
            topRoller.move_velocity(-200);
            indexRoller.move_velocity(0);
            break;
        case IntakeMode::MIDDLE_GOAL:
            backRoller.move_velocity(200);
            frontRoller.move_velocity(200);
            topRoller.move_velocity(-200);
            indexRoller.move_velocity(200);
            break;
        case IntakeMode::LOW_GOAL:  // note top roller is unneeded here
            backRoller.move_velocity(200);
            frontRoller.move_velocity(-200);
            indexRoller.move_velocity(200);
            break;
        case IntakeMode::TOP_GOAL:
            backRoller.move_velocity(200);
            frontRoller.move_velocity(200);
            topRoller.move_velocity(200);
            indexRoller.move_velocity(200);
            break;
        case IntakeMode::OFF:
        default:
            backRoller.move_velocity(0);
            frontRoller.move_velocity(0);
            topRoller.move_velocity(0);
            indexRoller.move_velocity(0);
            break;
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
    controller.print(1, 0, "Drive Mode: %s", (driveMode == DriveMode::TANK) ? "TANK  " : "ARCADE");
    allianceColor = autonRoutes[autonSelection].alliance;  // ensure teleop color matches selected route

    // Stop auton selector task during driver control
    if (autonSelectorTask != nullptr) {
        autonSelectorTask->remove();
        delete autonSelectorTask;
        autonSelectorTask = nullptr;
    }

    pros::lcd::clear_line(1);
    pros::lcd::print(2, "Status: DRIVING");

    while (true) {
        controlDrivetrain();
        controlIntake();
        controlTubeMech();
        ticks++;
        pros::delay(LOOP_DELAY);
    }
}
