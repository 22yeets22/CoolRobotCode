#include "main.h"

#include <algorithm>
#include <set>
#include <string>

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
    HIGH_GOAL,
    MIDDLE_GOAL
};

struct RumbleEvent {
    int timeMs;           // remaining match time to trigger
    const char* pattern;  // rumble pattern
    bool triggered;       // has it fired already
};

struct MotorEntry {
    int port;
    const char* name;
};

// ### HARDWARE ###
// Pneumatics
pros::adi::DigitalOut loaderSolenoid('A');      // loader solenoid
pros::adi::DigitalOut descoreSolenoid('B');     // descoring solenoid
pros::adi::DigitalOut middleGoalSolenoid('C');  // middle goal solenoid

pros::Imu inertial(13);     // inertial sensor
pros::Optical optical(16);  // optical sensor for intake colorsort

// ## MOTORS ##
// Drivetrain motor groups
pros::MotorGroup leftMotors({-1, 11, -12}, pros::MotorGears::blue);
pros::MotorGroup rightMotors({10, 19, -20}, pros::MotorGears::blue);

// Intake motors
pros::Motor intake(-2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor topRoller(14, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor middleRoller(-15, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

MotorEntry motors[] = {
    {1, "Ldrve"},
    {11, "Ldrve"},
    {12, "Ldrve"},
    {10, "Rdrve"},
    {19, "Rdrve"},
    {20, "Rdrve"},
    {2, "intake"},
    {14, "top"},
    {15, "mid"},
};

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                 // left motor group
                              &rightMotors,                // right motor group
                              10.6,                        // track width (dist between center of wheels width)
                              lemlib::Omniwheel::NEW_325,  // using 3.25 omnis
                              480,                         // drivetrain rpm
                              5                            // horizontal drift is 6 (bc center traction - should be 8, 6 safer)
);

// lateral motion controller
lemlib::ControllerSettings lateralController(14.5,  // proportional gain (kP) (10) - 14
                                             0,     // integral gain (kI)
                                             4.5,   // derivative gain (kD) (2) - 2.75
                                             3,     // anti windup
                                             1,     // small error range, in inches
                                             200,   // small error range timeout, in milliseconds
                                             2,     // large error range, in inches
                                             500,   // large error range timeout, in milliseconds
                                             3      // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2.4,  // proportional gain (kP) (2) - 2.4
                                             0,    // integral gain (kI)
                                             20,   // derivative gain (kD) (10) - 20
                                             3,    // anti windup
                                             1,    // small error range, in inches
                                             100,  // small error range timeout, in milliseconds
                                             3,    // large error range, in inches
                                             500,  // large error range timeout, in milliseconds
                                             0     // maximum acceleration (slew)
);

pros::Rotation horizontalEncoder(-9);  // horizontal tracking wheel on port 9, reversed
lemlib::TrackingWheel horizontalTrackingWheel(&horizontalEncoder, lemlib::Omniwheel::NEW_2, -5.75);

// sensors for odometry
lemlib::OdomSensors odomSensors(nullptr,                   // apparently if two nullptrs for the vertical tracking wheels, it will use the drivetrain
                                nullptr,                   // vertical tracking wheel 2
                                &horizontalTrackingWheel,  // horizontal tracking wheel
                                nullptr,                   // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                                &inertial                  // inertial sensor
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
const bool COLOR_SORT_ENABLED = true;          // ### whether color sorting is enabled
const bool DEBUG_PRINT_ENABLED = true;         // ### whether debug printing task is enabled
const bool CONTROLLER_DISPLAY_ENABLED = true;  // whether controller display task is enabled

const int LOOP_DELAY = 20;  // loop delay in milliseconds

// Colorsort
const int OPTICAL_INTEGRATION_TIME = LOOP_DELAY * 3;  // 60 ms
const int COLORSORT_TICKER_LIMIT = 10;
const int OPTICAL_LED_PWM = 100;
int colorSortTicker = 0;
bool opticalLedOn = false;  // track last set PWM value

// ### AUTON VARIABLES ###
int autonSelection = 1;  // ### DEFAULT AUTON INDEX

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

bool autonLocked = true;                                             // whether the auton selection is confirmed yet
AllianceColor allianceColor = autonRoutes[autonSelection].alliance;  // default alliance color follows first route

// ### CONTROLLER ###
const double MATCH_LENGTH_MS = 1.75 * 60 * 1000;  // 1:45 match
const int MOTOR_OVERHEAT_THRESHOLD = 50;          // 55 Celsius is the thresh, 50 is safe (https://www.vexforum.com/t/v5-overheating-threshold/64984/2)

// Controller Rumbles
RumbleEvent rumbles[] = {
    {60000, "..", false},  // 1m left
    {30000, "..", false},  // 30s
    {10000, "..", false}   // 10s
};

// ### GLOBAL VARIABLES ###
DriveMode driveMode = DriveMode::ARCADE;  // default drive mode
IntakeMode intakeMode = IntakeMode::OFF;

bool loaderMechDeployed = false;
bool descoreMechDeployed = false;

// Button press tracking to detect new presses
bool r1Pressed = false;
bool r2Pressed = false;
bool l1Pressed = false;
bool l2Pressed = false;
bool aPressed = false;
bool xPressed = false;
bool rightPressed = false;

// Global task pointers for task management
pros::Task* debugPrintTask = nullptr;
pros::Task* autonSelectorTask = nullptr;
pros::Task* controlIntakeTask = nullptr;
pros::Task* controllerDisplayTask = nullptr;

void controllerDisplay(void* param) {
    const double driverStart = pros::millis();  // start time of driver control

    std::set<std::string> lastProblemMotors;

    while (true) {  // a 400ms loop with delay bc controller slow
        // 1. Drive mode
        controller.print(0, 0, "Drive Mode: %s", (driveMode == DriveMode::TANK) ? "TANK  " : "ARCADE");
        pros::delay(100);

        // 2. Motor status (disconnect / overheat)
        std::string motorEvents;
        std::set<std::string> currentProblemMotors;
        bool newProblem = false;
        for (const auto& entry : motors) {
            pros::Motor motor(entry.port);

            std::string prefix;
            if (!motor.is_installed())
                prefix = "D-";  // disconnect
            else if (motor.get_temperature() >= MOTOR_OVERHEAT_THRESHOLD)
                prefix = "O-";  // overheat

            if (!prefix.empty()) {
                currentProblemMotors.insert(entry.name);
                if (!motorEvents.empty()) motorEvents += ' ';
                motorEvents += prefix + entry.name;

                if (lastProblemMotors.find(entry.name) == lastProblemMotors.end())
                    newProblem = true;
            }

            pros::delay(20);
        }

        if (newProblem) controller.rumble("--");
        if (!motorEvents.empty()) motorEvents += "               ";  // padding to clear old text
        controller.print(1, 0, "%s", motorEvents.empty() ? "All Motors Normal     " : motorEvents.c_str());
        lastProblemMotors = currentProblemMotors;
        pros::delay(100);

        // 3. Battery stats
        double voltage = pros::battery::get_voltage() / 1000.0;
        double current = pros::battery::get_current() / 1000.0;
        double power = voltage * current;
        controller.print(2, 0, "%.2fV, %.2fA, %.2fW    ", voltage, current, power);
        pros::delay(100);

        double elapsed = pros::millis() - driverStart;
        double remaining = MATCH_LENGTH_MS - elapsed;

        // 4. Timed rumble events
        if (pros::competition::is_connected()) {  // only rumble if competition switch is connected
            for (auto& event : rumbles) {
                if (!event.triggered && remaining <= event.timeMs) {
                    controller.rumble(event.pattern);
                    event.triggered = true;
                }
            }
        }
        pros::delay(100);
    }
}

void debugPrint(void* param) {
    // Separate task for printing robot position and status to screens
    while (true) {
        lemlib::Pose pose = chassis.getPose();

        // Print to brain LCD
        pros::lcd::print(3, "X: %.1f", pose.x);
        pros::lcd::print(4, "Y: %.1f", pose.y);
        pros::lcd::print(5, "T: %.1f", pose.theta);

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

            pros::lcd::clear_line(2);
            pros::lcd::print(2, "Status: %s", autonLocked ? "LOCKED" : "UNLOCKED");

            // Wait until button is released
            while (pros::lcd::read_buttons() & LCD_BTN_CENTER) {
                pros::delay(LOOP_DELAY);
            }
        }

        pros::delay(LOOP_DELAY);  // small loop delay
    }
}

void autonomous() {
    allianceColor = autonRoutes[autonSelection].alliance;  // sync alliance color with current autonomous choice
    // Stop auton selector task start of auton
    if (autonSelectorTask != nullptr) {
        autonSelectorTask->remove();
        delete autonSelectorTask;
        autonSelectorTask = nullptr;
    }

    switch (autonSelection) {
        case 0:  // Red Nothing - do nothing
        case 3:  // Blue Nothing - do nothing
            break;
        case 4:                                // Blue Left
        case 1: {                              // Red left
            chassis.setPose(-46.2, 14.1, 90);  // -45.773, 14.6

            intakeMode = IntakeMode::FORWARD;

            // chassis.moveToPose(-24, 24, 60, 3000, {.maxSpeed = 80}, false);  // grab the 3 balls
            // chassis.moveToPose(-26, 22.3, 60, 3000, {.maxSpeed = 80}, false);  // grab the 3 balls
            chassis.moveToPose(-22, 24, 60, 3000, {.lead = 0.3, .maxSpeed = 65}, false);  // grab the 3 balls

            // chassis.waitUntil(11.5);
            // loaderSolenoid.set_value(true);
            // chassis.waitUntilDone();
            // loaderSolenoid.set_value(false);

            // chassis.moveToPose(-15, 15, -45, 3500, {.forwards = false, .maxSpeed = 50});
            chassis.moveToPose(-10.5 - 1.5, 11 + 1.5, -45, 3500, {.forwards = false, .maxSpeed = 50});
            chassis.waitUntil(20);  // (4)
            // outtake into middle goal
            intakeMode = IntakeMode::REVERSE;
            pros::delay(250);
            intakeMode = IntakeMode::MIDDLE_GOAL;
            chassis.waitUntilDone();
            pros::delay(1000);

            // go to loader
            // chassis.moveToPoint(-40, 40, 2500, {.maxSpeed = 100, .minSpeed = 20}, false);
            chassis.moveToPoint(-42, 47, 3000, {.maxSpeed = 65, .minSpeed = 5}, false);
            chassis.moveToPose(-44, 47, -90, 4000, {.maxSpeed = 70}, false);  // lower lead = sharper curve
            loaderSolenoid.set_value(true);

            // back to forward
            intakeMode = IntakeMode::FORWARD;

            chassis.moveToPose(-60.5, 47, -90, 1000, {.maxSpeed = 60}, false);  // nudge forward a bit
            pros::delay(800);

            // chassis.moveToPose(pose.x - 12, 47, -90, 500, {.maxSpeed = 20}, false);   // nudge forward a bit
            // chassis.moveToPose(pose.x - 15, 47, -90, 2000, {.maxSpeed = 20}, false);  // nudge forward a lot

            chassis.moveToPoint(-29.5, 48 - 0.5, 3500, {.forwards = false, .maxSpeed = 65});  // back up to goal straight
            chassis.waitUntil(5);
            loaderSolenoid.set_value(false);  // loader up
            chassis.waitUntil(20);

            intakeMode = IntakeMode::HIGH_GOAL;  // score
            // loop for up to 2460ms in the optical integration time steps
            for (int elapsed = 0; elapsed < 2460; elapsed += OPTICAL_INTEGRATION_TIME) {
                if (colorSortTicker > 5) {
                    break;
                }
                pros::delay(OPTICAL_INTEGRATION_TIME);
            }
            intakeMode = IntakeMode::OFF;

            // goal rush
            chassis.moveToPoint(-42, 48, 2500, {.maxSpeed = 70});
            chassis.moveToPose(-14, 59, -90, 3500, {.forwards = false, .lead = 0.1, .maxSpeed = 60});
            break;
        }
        case 5:                                 // Blue Right
        case 2: {                               // Red Right
            chassis.setPose(-46.2, -14.1, 90);  // -45.773, -14.6
            intakeMode = IntakeMode::FORWARD;

            chassis.moveToPose(-26, -22.3, 120, 2000, {.maxSpeed = 70}, false);  // grab the 3 balls

            // go to loader
            chassis.moveToPoint(-40, -45, 3000, {.maxSpeed = 65, .minSpeed = 5}, false);
            chassis.moveToPose(-44, -47 - 1, -90, 4000, {.maxSpeed = 60}, false);  // lower lead = sharper curve
            loaderSolenoid.set_value(true);

            chassis.moveToPose(-61, -47 - 1, -90, 2000, {.maxSpeed = 60}, false);  // nudge forward a bit
            pros::delay(1000);

            // move to score
            chassis.moveToPose(-29, -48 - 0.5, -90, 5500, {.forwards = false, .maxSpeed = 80});  // back up to goal straight
            chassis.waitUntil(5);
            loaderSolenoid.set_value(false);  // loader up
            chassis.waitUntil(22);
            intakeMode = IntakeMode::HIGH_GOAL;  // score
            chassis.waitUntilDone();
            pros::delay(2000);
            intakeMode = IntakeMode::OFF;

            chassis.moveToPose(-38, -48, -90, 2000, {.maxSpeed = 70});
            chassis.moveToPose(-15, -36, -90, 3000, {.forwards = false, .lead = 0.2, .maxSpeed = 60});

            break;
        }
        default:
            // Do nothing (basically case 0)
            break;
    }
}

void controlDrivetrain() {
    // Toggle drive mode with RIGHT button
    bool rightCurrentState = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    if (rightCurrentState && !rightPressed) {
        // Toggle between modes
        driveMode = (driveMode == DriveMode::TANK) ? DriveMode::ARCADE : DriveMode::TANK;
    }
    rightPressed = rightCurrentState;

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
    // Sort by color
    int proximity = optical.get_proximity();  // higher = closer
    if (proximity > 180) {
        // object detected
        double hue = optical.get_hue();
        if (hue >= 325 && hue <= 360 || hue <= 10) {  // detected red 342-1
            return (allianceColor == AllianceColor::BLUE);
        } else if (hue >= 215 && hue <= 245) {  // detected blue 227-230
            return (allianceColor == AllianceColor::RED);
        }
    }
    return false;
}

void setIntakeMode(IntakeMode mode, int ticks) {
    if (COLOR_SORT_ENABLED && (mode == IntakeMode::HIGH_GOAL)) {
        if (!opticalLedOn) {
            optical.set_led_pwm(OPTICAL_LED_PWM);
            opticalLedOn = true;
        }
        if (ticks % (OPTICAL_INTEGRATION_TIME / LOOP_DELAY) == 0) {  // update every few ticks - eq to integration time
            if (isOtherAllianceBlockDetected()) {
                pros::lcd::print(6, "bad block detected! %d %f", optical.get_proximity(), optical.get_hue());
                colorSortTicker = std::min(colorSortTicker + 4, COLORSORT_TICKER_LIMIT);
            } else {
                pros::lcd::print(6, "nobody or good ball! %d %f", optical.get_proximity(), optical.get_hue());
                colorSortTicker = std::max(colorSortTicker - 3, 0);
            }
        }
    } else {
        if (opticalLedOn) {
            optical.set_led_pwm(0);
            opticalLedOn = false;
        }
        colorSortTicker = 0;
    }

    switch (mode) {
        case IntakeMode::FORWARD:
            middleGoalSolenoid.set_value(false);
            intake.move(127);
            middleRoller.move(64);
            topRoller.move(-16);
            break;
        case IntakeMode::REVERSE:
            middleGoalSolenoid.set_value(false);
            intake.move(-127);
            middleRoller.move(100);
            topRoller.move(-64);
            break;
        case IntakeMode::HIGH_GOAL:
            middleGoalSolenoid.set_value(false);
            intake.move(127);
            middleRoller.move(127);
            topRoller.move(127);
            break;
        case IntakeMode::MIDDLE_GOAL:
            middleGoalSolenoid.set_value(true);
            intake.move(127);
            middleRoller.move(127);
            topRoller.move(127);
            break;
        case IntakeMode::OFF:
        default:
            intake.move(0);
            middleRoller.move(0);
            topRoller.move(0);
            break;
    }
}

void controlIntake(void* param) {
    int ticks = 0;

    while (true) {
        if (!pros::competition::is_autonomous()) {
            bool xCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
            if (xCurrent && !xPressed) {
                intakeMode = (intakeMode == IntakeMode::HIGH_GOAL) ? IntakeMode::OFF : IntakeMode::HIGH_GOAL;
            }
            xPressed = xCurrent;

            bool aCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
            if (aCurrent && !aPressed) {
                intakeMode = (intakeMode == IntakeMode::MIDDLE_GOAL) ? IntakeMode::OFF : IntakeMode::MIDDLE_GOAL;
            }
            aPressed = aCurrent;

            bool r2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
            if (r2Current && !r2Pressed) {
                intakeMode = (intakeMode == IntakeMode::FORWARD) ? IntakeMode::OFF : IntakeMode::FORWARD;
            }
            r2Pressed = r2Current;

            bool r1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            if (r1Current && !r1Pressed) {
                intakeMode = (intakeMode == IntakeMode::REVERSE) ? IntakeMode::OFF : IntakeMode::REVERSE;
            }
            r1Pressed = r1Current;
        }

        setIntakeMode(intakeMode, ticks);

        ticks++;

        pros::delay(LOOP_DELAY);
    }
}

void controlLoaderMech() {
    // Toggles the loader mech piston with the L2 button
    bool l2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    if (l2Current && !l2Pressed) {
        loaderMechDeployed = !loaderMechDeployed;
        loaderSolenoid.set_value(loaderMechDeployed);
    }
    l2Pressed = l2Current;
}

void controlDescoreMech() {
    // Toggles the sweeper mech piston with the L1 button
    bool l1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    if (l1Current && !l1Pressed) {
        descoreMechDeployed = !descoreMechDeployed;
        descoreSolenoid.set_value(!descoreMechDeployed);  // invert it, so it starts up
    }
    l1Pressed = l1Current;
}

void initialize() {
    printf("Now initializing LCD...\n");
    pros::lcd::initialize();

    printf("Now calibrating inertial + chassis...\n");
    // calibrate chassis + initialize lcd
    inertial.reset();
    pros::delay(150);
    chassis.calibrate();

    pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonRoutes[autonSelection].name);
    pros::lcd::print(1, "Left/Right change, center to confirm");
    pros::lcd::print(2, "Status: %s", autonLocked ? "LOCKED" : "UNLOCKED");

    // Create debug print task that runs separately
    if (DEBUG_PRINT_ENABLED) {
        debugPrintTask = new pros::Task(debugPrint, nullptr, "Debug Print");
        if (debugPrintTask != nullptr) debugPrintTask->set_priority(4);  // prio ranges from 1-16, default 8
    }

    // Display auton selection interface
    autonSelectorTask = new pros::Task(autonSelector, nullptr, "Auton Selector");
    if (autonSelectorTask != nullptr) autonSelectorTask->set_priority(6);  // prio ranges from 1-16, default 8

    controlIntakeTask = new pros::Task(controlIntake, nullptr, "Intake Control");
    if (controlIntakeTask != nullptr) controlIntakeTask->set_priority(8);
}

void opcontrol() {
    if (CONTROLLER_DISPLAY_ENABLED) {
        controllerDisplayTask = new pros::Task(controllerDisplay, nullptr, "Controller Display");
        if (controllerDisplayTask != nullptr) controllerDisplayTask->set_priority(4);  // prio ranges from 1-16, default 8
    }

    chassis.cancelAllMotions();  // cancel any auton motions

    pros::lcd::clear_line(1);
    pros::lcd::print(2, "Status: DRIVING");

    while (true) {
        controlDrivetrain();
        controlLoaderMech();
        controlDescoreMech();
        pros::delay(LOOP_DELAY);
    }
}