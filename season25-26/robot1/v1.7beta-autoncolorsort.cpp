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

struct RumbleEvent {
    int timeMs;           // remaining match time to trigger
    const char* pattern;  // rumble pattern
    bool triggered;       // has it fired already
};

// ### HARDWARE ###
// Pneumatics
pros::adi::DigitalOut hoodSolenoid('A');     // hood solenoid
pros::adi::DigitalOut tubeSolenoid('B');     // tube solenoid
pros::adi::DigitalOut descoreSolenoid('C');  // sweeper solenoid

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
                              6                            // horizontal drift is 6 (bc center traction - should be 8, 6 safer)
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
                                             15     // maximum acceleration (slew)
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

const bool COLOR_SORT_ENABLED = true;          // whether color sorting is enabled
const bool DEBUG_PRINT_ENABLED = true;         // whether debug printing task is enabled
const bool CONTROLLER_DISPLAY_ENABLED = true;  // whether controller display task is enabled

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

bool autonLocked = true;  // whether the auton selection is confirmed yet
int autonSelection = 2;
AllianceColor allianceColor = autonRoutes[autonSelection].alliance;  // default alliance color follows first route

// ASSET(redright_txt);

// ### GLOBAL VARIABLES ###
int colorSortTicker = 0;    // tick count for colorsorting
bool opticalLedOn = false;  // track last set PWM value

DriveMode driveMode = DriveMode::ARCADE;  // default drive mode

// Intake control state
IntakeMode intakeMode = IntakeMode::OFF;
bool tubeMechDeployed = false;

bool descoreMechDeployed = false;

// Button press tracking to detect new presses
bool r1Pressed = false;
bool r2Pressed = false;
bool l1Pressed = false;
bool l2Pressed = false;
bool aPressed = false;
bool bPressed = false;
bool xPressed = false;
bool yPressed = false;

// Global task pointers for task management
pros::Task* debugPrintTask = nullptr;
pros::Task* autonSelectorTask = nullptr;
pros::Task* controlIntakeTask = nullptr;
pros::Task* controllerDisplayTask = nullptr;

void controllerDisplay(void* param) {
    const double MATCH_LENGTH_MS = 1.75 * 60 * 1000;  // 1:45 match
    const double matchStart = pros::millis();         // start time of driver control
    const int MOTOR_OVERHEAT_THRESHOLD = 55;          // 55 Celsius (https://www.vexforum.com/t/v5-overheating-threshold/64984/2)

    // Define all rumble events in one array
    RumbleEvent rumbles[] = {
        {60000, "..", false},  // 1m
        {30000, "..", false},  // 30s
        {10000, "..", false}   // 10s
    };

    while (true) {  // a 200ms loop with delay bc controller slow
        lemlib::Pose pose = chassis.getPose();
        controller.print(0, 0, "Drive Mode: %s", (driveMode == DriveMode::TANK) ? "TANK  " : "ARCADE");
        pros::delay(50);
        controller.print(1, 0, "Intake: %s", (intakeMode == IntakeMode::OFF) ? "OFF       " : (intakeMode == IntakeMode::FORWARD)   ? "FORWARD   "
                                                                                          : (intakeMode == IntakeMode::REVERSE)     ? "REVERSE   "
                                                                                          : (intakeMode == IntakeMode::LOW_GOAL)    ? "LOW GOAL  "
                                                                                          : (intakeMode == IntakeMode::MIDDLE_GOAL) ? "MIDDLE GOAL"
                                                                                                                                    : "TOP GOAL  ");
        pros::delay(50);
        double voltage = pros::battery::get_voltage() / 1000.0;
        double current = pros::battery::get_current() / 1000.0;
        double power = voltage * current;
        controller.print(2, 0, "Battery: %.2fV, %.2fA, %.2fW", voltage, current, power);

        pros::delay(50);
        double elapsed = pros::millis() - matchStart;
        double remaining = MATCH_LENGTH_MS - elapsed;

        // Process all rumble events
        for (auto& event : rumbles) {
            if (!event.triggered && remaining <= event.timeMs) {
                controller.rumble(event.pattern);
                event.triggered = true;
            }
        }

        pros::delay(50);
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
                printf("Left button pressed\n");
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
                printf("Right button pressed\n");
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
            printf("Center button pressed\n");
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
    printf("Now initializing motors.\n");

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
    optical.disable_gesture();
    optical.set_integration_time(OPTICAL_INTEGRATION_TIME);  // 3-712 ms limits
    optical.set_led_pwm(0);                                  // 0-100

    printf("Now initializing LCD...\n");
    pros::lcd::initialize();

    printf("Now calibrating inertial + chassis...\n");
    // calibrate chassis + initialize lcd
    inertial.reset();
    pros::delay(200);
    chassis.calibrate();

    pros::lcd::print(0, "Auton (%i out of %i): %s", autonSelection + 1, NUM_AUTONS, autonRoutes[autonSelection].name);
    pros::lcd::print(1, "Left/Right change, center to confirm");
    pros::lcd::print(2, "Status: UNLOCKED");

    // Create debug print task that runs separately
    if (DEBUG_PRINT_ENABLED) {
        debugPrintTask = new pros::Task(debugPrint, nullptr, "Debug Print");
        debugPrintTask->set_priority(4);  // prio ranges from 1-16, default 8
    }

    if (CONTROLLER_DISPLAY_ENABLED) {
        controllerDisplayTask = new pros::Task(controllerDisplay, nullptr, "Controller Display");
        controllerDisplayTask->set_priority(4);
    }

    // Display auton selection interface
    autonSelectorTask = new pros::Task(autonSelector, nullptr, "Auton Selector");
    autonSelectorTask->set_priority(6);  // prio ranges from 1-16, default 8

    // Controls intake + colorsorting for both driver control + auton
    controlIntakeTask = new pros::Task(controlIntake, nullptr, "Intake Control");
    controlIntakeTask->set_priority(8);
}

void autonomous() {
    allianceColor = autonRoutes[autonSelection].alliance;  // sync alliance color with current autonomous choice

    switch (autonSelection) {
        case 0:  // Red Nothing - do nothing
        case 3:  // Blue Nothing - do nothing
            break;
        case 4:    // Blue Left
        case 1: {  // Red left
            chassis.setPose(-48.821, 8.244, 90);

            intakeMode = IntakeMode::FORWARD;
            hoodSolenoid.set_value(false);

            chassis.moveToPose(-24, 24, 45, 3000, {.maxSpeed = 80}, false);  // grab the 3 balls

            chassis.moveToPose(-14.5, 14.5, 135, 2500, {.maxSpeed = 70});
            chassis.waitUntil(3);  // (4)
            // outtake into middle goal
            intakeMode = IntakeMode::MIDDLE_GOAL;
            chassis.waitUntilDone();
            pros::delay(1500);

            // switch back
            indexRoller.move_velocity(0);
            topRoller.move_velocity(200);

            // back up a lil
            chassis.moveToPose(-24, 24, 135, 2500, {.forwards = false, .maxSpeed = 50, .minSpeed = 10}, false);

            chassis.moveToPose(-48, 47, -90, 4000, {.lead = 0.35, .maxSpeed = 80}, false);

            tubeSolenoid.set_value(true);

            lemlib::Pose pose = chassis.getPose();
            chassis.moveToPose(pose.x - 15, 47, -90, 1500, {.maxSpeed = 60}, false);  // nudge forward a bit
            pros::delay(2000);

            // chassis.moveToPose(pose.x - 12, 47, -90, 500, {.maxSpeed = 20}, false);   // nudge forward a bit
            // chassis.moveToPose(pose.x - 15, 47, -90, 2000, {.maxSpeed = 20}, false);  // nudge forward a lot

            chassis.moveToPoint(pose.x - 7, 47, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 10}, false);  // back up no turn
            tubeSolenoid.set_value(false);

            chassis.moveToPose(pose.x + 11, 48, 90, 4000, {.maxSpeed = 70});  // back up to score
            chassis.waitUntil(3.5);
            intakeMode = IntakeMode::TOP_GOAL;
            hoodSolenoid.set_value(true);
            chassis.waitUntilDone();

            break;
        }
        case 5:    // Blue Right
        case 2: {  // Red Right (WIP!)
            chassis.setPose(-48.821, -8.244, 90);

            intakeMode = IntakeMode::FORWARD;
            hoodSolenoid.set_value(false);

            chassis.moveToPose(-23.5, -23.5, 180 - 45, 2500, {.maxSpeed = 80}, false);  // grab the 3 balls

            pros::delay(500);
            chassis.moveToPose(-48, -47, 180 + 45, 4000, {.maxSpeed = 100}, false);

            chassis.turnToHeading(-90, 1500, {}, false);
            tubeSolenoid.set_value(true);

            pros::delay(200);

            lemlib::Pose pose = chassis.getPose();
            chassis.moveToPose(pose.x - 15, -47, -90, 1500, {.maxSpeed = 60}, false);  // nudge forward a bit
            pros::delay(2000);

            chassis.moveToPose(pose.x - 4, -47, -90, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 10}, false);  // back up no turn
            tubeSolenoid.set_value(false);

            chassis.moveToPose(pose.x + 16.5, -48, 90, 4000, {.maxSpeed = 70});  // back up to score
            chassis.waitUntil(5);
            intakeMode = IntakeMode::TOP_GOAL;
            hoodSolenoid.set_value(true);
            chassis.waitUntilDone();

            break;
        }
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
        int proximity = optical.get_proximity();  // higher = closer
        if (proximity > 35) {
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

void setIntake(IntakeMode mode, int ticks) {
    if (mode != IntakeMode::FORWARD && mode != IntakeMode::TOP_GOAL) {
        if (opticalLedOn) {
            optical.set_led_pwm(0);
            opticalLedOn = false;
        }
        colorSortTicker = 0;
    } else {
        if (!opticalLedOn) {
            optical.set_led_pwm(OPTICAL_LED_PWM);
            opticalLedOn = true;
        }
        if (ticks % (OPTICAL_INTEGRATION_TIME / LOOP_DELAY) == 0) {  // update every few ticks - eq to integration time
            if (isOtherAllianceBlockDetected()) {
                colorSortTicker = std::min(colorSortTicker + 5, COLORSORT_TICKER_LIMIT);
            } else {
                colorSortTicker = std::max(colorSortTicker - 3, 0);
            }
        }
    }
    // TODO: maybe make colorsort threshold > 5, then make decrease slower to compensate
    // This way the sensor can double check to make sure the ball really is the right color
    int topRollerSpeed = colorSortTicker >= 3 ? -200 : 200;

    switch (mode) {
        case IntakeMode::FORWARD:
            backRoller.move_velocity(200);
            frontRoller.move_velocity(200);
            topRoller.move_velocity(topRollerSpeed);
            indexRoller.move_velocity(0);
            break;
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
        case IntakeMode::LOW_GOAL:
            backRoller.move_velocity(200);
            frontRoller.move_velocity(-200);
            topRoller.move_velocity(0);
            indexRoller.move_velocity(100);
            break;
        case IntakeMode::TOP_GOAL:
            backRoller.move_velocity(200);
            frontRoller.move_velocity(200);
            topRoller.move_velocity(topRollerSpeed);
            indexRoller.move_velocity(150);
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

void controlIntake(void* param) {
    int ticks = 0;

    while (true) {
        // If in driver control, read them buttons, otherwise auton just uses the current intake mode
        if (!pros::competition::is_autonomous()) {
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
                }
            }
            xPressed = xCurrent;
        }

        // Execute motor commands based on active mode
        setIntake(intakeMode, ticks);
        ticks++;

        pros::delay(LOOP_DELAY);
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

void controlDescoreMech() {
    // Toggles the sweeper mech piston with the L2 button
    bool l2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    if (l2Current && !l2Pressed) {
        descoreMechDeployed = !descoreMechDeployed;
        descoreSolenoid.set_value(!descoreMechDeployed);  // the mech is deployed whenever the solenoid is retracted
    }
    l2Pressed = l2Current;
}

void opcontrol() {
    allianceColor = autonRoutes[autonSelection].alliance;  // ensure teleop color matches selected route

    // Stop auton selector task during driver control
    if (autonSelectorTask != nullptr) {
        autonSelectorTask->remove();
        delete autonSelectorTask;
        autonSelectorTask = nullptr;
    }

    chassis.cancelAllMotions();  // cancel any auton motions

    pros::lcd::clear_line(1);
    pros::lcd::print(2, "Status: DRIVING");

    while (true) {
        controlDrivetrain();
        controlTubeMech();
        controlDescoreMech();
        pros::delay(LOOP_DELAY);
    }
}
