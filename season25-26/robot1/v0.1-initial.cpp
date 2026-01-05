#include "main.h"

#include "lemlib/api.hpp"  // https://lemlib.readthedocs.io/en/stable/index.html

// Drivetrain motor groups
pros::MotorGroup left_motors({-1, -2, 3}, pros::MotorGears::blue);
pros::MotorGroup right_motors({-8, 9, 10}, pros::MotorGears::blue);

// Intake motors
pros::Motor front_roller(19, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor top_roller(20, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor back_roller(12, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor cycle_roller(11, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors,                // left motor group
                              &right_motors,               // right motor group
                              12.7,                        // track width (dist between center of wheels width)
                              lemlib::Omniwheel::NEW_275,  // using 2.75 omnis
                              450,                         // drivetrain rpm
                              5                            // horizontal drift is 5 (for now)
);

// lateral motion controller
lemlib::ControllerSettings lateralController(5,    // proportional gain (kP) (10)
                                             0,    // integral gain (kI)
                                             3,    // derivative gain (kD)
                                             3,    // anti windup
                                             1,    // small error range, in inches
                                             100,  // small error range timeout, in milliseconds
                                             3,    // large error range, in inches
                                             500,  // large error range timeout, in milliseconds
                                             30    // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1,    // proportional gain (kP) (2)
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
lemlib::OdomSensors odomSensors(nullptr,  // apparently if two nullptrs for the vertical tracking wheels, it will use the drivetrain
                                nullptr,  // vertical tracking wheel 2
                                nullptr,  // horizontal tracking wheel
                                nullptr,  // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                                nullptr   // inertial sensor
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

pros::Controller controller(pros::E_CONTROLLER_MASTER);

int autonSelection = 1;
const char* autonNames[] = {
    "Do Nothing",
    "Red Left",
    "Red Right",
    "Blue Left",
    "Blue Right"};
const int NUM_AUTONS = 5;  // total number of auton routines

const int LOOP_DELAY = 20;  // loop delay in milliseconds

void initalize() {
    // fix up the 5.5w
    left_motors.set_gearing(pros::MotorGears::green, 2);   // set gearing for 5.5w motor
    right_motors.set_gearing(pros::MotorGears::green, 0);  // set gearing for 5.5w motor

    // calibrate chassis + initialize lcd
    pros::lcd::initialize();
    chassis.calibrate();
}

void autonomous() {
    chassis.setPose(0, 0, 0);

    switch (autonSelection) {
        case 1:
            // Red Left
            chassis.moveToPoint(0, 3, 4000);
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
            // Blue Left
            // chassis.follow(blue_left, 15, 2000);  // lookahead,timeout
            break;
        case 4:
            // Blue Right
            // chassis.follow(blue_right, 15, 2000);  // lookahead,timeout
        default:
            // Do nothing (basically case 0)
            break;
    }
}

void controlDrivetrain() {
    int left = controller.get_analog(ANALOG_LEFT_Y);
    int right = controller.get_analog(ANALOG_RIGHT_Y);

    chassis.tank(left, right);

    // print robot location to the brain screen
    controller.print(0, 1, "X:%.1f Y:%.1f T:%.1f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
}

// Toggle states for intake control
bool intakeForward = false;
bool intakeReverse = false;
bool cycleForward = false;
bool cycleReverse = false;

// Button press tracking to detect new presses
bool r1Pressed = false;
bool r2Pressed = false;
bool l1Pressed = false;
bool l2Pressed = false;

void controlIntake() {
    // R1: Toggle intake forward (front, top, back rollers)
    bool r1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    if (r1Current && !r1Pressed) {
        intakeForward = !intakeForward;
        if (intakeForward) intakeReverse = false;  // Turn off reverse if forward is on
    }
    r1Pressed = r1Current;

    // R2: Toggle intake reverse
    bool r2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    if (r2Current && !r2Pressed) {
        intakeReverse = !intakeReverse;
        if (intakeReverse) intakeForward = false;  // Turn off forward if reverse is on
    }
    r2Pressed = r2Current;

    // L1: Toggle cycle roller forward
    bool l1Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    if (l1Current && !l1Pressed) {
        cycleForward = !cycleForward;
        if (cycleForward) cycleReverse = false;
    }
    l1Pressed = l1Current;

    // L2: Toggle cycle roller reverse
    bool l2Current = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    if (l2Current && !l2Pressed) {
        cycleReverse = !cycleReverse;
        if (cycleReverse) cycleForward = false;
    }
    l2Pressed = l2Current;

    // X button: front and top rollers spin in opposite directions
    bool oppositeMode = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    // Apply intake motor states
    if (oppositeMode) {
        // Front spins one way, top spins the opposite way
        front_roller.move(-100);
        top_roller.move(-100);
        back_roller.move(100);
    } else if (intakeForward) {
        front_roller.move(-100);  // Reversed
        top_roller.move(100);
        back_roller.move(100);
    } else if (intakeReverse) {
        front_roller.move(100);  // Reversed
        top_roller.move(-100);
        back_roller.move(-100);
    } else {
        front_roller.move(0);
        top_roller.move(0);
        back_roller.move(0);
    }

    // Apply cycle roller states
    if (cycleForward) {
        cycle_roller.move(100);
    } else if (cycleReverse) {
        cycle_roller.move(-100);
    } else {
        cycle_roller.move(0);
    }
}

void opcontrol() {
    while (true) {
        controlDrivetrain();
        controlIntake();
        pros::delay(LOOP_DELAY);
    }
}
