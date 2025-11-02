#include "main.h"

#include "lemlib/api.hpp"  // https://lemlib.readthedocs.io/en/stable/index.html

// Enums
enum class DriveMode {
    TANK,
    ARCADE
};

// Pneumatics
pros::ADIDigitalOut hoodSolenoid('A');  // hood solenoid
pros::ADIDigitalOut tubeSolenoid('B');  // tube solenoid

// Drivetrain motor groups
pros::MotorGroup leftMotors({-1, -2, 3}, pros::MotorGears::blue);
pros::MotorGroup rightMotors({-8, 9, 10}, pros::MotorGears::blue);

// Intake motors
pros::Motor frontRoller(19, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor topRoller(20, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor backRoller(12, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor cycleRoller(11, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,                 // left motor group
                              &rightMotors,                // right motor group
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
    leftMotors.set_gearing(pros::MotorGears::green, 2);   // set gearing for 5.5w motor
    rightMotors.set_gearing(pros::MotorGears::green, 0);  // set gearing for 5.5w motor

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

    // print robot location to the controller screen
    controller.print(0, 1, "X:%.1f Y:%.1f T:%.1f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
}

DriveMode driveMode = DriveMode::ARCADE;  // default drive mode

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
        frontRoller.move(-100);
        topRoller.move(-100);
        backRoller.move(100);
    } else if (intakeForward) {
        frontRoller.move(-100);  // Reversed
        topRoller.move(100);
        backRoller.move(100);
    } else if (intakeReverse) {
        frontRoller.move(100);  // Reversed
        topRoller.move(-100);
        backRoller.move(-100);
    } else {
        frontRoller.move(0);
        topRoller.move(0);
        backRoller.move(0);
    }

    // Apply cycle roller states
    if (cycleForward) {
        cycleRoller.move(100);
    } else if (cycleReverse) {
        cycleRoller.move(-100);
    } else {
        cycleRoller.move(0);
    }
}

bool tubeMechDeployed = false;
bool aPressed = false;

void controlPneumatics() {
    // Example: Toggles the tube mech piston with the A button
    bool aCurrent = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    if (aCurrent && !aPressed) {
        tubeMechDeployed = !tubeMechDeployed;
    }
    aPressed = aCurrent;
}

void opcontrol() {
    while (true) {
        controlDrivetrain();
        controlIntake();
        pros::delay(LOOP_DELAY);
    }
}

// #include "main.h"

// #include "lemlib/api.hpp"  // https://lemlib.readthedocs.io/en/stable/index.html

// // Drivetrain motors
// pros::Motor left_motor_1(1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// pros::Motor left_motor_2(2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// pros::Motor left_motor_3(3, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// pros::Motor right_motor_1(-4, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// pros::Motor right_motor_2(-5, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
// pros::Motor right_motor_3(-6, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

// // Drivetrain motor groups
// pros::MotorGroup left_motors({1, 2, 3}, pros::MotorGears::blue);
// pros::MotorGroup right_motors({-4, -5, -6}, pros::MotorGears::blue);

// pros::Imu imu(20);  // IMU on port 20

// // horizontal tracking wheel
// pros::Rotation horizontalEnc(10);                                                  // negative makes reversed
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -1);  // 1 inch behind from the center of rotation

// // drivetrain settings
// lemlib::Drivetrain drivetrain(&left_motors,                // left motor group
//                               &right_motors,               // right motor group
//                               12.4,                        // track width (dist between center of wheels width)
//                               lemlib::Omniwheel::NEW_275,  // using 2.75 omnis
//                               480,                         // drivetrain rpm
//                               2                            // horizontal drift is 2 (for now)
// );

// // lateral motion controller
// lemlib::ControllerSettings lateralController(10,   // proportional gain (kP)
//                                              0,    // integral gain (kI)
//                                              3,    // derivative gain (kD)
//                                              3,    // anti windup
//                                              1,    // small error range, in inches
//                                              100,  // small error range timeout, in milliseconds
//                                              3,    // large error range, in inches
//                                              500,  // large error range timeout, in milliseconds
//                                              20    // maximum acceleration (slew)
// );

// // angular motion controller
// lemlib::ControllerSettings angularController(2,    // proportional gain (kP)
//                                              0,    // integral gain (kI)
//                                              10,   // derivative gain (kD)
//                                              3,    // anti windup
//                                              1,    // small error range, in degrees
//                                              100,  // small error range timeout, in milliseconds
//                                              3,    // large error range, in degrees
//                                              500,  // large error range timeout, in milliseconds
//                                              0     // maximum acceleration (slew)
// );

// // sensors for odometry
// lemlib::OdomSensors odomSensors(nullptr,      // apparently if two nullptrs for the vertical tracking wheels, it will use the drivetrain
//                                 nullptr,      // vertical tracking wheel 2
//                                 &horizontal,  // horizontal tracking wheel
//                                 nullptr,      // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                                 &imu          // inertial sensor
// );

// // input curve for throttle input during driver control
// lemlib::ExpoDriveCurve throttleCurve(10,   // joystick deadband out of 127
//                                      15,   // minimum output where drivetrain will move out of 127
//                                      1.02  // expo curve gain
// );

// // input curve for steer input during driver control
// lemlib::ExpoDriveCurve steerCurve(10,   // joystick deadband out of 127
//                                   15,   // minimum output where drivetrain will move out of 127
//                                   1.02  // expo curve gain
// );

// // create the chassis
// lemlib::Chassis chassis(drivetrain, lateralController, angularController, odomSensors, &throttleCurve, &steerCurve);

// pros::Controller master(pros::E_CONTROLLER_MASTER);

// int autonSelection = 0;
// const char* autonNames[] = {
//     "Do Nothing",
//     "Red Left",
//     "Red Right",
//     "Blue Left",
//     "Blue Right"};
// const int NUM_AUTONS = 5;  // total number of auton routines

// const int LOOP_DELAY = 30;  // loop delay in milliseconds

// ASSET(test_txt);

// void initialize() {
//     pros::lcd::initialize();
//     chassis.calibrate();

//     // Display initial selection and instructions
//     pros::lcd::clear();
//     pros::lcd::print(0, "Auton: %s", autonNames[autonSelection]);
//     pros::lcd::print(1, "Left/Right to change");
//     pros::lcd::print(2, "Center to confirm");

//     // print position to brain screen
//     pros::Task odomTask([&]() {
//         while (true) {
//             // print robot location to the brain screen
//             pros::lcd::print(3, "X: %f", chassis.getPose().x);            // x
//             pros::lcd::print(4, "Y: %f", chassis.getPose().y);            // y
//             pros::lcd::print(5, "Heading: %f", chassis.getPose().theta);  // heading
//             // delay to save resources
//             pros::delay(LOOP_DELAY);
//         }
//     });

//     while (true) {
//         int buttons = pros::lcd::read_buttons();

//         if (buttons & LCD_BTN_LEFT) {
//             // Move left
//             autonSelection = (autonSelection - 1 + NUM_AUTONS) % NUM_AUTONS;
//             pros::lcd::clear_line(0);  // only clear the line with auton name
//             pros::lcd::print(0, "Auton: %s", autonNames[autonSelection]);
//             pros::delay(300);  // debounce
//         }
//         if (buttons & LCD_BTN_RIGHT) {
//             // Move right
//             autonSelection = (autonSelection + 1) % NUM_AUTONS;
//             pros::lcd::clear_line(0);
//             pros::lcd::print(0, "Auton: %s", autonNames[autonSelection]);
//             pros::delay(300);  // debounce
//         }

//         pros::delay(LOOP_DELAY);  // small loop delay
//     }
// }

// void autonomous() {
//     switch (autonSelection) {
//         case 1:
//             // Red Left
//             chassis.follow(test_txt, 15, 4000);
//             // chassis.follow(red_left, 15, 2000);  // lookahead,timeout
//             break;
//         case 2:
//             // Red Right
//             // chassis.follow(red_right, 15, 2000);  // lookahead,timeout
//             break;
//         case 3:
//             // Blue Left
//             // chassis.follow(blue_left, 15, 2000);  // lookahead,timeout
//             break;
//         case 4:
//             // Blue Right
//             // chassis.follow(blue_right, 15, 2000);  // lookahead,timeout
//         default:
//             // Do nothing (basically case 0)
//             break;
//     }
// }

// void controlDrivetrain() {
//     while (true) {
//         int left = master.get_analog(ANALOG_LEFT_Y);
//         int right = master.get_analog(ANALOG_RIGHT_Y);

//         chassis.tank(left, right);

//         pros::delay(LOOP_DELAY);
//     }
// }

// void opcontrol() {
//     controlDrivetrain();
// }
