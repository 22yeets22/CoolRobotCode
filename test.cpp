#include "main.h"

#include "lemlib/api.hpp"  // https://lemlib.readthedocs.io/en/stable/index.html

// Drivetrain motor groups
pros::MotorGroup left_motors({-1, -2, 3}, pros::MotorGears::blue);
// the 5.5w motor is on port 5
pros::MotorGroup l_motors({7, 5, 4});

// before lemlib runs:
l_motors.set_gearing(pros::MotorGears::green, 1);  // set gearing for 5.5w motor

pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGears::blue);

// Intake motors
pros::Motor front_roller(5, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor top_roller(5, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor back_roller(5, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor cycle_roller(5, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

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

const int LOOP_DELAY = 30;  // loop delay in milliseconds

void initalize() {
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
    while (true) {
        int left = controller.get_analog(ANALOG_LEFT_Y);
        int right = controller.get_analog(ANALOG_RIGHT_Y);

        chassis.tank(left, right);

        // print robot location to the brain screen
        controller.print(0, 1, "X:%.1f Y:%.1f T:%.1f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

        pros::delay(LOOP_DELAY);
    }
}

void opcontrol() {
    controlDrivetrain();
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