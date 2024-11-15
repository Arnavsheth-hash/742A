#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

pros::adi::Pneumatics clampA('G', false, false);
pros::adi::Pneumatics clampB('H', false, false);
pros::adi::Pneumatics arm('F', false, false);
pros::adi::Pneumatics intakeRaise('A', false, false);

pros::MotorGroup intake({19, -11}, pros::MotorGears::blue);

// left motor group
pros::MotorGroup left_motor_group({-10, -7, -9}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({2, 3, 8}, pros::MotorGears::blue);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.45, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(6);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(-18);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(5);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 2.5);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.125);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              40, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              45, // derivative gain (kD)
                                              0, // anti windup
                                              2, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// initialize function. Runs on program startup
void initialize() {
   pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	chassis.setPose(0,0,180);

    chassis.moveToPoint(0, 37.5, 1000, {.forwards = false, .minSpeed = 25});

    chassis.turnToPoint(6.5, 49.5, 450, {.forwards = false});

    chassis.moveToPoint(6.5, 49.5, 1000, {.forwards = false});

    pros::delay(500);

    clampA.toggle();
    clampB.toggle();

    pros::delay(250);

    chassis.turnToPoint(15, 24, 1000, {}, false);

    intake.move_voltage(10000);

    chassis.moveToPoint(15, 22,  1000,{}, false);

    pros::delay(750);

    chassis.moveToPoint(0, 10, 1500, {.forwards = false}, false);

    intake.move_voltage(0);

    clampA.toggle();
    clampB.toggle();

    intake.move_voltage(11000);

    chassis.moveToPose(14, 35, -86, 2500, {}, false);

    chassis.moveToPoint(14, 31.5, 2500, {.forwards = false});

    intake.move_voltage(0);

    chassis.turnToPoint(29, 31.5, 1000, {.forwards = false});

    chassis.moveToPoint(29, 31.5, 1000, {.forwards = false});

    pros::delay(500);

    clampA.toggle();
    clampB.toggle();

    pros::delay(250);

    intakeRaise.toggle();

    chassis.turnToPoint(53, 8, 1000, {}, false);

    intake.move_voltage(11000);

    chassis.moveToPoint(52, 11, 1500, {}, false);

    intakeRaise.toggle();

    pros::delay(1000);

    chassis.moveToPoint(48, 16, 1500, {.forwards = false, .maxSpeed = 60}, false);

    pros::delay(500);

    chassis.moveToPoint(52, 11, 1500, {}, false);

    chassis.turnToHeading(180, 1000, {}, false);

    pros::delay(500);

    intake.move_velocity(3000);

    chassis.moveToPoint(-52, 40, 2000, {.forwards = false, .maxSpeed = 75});








}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {


    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX, true);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            clampA.toggle();
            clampB.toggle();
            controller.rumble(".");
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            arm.toggle();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move_voltage(11000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_voltage(-11000);
        } else {
            intake.move_voltage(0);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            intakeRaise.toggle();
        }



        // delay to save resources
        pros::delay(25);


    }
}
