#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// motor groups
pros::MotorGroup leftMotors({-10,-9},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({7, 8}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)
pros :: MotorGroup dr4b({-1,11}, pros::MotorGearset::red);
pros :: Motor claw (-2, pros::MotorGearset::green);
pros :: Motor intake(19, pros::MotorGearset::blue);
pros :: Motor claw_wrist(3, pros::MotorGearset::green);
// Inertial Sensor on port 12
pros :: Imu imu (12);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 4, not reversed
pros::Rotation horizontalEnc(4);
// vertical tracking wheel encoder. Rotation sensor, port 18, reversed
pros::Rotation verticalEnc(18);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.5);
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.625, // 10.625 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);
// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            10, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            2.5 // maximum acceleration (slew)
);
// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);
// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);
// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::delay(5000); // wait for imu to calibrate
    chassis.setPose(0, 0, 0); // set the starting pose of the robot to x: 0, y: 0, theta: 0
    pros:: delay(3000);




    dr4b.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); // set lift to hold mode
    claw_wrist.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set lift to hold mode
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set lift to hold mode
    dr4b.tare_position(); // set lift to 0
    claw_wrist.tare_position(); // set lift to 0
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms
    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs
    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}
/**
 * Runs while the robot is disabled
 */
void disabled() {}
/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}
/**
 * Competition autonomous is empty; the practice sequence is in the A-button block below.
 */
void autonomous() {}
/**
 * Runs in driver control
 */
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // controller
    // loop to continuously update motors
    pros::lcd::print(0, "Heading: %f", imu.get_heading());
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            dr4b.move(127 * 0.75);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            dr4b.move(-127 * 0.25);
        } else {
            dr4b.move(0);
        }
         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            claw.move(-127);
            claw_wrist.move(127);
            dr4b.move(127);
            pros::delay(3000);
            claw.move(0);
            claw_wrist.move(0);
            dr4b.move(0);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
            claw.move(127);
         } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(-127);
            claw.move(-127);
        } else {
            intake.move(0);
            claw.move(0);
        }
         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            claw_wrist.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            //relative wris position flat;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            claw_wrist.move(-127);
        } else {
            claw_wrist.move(0);
        }// delay to save resources
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            dr4b.move_absolute(100, 200);
            claw_wrist.move_absolute(0, 200);
         }
         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            dr4b.move_absolute(0, 200);
            claw_wrist.move_absolute(200, 200);
            claw.move(127);
            rightMotors.move(127);
            leftMotors.move(127);
            pros::delay(3000);
            claw.move(0);
            rightMotors.move(0);
            leftMotors.move(0);
         }
        // Press X once: wrist up, lift up, and claw outtake together.
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            chassis.tank(0, 0, true); // stay still during this macro
            intake.move(0);
            // Start all three gently, then increase power after 0.1 second.
            claw_wrist.move(40);
            dr4b.move(40);
            claw.move(60);
            pros::delay(100);

            claw_wrist.move(80);
            dr4b.move(60);
            claw.move(100);
            pros::delay(200);
            dr4b.brake();         // 0.3-second lift pulse; tune toward a 3-inch rise

            pros::delay(500);
            claw_wrist.move(40);  // slow the wrist before stopping
            pros::delay(200);
            claw_wrist.brake();   // wrist runs for 1 second total

            pros::delay(800);
            claw.move(60);        // slow the outtake before stopping
            pros::delay(200);
            claw.brake();         // outtake runs for 2 seconds total
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        }

        // Press A once to run these steps. Holding A does not repeat them.
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            dr4b.brake();
            claw_wrist.brake();
            intake.move(0);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
            // 1. Push backward 3 inches and give the toggle one short pulse.
            chassis.setPose(0, 0, 0); // measure this move from zero
            claw.move(127);
            chassis.moveToPoint(0, -3, 3000, {.forwards = false, .maxSpeed = 30}, true);
            pros::delay(1400); // first timing estimate for the consistent yellow start
            claw.brake();
            chassis.waitUntilDone();
            pros::delay(3000); // stop for 3 seconds; do not pulse the toggle again

            // 2. Drive forward 4 inches away from the toggle.
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 4, 2000, {.maxSpeed = 50}, false);

            // 3. Turn around 180 degrees after the drive finishes.
            chassis.turnToHeading(180, 3000, {.maxSpeed = 50}, false);
            chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
        }
        pros::delay(10);
    }
}
