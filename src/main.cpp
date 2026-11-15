#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-20, -15},
                            pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({11, 16}, pros::MotorGearset::blue); 

pros:: MotorGroup dr4b ({-13, -14}, pros::MotorGearset::red); 

pros::Motor claw(12, pros::MotorGearset::green);

pros::Motor claw_spinner(-19, pros::MotorGearset::green);

// Inertial Sensor on port 12
pros :: Imu imu (2); 

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 4, not reversed
pros::Rotation horizontalEnc(13);
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
lemlib::ControllerSettings linearController(12, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            12, // derivative gain (kD)
                                            0, // anti windup
                                            0.5, // small error range, in inches
                                            60, // small error settle time, in milliseconds
                                            0.8, // large error range, in inches
                                            125, // large error settle time, in milliseconds
                                            5 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             13, // derivative gain (kD)
                                             0, // anti windup
                                             2, // small error range, in degrees
                                             60, // small error settle time, in milliseconds
                                             4, // large error range, in degrees
                                             120, // large error settle time, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
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

lemlib::Pose current = chassis.getPose();

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::delay(300); // wait for imu to calibrate
    chassis.setPose(0, 0, 0); // set the starting pose of the robot to x: 0, y: 0, theta: 0
    pros:: delay(300);

    dr4b.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); // hold both lift motors when stopped
    dr4b.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set claw to hold position when no power is applied
    claw_spinner.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set claw spinner to hold position when no power is applied
    
    
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

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void autonomous() {
    // Motor degrees above the lift's starting position. Tune on the field.
    constexpr double goalClearanceDeg = 180;
    constexpr double goalSeatDeg = 98; // seat the pin a little deeper before release
    dr4b.tare_position_all();
    dr4b.move_absolute(goalClearanceDeg, 50);
    claw.move(127);
    claw_spinner.move(127);
    chassis.setPose(0, 0, 0);

    // Activate the toggler, then line up for the goal.
    constexpr float goalLaneY = 19.0f; // travel farther after the toggler to line up the drop
    chassis.moveToPoint(0, 2, 325, {.maxSpeed = 55}, false);
    chassis.moveToPoint(0, -5, 450, {.forwards = false, .maxSpeed = 75}, false);
    chassis.moveToPoint(0, goalLaneY, 1100, {.maxSpeed = 110}, false);
    constexpr float goalHeadingDeg = 97.0f; // compensate for the turn ending slightly short
    chassis.turnToHeading(goalHeadingDeg, 720,
                         {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 95}, false);
    chassis.moveToPoint(17.5f, goalLaneY, 925, {.maxSpeed = 105}, false);
    chassis.moveToPoint(23, goalLaneY, 700, {.maxSpeed = 32}, false);

    // Maintain light contact so the chassis does not roll off the goal during scoring.
    constexpr int goalContactPower = 12;
    chassis.tank(goalContactPower, goalContactPower, true);

    // Lower the pin into the goal with the claw closed. Open after the seating move.
    claw_spinner.brake();
    dr4b.move_absolute(goalSeatDeg, 35);
    pros::delay(500);
    dr4b.brake();
    claw.move_voltage(-6000);
    pros::delay(700);
    claw.brake();
    pros::delay(50);

    chassis.tank(0, 0, true);
    chassis.moveToPoint(0, goalLaneY, 1100, {.forwards = false, .maxSpeed = 110}, false);

    // Aim at the pin in field coordinates rather than projecting the current heading.
    constexpr float pickupX = -0.25f; // turn a little farther counterclockwise toward the cup
    constexpr float pickupY = 34.0f;
    constexpr int pickupLowerTimeMs = 425;
    chassis.turnToPoint(pickupX, pickupY, 950,
                        {.maxSpeed = 100, .minSpeed = 20, .earlyExitRange = 1.25f}, true);
    dr4b.move_voltage(-3000);
    pros::delay(pickupLowerTimeMs);
    dr4b.brake();
    chassis.waitUntilDone();

    // Feed the pin inward on contact, then close at the end of the slower approach.
    claw_spinner.move(127);
    chassis.moveToPoint(pickupX, pickupY, 1250, {.maxSpeed = 48}, false);
    claw.move(127);
    pros::delay(400);
    claw.brake();

    // Raise while backing onto the same line as the alliance goal, then turn left.
    // The short approach and lift positions are starting values for the field.
    constexpr float bigGoalApproachX = -1.5f;
    constexpr double bigGoalClearanceDeg = 260;
    constexpr double bigGoalReleaseDeg = 215;
    dr4b.move_absolute(bigGoalClearanceDeg, 50);
    chassis.moveToPoint(pickupX, goalLaneY, 1050, {.forwards = false, .maxSpeed = 80}, false);
    chassis.turnToHeading(270, 900,
                         {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 95},
                         false);
    chassis.moveToPoint(bigGoalApproachX, goalLaneY, 450, {.maxSpeed = 25}, false);
    chassis.tank(0, 0, true);

    // Lower the held pin onto the big goal, then open the claw.
    claw_spinner.brake();
    dr4b.move_absolute(bigGoalReleaseDeg, 35);
    pros::delay(500);
    dr4b.brake();
    claw.move_voltage(-6000);
    pros::delay(700);
    claw.brake();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    pros::lcd::print(0, "Heading: %f", imu.get_heading());
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
    
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        claw.move(127);
        claw_spinner.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        claw.move(-127);
        claw_spinner.move(-127);
        }
        else {
        claw.move(0);
        claw_spinner.move(0);
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        dr4b.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        dr4b.move(-127);
        }
        else {
        dr4b.move(0);
        }
        pros::delay(10);
 }
}

