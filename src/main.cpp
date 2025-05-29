#include "main.h"
#include "pros/motor_group.hpp"
#include "Customs\intake.cpp"

// Constants for swerve drive
const double WHEELBASE = 12.408980;  // Distance between front and back wheels (in inches)
const double TRACKWIDTH = 12.511438; // Distance between left and right wheels (in inches)
const double RADIUS = std::sqrt((WHEELBASE * WHEELBASE) + (TRACKWIDTH * TRACKWIDTH)) / 2.0;

// Create swerve modules
SwerveModule frontLeft(13, 14, 7);
SwerveModule frontRight(-17, -18, 9);
SwerveModule backLeft(16, 15, 8);
SwerveModule backRight(19, 20, 10);

void initialize() {
    pros::lcd::initialize();
}

void autonomous() {
    // Autonomous code can be added here
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    Intake intake({-6, 5, 1}); // Initialize intake with motors on ports 1 and 2

    Odometry odometry;

    while (true) {

        // Update odometry with tracking wheel and gyro readings

        //intake control
        intake.control(master);

        double x = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
        double y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
        double rotation = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

        // Deadband to prevent small joystick movements from affecting the motors
        if (std::abs(x) < .07) x = 0.0;
        if (std::abs(y) < .07) y = 0.0;
        if (std::abs(rotation) < .07) rotation = 0.0;

        double frontLeftAngle, frontRightAngle, backLeftAngle, backRightAngle;

        if (x != 0 || y != 0 || rotation != 0) {
            // Calculate angles for each module
            frontLeftAngle = std::atan2(y + rotation * (WHEELBASE / 2), x - rotation * (TRACKWIDTH / 2)) * (180.0 / M_PI);
            frontRightAngle = std::atan2(y + rotation * (WHEELBASE / 2), x + rotation * (TRACKWIDTH / 2)) * (180.0 / M_PI);
            backLeftAngle = std::atan2(y - rotation * (WHEELBASE / 2), x - rotation * (TRACKWIDTH / 2)) * (180.0 / M_PI) + 180;
            backRightAngle = std::atan2(y - rotation * (WHEELBASE / 2), x + rotation * (TRACKWIDTH / 2)) * (180.0 / M_PI) + 180;

            // Calculate speeds for each module
            double frontLeftSpeed = std::sqrt(std::pow(x - rotation * (TRACKWIDTH / 2), 2) + std::pow(y + rotation * (WHEELBASE / 2), 2));
            double frontRightSpeed = std::sqrt(std::pow(x + rotation * (TRACKWIDTH / 2), 2) + std::pow(y + rotation * (WHEELBASE / 2), 2));
            double backLeftSpeed = std::sqrt(std::pow(x - rotation * (TRACKWIDTH / 2), 2) + std::pow(y - rotation * (WHEELBASE / 2), 2));
            double backRightSpeed = std::sqrt(std::pow(x + rotation * (TRACKWIDTH / 2), 2) + std::pow(y - rotation * (WHEELBASE / 2), 2));

            // Reverse speeds for front-right and back-left modules when turning in place
            if (x == 0 && y == 0 && rotation != 0) {
                frontRightSpeed = -frontRightSpeed;
                backLeftSpeed = -backLeftSpeed;
            }

            // Normalize speeds to ensure they stay within range [0, 1]
            double maxSpeed = std::max({frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed});
            if (maxSpeed > 1.0) {
                frontLeftSpeed /= maxSpeed;
                frontRightSpeed /= maxSpeed;
                backLeftSpeed /= maxSpeed;
                backRightSpeed /= maxSpeed;
            }

            // Set module speeds and angles
            frontLeft.setModuleSpeedAndAngle(frontLeftSpeed, frontLeftAngle);
            frontRight.setModuleSpeedAndAngle(frontRightSpeed, frontRightAngle);
            backLeft.setModuleSpeedAndAngle(backLeftSpeed, backLeftAngle);
            backRight.setModuleSpeedAndAngle(backRightSpeed, backRightAngle);
        } else {
            frontLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
        }

        // Display actual angles from the swerve modules' rotation sensors
        pros::lcd::set_text(1, 
            "FL Target: " + std::to_string(frontLeftAngle) + 
            " Current: " + std::to_string(frontLeft.getCurrentAngle()));
        pros::lcd::set_text(2, 
            "FR Target: " + std::to_string(frontRightAngle) + 
            " Current: " + std::to_string(frontRight.getCurrentAngle()));
        pros::lcd::set_text(3, 
            "BL Target: " + std::to_string(backLeftAngle) + 
            " Current: " + std::to_string(backLeft.getCurrentAngle()));
        pros::lcd::set_text(4, 
            "BR Target: " + std::to_string(backRightAngle) + 
            " Current: " + std::to_string(backRight.getCurrentAngle()));

        pros::delay(20);
    }
}