#pragma once

#include "SwerveModule.hpp"
#include "Odometry.hpp"
#include "pros/imu.hpp"


class Chassis {
private:
    // Constants for swerve drive
    const double WHEELBASE = 12.408980;  // Distance between front and back wheels (in inches)
    const double TRACKWIDTH = 12.511438; // Distance between left and right wheels (in inches)
    const double RADIUS = std::sqrt((WHEELBASE * WHEELBASE) + (TRACKWIDTH * TRACKWIDTH)) / 2.0;

    // Swerve modules
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    // Sensors
    Odometry odometry;
    pros::IMU imu;

public:
    /**
     * Constructor for the Chassis class.
     * Initializes swerve modules and sensors.
     * @param imuPort Port number for the IMU sensor.
     * @param forwardWheel Forward tracking wheel object.
     * @param sidewaysWheel Sideways tracking wheel object.
     */
    Chassis(int imuPort, const TrackingWheel& forwardWheel, const TrackingWheel& sidewaysWheel)
        : frontLeft(13, 14, 7), // Example motor ports for front-left module
          frontRight(17, 18, 9), // Example motor ports for front-right module
          backLeft(15, 16, 8), // Example motor ports for back-left module
          backRight(19, 20, 10), // Example motor ports for back-right module
          odometry(forwardWheel, sidewaysWheel, imuPort),
          imu(imuPort) {}

    /**
     * Calibrates all sensors.
     * This method should be called during initialization.
     */
    void calibrateSensors() {
        pros::lcd::set_text(1, "Calibrating IMU...");
        imu.reset();
        while (imu.is_calibrating()) {
            pros::delay(50); // Wait for IMU calibration to complete
        }
        pros::lcd::set_text(1, "IMU Calibration Complete!");

        // Reset odometry sensors
        odometry.getForwardWheel().resetSensor();
        odometry.getSidewaysWheel().resetSensor();
        pros::lcd::set_text(2, "Tracking Wheels Reset!");
    }

    /**
     * Updates the odometry system.
     */
    void updateOdometry() {
        odometry.update();
    }

    /**
     * Gets the odometry object for position tracking.
     * @return Reference to the odometry object.
     */
    Odometry& getOdometry() {
        return odometry;
    }

    /**
     * Gets the IMU object for heading tracking.
     * @return Reference to the IMU object.
     */
    pros::IMU& getIMU() {
        return imu;
    }

    /**
     * Controls the swerve modules based on joystick inputs.
     * @param x X-axis input from the joystick.
     * @param y Y-axis input from the joystick.
     * @param rotation Rotation input from the joystick.
     */
    void controlSwerve(double x, double y, double rotation) {
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
    }
};