#include "main.h"
#include "pros/motor_group.hpp"
#include "Customs\intake.cpp"

/* ------------------------------------- */
// Create swerve modules
SwerveModule frontLeft(13, 14, 7);
SwerveModule frontRight(-17, -18, 9);
SwerveModule backLeft(16, 15, 8);
SwerveModule backRight(19, 20, 10);

TrackingWheel forwardWheel(2.75, 0.0, 5.0, 1.0, 11);  // Forward tracking wheel
TrackingWheel sidewaysWheel(2.75, 5.0, 0.0, 1.0, 12); // Sideways tracking wheel
Chassis chassis(21, forwardWheel, sidewaysWheel);      // IMU on port 21
/* ------------------------------------- */

void initialize() {
    pros::lcd::initialize();

    // Calibrate sensors
    chassis.calibrateSensors();
}

void autonomous() {
    static int autonRunCount = 0; // Tracks how many times auton has run

    if (autonRunCount == 0) {
        
    } else if (autonRunCount == 1) {
        
    } else {
        return; // Do nothing after the second execution
    }

    autonRunCount++; // Increment the counter
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    Intake intake({-6, 5, 1}); // Initialize intake with motors on ports 1 and 2

    Odometry odometry(
        TrackingWheel(2.125, 0, 0, 1, 11), // Forward tracking wheel
        TrackingWheel(2.125, 0, 0, 1, 12), // Sideways tracking wheel
        21 // IMU port
    );  

    while (true) {
        // Update odometry with tracking wheel and gyro readings
        odometry.update();

        //intake control
        intake.control(master);

        double x = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127.0;
        double y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
        double rotation = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0;

        chassis.controlSwerve(x, y, rotation); // Control the swerve drive

        double xPosition = odometry.getPosition().x; // Store odometry position for testing
        pros::lcd::set_text(5, "Odometry X: " + std::to_string(xPosition)); // Display odometry position
        double yPosition = odometry.getPosition().y; // Store odometry position for testing
        pros::lcd::set_text(6, "Odometry Y: " + std::to_string(yPosition)); // Display odometry position
        double theta = odometry.getPosition().theta; // Store odometry heading for testing
        pros::lcd::set_text(7, "Odometry Theta: " + std::to_string(theta)); // Display odometry heading

        pros::delay(20);
    }
}