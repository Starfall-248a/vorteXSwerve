#include "TrackingWheel.hpp"
#include "pros/imu.hpp"

#pragma once

class Odometry {
public:
    struct Position {
        double x;      // Robot's x position
        double y;      // Robot's y position
        double theta;  // Robot's heading (in radians)
    };

private:
    Position position = {0.0, 0.0, 0.0}; // Robot's position and heading

    double prevForward = 0.0; // Previous forward tracking wheel position
    double prevSideways = 0.0; // Previous sideways tracking wheel position
    double prevTheta = 0.0; // Previous IMU heading

    TrackingWheel forwardWheel;  // Forward tracking wheel
    TrackingWheel sidewaysWheel; // Sideways tracking wheel
    pros::IMU imu;               // IMU for heading

public:
    /**
     * Constructor for the Odometry class.
     * @param forwardWheel Forward tracking wheel object.
     * @param sidewaysWheel Sideways tracking wheel object.
     * @param imuPort Port number for the IMU sensor.
     */
    Odometry(const TrackingWheel& forwardWheel, const TrackingWheel& sidewaysWheel, int imuPort)
        : forwardWheel(forwardWheel), sidewaysWheel(sidewaysWheel), imu(imuPort) {}

    /**
     * Updates the robot's position and heading based on tracking wheel and IMU readings.
     */
    void update() {
        double forwardInches = forwardWheel.ticksToInches();
        double sidewaysInches = sidewaysWheel.ticksToInches();

        // Calculate changes in tracking wheel positions
        double deltaForward = forwardInches - prevForward;
        double deltaSideways = sidewaysInches - prevSideways;

        // Update previous tracking wheel positions
        prevForward = forwardInches;
        prevSideways = sidewaysInches;

        // Get current heading from IMU (convert to radians)
        double gyroHeading = imu.get_rotation() * (M_PI / 180.0);

        // Calculate change in heading
        double deltaTheta = gyroHeading - prevTheta;
        prevTheta = gyroHeading;

        // Transform local changes into global coordinates
        double avgTheta = position.theta + (deltaTheta / 2.0); // Average heading during movement
        double globalDeltaX = deltaForward * std::cos(avgTheta) - deltaSideways * std::sin(avgTheta);
        double globalDeltaY = deltaForward * std::sin(avgTheta) + deltaSideways * std::cos(avgTheta);

        // Update global position and heading
        position.x += globalDeltaX;
        position.y += globalDeltaY;
        position.theta = gyroHeading; // Update heading directly from IMU
    }
    
    void resetSensors() {
        // Reset IMU
        imu.reset();
        while (imu.is_calibrating()) {
            pros::delay(50); // Wait for IMU calibration to complete
        }

        // Reset tracking wheels
        forwardWheel.resetSensor();
        sidewaysWheel.resetSensor();
    }

    /**
     * Gets the forward tracking wheel object.
     * @return Reference to the forward tracking wheel.
     */
    TrackingWheel& getForwardWheel() {
        return forwardWheel;
    }

    /**
     * Gets the sideways tracking wheel object.
     * @return Reference to the sideways tracking wheel.
     */
    TrackingWheel& getSidewaysWheel() {
        return sidewaysWheel;
    }

    /**
     * Gets the robot's current position and heading.
     * @return A Position struct containing x, y, and theta.
     */
    Position getPosition() const {
        return position;
    }
};