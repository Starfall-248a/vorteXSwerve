#ifndef SWERVE_MODULE_HPP
#define SWERVE_MODULE_HPP

#include "pros/screen.hpp"
#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include <cmath>

class SwerveModule {
private:
    pros::Motor motor1;
    pros::Motor motor2;
    pros::Rotation rotationSensor;

    double prevError = 0.0;
    double integral = 0.0;


    // PID constants
    const double kP = 18.2;  // Proportional gain
    const double kI = 0.0;  // Integral gain
    const double kD = 265;  // Derivative gain
    const double integralLimit = 5.0; // Limit the integral term

    /**
     * PID controller for angle adjustment.
     */
    double calculatePID(double targetAngle, double currentAngle) {
        double error = targetAngle - currentAngle;

        // Wrap error to [-180, 180] range
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Deadband to prevent oscillations
        if (std::abs(error) < 3.0) return 0.0; // No correction if error is less than 1 degree

        integral += error;
        integral = std::clamp(integral, -integralLimit, integralLimit); // Limit the integral term
        double derivative = error - prevError;
        prevError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

public:
    /**
     * Constructor for the SwerveModule class.
     * @param motor1Port Port number for the first motor.
     * @param motor2Port Port number for the second motor.
     * @param rotationPort Port number for the rotation sensor.
     */
    SwerveModule(int motor1Port, int motor2Port, int rotationPort)
        : motor1(motor1Port), motor2(motor2Port), rotationSensor(rotationPort) {}

    /**
     * Calculates and sets the motor speeds for the swerve module.
     * @param x X-axis input for strafing.
     * @param y Y-axis input for forward/backward motion.
     * @param rotation Rotation input for turning.
     * @param moduleX X position of the module relative to the robot center.
     * @param moduleY Y position of the module relative to the robot center.
     */
    void setModuleSpeed(double x, double y, double rotation, double moduleX, double moduleY, double radius) {
        double deltaX = x + rotation * (moduleY / radius);
        double deltaY = y - rotation * (moduleX / radius);
        double speed = std::sqrt(deltaX * deltaX + deltaY * deltaY);
        double targetAngle = std::atan2(deltaY, deltaX) * (180.0 / M_PI); // Convert to degrees

        double currentAngle = rotationSensor.get_angle() / 100.0; // Convert to degrees

        // Wrap current angle to [-180, 180]
        if (currentAngle > 180) currentAngle -= 360;
        if (currentAngle < -180) currentAngle += 360;

        double correction = calculatePID(targetAngle, currentAngle);

        double speed1 = speed + correction * 0.5;
        double speed2 = speed - correction * 0.5;

        if (std::abs(speed1) < 0.05 && std::abs(speed2) < 0.05) {
            motor1.move_velocity(0);
            motor2.move_velocity(0);
        } else {
            motor1.move_velocity(speed1 * 600);
            motor2.move_velocity(speed2 * 600);
        }
    }

    /**
     * Sets the speed of the swerve module.
     * @param speed The desired speed (0.0 to 1.0).
     */
    void setModuleSpeed(double speed) {
        motor1.move_velocity(speed * 600); // Scale speed to motor velocity range
        motor2.move_velocity(speed * 600); // Both motors drive at the same speed
    }

    /**
     * Sets the rotor angle of the swerve module to the specified target angle.
     * @param targetAngle The desired angle in degrees.
     */
    void setModuleAngle(double targetAngle) {
        double currentAngle = rotationSensor.get_position(); // Convert to degrees

        // Wrap current angle to [-180, 180]
        if (currentAngle > 180) currentAngle -= 360;
        if (currentAngle < -180) currentAngle += 360;

        // Calculate the correction using the PID controller
        double correction = calculatePID(targetAngle, currentAngle);

        // Apply the correction to the motors
        double speed1 = correction * 0.5;
        double speed2 = -correction * 0.5;

        // Set motor velocities to adjust the angle
        motor1.move_velocity(speed1 * 600); // Scale correction to motor velocity range
        motor2.move_velocity(speed2 * 600); // Scale correction to motor velocity range
    }

    /**
     * Sets the speed and angle of the swerve module.
     * @param speed The desired speed (0.0 to 1.0).
     * @param targetAngle The desired angle in degrees.
     */
    void setModuleSpeedAndAngle(double speed, double targetAngle) {
        double currentAngle = wrapAngle(rotationSensor.get_angle() / 100.0); // Convert to degrees
        targetAngle = wrapAngle(targetAngle);

        // Check if reversing the speed is more efficient
        double angleDifference = wrapAngle(targetAngle - currentAngle);
        if (std::abs(angleDifference) > 90) {
            targetAngle = wrapAngle(targetAngle + 180);
            speed = -speed; // Reverse the speed
        }

        // Calculate the correction using the PID controller
        double correction = calculatePID(targetAngle, currentAngle);

        // Apply angle correction to the motors
        if (std::abs(angleDifference) < 5.0) {
            // Prioritize driving the wheel
            motor1.move_velocity(speed * 600);
            motor2.move_velocity(speed * 600);
        } else {
            // Apply angle correction
            double rotationSpeed1 = correction * 600;
            double rotationSpeed2 = -correction * 600;
            double currentSpeed1 = motor1.get_actual_velocity(); // Get current motor speed
            double currentSpeed2 = motor2.get_actual_velocity();

            double targetSpeed1 = rotationSpeed1 + speed * 600;
            double targetSpeed2 = rotationSpeed2 + speed * 600;

            motor1.move_velocity(rampSpeed(currentSpeed1, targetSpeed1, 75));
            motor2.move_velocity(rampSpeed(currentSpeed2, targetSpeed2, 75));
        }
    }

    /**
     * Stops the motors of the swerve module.
     */
    void stop() {
        motor1.move_velocity(0);
        motor2.move_velocity(0);
    }

    /**
     * Gets the current angle of the swerve module.
     * @return The current angle in degrees.
     */
    double getCurrentAngle() {
        return rotationSensor.get_angle() / 100.0; // Convert to degrees
    }

    /**
     * Wraps the angle to the range [-180, 180].
     * @param angle The angle to wrap.
     * @return The wrapped angle.
     */
    double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Ramps the speed from the current speed to the target speed at the specified rate.
     * @param currentSpeed The current speed.
     * @param targetSpeed The target speed.
     * @param rampRate The rate at which to change the speed.
     * @return The new speed, ramped up or down.
     */
    double rampSpeed(double currentSpeed, double targetSpeed, double rampRate) {
        if (std::abs(targetSpeed - currentSpeed) < rampRate) {
            return targetSpeed; // Close enough to target speed
        }
        return currentSpeed + (targetSpeed > currentSpeed ? rampRate : -rampRate);
    }
};

#endif