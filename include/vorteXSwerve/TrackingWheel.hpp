#include <cmath>
#include "pros/rotation.hpp"

#pragma once

class TrackingWheel {
private:
    double diameter; // Diameter of the tracking wheel (in inches)
    double offsetX;  // X offset from the center of the robot (in inches)
    double offsetY;  // Y offset from the center of the robot (in inches)
    double gearRatio; // Gear ratio between the encoder and the wheel
    const double TICKS_PER_REVOLUTION = 360.0; // Rotation sensor ticks per revolution
    pros::Rotation rotationSensor; // Rotation sensor for the tracking wheel

public:
    /**
     * Constructor for the TrackingWheel class.
     * @param diameter Diameter of the tracking wheel (in inches).
     * @param offsetX X offset from the center of the robot (in inches).
     * @param offsetY Y offset from the center of the robot (in inches).
     * @param gearRatio Gear ratio between the encoder and the wheel.
     * @param rotationPort Port number for the rotation sensor.
     */
    TrackingWheel(double diameter, double offsetX, double offsetY, double gearRatio, int sensorPort)
        : diameter(diameter), offsetX(offsetX), offsetY(offsetY), gearRatio(gearRatio), rotationSensor(sensorPort) {}

    /**
     * Converts ticks from the rotation sensor to inches.
     * @return Distance traveled by the tracking wheel in inches.
     */
    double ticksToInches() const {
        double wheelCircumference = diameter * M_PI; // Calculate wheel circumference
        double inchesPerTick = (wheelCircumference / TICKS_PER_REVOLUTION) * gearRatio; // Adjust for gear ratio
        return rotationSensor.get_position() * inchesPerTick; // Convert ticks to inches
    }

    /**
     * Gets the X offset of the tracking wheel.
     * @return X offset in inches.
     */
    double getOffsetX() const { return offsetX; }

    /**
     * Gets the Y offset of the tracking wheel.
     * @return Y offset in inches.
     */
    double getOffsetY() const { return offsetY; }

    /**
     * Resets the rotation sensor's position to zero.
     */
    void resetSensor() {
        rotationSensor.set_position(0);
    }
};