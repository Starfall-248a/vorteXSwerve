#include <cmath>
#include "pros/imu.hpp"

class Odometry {
private:
    double x = 0.0; // Robot's x position
    double y = 0.0; // Robot's y position
    double theta = 0.0; // Robot's heading (in radians)

    double prevForward = 0.0; // Previous forward tracking wheel position
    double prevSideways = 0.0; // Previous sideways tracking wheel position
    double prevTheta = 0.0; // Previous gyro heading

    const double TRACKING_WHEEL_DIAMETER = 2.75; // Diameter of tracking wheels (in inches)
    const double TRACKING_WHEEL_CIRCUMFERENCE = TRACKING_WHEEL_DIAMETER * M_PI;
    const double TICKS_PER_REVOLUTION = 360.0; // Rotation sensor ticks per revolution

    double ticksToInches(double ticks) {
        return (ticks / TICKS_PER_REVOLUTION) * TRACKING_WHEEL_CIRCUMFERENCE;
    }

public:
    /**
     * Updates the robot's position and heading based on tracking wheel and gyro readings.
     * @param forwardTicks Current ticks from the forward tracking wheel.
     * @param sidewaysTicks Current ticks from the sideways tracking wheel.
     * @param gyroHeading Current heading from the gyro (in radians).
     */
    void update(double forwardTicks, double sidewaysTicks, double gyroHeading) {
        // Convert tracking wheel ticks to inches
        double forwardInches = ticksToInches(forwardTicks);
        double sidewaysInches = ticksToInches(sidewaysTicks);

        // Calculate changes in tracking wheel positions
        double deltaForward = forwardInches - prevForward;
        double deltaSideways = sidewaysInches - prevSideways;

        // Update previous tracking wheel positions
        prevForward = forwardInches;
        prevSideways = sidewaysInches;

        // Calculate change in heading
        double deltaTheta = gyroHeading - prevTheta;
        prevTheta = gyroHeading;

        // Transform local changes into global coordinates
        double avgTheta = theta + (deltaTheta / 2.0); // Average heading during movement
        double globalDeltaX = deltaForward * std::cos(avgTheta) - deltaSideways * std::sin(avgTheta);
        double globalDeltaY = deltaForward * std::sin(avgTheta) + deltaSideways * std::cos(avgTheta);

        // Update global position and heading
        x += globalDeltaX;
        y += globalDeltaY;
        theta = gyroHeading; // Update heading directly from gyro
    }

    /**
     * Gets the robot's current position and heading.
     * @return A tuple containing (x, y, theta).
     */
    std::tuple<double, double, double> getPosition() {
        return {x, y, theta};
    }
};