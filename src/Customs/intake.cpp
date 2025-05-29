#include "main.h"
#include "pros/motors.hpp"

class Intake {
private:
    pros::MotorGroup intakeMotors;

public:
    /**
     * Constructor for the Intake class.
     * @param motorPorts A vector of motor ports for the intake motors.
     */
    Intake(const std::vector<int>& motorPorts) : intakeMotors({motorPorts.begin(), motorPorts.end()}) {}

    /**
     * Controls the intake motors using the controller bumpers.
     * @param controller The controller object.
     */
    void control(pros::Controller& controller) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeMotors.move_velocity(600); // Intake in
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeMotors.move_velocity(-600); // Intake out
        } else {
            intakeMotors.move_velocity(0); // Stop intake
        }

        
    }
};

