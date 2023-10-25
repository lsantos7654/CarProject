#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <wiringPi.h>

class L298NHardware : public hardware_interface::SystemInterface
{
public:
    L298NHardware()
    {
        // Initialize wiringPi
        wiringPiSetupGpio(); // Using Broadcom GPIO pin numbering
    }

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override
    {
        // Initialize GPIOs for direction control
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
        pinMode(IN3, OUTPUT);
        pinMode(IN4, OUTPUT);
        
        // Initialize PWM channels
        pinMode(ENA, PWM_OUTPUT);
        pinMode(ENB, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetRange(1024);
        pwmSetClock(375);

        return hardware_interface::return_type::OK;
    }

    // ... other methods as in the previous example ...

    hardware_interface::return_type write() override
    {
        // Assuming you have some member variables or handles set up to receive the commands:
        // motor1_speed, motor2_speed, motor1_direction, motor2_direction

        // Set direction
        if(motor1_direction == FORWARD) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
        
        // ... similar code for motor2 ...

        // Set speed
        pwmWrite(ENA, motor1_speed);
        pwmWrite(ENB, motor2_speed);

        return hardware_interface::return_type::OK;
    }

private:
    // GPIO and PWM pin definitions
    const int IN1 = 17;
    const int IN2 = 27;
    const int IN3 = 23;
    const int IN4 = 24;
    const int ENA = 12;
    const int ENB = 13;

    // Definitions for FORWARD and REVERSE
    const int FORWARD = 1;
    const int REVERSE = 0;

    // Member variables to hold command values
    int motor1_speed = 0;
    int motor2_speed = 0;
    int motor1_direction = FORWARD;
    int motor2_direction = FORWARD;
};

