#include <pigpio.h>
#include <unistd.h>
#include <iostream>


int main() {
    // Initialize pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialization failed." << std::endl;
        return 1;
    }

    // Define GPIO pin numbers
    int IN1 = 17;   // Motor driver input 1
    int IN2 = 27;   // Motor driver input 2
    int motor_pwm = 18; // Motor driver PWM input

    // Set GPIO pin modes
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(motor_pwm, PI_OUTPUT);

    // Set PWM frequency
    int pwm_frequency = 10000; // 10 kHz
    gpioSetPWMfrequency(motor_pwm, pwm_frequency);

    // Motor control loop
    while (1) {
        // Drive the motor in forward direction
        gpioWrite(IN1, 1);
        gpioWrite(IN2, 0);

        // Set PWM duty cycle (0 to 255, 0% to 100%)
        int pwm_duty_cycle = 128; // Adjust as needed
        gpioPWM(motor_pwm, pwm_duty_cycle);

        // Wait for some time
        usleep(1000000); // 1 second

        // Reverse the motor direction
        gpioWrite(IN1, 0);
        gpioWrite(IN2, 1);

        // Set PWM duty cycle
        gpioPWM(motor_pwm, pwm_duty_cycle);

        // Wait for some time
        usleep(1000000); // 1 second
    }

    // Cleanup
    gpioTerminate();

    return 0;
}
