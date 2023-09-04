#include <pigpiod_if2.h>
#include <unistd.h>
#include <iostream>

int IN1 = 6;
int IN2 = 5;
int motor_pwm = 12; // Motor driver PWM input

int pi;

int main() {
    // Initialize pigpio
    pi = pigpio_start(NULL,NULL);

    // Set GPIO pin modes
    set_mode(pi,IN1, PI_OUTPUT);
    set_mode(pi,IN2, PI_OUTPUT);
    set_mode(pi,motor_pwm, PI_OUTPUT);

    // Set PWM frequency
    int pwm_frequency = 10000; // 10 kHz
    set_PWM_frequency(pi,motor_pwm, pwm_frequency);

    // Motor control loop
    while (1) {
        // Drive the motor in forward direction
        gpio_write(pi,IN1, 1);
        gpio_write(pi,IN2, 0);

        // Set PWM duty cycle (0 to 255, 0% to 100%)
        int pwm_duty_cycle = 128; // Adjust as needed
        set_PWM_dutycycle(pi,motor_pwm, pwm_duty_cycle);

        // Wait for some time
        usleep(700); // 1 second

        gpio_write(pi,IN1, 0);
        usleep(100);

        // Reverse the motor direction
        gpio_write(pi,IN1, 0);
        gpio_write(pi,IN2, 1);

        // Set PWM duty cycle
        set_PWM_dutycycle(pi,motor_pwm, pwm_duty_cycle);

        // Wait for some time
        usleep(700); // 1 second

        gpio_write(pi,IN2, 0);
        usleep(100);
    }

    // Cleanup
    pigpio_stop(pi);

    return 0;
}
