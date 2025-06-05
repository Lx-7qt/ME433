#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define MOTOR_FWD_PIN   16
#define MOTOR_REV_PIN   17

int main() {
    // Initialize stdio so we can use printf() / getchar() over USB‐serial
    stdio_init_all();

    // Configure both pins as PWM outputs
    gpio_set_function(MOTOR_FWD_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_REV_PIN, GPIO_FUNC_PWM);

    // Determine which PWM slice and channel each pin belongs to
    uint slice_fwd = pwm_gpio_to_slice_num(MOTOR_FWD_PIN);
    uint chan_fwd  = pwm_gpio_to_channel(MOTOR_FWD_PIN);
    uint slice_rev = pwm_gpio_to_slice_num(MOTOR_REV_PIN);
    uint chan_rev  = pwm_gpio_to_channel(MOTOR_REV_PIN);

    //set PWM_WRAP = 100, which means “1 count = 1%.”
    pwm_set_wrap(slice_fwd, 100);
    pwm_set_wrap(slice_rev, 100);

    // Start both slices with 0% duty
    pwm_set_chan_level(slice_fwd, chan_fwd, 0);
    pwm_set_chan_level(slice_rev, chan_rev, 0);
    pwm_set_enabled(slice_fwd, true);
    pwm_set_enabled(slice_rev, true);

    int duty = 0; // signed duty cycle in percent

    /*
    printf("\n=== Motor Control (HW16) ===\n");
    printf("Press '+' to increase duty, '-' to decrease duty.\n");
    printf("Duty range is –100..+100 (%%). Negative spins reverse.\n\n");
    printf("Current duty: %d%%\n", duty);*/

    // Main loop: poll getchar() (non‐blocking). Update PWM whenever '+' or '‐' is pressed.
    while (true) {
        int c = getchar_timeout_us(0);  // non‐blocking read, returns PICO_ERROR_TIMEOUT if no char

        if (c == PICO_ERROR_TIMEOUT) {
            // no key pressed—just spin
            tight_loop_contents();
            continue;
        }

        // If we got here, 'c' is some character from USB‐serial
        if (c == '+') {
            if (duty < 100) duty++;
        } else if (c == '-') {
            if (duty > -100) duty--;
        } else {
            // Ignore any other key
            continue;
        }

        // Update PWM outputs based on sign of 'duty'
        if (duty >= 0) {
            // Forward: set forward‐channel = duty%, reverse‐channel = 0%
            pwm_set_chan_level(slice_fwd, chan_fwd, duty);
            pwm_set_chan_level(slice_rev, chan_rev, 0);
        } else {
            // Reverse: forward‐channel = 0%, reverse‐channel = abs(duty)%
            pwm_set_chan_level(slice_fwd, chan_fwd, 0);
            pwm_set_chan_level(slice_rev, chan_rev, -duty);
        }

        // Echo back the new duty
        printf("Duty cycle: %d%%\n", duty);
    }

    return 0;
}
