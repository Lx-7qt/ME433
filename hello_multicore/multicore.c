/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#define FLAG_VALUE 123
// ---------------------------------------------------------------------------
// Command codes sent over FIFO
// ---------------------------------------------------------------------------
#define CMD_READ_ADC    10    // Core 0 → Core 1: Read ADC0 and return raw value
#define CMD_LED_ON      11    // Core 0 → Core 1: Turn on LED (GP15)
#define CMD_LED_OFF     12    // Core 0 → Core 1: Turn off LED (GP15)

// A simple ACK value for LED commands:
#define ACK_OK          0x5555AAAA

// A READY value for the initial handshake:
#define HANDSHAKE_READY 0xA5A5A5A5

// ---------------------------------------------------------------------------
// Pins & ADC
// ---------------------------------------------------------------------------
// LED pin on Core 1:
#define LED_PIN         15

// ADC0 pin on Core 1 (GPIO26):
#define ADC_GPIO_PIN    26

void core1_entry() {
    // 1) Initialize GPIO15 as output for LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);  // start LED off

    // 2) Initialize ADC0 (GPIO26)
    adc_init();
    adc_gpio_init(ADC_GPIO_PIN);  // configure GPIO26 for ADC
    adc_select_input(0);          // ADC0 → GPIO26

    // 3) Send handshake READY to Core 0
    multicore_fifo_push_blocking(HANDSHAKE_READY);

    // 4) Main loop: wait for commands from Core 0
    while (true) {
        uint32_t cmd = multicore_fifo_pop_blocking();
        switch (cmd) {
            case CMD_READ_ADC: {
                // Read raw 12-bit ADC value
                uint16_t raw = adc_read(); // 0..4095
                // Send raw reading back to Core 0 (fits in 32-bit FIFO)
                multicore_fifo_push_blocking((uint32_t)raw);
                break;
            }
            case CMD_LED_ON:
                gpio_put(LED_PIN, 1);
                multicore_fifo_push_blocking(ACK_OK);
                break;
            case CMD_LED_OFF:
                gpio_put(LED_PIN, 0);
                multicore_fifo_push_blocking(ACK_OK);
                break;
            default:
                // Unknown command: ignore or optionally send an error code
                multicore_fifo_push_blocking(0xDEADBEEF);
                break;
        }
    }
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Hello, multicore!\n");

    /// \tag::setup_multicore[]

    // Launch Core 1
    multicore_launch_core1(core1_entry);

    // Wait for handshake from Core 1
    uint32_t h = multicore_fifo_pop_blocking();
    if (h != HANDSHAKE_READY) {
        printf("Core 0: Handshake failed, got 0x%08X instead\n", h);
        while (true) {
            tight_loop_contents();
        }
    }
    // Send handshake back (not strictly needed, but confirms both are alive)
    multicore_fifo_push_blocking(HANDSHAKE_READY);
    printf("Core 0: Handshake complete. Enter commands:\n");
    printf("   '0' → read ADC0 voltage\n");
    printf("   '1' → turn ON LED on GPIO15\n");
    printf("   '2' → turn OFF LED on GPIO15\n");

    // Main loop: read user input from USB, send commands to Core 1, wait for response
    while (true) {
        // Prompt user
        printf("\nEnter 0, 1, or 2: ");
        fflush(stdout);

        // Read a single character (blocking) via getchar_timeout_us(-1)
        int ch_int = getchar_timeout_us(-1);
        if (ch_int < 0) {
            // Should never happen when blocking with -1; skip iteration
            continue;
        }
        char c = (char)ch_int;
        // Skip CR / LF if needed
        if (c == '\r' || c == '\n') {
            continue;
        }
        printf("%c\n", c);

        switch (c) {
            case '0': {
                // Command: read ADC0
                multicore_fifo_push_blocking(CMD_READ_ADC);
                // Wait for raw ADC result
                uint32_t raw32 = multicore_fifo_pop_blocking();
                uint16_t raw = (uint16_t)(raw32 & 0xFFFF);
                printf("ADC0 = %5u V\n", raw);
                break;
            }
            case '1': {
                // Command: turn LED ON
                multicore_fifo_push_blocking(CMD_LED_ON);
                uint32_t ack = multicore_fifo_pop_blocking();
                if (ack == ACK_OK) {
                    printf("LED on GPIO15 is now ON\n");
                } else {
                    printf("Unexpected ACK 0x%08X\n", ack);
                }
                break;
            }
            case '2': {
                // Command: turn LED OFF
                multicore_fifo_push_blocking(CMD_LED_OFF);
                uint32_t ack = multicore_fifo_pop_blocking();
                if (ack == ACK_OK) {
                    printf("LED on GPIO15 is now OFF\n");
                } else {
                    printf("Unexpected ACK 0x%08X\n", ack);
                }
                break;
            }
            default:
                printf("Invalid input '%c'. Please enter 0, 1, or 2.\n", c);
                break;
        }
    }

    return 0;
    }



