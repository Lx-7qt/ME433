#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// I2C and MCP23008 definitions
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9
#define I2C_BAUDRATE    100000       // 400 kHz

// MCP23008 7-bit base address when A2,A1,A0 = 0,0,0
#define MCP23008_ADDR   0x20

// MCP23008 register addresses (per datasheet)
#define MCP_IODIR       0x00    // I/O Direction register
#define MCP_GPIO        0x09    // GPIO register (read inputs / reflect OLAT)
#define MCP_OLAT        0x0A    // Output Latch register (write outputs)

// Heartbeat settings
#define HEARTBEAT_LED_PIN       25      // On-board LED on Pico
#define HEARTBEAT_INTERVAL_MS   500     // Toggle every 500 ms

// Helper: write a single byte to a MCP23008 register
static void mcp_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t buf[2] = { reg_addr, value };
    int ret = i2c_write_blocking(I2C_PORT, MCP23008_ADDR, buf, 2, false);
    if (ret < 0) {
        // If write fails, hang or print error
        printf("I2C write error: register 0x%02X\n", reg_addr);
        // Optionally, you could handle retries here.
    }
}

// Helper: read a single byte from a MCP23008 register
static uint8_t mcp_read_register(uint8_t reg_addr) {
    uint8_t value = 0;
    // First, tell the MCP23008 which register we want to read:
    int ret = i2c_write_blocking(I2C_PORT, MCP23008_ADDR, &reg_addr, 1, true);
    if (ret < 0) {
        printf("I2C write (to set read reg) error: 0x%02X\n", reg_addr);
        return 0xFF;
    }
    // Then read 1 byte from that register:
    ret = i2c_read_blocking(I2C_PORT, MCP23008_ADDR, &value, 1, false);
    if (ret < 0) {
        printf("I2C read error: reg 0x%02X\n", reg_addr);
        return 0xFF;
    }
    return value;
}
#ifdef CYW43_WL_GPIO_LED_PIN
 #include "pico/cyw43_arch.h"
 #endif
 
 // Set an LED_TYPE variable - 0 is default, 1 is connected to WIFI chip
 // Note that LED_TYPE == 1 is only supported when initially compiled for
 // a board with PICO_CYW43_SUPPORTED (eg pico_w), else the required
 // libraries won't be present
 bi_decl(bi_program_feature_group(0x1111, 0, "LED Configuration"));
 #if defined(PICO_DEFAULT_LED_PIN)
     // the tag and id are not important as picotool filters based on the
     // variable name, so just set them to 0
     bi_decl(bi_ptr_int32(0x1111, 0, LED_TYPE, 0));
     bi_decl(bi_ptr_int32(0x1111, 0, LED_PIN, PICO_DEFAULT_LED_PIN));
 #elif defined(CYW43_WL_GPIO_LED_PIN)
     bi_decl(bi_ptr_int32(0x1111, 0, LED_TYPE, 1));
     bi_decl(bi_ptr_int32(0x1111, 0, LED_PIN, CYW43_WL_GPIO_LED_PIN));
 #else
     bi_decl(bi_ptr_int32(0x1111, 0, LED_TYPE, 0));
     bi_decl(bi_ptr_int32(0x1111, 0, LED_PIN, 25));
 #endif
  // Perform initialisation
  int pico_led_init(void) {
    if (LED_TYPE == 0) {
        // A device like Pico that uses a GPIO for the LED so we can
        // use normal GPIO functionality to turn the led on and off
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        return PICO_OK;
#ifdef CYW43_WL_GPIO_LED_PIN
    } else if (LED_TYPE == 1) {
        // For Pico W devices we need to initialise the driver etc
        return cyw43_arch_init();
#endif
    } else {
        return PICO_ERROR_INVALID_DATA;
    }
}
// Turn the led on or off
void pico_set_led(bool led_on) {
   if (LED_TYPE == 0) {
       // Just set the GPIO on or off
       gpio_put(LED_PIN, led_on);
#ifdef CYW43_WL_GPIO_LED_PIN
   } else if (LED_TYPE == 1) {
       // Ask the wifi "driver" to set the GPIO on or off
       cyw43_arch_gpio_put(LED_PIN, led_on);
#endif
   }
}
int main() {
    stdio_init_all();

    // Initialize I2C0 at 400 kHz
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    // Note: external pull-ups (10 kΩ) must be present on SDA & SCL lines

    // Initialize heartbeat LED (on-board)
    int rc = pico_led_init();
    if (rc != PICO_OK) {
        printf("LED init failed (%d)\n", rc);
        return 1;
    }
    pico_set_led(true);
    // Small delay before talking to MCP, give it time after power-up
    sleep_ms(10);

    // === MCP23008 Initialization ===
    // Set IODIR:
    //   Bit = 1 → corresponding pin is input
    //   Bit = 0 → corresponding pin is output
    // We want GP0 (bit 0) as input (button), GP7 (bit 7) as output (LED).
    // All other bits 1–6: make inputs (we're not using them).
    // So IODIR = 0b0111 1111 = 0x7F
    mcp_write_register(MCP_IODIR, 0x7F);

    // Initialize the outputs latch (GP7 low initially). OLAT bit7 = 0 → LED off
    mcp_write_register(MCP_OLAT, 0x00);

    // === Main Loop ===
    absolute_time_t last_heartbeat = get_absolute_time();
    bool heartbeat_state = false;

    while (true) {
        // 1) Handle heartbeat (toggle Pico LED every HEARTBEAT_INTERVAL_MS)
        absolute_time_t now = get_absolute_time();
        int64_t elapsed_us = absolute_time_diff_us(last_heartbeat, now);
        if (elapsed_us >= HEARTBEAT_INTERVAL_MS * 1000) {
            heartbeat_state = !heartbeat_state;
            pico_set_led(heartbeat_state);
            last_heartbeat = now;
        }

        // 2) Read button state from MCP23008 GP0 (GPIO register bit 0)
        uint8_t gpio_state = mcp_read_register(MCP_GPIO);
        // If bit 0 == 0 → button pressed (external pull-up keeps it high when released)
        bool button_pressed = ((gpio_state & 0x01) == 0);

        // 3) Control GP7 (OLAT bit 7) accordingly
        uint8_t olat_val = 0x00;
        if (button_pressed) {
            olat_val |= (1 << 7);    // set bit 7 → turn LED on
        }
        // else leave bit 7 = 0 → LED off

        mcp_write_register(MCP_OLAT, olat_val);

        // 4) Small delay so we don't hammer I2C bus too hard.
        //    Also keeps button sampling at ~100 Hz.
        sleep_ms(10);
    }

    return 0;
}
