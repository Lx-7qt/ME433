#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "ssd1306.h"
#include "font.h"
#include "pico/binary_info.h"

// I2C defines for SSD1306 (128×32 OLED over I2C)
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9
#define I2C_BAUD        400000   // 400 kHz

// ADC0 is on GPIO26 (ADC input 0)
#define ADC_GPIO_PIN         26
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
// Helper prototypes
void drawChar(int x, int y, char c);
void drawString(int x, int y, const char *s);

int main() {
    stdio_init_all();

    // 1) Initialize I2C0 for SSD1306
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);


    // Initialize heartbeat LED (on-board)
    int rc = pico_led_init();
    if (rc != PICO_OK) {
        printf("LED init failed (%d)\n", rc);
        return 1;
    }

    // 3) Initialize ADC0 (on GPIO26)
    adc_init();
    adc_gpio_init(ADC_GPIO_PIN);
    adc_select_input(0);  // ADC0 → GPIO26

    // 4) Initialize the SSD1306
    ssd1306_setup();     // this issues the sequence of commands to turn on the display
    ssd1306_clear();
    ssd1306_update();

    // 5) Main loop: blink LED+pixel at 1 Hz, read ADC, compute FPS, draw text, update display
    const uint64_t BLINK_INTERVAL_US = 500000; // toggle every 500 ms → 1 Hz blink
    uint64_t last_blink_time = to_us_since_boot(get_absolute_time());
    bool heartbeat_state = false;

    while (true) {
        // --- 5.a) Check if it’s time to toggle heartbeat (LED + a pixel at (0,0)) ---
        uint64_t now_us = to_us_since_boot(get_absolute_time());
        if (now_us - last_blink_time >= BLINK_INTERVAL_US) {
            heartbeat_state = !heartbeat_state;
            pico_set_led(heartbeat_state);
            // also toggle pixel at (0,0) in the frame buffer
            ssd1306_drawPixel(0, 0, heartbeat_state ? 1 : 0);
            // We don’t call update() here yet, because we will update after drawing text too.
            last_blink_time = now_us;
        }

        // --- 5.b) Read ADC0 value and convert to voltage ---
        uint16_t raw = adc_read(); 
        // On Pico: raw is 12-bit (0..4095). Reference voltage is 3.3 V.
        float voltage = (3.3f * (float)raw) / (float)(1 << 12);

        // --- 5.c) Begin timing for FPS measurement ---
        uint64_t t_frame_start = to_us_since_boot(get_absolute_time());

        // --- 5.d) Clear the frame buffer (retain the blinked pixel at (0,0) as toggled above) ---
        // We’ll clear everything except (0,0) if it’s on. 
        // Easiest: call ssd1306_clear() and then re‐draw the blinked pixel if needed.
        bool pixel00_on = heartbeat_state;
        ssd1306_clear();
        if (pixel00_on) {
            ssd1306_drawPixel(0, 0, 1);
        }

        // --- 5.e) Draw ADC voltage text at (x=0, y=0) below the blink pixel (y=8) ---
        char line1[32];
        snprintf(line1, sizeof(line1), "ADC0: %.2f V", voltage);
        drawString(0, 8, line1);

        // --- 5.h) Finally, push the entire buffer up to the display ---
        ssd1306_update();
        // --- 5.f) After drawing ADC text, measure how long it took so far, compute FPS ---
        uint64_t t_mid = to_us_since_boot(get_absolute_time());
        float elapsed_us = (float)(t_mid - t_frame_start);
        float fps = 1e6f / elapsed_us;
                // --- 5.g) Draw FPS at (x=0, y=16) (third text line) ---
                char line2[32];
                snprintf(line2, sizeof(line2), "FPS: %.1f", fps);
                drawString(0, 16, line2);
        ssd1306_update();
        // Loop will continue immediately; text and pixel will be redrawn next iteration
    }

    return 0;
}

//------------------------------------------------------------------------------------------------
// drawChar(x,y,c):
//   Render a single ASCII character ‘c’ (0x20..0x7F) at pixel‐coordinates (x,y).
//   Uses the 5×8 bitmap from font.h. 
//------------------------------------------------------------------------------------------------
void drawChar(int x, int y, char c) {
    if (c < 0x20 || c > 0x7F) {
        // Out of our font range: treat as space
        c = 0x20;
    }
    int index = c - 0x20; // ASCII offset into font table

    // Each character is 5 columns wide, 8 rows tall.
    // We will draw each bit in ASCII[index][col] as a pixel on/off.
    for (int col = 0; col < 5; col++) {
        uint8_t columnBits = ASCII[index][col];
        for (int row = 0; row < 8; row++) {
            // If the bit ‘row’ is set in columnBits, turn on pixel; otherwise off.
            uint8_t pixelOn = (columnBits >> row) & 0x01;
            ssd1306_drawPixel(x + col, y + row, pixelOn);
        }
    }
    // Optionally leave one blank column of spacing:
    for (int row = 0; row < 8; row++) {
        ssd1306_drawPixel(x + 5, y + row, 0);
    }
}

//------------------------------------------------------------------------------------------------
// drawString(x,y,s):
//   Render a null‐terminated string starting at (x,y). Each character is 6 pixels wide
//   (5 for the glyph + 1 pixel padding). 
//------------------------------------------------------------------------------------------------
void drawString(int x, int y, const char *s) {
    int origX = x;
    while (*s) {
        // If we run off the right edge (128px), wrap to next line:
        if (x + 6 > 128) {
            x = origX;
            y += 8;
            if (y + 8 > 32) {
                // No more room on screen
                return;
            }
        }
        drawChar(x, y, *s);
        x += 6; // 5 for glyph + 1px spacing
        s++;
    }
}
