 #include <stdio.h>
 #include <stdlib.h>
 #include <math.h>
 #include "pico/stdlib.h"
 #include "hardware/pwm.h"
 #include "hardware/regs/clocks.h"
 #include "hardware/clocks.h"
 #include "hardware/pio.h"
 #include "ws2812.pio.h"
 
 
 #define IS_RGBW false
 #define NUM_PIXELS 4
 
 #ifdef PICO_DEFAULT_WS2812_PIN
   #define WS2812_PIN PICO_DEFAULT_WS2812_PIN
 #else
   #define WS2812_PIN 2
 #endif
 
 // Put a 24-bit word into the PIO FIFO (GRB order)
 static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
     pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
 }
 
 // Pack RGB into a 32-bit word for WS2812 protocol (GRB ordering)
 static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
     return
         ((uint32_t)(r) << 8)  |  // R → bits [15:8]
         ((uint32_t)(g) << 16) |  // G → bits [23:16]
         (uint32_t)(b);           // B → bits [7:0]
 }
 
 // Simple HSB→RGB conversion (hue: 0–360, sat: 0–1, brightness: 0–1)
 typedef struct {
     uint8_t r;
     uint8_t g;
     uint8_t b;
 } wsColor;
 
 wsColor HSBtoRGB(float hue, float sat, float brightness) {
     float red = 0.0f, green = 0.0f, blue = 0.0f;
     if (sat <= 0.0f) {
         // Grayscale
         red = brightness;
         green = brightness;
         blue = brightness;
     } else {
         if (hue >= 360.0f) hue = 0.0f;
         float h = hue / 60.0f;
         int   slice = (int)h;
         float frac  = h - (float)slice;
         float p = brightness * (1.0f - sat);
         float q = brightness * (1.0f - sat * frac);
         float t = brightness * (1.0f - sat * (1.0f - frac));
         switch (slice) {
             case 0: red = brightness; green = t;       blue = p;       break;
             case 1: red = q;         green = brightness; blue = p;       break;
             case 2: red = p;         green = brightness; blue = t;       break;
             case 3: red = p;         green = q;         blue = brightness; break;
             case 4: red = t;         green = p;         blue = brightness; break;
             case 5: red = brightness; green = p;         blue = q;       break;
             default: red = 0.0f;     green = 0.0f;      blue = 0.0f;    break;
         }
     }
     wsColor c;
     c.r = (uint8_t)(red * 255.0f);
     c.g = (uint8_t)(green * 255.0f);
     c.b = (uint8_t)(blue * 255.0f);
     return c;
 }
 
 // Initialize the PIO state machine for WS2812
 static PIO ws_pio;
 static uint ws_sm;
 static uint ws_offset;
 
 void init_ws2812() {
     bool ok = pio_claim_free_sm_and_add_program_for_gpio_range(
         &ws2812_program, &ws_pio, &ws_sm, &ws_offset, WS2812_PIN, 1, true
     );
     if (!ok) {
         printf("PIO allocation for WS2812 failed!\n");
         while (1) tight_loop_contents();
     }
     ws2812_program_init(ws_pio, ws_sm, ws_offset, WS2812_PIN, 800000, IS_RGBW);
 }
 
 
 #define SERVO_PIN 15
 
 // Servo PWM parameters (50 Hz)
 #define SERVO_FREQ_HZ    50.0f
 #define SERVO_PERIOD_US  20000u   // 1/50 Hz = 20 ms = 20000 µs
 #define SERVO_MIN_US     500u     // 0° → 0.5 ms pulse
 #define SERVO_MAX_US     2500u    // 180° → 2.5 ms pulse
 
 static uint servo_slice;
 static uint16_t servo_wrap;
 static float    servo_clkdiv;
 
 // Initialize the PWM slice for SERVO_PIN at 50 Hz
 void servo_init() {
     // Assign SERVO_PIN as PWM
     gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
     servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);
 
     // Compute divider & wrap so that F_pwm = 50 Hz
     //
     // F_sys = 150 MHz. We want F_pwm = 50 Hz → period = 20 ms.
     // The PWM counter rolls over at (wrap+1). Effective PWM frequency:
     //   F_pwm = F_sys / clkdiv / (wrap + 1)
     // → wrap = (F_sys / clkdiv / F_pwm) - 1
     //
     // We pick clkdiv = 150.0f → F_sys/clkdiv = 1 MHz. Then wrap+1 = 1 MHz / 50 = 20000 → wrap = 19999.
     // 19999 < 65535 → valid. This gives us a 1 µs resolution roughly.
     servo_clkdiv = 150.0f;
     pwm_set_clkdiv(servo_slice, servo_clkdiv);
 
     servo_wrap = 20000 - 1; // count from 0..19999 → 20000 ticks = 20 ms
     pwm_set_wrap(servo_slice, servo_wrap);
 
     pwm_set_enabled(servo_slice, true);
 }
 
 // Set servo angle (0.0–180.0). Maps to pulse width 0.5 ms–2.5 ms.
 void servo_set_angle(float angle_deg) {
     if (angle_deg < 0.0f) angle_deg = 0.0f;
     if (angle_deg > 180.0f) angle_deg = 180.0f;
     // Linear interp: pulse_us = SERVO_MIN_US + (angle/180)*(SERVO_MAX_US - SERVO_MIN_US)
     float pulse_us = (float)SERVO_MIN_US + (angle_deg / 180.0f) * ((float)SERVO_MAX_US - (float)SERVO_MIN_US);
     // Convert pulse_us to a PWM compare value: level = (pulse_us / period_us) * (wrap+1)
     uint32_t level = (uint32_t)((pulse_us / (float)SERVO_PERIOD_US) * (float)(servo_wrap + 1));
     pwm_set_gpio_level(SERVO_PIN, level);
 }
 
 // ─────────────────────────────────────────────────────────────────────
 
 int main() {
     stdio_init_all();
     sleep_ms(2000); // allow USB serial to initialize
 
     printf("Initializing WS2812 and Servo...\n");
 
     // 1) Initialize WS2812 PIO + state machine
     init_ws2812();
 
     // 2) Initialize Servo PWM
     servo_init();
 
     // Variables for animation
     const int STEPS = 500;            // 500 steps → 500 * 10 ms = 5000 ms = 5 s
     const int DELAY_MS = 10;          // 10 ms per step
     const float  HUE_INCREMENT = 360.0f / (float)STEPS;   // Hue step per iteration
     const float  ANGLE_INCREMENT = 180.0f / (float)STEPS; // Angle step per iteration
 
     printf("Entering main loop. Cycling servo 0→180° in 5s and WS2812 rainbow.\n");
 
     while (true) {
         float hue_base = 0.0f;
         float angle    = 0.0f;
 
         for (int i = 0; i < STEPS; i++) {
             // → Compute this step’s servo angle
             angle = (float)i * ANGLE_INCREMENT;
             servo_set_angle(angle);
 
             // → Compute base hue for this frame
             hue_base = fmodf((float)i * HUE_INCREMENT, 360.0f);
 
             // → Drive each of the 4 LEDs, offset by 360/4 = 90° in hue
             for (int led = 0; led < NUM_PIXELS; led++) {
                 float hue_led = hue_base + (float)led * (360.0f / (float)NUM_PIXELS);
                 if (hue_led >= 360.0f) hue_led -= 360.0f;
 
                 wsColor col = HSBtoRGB(hue_led, 1.0f /*full sat*/, 0.2f /*moderate brightness*/);
                 put_pixel(ws_pio, ws_sm, urgb_u32(col.r, col.g, col.b));
             }
 
             // Wait 10 ms until next step
             sleep_ms(DELAY_MS);
         }
 
         // After 5 s, loop repeats: servo jumps back to 0°, rainbow resets
     }
 
     return 0;
 }
 