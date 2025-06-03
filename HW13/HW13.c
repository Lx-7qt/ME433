/**
 * MPU6050 + SSD1306 with Debug Prints
 *
 * - Blinks the on‐board LED (GP25) once per second as a heartbeat.
 * - Prints status/debug lines over USB‐CDC so you can see where things may be failing.
 * - At 10 Hz, reads raw AX/AY/… from MPU6050, prints them, draws a line on the OLED,
 *   and prints “OLED updated” each cycle.
 *
 * Wiring remains:
 *   Pico GPIO8  → SDA (both MPU6050 & SSD1306 share this)
 *   Pico GPIO9  → SCL (both MPU6050 & SSD1306 share this)
 *   MPU6050 VCC → 3.3 V, GND → GND
 *   SSD1306 VCC → 3.3 V, GND → GND
 */

 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include "pico/stdlib.h"
 #include "hardware/i2c.h"
 #include "hardware/gpio.h"
 #include "ssd1306.h"
 
 // I2C configuration
 #define I2C_PORT        i2c0
 #define I2C_SDA_PIN     8
 #define I2C_SCL_PIN     9
 #define I2C_BAUDRATE    400000    // 400 kHz
 
 // MPU6050 address
 #define MPU6050_ADDR    0x68
 
 // MPU6050 Registers
 #define WHO_AM_I        0x75
 #define PWR_MGMT_1      0x6B
 #define ACCEL_CONFIG    0x1C
 #define GYRO_CONFIG     0x1B
 #define ACCEL_XOUT_H    0x3B
 
 // Debug LED (heartbeat)
 #define HEARTBEAT_LED   25
 
 // Scale factor: ±2 g → 1 g = 16384 LSB → 1/16384 ≈ 0.000061
 #define ACCEL_SENSITIVITY 0.000061f
 // On‐screen scale (pixels per g)
 #define PIXELS_PER_G   30.0f
 
 // Center of OLED
 #define CENTER_X 64
 #define CENTER_Y 16
 
 // Buffer for burst‐reading 14 bytes from MPU6050
 static uint8_t mpu_raw[14];
 
 // =================================================================================
 // Utility: write a single byte to an MPU6050 register
 // =================================================================================
 static void write_register(uint8_t reg, uint8_t value) {
     uint8_t buf[2] = {reg, value};
     int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
     if (ret < 0) {
         printf(">> I2C write err @ reg 0x%02X\n", reg);
     }
 }
 
 // =================================================================================
 // Utility: read a single byte from an MPU6050 register
 // =================================================================================
 static uint8_t read_register(uint8_t reg) {
     int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
     if (ret < 0) {
         printf(">> I2C write (set read) err @ reg 0x%02X\n", reg);
         return 0xFF;
     }
     uint8_t val = 0;
     ret = i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, false);
     if (ret < 0) {
         printf(">> I2C read err @ reg 0x%02X\n", reg);
         return 0xFF;
     }
     return val;
 }
 
 // =================================================================================
 // Initialize MPU6050
 //  • Check WHO_AM_I (should be 0x68 or possibly 0x98).
 //  • Write 0x00 to PWR_MGMT_1 to wake up.
 //  • Write 0x00 to ACCEL_CONFIG for ±2 g.
 //  • Write 0x18 to GYRO_CONFIG for ±2000 °/s.
 // =================================================================================
 static bool mpu6050_init() {
     printf("MPU6050: Reading WHO_AM_I...\n");
     uint8_t who = read_register(WHO_AM_I);
     printf("MPU6050: WHO_AM_I = 0x%02X\n", who);
     if (who != 0x68 && who != 0x98) {
         printf("MPU6050: Unexpected WHO_AM_I → halting.\n");
         // Blink heartbeat LED fast to indicate error
         gpio_init(HEARTBEAT_LED);
         gpio_set_dir(HEARTBEAT_LED, GPIO_OUT);
         while (true) {
             gpio_put(HEARTBEAT_LED, 1);
             sleep_ms(100);
             gpio_put(HEARTBEAT_LED, 0);
             sleep_ms(100);
         }
     }
 
     printf("MPU6050: Waking up (PWR_MGMT_1=0x00)...\n");
     write_register(PWR_MGMT_1, 0x00);
     sleep_ms(10);
 
     printf("MPU6050: Setting ACCEL_CONFIG=0x00 (±2 g)...\n");
     write_register(ACCEL_CONFIG, 0x00);
     sleep_ms(10);
 
     printf("MPU6050: Setting GYRO_CONFIG=0x18 (±2000 °/s)...\n");
     write_register(GYRO_CONFIG, 0x18);
     sleep_ms(10);
 
     printf("MPU6050: Initialization complete.\n");
     return true;
 }
 
 // =================================================================================
 // Read raw accelerometer, temperature, gyro (14 bytes) in one burst
 // Combined as signed 16‐bit values.
 // =================================================================================
 static void mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                              int16_t *temp, int16_t *gx, int16_t *gy, int16_t *gz) {
     uint8_t reg = ACCEL_XOUT_H;
     int ret = i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
     if (ret < 0) {
         printf(">> I2C write (start burst) err\n");
     }
     ret = i2c_read_blocking(I2C_PORT, MPU6050_ADDR, mpu_raw, 14, false);
     if (ret < 0) {
         printf(">> I2C read (burst 14 bytes) err\n");
     }
 
     *ax   = (int16_t)((mpu_raw[0] << 8)  | mpu_raw[1]);
     *ay   = (int16_t)((mpu_raw[2] << 8)  | mpu_raw[3]);
     *az   = (int16_t)((mpu_raw[4] << 8)  | mpu_raw[5]);
     *temp = (int16_t)((mpu_raw[6] << 8)  | mpu_raw[7]);
     *gx   = (int16_t)((mpu_raw[8] << 8)  | mpu_raw[9]);
     *gy   = (int16_t)((mpu_raw[10] << 8) | mpu_raw[11]);
     *gz   = (int16_t)((mpu_raw[12] << 8) | mpu_raw[13]);
 }
 
 // =================================================================================
 // Simple Bresenham line for the OLED
 // =================================================================================
 static void drawLine(int x0, int y0, int x1, int y1) {
     int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
     int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
     int err = dx + dy;
     while (true) {
         ssd1306_drawPixel(x0, y0, 1);
         if (x0 == x1 && y0 == y1) break;
         int e2 = 2 * err;
         if (e2 >= dy) {
             err += dy;
             x0 += sx;
         }
         if (e2 <= dx) {
             err += dx;
             y0 += sy;
         }
     }
 }
 
 // =================================================================================
 // main()
 // =================================================================================
 int main() {
     stdio_init_all();
     // Wait for USB‐CDC to connect before printing
     while (!stdio_usb_connected()) {
         sleep_ms(100);
     }
     printf("=== MPU6050 + SSD1306 Debug Sketch ===\n");
 
     // 1) Initialize heartbeat LED on GPIO25
     gpio_init(HEARTBEAT_LED);
     gpio_set_dir(HEARTBEAT_LED, GPIO_OUT);
 
     // 2) Initialize I2C
     printf("I2C: Initializing on SDA=GPIO%u, SCL=GPIO%u @ %u Hz\n",
            I2C_SDA_PIN, I2C_SCL_PIN, I2C_BAUDRATE);
     i2c_init(I2C_PORT, I2C_BAUDRATE);
     gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
     gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
     gpio_pull_up(I2C_SDA_PIN);
     gpio_pull_up(I2C_SCL_PIN);
     printf("I2C: Done.\n");
 
     // 3) Initialize SSD1306
     printf("OLED: Running ssd1306_setup()...\n");
     ssd1306_setup();
     ssd1306_clear();
     ssd1306_update();
     printf("OLED: Cleared and updated.\n");
 
     // 4) Initialize MPU6050
     printf("MPU6050: Calling init...\n");
     bool ok = mpu6050_init();
     if (!ok) {
         printf("MPU6050: init() returned false. Halting.\n");
         while (true) {
             tight_loop_contents();
         }
     }
 
     // 5) Variables for raw readings
     int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
     int16_t raw_temp = 0, raw_gx = 0, raw_gy = 0, raw_gz = 0;
 
     // 6) Precompute OLED center
     const int cx = CENTER_X, cy = CENTER_Y;
 
     // 7) Main loop at ~10 Hz (every 100 ms)
     absolute_time_t next_frame = make_timeout_time_ms(0);
     while (true) {
         // 7.a) Heartbeat toggle once per second
         static uint32_t last_heartbeat = 0;
         uint32_t now_ms = to_ms_since_boot(get_absolute_time());
         if (now_ms - last_heartbeat >= 500) {
             gpio_put(HEARTBEAT_LED, (now_ms / 500) & 1);
             last_heartbeat = now_ms;
         }
 
         // 7.b) Read raw data from MPU6050
         mpu6050_read_raw(&raw_ax, &raw_ay, &raw_az,
                          &raw_temp, &raw_gx, &raw_gy, &raw_gz);
         printf("Raw AX=%6d AY=%6d AZ=%6d  T=%6d  GX=%6d GY=%6d GZ=%6d\n",
                raw_ax, raw_ay, raw_az, raw_temp, raw_gx, raw_gy, raw_gz);
 
         // 7.c) Convert to g
         float ax_g = raw_ax * ACCEL_SENSITIVITY;
         float ay_g = raw_ay * ACCEL_SENSITIVITY;
         printf("Accel: X=%.3f g  Y=%.3f g\n", ax_g, ay_g);
 
         // 7.d) Map to pixel offsets
         int px = cx + (int)(ax_g * PIXELS_PER_G);
         int py = cy + (int)(ay_g * PIXELS_PER_G);
         if (px < 0)   px = 0;
         if (px > 127) px = 127;
         if (py < 0)   py = 0;
         if (py > 31)  py = 31;
         printf("Mapped endpoint: (%d, %d)\n", px, py);
 
         // 7.e) Clear OLED buffer
         ssd1306_clear();
 
         // 7.f) Draw line from center to (px,py)
         drawLine(cx, cy, px, py);
 
         // 7.g) Update OLED
         ssd1306_update();
         printf("OLED: updated frame.\n\n");
 
         // 7.h) Wait until next 100 ms tick
         next_frame = delayed_by_ms(next_frame, 100);
         sleep_until(next_frame);
     }
 
     return 0;
 }
 