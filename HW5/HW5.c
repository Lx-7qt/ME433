 #include <stdio.h>
 #include <math.h>
 #include <inttypes.h>
 #include "pico/stdlib.h"
 #include "hardware/spi.h"
 
 // === SPI‐and‐GPIO pin definitions ===
 #define SPI_PORT        spi0
 #define PIN_MISO        16      // SPI0 RX
 #define PIN_SCK         18      // SPI0 SCK
 #define PIN_MOSI        19      // SPI0 TX
 #define PIN_DAC_CS      17      // Chip select for MCP4912
 #define PIN_SRAM_CS     20      // Chip select for 23K256 SPI SRAM
 
 // === Constants ===
 #define CPU_FREQ_HZ     150000000UL   // RP2350 core clock: 150 MHz
 #define CYCLES_PER_US   (CPU_FREQ_HZ / 1000000UL)  // 150 cycles per μs
 
 // At 1 ms between samples, 1000 samples produce a 1 Hz sine wave.
 #define NUM_SAMPLES     1000
 #define SAMPLE_DELAY_US 1000        // 1000 µs = 1 ms = 1/sample kHz
 
 // === Utility: CS assert/deassert ===
 static inline void cs_select(uint cs_pin) {
     asm volatile("nop \n nop \n nop");
     gpio_put(cs_pin, 0);
     asm volatile("nop \n nop \n nop");
 }
 
 static inline void cs_deselect(uint cs_pin) {
     asm volatile("nop \n nop \n nop");
     gpio_put(cs_pin, 1);
     asm volatile("nop \n nop \n nop");
 }
 
 // === Benchmarking floating‐point operations ===
 static void benchmark_fpu(void) {
     volatile float f1 = 0.0f, f2 = 0.0f;
     volatile float f_add = 0.0f, f_sub = 0.0f, f_mul = 0.0f, f_div = 0.0f;
 
     // Prompt for two floats
     printf("Enter two floats to benchmark: ");
     fflush(stdout);
     // Block until user enters two floats
     scanf("%f %f", (float *)&f1, (float *)&f2);
 
     // 1) Addition benchmark
     uint64_t t0 = to_us_since_boot(get_absolute_time());
     for (int i = 0; i < 1000; i++) {
         f_add = f1 + f2;
     }
     uint64_t t1 = to_us_since_boot(get_absolute_time());
     uint64_t delta_us_add = t1 - t0;
     uint64_t cycles_add_total = delta_us_add * CYCLES_PER_US; // total cycles for 1000 ops
     uint64_t avg_cycles_add = cycles_add_total / NUM_SAMPLES;
 
     // 2) Subtraction
     t0 = to_us_since_boot(get_absolute_time());
     for (int i = 0; i < 1000; i++) {
         f_sub = f1 - f2;
     }
     t1 = to_us_since_boot(get_absolute_time());
     uint64_t delta_us_sub = t1 - t0;
     uint64_t avg_cycles_sub = (delta_us_sub * CYCLES_PER_US) / NUM_SAMPLES;
 
     // 3) Multiplication
     t0 = to_us_since_boot(get_absolute_time());
     for (int i = 0; i < 1000; i++) {
         f_mul = f1 * f2;
     }
     t1 = to_us_since_boot(get_absolute_time());
     uint64_t delta_us_mul = t1 - t0;
     uint64_t avg_cycles_mul = (delta_us_mul * CYCLES_PER_US) / NUM_SAMPLES;
 
     // 4) Division
     t0 = to_us_since_boot(get_absolute_time());
     for (int i = 0; i < 1000; i++) {
         f_div = f1 / f2;
     }
     t1 = to_us_since_boot(get_absolute_time());
     uint64_t delta_us_div = t1 - t0;
     uint64_t avg_cycles_div = (delta_us_div * CYCLES_PER_US) / NUM_SAMPLES;
 
     // Print results
     printf("\nFloating‐Point Benchmark (avg cycles/op):\n");
     printf("  Add: %llu cycles  (total %llu us for 1000 ops)\n",
            (unsigned long long)avg_cycles_add, (unsigned long long)delta_us_add);
     printf("  Sub: %llu cycles  (total %llu us for 1000 ops)\n",
            (unsigned long long)avg_cycles_sub, (unsigned long long)delta_us_sub);
     printf("  Mul: %llu cycles  (total %llu us for 1000 ops)\n",
            (unsigned long long)avg_cycles_mul, (unsigned long long)delta_us_mul);
     printf("  Div: %llu cycles  (total %llu us for 1000 ops)\n\n",
            (unsigned long long)avg_cycles_div, (unsigned long long)delta_us_div);
 
     // Use results minimally so optimizer can’t remove operations
     if (f_add + f_sub + f_mul + f_div == 0.0f) {
         printf("Impossible!\n");
     }
 }
 
 // === MCP4912 (10-bit SPI DAC) functions ===
 // We will only use DAC channel A (channel=0). BUF=1, GA=1, SHDN=1 .
 static void dac_write(uint8_t channel, uint16_t value10bit) {
     // Clip to 0..1023
     uint16_t data10 = (value10bit & 0x03FF) << 2; // place bits into D9..D0 → bits11..2
 
     // Control bits for channel A vs B (bits15..12):
     //   channel=0 (DAC_A):  bit15..12 = 0 1 1 1 → 0x7 << 12 = 0x7000
     //   channel=1 (DAC_B):  bit15..12 = 1 1 1 1 → 0xF << 12 = 0xF000
     uint16_t command = (channel == 0 ? 0x7000 : 0xF000) | data10;
 
     // Split high/low byte
     uint8_t buf[2];
     buf[0] = (uint8_t)((command >> 8) & 0xFF);
     buf[1] = (uint8_t)( command        & 0xFF);
 
     cs_select(PIN_DAC_CS);
     spi_write_blocking(SPI_PORT, buf, 2);
     cs_deselect(PIN_DAC_CS);
 }
 
 // === 23K256 SPI SRAM functions ===
 // Commands:
 #define SRAM_CMD_WRITE        0x02
 #define SRAM_CMD_READ         0x03
 #define SRAM_CMD_RDMODE       0x01
 // Mode bytes:
 #define SRAM_MODE_BYTE        0x40    // 0b0100_0000 → SEQUENTIAL mode (wrap at 32768)
 
 static void sram_init(void) {
     // Put 23K256 into SEQUENTIAL mode:
     cs_select(PIN_SRAM_CS);
     uint8_t tx[2] = { SRAM_CMD_RDMODE, SRAM_MODE_BYTE };
     spi_write_blocking(SPI_PORT, tx, 2);
     cs_deselect(PIN_SRAM_CS);
 }
 
 static void sram_write_bytes(uint16_t address, const uint8_t *data, size_t len) {
     // In SEQUENTIAL mode, address wraps through full 0x0000..0x7FFF
     uint8_t tx[3];
     tx[0] = SRAM_CMD_WRITE;
     tx[1] = (uint8_t)((address >> 8) & 0xFF);
     tx[2] = (uint8_t)( address        & 0xFF);
 
     cs_select(PIN_SRAM_CS);
     spi_write_blocking(SPI_PORT, tx, 3);
     spi_write_blocking(SPI_PORT, data, len);
     cs_deselect(PIN_SRAM_CS);
 }
 
 static void sram_read_bytes(uint16_t address, uint8_t *out_buf, size_t len) {
     uint8_t tx[3];
     tx[0] = SRAM_CMD_READ;
     tx[1] = (uint8_t)((address >> 8) & 0xFF);
     tx[2] = (uint8_t)( address        & 0xFF);
 
     cs_select(PIN_SRAM_CS);
     spi_write_blocking(SPI_PORT, tx, 3);
     // Clock out 'len' bytes by sending dummy 0x00 and reading back
     spi_read_blocking(SPI_PORT, 0x00, out_buf, len);
     cs_deselect(PIN_SRAM_CS);
 }
 
 // Write a single 32-bit float into SRAM at byte‐address "addr".
 static void sram_write_float(uint16_t addr, float f) {
     // Reinterpret float as uint32_t bit‐pattern
     union {
         float    f;
         uint32_t u;
     } conv;
     conv.f = f;
 
     // Break 32-bit into 4 bytes, MSB first
     uint8_t bytes[4];
     bytes[0] = (uint8_t)((conv.u >> 24) & 0xFF);
     bytes[1] = (uint8_t)((conv.u >> 16) & 0xFF);
     bytes[2] = (uint8_t)((conv.u >>  8) & 0xFF);
     bytes[3] = (uint8_t)( conv.u        & 0xFF);
 
     sram_write_bytes(addr, bytes, 4);
 }
 
 // Read a single 32-bit float from SRAM at byte‐address "addr".
 static float sram_read_float(uint16_t addr) {
     uint8_t bytes[4];
     sram_read_bytes(addr, bytes, 4);
 
     // Reassemble into uint32_t
     uint32_t u = ((uint32_t)bytes[0] << 24)
                | ((uint32_t)bytes[1] << 16)
                | ((uint32_t)bytes[2] <<  8)
                |  (uint32_t)bytes[3];
 
     union {
         uint32_t u;
         float    f;
     } conv;
     conv.u = u;
     return conv.f;
 }
 
 int main() {
     stdio_init_all();
     while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
     /*******************************************************
      * Part 1: Benchmark FPU operations
      *******************************************************/
     benchmark_fpu();
     // Pause briefly so user can see text before DAC begins toggling
     sleep_ms(1000);
 
     /*******************************************************
      * Part 2: SPI + External SRAM + MCP4912 DAC (HW5)
      *******************************************************/
     // Initialize SPI0 at 1 MHz
     
     spi_init(SPI_PORT, 1000 * 1000);
     gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
     gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
     gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
 
     // Configure chip selects
     gpio_set_function(PIN_DAC_CS,   GPIO_FUNC_SIO);
     gpio_set_function(PIN_SRAM_CS,  GPIO_FUNC_SIO);
     gpio_set_dir(PIN_DAC_CS,   GPIO_OUT);
     gpio_set_dir(PIN_SRAM_CS,  GPIO_OUT);
     // Deselect both by driving high
     gpio_put(PIN_DAC_CS,  1);
     gpio_put(PIN_SRAM_CS, 1);
 
     // 1) Initialize 23K256 into SEQUENTIAL mode
     sram_init();
 
     // 2) Precompute 1000 float samples for one 1 Hz sine cycle → 0..3.3 V
     //    Store each float into SRAM at address = index * 4
     for (int i = 0; i < NUM_SAMPLES; i++) {
         float angle = 2.0f * (float)M_PI * (float)i / (float)NUM_SAMPLES;
         // Map sin ∈ [–1..1] → [0..1], then scale to [0..3.3]
         float v = (sinf(angle) + 1.0f) * 0.5f * 3.3f;
         uint16_t byte_addr = (uint16_t)(i * 4);
         sram_write_float(byte_addr, v);
         // Tiny delay so SPI bus doesn’t get overwhelmed
         sleep_us(10);
     }
 
     // 3) Enter infinite loop: read one float/sample every 1 ms, send to DAC, wrap around
     int read_index = 0;
     while (true) {
         uint16_t byte_addr = (uint16_t)(read_index * 4);
         float sample_v = sram_read_float(byte_addr);
 
         // Convert sample_v ∈ [0..3.3] to 10-bit DAC code [0..1023]:
         // value10bit = round((sample_v / 3.3) * 1023)
         if (sample_v < 0.0f) sample_v = 0.0f;
         if (sample_v > 3.3f) sample_v = 3.3f;
         uint16_t code10 = (uint16_t)((sample_v / 3.3f) * 1023.0f + 0.5f);
 
         // Output via MCP4912 on channel A (channel=0)
         dac_write(0, code10);
 
         // Advance index, wrap at NUM_SAMPLES
         read_index = (read_index + 1) % NUM_SAMPLES;
 
         // 1 ms delay → drives 1000 samples into one second = 1 Hz sine
         sleep_us(SAMPLE_DELAY_US);
     }
 
     return 0;
 }
 