#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Defines
#define SPI_PORT    spi0
#define PIN_MISO    16
#define PIN_CS      17
#define PIN_SCK     18
#define PIN_MOSI    19

// Waveform parameters
#define N_SINE      100     // number of samples per 2 Hz sine cycle (at 200 Hz update rate)
#define N_TRI       200     // number of samples per 1 Hz triangle cycle (at 200 Hz update rate)
#define SAMPLE_RATE 200     // samples per second (i.e. 200 Hz)

static uint16_t sine_table[N_SINE];
static uint16_t tri_table[N_TRI];

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

// Build lookup tables for a 2 Hz sine and a 1 Hz triangle, each scaled to 10-bit (0..1023)
void build_tables() {
    // 2 Hz sine: N_SINE = 100 points
    for (int i = 0; i < N_SINE; i++) {
        float angle = 2.0f * M_PI * i / N_SINE;
        // Map sin ∈ [–1..1] to [0..1]
        float v = (sinf(angle) + 1.0f) * 0.5f;
        // Scale to 10-bit range [0..1023]
        sine_table[i] = (uint16_t)(v * 1023.0f + 0.5f);
    }
    // 1 Hz triangle: N_TRI = 200 points
    for (int i = 0; i < N_TRI; i++) {
        float t = (float)i / N_TRI;  // t ∈ [0..1)
        float v;
        if (t < 0.5f) {
            v = t * 2.0f;       // rising edge (0→1)
        } else {
            v = (1.0f - t) * 2.0f; // falling edge (1→0)
        }
        tri_table[i] = (uint16_t)(v * 1023.0f + 0.5f);
    }
}

/**
 * @brief  Write a 10-bit value into MCP4912, with BUF=1, GA=1, SHDN=1.
 * @param  channel  0 → DAC_A; 1 → DAC_B
 * @param  value    10-bit data (0..1023). We will left-shift it 2 bits so it sits at D9..D0 → bits 11..2.
 *
 * Control-bit layout (bits 15..12):
 *   bit15 = A/B   (0 = write to DAC_A; 1 = write to DAC_B)
 *   bit14 = BUF   (1 = buffered mode)
 *   bit13 = GA    (1 = gain = 1×; 0 = gain = 2×)
 *   bit12 = SHDN  (1 = active; 0 = shutdown)
 *
 * Because we want BUF=1, GA=1, SHDN=1:
 *   - If channel == 0 (DAC_A), bits15..12 = 0 1 1 1 → 0x7 << 12 = 0x7000
 *   - If channel == 1 (DAC_B), bits15..12 = 1 1 1 1 → 0xF << 12 = 0xF000
 *
 * Then bits11..2 = 10-bit data << 2, and bits1..0 = 0.
 */
void dac_write(uint8_t channel, uint16_t value) {
    // Clip to 10 bits and shift left 2
    uint16_t data10 = (value & 0x03FF) << 2;

    uint16_t command;
    if (channel == 0) {
        // DAC_A: 0b0 1 1 1 → 0x7000
        command = 0x7000 | data10;
    } else {
        // DAC_B: 0b1 1 1 1 → 0xF000
        command = 0xF000 | data10;
    }

    // Split 16-bit command into two bytes (MSB first)
    uint8_t buf[2];
    buf[0] = (uint8_t)((command >> 8) & 0xFF);
    buf[1] = (uint8_t)( command        & 0xFF);

    // Assert CS, send both bytes, de-assert CS
    cs_select(PIN_CS);
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect(PIN_CS);
}

int main() {
    stdio_init_all();

    // Initialize SPI0 at 1 MHz
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // CS is active-low. Drive it high to start.
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // Build lookup tables for both waveforms
    build_tables();

    int sin_idx = 0;
    int tri_idx = 0;
    // 200 samples/sec → delay = 1 000 000 / 200 = 5000 µs
    uint32_t delay_us = 1000000 / SAMPLE_RATE;

    while (true) {
        // Output one sample of 2 Hz sine on DAC_A (channel=0)
        dac_write(0, sine_table[sin_idx]);
        // Output one sample of 1 Hz triangle on DAC_B (channel=1)
        dac_write(1, tri_table[tri_idx]);

        // Advance indices (with wrap-around)
        sin_idx = (sin_idx + 1) % N_SINE;
        tri_idx = (tri_idx + 1) % N_TRI;

        // Wait until the next sample time
        sleep_us(delay_us);
    }
    return 0;
}
