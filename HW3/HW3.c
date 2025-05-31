#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#define BUTTON_PIN  3

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
 
 #ifndef LED_DELAY_MS
 #define LED_DELAY_MS 250
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
void button_callback(uint gpio, uint32_t events) {
    // toggle LED
    pico_set_led(false);
    printf("Enter a number!\n");

}
int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    int rc = pico_led_init();
    if (rc != PICO_OK) {
        printf("LED init failed (%d)\n", rc);
        return 1;
    }
    printf("Start!\n");
    pico_set_led(true);
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_down(BUTTON_PIN);
    adc_init(); // init the adc module
    adc_gpio_init(26); // set ADC0 pin to be adc input instead of GPIO
    adc_select_input(0); // select to read from ADC0
    gpio_set_irq_enabled_with_callback(
        BUTTON_PIN,
        GPIO_IRQ_EDGE_FALL,
        true,
        &button_callback
    );
    while (1) {
        int num;
        scanf("%d", &num);
        for(int j=0;j<num;j++){
            uint16_t result = adc_read();
            printf("%u V\n", result);
        }
        sleep_ms(50);
    }
}