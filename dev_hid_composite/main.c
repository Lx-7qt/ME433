/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include "usb_descriptors.h"
#include "hardware/gpio.h"
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

 #include <stdlib.h>
 #include <stdio.h>
 #include <string.h>
 
 #include "bsp/board_api.h"
 #include "tusb.h"
 
 #include "usb_descriptors.h"
 #include "hardware/gpio.h"
 //--------------------------------------------------------------------+
 // MACRO CONSTANT TYPEDEF PROTYPES
 //--------------------------------------------------------------------+
 // Button pins (wired as active‐LOW with internal pull‐ups)
#define PIN_BTN_UP      2
#define PIN_BTN_DOWN    3
#define PIN_BTN_LEFT    6
#define PIN_BTN_RIGHT   8
#define PIN_BTN_MODE    10

// Indicator LED pin (onboard LED)
#define PIN_LED_MODE    15
#define MOUSE_DELTA     5
 /* Blink pattern
  * - 250 ms  : device not mounted
  * - 1000 ms : device mounted
  * - 2500 ms : device is suspended
  */
 enum  {
   BLINK_NOT_MOUNTED = 250,
   BLINK_MOUNTED = 1000,
   BLINK_SUSPENDED = 2500,
 };
 
 static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
 static bool remote_mode = false;               // false = regular (buttons), true = remote (circle)
static bool last_mode_button_state = false;    // for edge detection on mode‐toggle button
// angle (in radians) for circular movement
static float circle_angle = 0.0f;
 void led_blinking_task(void);
 void hid_task(void);
 
 /*------------- MAIN -------------*/
 int main(void)
 {
   board_init();
   gpio_init(PIN_BTN_UP);
   gpio_set_dir(PIN_BTN_UP, GPIO_IN);
   gpio_pull_up(PIN_BTN_UP);   // active‐LOW
 
   gpio_init(PIN_BTN_DOWN);
   gpio_set_dir(PIN_BTN_DOWN, GPIO_IN);
   gpio_pull_up(PIN_BTN_DOWN);
 
   gpio_init(PIN_BTN_LEFT);
   gpio_set_dir(PIN_BTN_LEFT, GPIO_IN);
   gpio_pull_up(PIN_BTN_LEFT);
 
   gpio_init(PIN_BTN_RIGHT);
   gpio_set_dir(PIN_BTN_RIGHT, GPIO_IN);
   gpio_pull_up(PIN_BTN_RIGHT);
 
   gpio_init(PIN_BTN_MODE);
   gpio_set_dir(PIN_BTN_MODE, GPIO_IN);
   gpio_pull_up(PIN_BTN_MODE);
 
   // Initialize LED pin
   gpio_init(PIN_LED_MODE);
   gpio_set_dir(PIN_LED_MODE, GPIO_OUT);
   gpio_put(PIN_LED_MODE, 0);
   // init device stack on configured roothub port
   tud_init(BOARD_TUD_RHPORT);
 
   if (board_init_after_tusb) {
     board_init_after_tusb();
   }
   
   while (1)
   {
     tud_task(); // tinyusb device task
     led_blinking_task();
 
     hid_task();
   }
 }
 
 //--------------------------------------------------------------------+
 // Device callbacks
 //--------------------------------------------------------------------+
 
 // Invoked when device is mounted
 void tud_mount_cb(void)
 {
   blink_interval_ms = BLINK_MOUNTED;
 }
 
 // Invoked when device is unmounted
 void tud_umount_cb(void)
 {
   blink_interval_ms = BLINK_NOT_MOUNTED;
 }
 
 // Invoked when usb bus is suspended
 // remote_wakeup_en : if host allow us  to perform remote wakeup
 // Within 7ms, device must draw an average of current less than 2.5 mA from bus
 void tud_suspend_cb(bool remote_wakeup_en)
 {
   (void) remote_wakeup_en;
   blink_interval_ms = BLINK_SUSPENDED;
 }
 
 // Invoked when usb bus is resumed
 void tud_resume_cb(void)
 {
   blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
 }
 
 //--------------------------------------------------------------------+
 // USB HID
 //--------------------------------------------------------------------+
 
 static void send_hid_report(uint8_t report_id, uint32_t btn)
 {
   // skip if hid is not ready yet
   if ( !tud_hid_ready() ) return;
 
   switch(report_id)
   {
     case REPORT_ID_KEYBOARD:
     {
       // use to avoid send multiple consecutive zero report for keyboard
       static bool has_keyboard_key = false;
 
       if ( btn )
       {
         uint8_t keycode[6] = { 0 };
         keycode[0] = HID_KEY_A;
 
         tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
         has_keyboard_key = true;
       }else
       {
         // send empty key report if previously has key pressed
         if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
         has_keyboard_key = false;
       }
     }
     break;
 
     case REPORT_ID_MOUSE:
     {
      if (!remote_mode)
      {
        // ---- REGULAR MODE: buttons move the cursor in 4 directions ----
        int8_t dx = 0;
        int8_t dy = 0;

        // Buttons are wired as active-LOW (pulled-up when unpressed)
        if (gpio_get(PIN_BTN_UP) == 0)    { dy -= MOUSE_DELTA; }
        if (gpio_get(PIN_BTN_DOWN) == 0)  { dy += MOUSE_DELTA; }
        if (gpio_get(PIN_BTN_LEFT) == 0)  { dx -= MOUSE_DELTA; }
        if (gpio_get(PIN_BTN_RIGHT) == 0) { dx += MOUSE_DELTA; }

        // If no button is pressed, dx=dy=0 (cursor stands still)
        tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, dx, dy, 0, 0);
      }
      else
      {
        // ---- REMOTE MODE: move in a slow circle ----
        // Use circle_angle (radians) to generate a small dx, dy.
        // Radius = MOUSE_DELTA. Angle increments by a fixed step each report.
        float rad = circle_angle;
        int8_t dx = (int8_t)roundf(MOUSE_DELTA * cosf(rad));
        int8_t dy = (int8_t)roundf(MOUSE_DELTA * sinf(rad));

        tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, dx, dy, 0, 0);

        // advance the angle a bit; wrap around at 2*pi
        circle_angle += 0.1f;               // adjust 0.1 to make circle slower/faster
        if (circle_angle > 2.0f * M_PI) {
          circle_angle -= 2.0f * M_PI;
        }
      }
     }
     break;
 
     case REPORT_ID_CONSUMER_CONTROL:
     {
       // use to avoid send multiple consecutive zero report
       static bool has_consumer_key = false;
 
       if ( btn )
       {
         // volume down
         uint16_t volume_down = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
         tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &volume_down, 2);
         has_consumer_key = true;
       }else
       {
         // send empty key report (release key) if previously has key pressed
         uint16_t empty_key = 0;
         if (has_consumer_key) tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &empty_key, 2);
         has_consumer_key = false;
       }
     }
     break;
 
     case REPORT_ID_GAMEPAD:
     {
       // use to avoid send multiple consecutive zero report for keyboard
       static bool has_gamepad_key = false;
 
       hid_gamepad_report_t report =
       {
         .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
         .hat = 0, .buttons = 0
       };
 
       if ( btn )
       {
         report.hat = GAMEPAD_HAT_UP;
         report.buttons = GAMEPAD_BUTTON_A;
         tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
 
         has_gamepad_key = true;
       }else
       {
         report.hat = GAMEPAD_HAT_CENTERED;
         report.buttons = 0;
         if (has_gamepad_key) tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
         has_gamepad_key = false;
       }
     }
     break;
 
     default: break;
   }
 }
 
 // Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
 // tud_hid_report_complete_cb() is used to send the next report after previous one is complete
 void hid_task(void)
 {
   uint32_t const btn = board_button_read();
 
   const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms ) return;
  start_ms += interval_ms;

  // ---- MODE TOGGLE LOGIC (edge detection) ----
  bool mode_pressed = (gpio_get(PIN_BTN_MODE) == 0); // active-LOW
  if ( mode_pressed && !last_mode_button_state )
  {
    // Rising edge (unpressed->pressed) of the mode button: switch modes
    remote_mode = !remote_mode;
    gpio_put(PIN_LED_MODE, remote_mode ? 1 : 0);
  }
  last_mode_button_state = mode_pressed;

  // If device is suspended and mode button is held, try remote wakeup
  if ( tud_suspended() && mode_pressed )
  {
    tud_remote_wakeup();
  }
  else
  {
    // Send the mouse report (buttons or circle, depending on mode)
    send_hid_report(REPORT_ID_MOUSE, btn);
  }
 }
 
 // Invoked when sent REPORT successfully to host
 // Application can use this to send the next report
 // Note: For composite reports, report[0] is report ID
 void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
 {
   (void) instance;
   (void) len;
 
   uint8_t next_report_id = report[0] + 1u;
 
   if (next_report_id < REPORT_ID_COUNT)
   {
     send_hid_report(next_report_id, board_button_read());
   }
 }
 
 // Invoked when received GET_REPORT control request
 // Application must fill buffer report's content and return its length.
 // Return zero will cause the stack to STALL request
 uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
 {
   // TODO not Implemented
   (void) instance;
   (void) report_id;
   (void) report_type;
   (void) buffer;
   (void) reqlen;
 
   return 0;
 }
 
 // Invoked when received SET_REPORT control request or
 // received data on OUT endpoint ( Report ID = 0, Type = 0 )
 void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
 {
   (void) instance;
 
   if (report_type == HID_REPORT_TYPE_OUTPUT)
   {
     // Set keyboard LED e.g Capslock, Numlock etc...
     if (report_id == REPORT_ID_KEYBOARD)
     {
       // bufsize should be (at least) 1
       if ( bufsize < 1 ) return;
 
       uint8_t const kbd_leds = buffer[0];
 
       if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
       {
         // Capslock On: disable blink, turn led on
         blink_interval_ms = 0;
         board_led_write(true);
       }else
       {
         // Caplocks Off: back to normal blink
         board_led_write(false);
         blink_interval_ms = BLINK_MOUNTED;
       }
     }
   }
 }
 
 //--------------------------------------------------------------------+
 // BLINKING TASK
 //--------------------------------------------------------------------+
 void led_blinking_task(void)
 {
   static uint32_t start_ms = 0;
   static bool led_state = false;
 
   // blink is disabled
   if (!blink_interval_ms) return;
 
   // Blink every interval ms
   if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
   start_ms += blink_interval_ms;
 
   board_led_write(led_state);
   led_state = 1 - led_state; // toggle
 }