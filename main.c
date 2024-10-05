#include "bsp/board.h"
#include "bsp/board_api.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include <pico.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUTTON_PRESSED 0
#define BUTTON_RELEASED 1
#define MAX_CAL_SAMPLE 200
#define DEADZONE 5
#define ADC_PIN 26
#define ADC_CHANNEL 0
#define BTN_PIN_1 14
#define BTN_PIN_2 15
#define DEBOUNCE_TIME 2 // milliseconds

enum {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 3000,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
typedef struct {
  uint8_t time;
  bool armed;
} debounce_timer_t;

void reset_debounce_timer(debounce_timer_t *dt);
void arm_debounce_timer(debounce_timer_t *dt);
void disarm_debounce_timer(debounce_timer_t *dt);
void buttons_task(bool *btn_state_1, bool *btn_state_2, debounce_timer_t *dt1,
                  debounce_timer_t *dt2);
void mask_buttons_state(uint32_t *btn_mask, bool btn_state_1, bool btn_state_2);

void led_blinking_task(void);
void hid_task(int8_t mapped, uint32_t btn_mask);
uint16_t update_min(uint16_t read, uint16_t min);
int8_t map_value(uint16_t val, uint16_t min, uint16_t max);
uint16_t update_max(uint16_t read, uint16_t max);

/*------------- MAIN -------------*/
int main(void) {
  // vars
  uint16_t min = UINT16_MAX;
  uint16_t max = 0;
  uint16_t read;
  int8_t mapped;
  uint32_t buttons_mask;
  debounce_timer_t dt1;
  debounce_timer_t dt2;
  bool btn_state1;
  bool btn_state2;
  // inits
  reset_debounce_timer(&dt1);
  reset_debounce_timer(&dt2);
  board_init();
  tusb_init();
  adc_init();
  adc_gpio_init(ADC_PIN);
  adc_select_input(ADC_CHANNEL);
  gpio_init(BTN_PIN_1);
  gpio_pull_up(BTN_PIN_1);
  gpio_init(BTN_PIN_2);
  gpio_pull_up(BTN_PIN_2);

  // main loop
  while (1) {
    // hall sensor pooling
    read = adc_read();
    buttons_task(&btn_state1, &btn_state2, &dt1, &dt2);
    mask_buttons_state(&buttons_mask, btn_state1, btn_state2);
    max = update_max(read, max);
    min = update_min(read, min);
    mapped = map_value(read, min, max);
    // usb tasks and reporting
    tud_task();
    led_blinking_task();
    hid_task(mapped, buttons_mask);
  }

  return 0;
}

// Standard functions for tinyUsb

// Invoked when device is mounted
void tud_mount_cb(void) { blink_interval_ms = BLINK_MOUNTED; }

// Invoked when device is unmounted
void tud_umount_cb(void) { blink_interval_ms = BLINK_NOT_MOUNTED; }

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) { blink_interval_ms = BLINK_MOUNTED; }

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, int8_t mapped,
                            uint32_t btn_mask) {
  // skip if hid is not ready yet
  if (!tud_hid_ready())
    return;
  // use to avoid send multiple consecutive zero report for keyboard
  static bool has_gamepad_key = false;

  hid_gamepad_report_t report = {.x = 0,
                                 .y = 0,
                                 .z = 0,
                                 .rz = 0,
                                 .rx = 0,
                                 .ry = 0,
                                 .hat = 0,
                                 .buttons = 0};

  if (mapped || btn_mask) {
    report.z = mapped;
    report.buttons = btn_mask;
    tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
    has_gamepad_key = true;
  } else {
    report.hat = GAMEPAD_HAT_CENTERED;
    report.buttons = 0;
    if (has_gamepad_key)
      tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
    has_gamepad_key = false;
  }
}

// main HID reporting task
void hid_task(int8_t mapped, uint32_t btn_mask) {
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if (board_millis() - start_ms < interval_ms)
    return; // not enough time
  start_ms += interval_ms;

  // Remote wakeup
  if (tud_suspended()) {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  } else {
    send_hid_report(REPORT_ID_GAMEPAD, mapped, btn_mask);
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
  // TODO not Implemented
  (void)instance;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {
  (void)instance;
}

// BLINKING TASK
void led_blinking_task(void) {
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // blink is disabled
  if (!blink_interval_ms)
    return;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms)
    return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

void read_buttons_state(bool *btn_state_1, bool *btn_state_2) {
  // read brns state
  uint32_t curr_state = gpio_get_all();
  *btn_state_1 = (bool)(curr_state & 1 << BTN_PIN_1);
  *btn_state_2 = (bool)(curr_state & 1 << BTN_PIN_2);
}

void reset_debounce_timer(debounce_timer_t *dt) {
  dt->time = 0;
  dt->armed = false;
}

void arm_debounce_timer(debounce_timer_t *dt) {
  dt->time = board_millis();
  dt->armed = true;
}

void disarm_debounce_timer(debounce_timer_t *dt) { dt->armed = false; }

void debounce_button(bool curr_state, bool *btn_state, debounce_timer_t *dt) {
  if (curr_state != *btn_state && !dt->armed) {
    arm_debounce_timer(dt);
  }
  if (dt->armed) {
    if ((board_millis() - dt->time) > DEBOUNCE_TIME) {
      if (curr_state != *btn_state) {
        *btn_state = curr_state;
      }
      disarm_debounce_timer(dt);
    }
  }
}

void buttons_task(bool *btn_state_1, bool *btn_state_2, debounce_timer_t *dt1,
                  debounce_timer_t *dt2) {
  bool c_state_1;
  bool c_state_2;
  read_buttons_state(&c_state_1, &c_state_2);
  debounce_button(c_state_1, btn_state_1, dt1);
  debounce_button(c_state_2, btn_state_2, dt2);
}

// HELPERS
void mask_buttons_state(uint32_t *btn_mask, bool btn_state_1,
                        bool btn_state_2) {
  *btn_mask &= ~(1 << BTN_PIN_1); // Clear bit for bool1
  *btn_mask &= ~(1 << BTN_PIN_2); // Clear bit for bool2

  // Set the bits if the bools are true
  if (btn_state_1) {
    *btn_mask |= (1 << BTN_PIN_1); // Set bit for bool1
  }
  if (btn_state_2) {
    *btn_mask |= (1 << BTN_PIN_2); // Set bit for bool2
  }
}
uint16_t update_min(uint16_t read, uint16_t min) {
  if (read < min) {
    return read;
  }
  return min;
}

int8_t map_value(uint16_t val, uint16_t min, uint16_t max) {
  int8_t r_min = INT8_MIN;
  int8_t r_max = INT8_MAX;
  int8_t mapped = r_min + ((val - min) * (r_max - r_min) / (max - min));
  return mapped;
}

uint16_t update_max(uint16_t read, uint16_t max) {
  if (read > max) {
    return read;
  }
  return max;
}
