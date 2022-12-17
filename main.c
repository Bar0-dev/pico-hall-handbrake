#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "bsp/board.h"
#include "tusb.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "usb_descriptors.h"

#define MAX_CAL_SAMPLE 200
#define DEADZONE 10

enum
{
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 3000,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void hid_task(int8_t mapped);
uint16_t update_min(uint16_t read, uint16_t min);
int8_t map_value(uint16_t val, uint16_t min, uint16_t max);
uint16_t update_max(uint16_t read, uint16_t max);

/*------------- MAIN -------------*/
int main(void)
{
  //vars
  uint16_t min = 0;
  uint16_t max = 0;
  uint16_t read;
  int8_t mapped;
  //inits
  board_init();
  tusb_init();
  adc_init();
  adc_gpio_init(26);
  adc_select_input(0);
  //initial calibration setup
  read = adc_read();
  min = update_min(read, min);
  max = update_max(read, max);

  //main loop
  while (1)
  {
    //hall sensor pooling
    read = adc_read();
    max = update_max(read, max);
    mapped = map_value(read, min, max);
    //usb tasks and reporting
    tud_task();
    led_blinking_task();
    hid_task(mapped);
  }

  return 0;
}

//Standard functions for tinyUsb

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
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

static void send_hid_report(uint8_t report_id, int8_t mapped)
{
  // skip if hid is not ready yet
  if (!tud_hid_ready())
    return;
  // use to avoid send multiple consecutive zero report for keyboard
  static bool has_gamepad_key = false;

  hid_gamepad_report_t report =
      {
          .x = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0, .hat = 0, .buttons = 0};

  if (mapped)
  {
    report.z = mapped;
    tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
    has_gamepad_key = true;
  }
  else
  {
    report.hat = GAMEPAD_HAT_CENTERED;
    report.buttons = 0;
    if (has_gamepad_key)
      tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
    has_gamepad_key = false;
  }
}

//main HID reporting task
void hid_task(int8_t mapped)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if (board_millis() - start_ms < interval_ms)
    return; // not enough time
  start_ms += interval_ms;

  // Remote wakeup
  if (tud_suspended())
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }
  else
  {
    send_hid_report(REPORT_ID_GAMEPAD, mapped);
  }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
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
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
  (void)instance;
}

// BLINKING TASK
void led_blinking_task(void)
{
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

// HELPERS
uint16_t update_min(uint16_t read, uint16_t min)
{
  uint32_t sum = 0;
  for (int i = 0; i < MAX_CAL_SAMPLE; i++)
  {
    sum += read;
  }
  return (sum / MAX_CAL_SAMPLE) + DEADZONE;
}

int8_t map_value(uint16_t val, uint16_t min, uint16_t max)
{
  int8_t r_min = 0;
  int8_t r_max = 127;
  int8_t mapped = r_min + ((val - min) * (r_max - r_min) / (max - min));
  return mapped;
}

uint16_t update_max(uint16_t read, uint16_t max)
{
  if (read > max)
  {
    return read;
  }
  return max;
}