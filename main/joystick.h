#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Hardware pin mapping for PS2 Joystick on YD-ESP32-S3:
//   VRX -> GPIO 2 (ADC1 Channel 1) -> Slew MG996R Servo on GPIO 1
//   VRY -> GPIO 3 (ADC1 Channel 2) -> Fine Nudge / Alternate Slew
//   SW  -> GPIO 4 (Digital Input, internal pull-up) -> Click to Toggle Center/Min/Max

#define JOYSTICK_ADC_VRX_PIN        GPIO_NUM_2
#define JOYSTICK_ADC_VRY_PIN        GPIO_NUM_3
#define JOYSTICK_SW_PIN             GPIO_NUM_4

#define JOYSTICK_MIN_SPEED_DPS      30
#define JOYSTICK_MAX_SPEED_DPS      500
#define JOYSTICK_DEFAULT_SPEED_DPS  260

void joystick_init(void);
void joystick_set_speed(int deg_per_sec);
int  joystick_get_speed(void);
void joystick_set_invert(bool invert);
bool joystick_get_invert(void);
void joystick_toggle_pose(void);

#ifdef __cplusplus
}
#endif