#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Single MG996R Servo on GPIO 1
#define SERVO_COUNT         1
#define SERVO_1_PIN         GPIO_NUM_1

// MG996R standard 50Hz PWM timings (500us - 2500us for full 0 - 180 degree span)
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_DEFAULT_ANGLE 90

#define SERVO_OFFSET_MIN    -45
#define SERVO_OFFSET_MAX    45

#define ACTION_NUM_SLOTS    8
#define ACTION_MAX_LEN      384
#define ACTION_MAX_STEPS    24

void servo_controller_init(void);

// Angle commands
void servo_set_angle(int angle);
void servo_set_angle_idx(int idx, int angle);
void servo_move_smooth(int angle, int duration_ms);
int  servo_get_angle(void);
int  servo_get_angle_idx(int idx);

// Power / Attach / Detach controls (duty = 0 prevents MG996R heat and stall buzzing)
void servo_detach(void);
void servo_detach_idx(int idx);
void servo_attach(void);
bool servo_is_enabled(void);
void servo_set_auto_idle(bool enabled);
bool servo_get_auto_idle(void);

// Calibration Trim Offsets
void servo_set_offset(int offset);
int  servo_get_offset(void);
void servo_save_offset(int offset);

// Programmable Sequence Routines
bool servo_action_set(int slot, const char *def_str);
const char* servo_action_get(int slot);
void servo_action_reset_defaults(void);
bool servo_action_run(int slot, bool loop);
bool servo_run_sequence_str(const char *steps_str, bool loop);
void servo_stop_motion(void);
bool servo_is_sequence_running(void);
int  servo_get_active_slot(void);
bool servo_is_sequence_looping(void);

#ifdef __cplusplus
}
#endif