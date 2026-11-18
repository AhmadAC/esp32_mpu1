#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// 3-Axis MP90S Robotic Arm Servo Mapping:
// S1: Claw (Gripper) on GPIO 1
// S2: Arm on GPIO 38
// S3: Shoulder on GPIO 39
#define SERVO_COUNT         3
#define SERVO_1_PIN         GPIO_NUM_1
#define SERVO_2_PIN         GPIO_NUM_38
#define SERVO_3_PIN         GPIO_NUM_39

// Standard 50Hz PWM timings (550us - 2450us for 0 - 180 degrees)
#define SERVO_MIN_PULSE_US  550
#define SERVO_MAX_PULSE_US  2450
#define SERVO_MIN_US        550
#define SERVO_MAX_US        2450
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_DEFAULT_ANGLE 90

#define SERVO_OFFSET_MIN    -45
#define SERVO_OFFSET_MAX    45

#define ACTION_NUM_SLOTS    8
#define ACTION_MAX_LEN      384
#define ACTION_MAX_STEPS    24

typedef struct {
    int s1;
    int s2;
    int s3;
    int move_ms;
    int wait_ms;
} servo_step_t;

void servo_controller_init(void);

// Angle commands
void servo_set_angle(int idx, int angle);
int  servo_get_angle(int idx);
void servo_set_all(int s1, int s2, int s3);
void servo_move_smooth(int s1, int s2, int s3, int duration_ms);
void servo_get_all_angles(int *s1, int *s2, int *s3);

// Power / Attach / Detach controls (duty = 0 prevents heating and stall buzz)
void servo_detach(int idx);
void servo_detach_all(void);
void servo_attach_all(void);
void servo_get_all_enabled(bool *e1, bool *e2, bool *e3);
void servo_set_auto_idle(bool enabled);
bool servo_get_auto_idle(void);

// Calibration Trim Offsets
void servo_set_offset(int idx, int offset);
void servo_save_offsets(int o1, int o2, int o3);
void servo_get_all_offsets(int *o1, int *o2, int *o3);

// Programmable Sequence Routines
bool servo_action_set(int slot, const char *def);
const char *servo_action_get(int slot);
void servo_action_reset_defaults(void);
bool servo_action_run(int slot, bool loop);
bool servo_run_sequence_str(const char *seq_str, bool loop);
void servo_stop_motion(void);
bool servo_is_sequence_running(void);
bool servo_is_sequence_looping(void);
int  servo_get_active_slot(void);

#ifdef __cplusplus
}
#endif