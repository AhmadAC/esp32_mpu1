#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maps the MPU-6050 orientation (relative to the stored neutral pose) onto the arm servos:
//   Roll  -> Servo 2 (Arm,      GPIO 38)
//   Pitch -> Servo 3 (Shoulder, GPIO 39)
//   Yaw   -> Servo 1 (Claw,     GPIO 1)
// servo_angle = 90 +/- gain * orientation_deg (clamped to 0..180)

typedef void (*mpu_control_print_fn_t)(const char *str);

void mpu_control_init(void);

void mpu_control_set_enabled(bool enabled);
bool mpu_control_is_enabled(void);

// Gain changes are applied immediately but only persisted by mpu_control_save_settings()
// (the app streams values while the slider moves and saves once on release).
void  mpu_control_set_gain(float gain);
float mpu_control_get_gain(void);

void mpu_control_set_axes(bool roll, bool pitch, bool yaw);
void mpu_control_get_axes(bool *roll, bool *pitch, bool *yaw);

void mpu_control_set_invert(bool roll, bool pitch, bool yaw);
void mpu_control_get_invert(bool *roll, bool *pitch, bool *yaw);

// Persists gain, axis enables and inversion to NVS flash
void mpu_control_save_settings(void);

// Prints the MPUCTRL line
void mpu_control_report(mpu_control_print_fn_t print_fn);

#ifdef __cplusplus
}
#endif
