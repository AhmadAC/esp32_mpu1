#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Orientation is reported RELATIVE to the stored neutral pose (Set Neutral / Zero Yaw).
// Convention (right-hand rule, sensor chip facing up when flat):
//   roll  = rotation about X, pitch = rotation about Y, yaw = rotation about Z (gyro integrated).
typedef struct {
    float roll;   // deg
    float pitch;  // deg
    float yaw;    // deg
    float ax, ay, az;  // g (bias corrected)
    float gx, gy, gz;  // deg/s (bias corrected)
    float temp;   // deg C
} imu_pose_t;

typedef void (*imu_print_fn_t)(const char *str);

void imu_init(void);

// Latest fused pose; false when the sensor is missing or the sample is stale.
bool imu_get(imu_pose_t *out);
bool imu_has_sensor(void);

// Telemetry streaming over BLE ("IMU:" lines, 20 Hz)
void imu_set_stream(bool enabled);
bool imu_get_stream(void);

// kind: "gyro", "accel", "neutral", "yaw", "clear". Runs asynchronously in the IMU task and
// reports through MPUCALPROG / MPUCALRES / MPUCAL notifications. Returns false when the kind is
// invalid or another calibration is still running.
bool imu_request_cal(const char *kind);

// Prints MPUHW and MPUCAL lines through print_fn
void imu_report_state(imu_print_fn_t print_fn);

// Formats one "IMU:" telemetry line; returns false when there is no fresh data.
bool imu_format_line(char *buf, size_t len);

#ifdef __cplusplus
}
#endif
