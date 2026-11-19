#include "imu.h"
#include "mpu6050.h"
#include "ble_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "IMU";

#define LOOP_MS               5        // 200 Hz sampling / fusion
#define TELEMETRY_DIV         10       // 20 Hz BLE telemetry
#define CAL_SAMPLES           400      // 2 s of samples per calibration
#define STALE_US              150000
#define RAD2DEG               57.29577951f
#define DEG2RAD               0.01745329252f
#define GYRO_DEADBAND_DPS     0.2f
#define GYRO_STILL_RANGE_LSB  400      // max peak-to-peak gyro noise while still
#define ACCEL_STILL_RANGE_LSB 700      // max peak-to-peak accel noise while still
#define MAX_READ_FAILS        50
#define MAHONY_KP             0.40f    // attitude filter proportional convergence rate

#define CAL_MAGIC             0x4D505545u

typedef struct {
    uint32_t magic;
    float    gyro_bias[3];   // deg/s
    float    accel_bias[3];  // g
    float    neutral_q[4];   // [w, x, y, z] neutral quaternion
    uint8_t  gyro_ok;
    uint8_t  accel_ok;
    uint8_t  neutral_ok;
    uint8_t  pad;
} imu_cal_t;

enum {
    REQ_NONE = 0,
    REQ_GYRO,
    REQ_ACCEL,
    REQ_NEUTRAL,
    REQ_YAW,
    REQ_CLEAR
};

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static imu_pose_t   s_pose;
static int64_t      s_pose_us   = 0;
static imu_cal_t    s_cal_shared;
static volatile bool s_sensor_ok = false;
static volatile bool s_stream    = false;
static volatile int  s_cal_req   = REQ_NONE;
static volatile bool s_cal_busy  = false;

static void cal_defaults(imu_cal_t *c) {
    memset(c, 0, sizeof(*c));
    c->magic = CAL_MAGIC;
    // Default neutral orientation: standing upright on pins (+X up, pitch -90 deg from flat)
    c->neutral_q[0] = 0.7071068f; // w = cos(-45 deg)
    c->neutral_q[1] = 0.0f;
    c->neutral_q[2] = -0.7071068f;// y = sin(-45 deg)
    c->neutral_q[3] = 0.0f;
    c->neutral_ok = 1;
}

static void cal_load(imu_cal_t *c) {
    cal_defaults(c);
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READONLY, &h) == ESP_OK) {
        imu_cal_t tmp;
        size_t len = sizeof(tmp);
        if (nvs_get_blob(h, "cal", &tmp, &len) == ESP_OK && len == sizeof(tmp) && tmp.magic == CAL_MAGIC) {
            *c = tmp;
        }
        nvs_close(h);
    }
}

static void cal_save(const imu_cal_t *c) {
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "cal", c, sizeof(*c));
        nvs_commit(h);
        nvs_close(h);
    }
}

static void cal_clear_nvs(void) {
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "cal");
        nvs_commit(h);
        nvs_close(h);
    }
}

static void cal_share(const imu_cal_t *c) {
    portENTER_CRITICAL(&s_lock);
    s_cal_shared = *c;
    portEXIT_CRITICAL(&s_lock);
}

static void format_cal_line(char *buf, size_t len, const imu_cal_t *c) {
    snprintf(buf, len,
             "MPUCAL:gyro=%d,accel=%d,nr=0.0,np=0.0,ny=0.0,gbx=%.2f,gby=%.2f,gbz=%.2f,abx=%.3f,aby=%.3f,abz=%.3f\n",
             c->gyro_ok ? 1 : 0, c->accel_ok ? 1 : 0,
             c->gyro_bias[0], c->gyro_bias[1], c->gyro_bias[2],
             c->accel_bias[0], c->accel_bias[1], c->accel_bias[2]);
}

static void notify_cal_state(const imu_cal_t *c) {
    char buf[240];
    format_cal_line(buf, sizeof(buf), c);
    ble_server_send_notify(buf);
}

static void notify_hw(bool ok) {
    char buf[48];
    snprintf(buf, sizeof(buf), "MPUHW:ok=%d,addr=%d\n", ok ? 1 : 0, (int)mpu6050_address());
    ble_server_send_notify(buf);
}

static void notify_result(const char *kind, bool ok, const char *msg) {
    char buf[160];
    snprintf(buf, sizeof(buf), "MPUCALRES:kind=%s,ok=%d,msg=%s\n", kind, ok ? 1 : 0, msg);
    ble_server_send_notify(buf);
    ESP_LOGI(TAG, "Calibration %s: %s (%s)", kind, ok ? "OK" : "FAILED", msg);
}

static void notify_progress(const char *kind, int pct) {
    char buf[64];
    snprintf(buf, sizeof(buf), "MPUCALPROG:kind=%s,pct=%d\n", kind, pct);
    ble_server_send_notify(buf);
}

static void publish_pose(const imu_pose_t *p, int64_t now_us) {
    portENTER_CRITICAL(&s_lock);
    s_pose = *p;
    s_pose_us = now_us;
    portEXIT_CRITICAL(&s_lock);
}

static bool pose_snapshot(imu_pose_t *out) {
    int64_t age;
    portENTER_CRITICAL(&s_lock);
    *out = s_pose;
    age = esp_timer_get_time() - s_pose_us;
    portEXIT_CRITICAL(&s_lock);
    return s_sensor_ok && s_pose_us != 0 && age < STALE_US;
}

bool imu_format_line(char *buf, size_t len) {
    imu_pose_t p;
    if (!pose_snapshot(&p)) return false;
    snprintf(buf, len, "IMU:%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f\n",
             p.roll, p.pitch, p.yaw, p.ax, p.ay, p.az, p.gx, p.gy, p.gz, p.temp);
    return true;
}

static void quat_mult(const float q1[4], const float q2[4], float out[4]) {
    out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void quat_conj(const float q[4], float out[4]) {
    out[0] =  q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

static void quat_to_euler_deg(const float q[4], float *roll, float *pitch, float *yaw) {
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // Roll (X-axis)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

    // Pitch (Y-axis)
    float sinp = 2.0f * (w * y - z * x);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);
    } else {
        *pitch = asinf(sinp) * RAD2DEG;
    }

    // Yaw (Z-axis)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}

static void imu_task(void *pvParameters) {
    (void)pvParameters;

    imu_cal_t cal;
    cal_load(&cal);
    cal_share(&cal);

    bool initialized = false;
    float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    float yaw_offset_deg = 0.0f;

    int64_t last_us = esp_timer_get_time();
    int64_t last_probe_us = 0;
    int tele_div = 0;
    int read_fails = 0;

    int collecting = REQ_NONE;
    int n = 0;
    int next_pct = 25;
    double sum[6] = { 0, 0, 0, 0, 0, 0 };
    int16_t mn[6] = { 0, 0, 0, 0, 0, 0 };
    int16_t mx[6] = { 0, 0, 0, 0, 0, 0 };

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_MS));
        int64_t now_us = esp_timer_get_time();

        // Sensor presence check / re-probe
        if (!s_sensor_ok) {
            if (now_us - last_probe_us > 1000000) {
                last_probe_us = now_us;
                if (mpu6050_init()) {
                    s_sensor_ok = true;
                    read_fails = 0;
                    initialized = false;
                    last_us = now_us;
                    ESP_LOGI(TAG, "MPU-6050 online");
                    notify_hw(true);
                }
            }
            continue;
        }

        // Handle calibration requests
        int req = s_cal_req;
        if (req != REQ_NONE && collecting == REQ_NONE) {
            s_cal_req = REQ_NONE;
            if (req == REQ_GYRO || req == REQ_ACCEL) {
                collecting = req;
                s_cal_busy = true;
                n = 0;
                next_pct = 25;
                for (int i = 0; i < 6; i++) { sum[i] = 0.0; mn[i] = 32767; mx[i] = -32768; }
                notify_progress(req == REQ_GYRO ? "gyro" : "accel", 0);
            } else if (req == REQ_NEUTRAL) {
                if (initialized) {
                    cal.neutral_q[0] = q[0];
                    cal.neutral_q[1] = q[1];
                    cal.neutral_q[2] = q[2];
                    cal.neutral_q[3] = q[3];
                    cal.neutral_ok = 1;
                    yaw_offset_deg = 0.0f;
                    cal_save(&cal);
                    cal_share(&cal);
                    notify_result("neutral", true, "Neutral upright pose saved");
                } else {
                    notify_result("neutral", false, "No sensor data yet");
                }
                notify_cal_state(&cal);
            } else if (req == REQ_YAW) {
                float r, p, y;
                float q_inv[4], q_rel[4];
                quat_conj(cal.neutral_q, q_inv);
                quat_mult(q_inv, q, q_rel);
                quat_to_euler_deg(q_rel, &r, &p, &y);
                yaw_offset_deg = y;
                notify_result("yaw", true, "Yaw zeroed at current heading");
                notify_cal_state(&cal);
            } else if (req == REQ_CLEAR) {
                cal_defaults(&cal);
                cal_clear_nvs();
                cal_share(&cal);
                initialized = false;
                yaw_offset_deg = 0.0f;
                notify_result("clear", true, "All MPU calibration cleared");
                notify_cal_state(&cal);
            }
        }

        // Read raw data from MPU-6050
        mpu6050_raw_t raw;
        if (!mpu6050_read_raw(&raw)) {
            if (++read_fails >= MAX_READ_FAILS) {
                ESP_LOGW(TAG, "MPU-6050 connection lost");
                s_sensor_ok = false;
                collecting = REQ_NONE;
                s_cal_busy = false;
                notify_hw(false);
            }
            continue;
        }
        read_fails = 0;

        // Process calibration sample collection
        if (collecting != REQ_NONE) {
            const int16_t v[6] = { raw.ax, raw.ay, raw.az, raw.gx, raw.gy, raw.gz };
            for (int i = 0; i < 6; i++) {
                sum[i] += (double)v[i];
                if (v[i] < mn[i]) mn[i] = v[i];
                if (v[i] > mx[i]) mx[i] = v[i];
            }
            n++;

            const char *kind = (collecting == REQ_GYRO) ? "gyro" : "accel";
            int pct = (n * 100) / CAL_SAMPLES;
            if (pct >= next_pct && pct < 100) {
                notify_progress(kind, pct);
                next_pct += 25;
            }

            if (n >= CAL_SAMPLES) {
                bool gyro_still = true;
                for (int i = 3; i < 6; i++) {
                    if ((mx[i] - mn[i]) > GYRO_STILL_RANGE_LSB) gyro_still = false;
                }

                if (collecting == REQ_GYRO) {
                    if (!gyro_still) {
                        notify_result(kind, false, "Sensor moved. Keep it still and retry");
                    } else {
                        for (int i = 0; i < 3; i++) {
                            cal.gyro_bias[i] = (float)(sum[3 + i] / n) / MPU6050_GYRO_LSB_PER_DPS;
                        }
                        cal.gyro_ok = 1;
                        cal_save(&cal);
                        cal_share(&cal);
                        initialized = false;
                        notify_result(kind, true, "Gyro bias saved");
                    }
                } else {
                    bool accel_still = true;
                    for (int i = 0; i < 3; i++) {
                        if ((mx[i] - mn[i]) > ACCEL_STILL_RANGE_LSB) accel_still = false;
                    }
                    float mean_ax = (float)(sum[0] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float mean_ay = (float)(sum[1] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float mean_az = (float)(sum[2] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float total_mag = sqrtf(mean_ax * mean_ax + mean_ay * mean_ay + mean_az * mean_az);

                    if (!gyro_still || !accel_still) {
                        notify_result(kind, false, "Sensor moved. Keep it still and retry");
                    } else if (total_mag < 0.75f || total_mag > 1.25f) {
                        notify_result(kind, false, "Invalid acceleration. Keep sensor resting still");
                    } else {
                        // Multi-axis stationary calibration: identifies gravity axis automatically
                        // Works whether resting standing upright on pins (+X=1g) or flat (+Z=1g)
                        float max_comp = fabsf(mean_ax);
                        int axis = 0; // 0=X, 1=Y, 2=Z
                        if (fabsf(mean_ay) > max_comp) { max_comp = fabsf(mean_ay); axis = 1; }
                        if (fabsf(mean_az) > max_comp) { max_comp = fabsf(mean_az); axis = 2; }

                        cal.accel_bias[0] = mean_ax;
                        cal.accel_bias[1] = mean_ay;
                        cal.accel_bias[2] = mean_az;
                        if (axis == 0) cal.accel_bias[0] -= copysignf(1.0f, mean_ax);
                        else if (axis == 1) cal.accel_bias[1] -= copysignf(1.0f, mean_ay);
                        else cal.accel_bias[2] -= copysignf(1.0f, mean_az);

                        cal.accel_ok = 1;
                        cal_save(&cal);
                        cal_share(&cal);
                        initialized = false;
                        notify_result(kind, true, "Accel bias saved (upright calibration OK)");
                    }
                }
                notify_cal_state(&cal);
                collecting = REQ_NONE;
                s_cal_busy = false;
                last_us = now_us;
            }
            continue;
        }

        // Convert raw to engineering units and remove calibrated biases
        float ax = (float)raw.ax / MPU6050_ACCEL_LSB_PER_G - cal.accel_bias[0];
        float ay = (float)raw.ay / MPU6050_ACCEL_LSB_PER_G - cal.accel_bias[1];
        float az = (float)raw.az / MPU6050_ACCEL_LSB_PER_G - cal.accel_bias[2];
        float gx = (float)raw.gx / MPU6050_GYRO_LSB_PER_DPS - cal.gyro_bias[0];
        float gy = (float)raw.gy / MPU6050_GYRO_LSB_PER_DPS - cal.gyro_bias[1];
        float gz = (float)raw.gz / MPU6050_GYRO_LSB_PER_DPS - cal.gyro_bias[2];
        float temp_c = (float)raw.temp / 340.0f + 36.53f;

        float gx_f = (fabsf(gx) < GYRO_DEADBAND_DPS) ? 0.0f : gx;
        float gy_f = (fabsf(gy) < GYRO_DEADBAND_DPS) ? 0.0f : gy;
        float gz_f = (fabsf(gz) < GYRO_DEADBAND_DPS) ? 0.0f : gz;

        float dt = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.05f) dt = 0.05f;

        // Quaternion Complementary Filter (Mahony algorithm - completely gimbal-lock free)
        float a_mag = sqrtf(ax * ax + ay * ay + az * az);
        float gx_rad = gx_f * DEG2RAD;
        float gy_rad = gy_f * DEG2RAD;
        float gz_rad = gz_f * DEG2RAD;

        if (!initialized) {
            if (a_mag > 0.8f && a_mag < 1.2f) {
                // Initialize quaternion aligned directly with measured gravity vector
                float ax_n = ax / a_mag, ay_n = ay / a_mag, az_n = az / a_mag;
                // Target: map [0, 0, 1] to [ax_n, ay_n, az_n]
                float dot_val = az_n;
                if (dot_val > 0.9999f) {
                    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
                } else if (dot_val < -0.9999f) {
                    q[0] = 0.0f; q[1] = 1.0f; q[2] = 0.0f; q[3] = 0.0f;
                } else {
                    float s = sqrtf(2.0f * (1.0f + dot_val));
                    q[0] = 0.5f * s;
                    q[1] = -ay_n / s;
                    q[2] =  ax_n / s;
                    q[3] = 0.0f;
                }
                initialized = true;
            }
        } else {
            // Accel correction
            if (a_mag > 0.8f && a_mag < 1.2f) {
                float ax_n = ax / a_mag, ay_n = ay / a_mag, az_n = az / a_mag;
                // Estimated gravity direction in body frame from quaternion
                float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
                float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
                float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

                // Error is cross product of measured gravity and estimated gravity
                float ex = (ay_n * vz - az_n * vy);
                float ey = (az_n * vx - ax_n * vz);
                float ez = (ax_n * vy - ay_n * vx);

                gx_rad += MAHONY_KP * ex;
                gy_rad += MAHONY_KP * ey;
                gz_rad += MAHONY_KP * ez;
            }

            // Integrate quaternion rate of change
            float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
            float dq0 = 0.5f * (-qx * gx_rad - qy * gy_rad - qz * gz_rad);
            float dq1 = 0.5f * ( qw * gx_rad + qy * gz_rad - qz * gy_rad);
            float dq2 = 0.5f * ( qw * gy_rad - qx * gz_rad + qz * gx_rad);
            float dq3 = 0.5f * ( qw * gz_rad + qx * gy_rad - qy * gx_rad);

            q[0] += dq0 * dt;
            q[1] += dq1 * dt;
            q[2] += dq2 * dt;
            q[3] += dq3 * dt;

            // Re-normalize quaternion
            float q_len = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (q_len > 0.0f) {
                q[0] /= q_len; q[1] /= q_len; q[2] /= q_len; q[3] /= q_len;
            }
        }

        // Calculate attitude RELATIVE to neutral stored orientation
        float q_rel[4];
        if (cal.neutral_ok) {
            float q_neut_inv[4];
            quat_conj(cal.neutral_q, q_neut_inv);
            quat_mult(q_neut_inv, q, q_rel);
        } else {
            memcpy(q_rel, q, sizeof(q_rel));
        }

        float r_deg = 0.0f, p_deg = 0.0f, y_deg = 0.0f;
        quat_to_euler_deg(q_rel, &r_deg, &p_deg, &y_deg);
        y_deg -= yaw_offset_deg;
        while (y_deg > 180.0f) y_deg -= 360.0f;
        while (y_deg < -180.0f) y_deg += 360.0f;

        imu_pose_t out;
        out.roll  = r_deg;
        out.pitch = p_deg;
        out.yaw   = y_deg;
        out.ax = ax; out.ay = ay; out.az = az;
        out.gx = gx; out.gy = gy; out.gz = gz;
        out.temp = temp_c;
        publish_pose(&out, now_us);

        // BLE Telemetry at 20 Hz
        if (++tele_div >= TELEMETRY_DIV) {
            tele_div = 0;
            if (s_stream && ble_server_is_connected()) {
                char line[128];
                if (imu_format_line(line, sizeof(line))) {
                    ble_server_send_notify(line);
                }
            }
        }
    }
}

void imu_init(void) {
    memset(&s_pose, 0, sizeof(s_pose));
    cal_defaults(&s_cal_shared);
    s_sensor_ok = mpu6050_init();
    if (s_sensor_ok) {
        ESP_LOGI(TAG, "MPU-6050 ready (SDA=GPIO%d, SCL=GPIO%d)", (int)MPU6050_SDA_PIN, (int)MPU6050_SCL_PIN);
    } else {
        ESP_LOGW(TAG, "MPU-6050 not detected (SDA=GPIO%d, SCL=GPIO%d). Will keep retrying.",
                 (int)MPU6050_SDA_PIN, (int)MPU6050_SCL_PIN);
    }
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, NULL, 6, NULL, 1);
}

bool imu_get(imu_pose_t *out) {
    if (out == NULL) return false;
    return pose_snapshot(out);
}

bool imu_has_sensor(void) {
    return s_sensor_ok;
}

void imu_set_stream(bool enabled) {
    s_stream = enabled;
}

bool imu_get_stream(void) {
    return s_stream;
}

bool imu_request_cal(const char *kind) {
    if (kind == NULL) return false;
    int req = REQ_NONE;
    if (strcmp(kind, "gyro") == 0) req = REQ_GYRO;
    else if (strcmp(kind, "accel") == 0) req = REQ_ACCEL;
    else if (strcmp(kind, "neutral") == 0) req = REQ_NEUTRAL;
    else if (strcmp(kind, "yaw") == 0) req = REQ_YAW;
    else if (strcmp(kind, "clear") == 0) req = REQ_CLEAR;
    if (req == REQ_NONE) return false;
    if (s_cal_busy || s_cal_req != REQ_NONE) return false;
    if (!s_sensor_ok) return false;
    s_cal_req = req;
    return true;
}

void imu_report_state(imu_print_fn_t print_fn) {
    if (print_fn == NULL) return;

    char hw[48];
    snprintf(hw, sizeof(hw), "MPUHW:ok=%d,addr=%d\n", s_sensor_ok ? 1 : 0, (int)mpu6050_address());
    print_fn(hw);

    imu_cal_t c;
    portENTER_CRITICAL(&s_lock);
    c = s_cal_shared;
    portEXIT_CRITICAL(&s_lock);

    char line[240];
    format_cal_line(line, sizeof(line), &c);
    print_fn(line);
}