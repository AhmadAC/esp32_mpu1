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
#define COMP_ALPHA            0.98f    // complementary filter gyro weight
#define GYRO_DEADBAND_DPS     0.2f
#define STALE_US              150000
#define RAD2DEG               57.29577951f
#define DEG2RAD               0.01745329252f
#define GYRO_STILL_RANGE_LSB  400      // max peak-to-peak gyro noise while "still"
#define ACCEL_STILL_RANGE_LSB 700      // max peak-to-peak accel noise while "still"
#define FLAT_XY_MAX_G         0.12f    // max |ax|,|ay| (about 7 deg) for accel calibration
#define MAX_READ_FAILS        50

#define CAL_MAGIC             0x4D505543u

typedef struct {
    uint32_t magic;
    float    gyro_bias[3];   // deg/s
    float    accel_bias[3];  // g
    float    neutral[3];     // deg (absolute roll, pitch, yaw)
    uint8_t  gyro_ok;
    uint8_t  accel_ok;
    uint8_t  pad[2];
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

static float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void cal_defaults(imu_cal_t *c) {
    memset(c, 0, sizeof(*c));
    c->magic = CAL_MAGIC;
}

static void cal_load(imu_cal_t *c) {
    cal_defaults(c);
    nvs_handle_t h;
    if (nvs_open("mpu_cfg", NVS_READONLY, &h) == ESP_OK) {
        imu_cal_t tmp;
        size_t len = sizeof(tmp);
        if (nvs_get_blob(h, "cal", &tmp, &len) == ESP_OK && len == sizeof(tmp) && tmp.magic == CAL_MAGIC) {
            *c = tmp;
            c->neutral[2] = 0.0f;  // yaw is relative to power-on heading
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
             "MPUCAL:gyro=%d,accel=%d,nr=%.1f,np=%.1f,ny=%.1f,gbx=%.2f,gby=%.2f,gbz=%.2f,abx=%.3f,aby=%.3f,abz=%.3f\n",
             c->gyro_ok ? 1 : 0, c->accel_ok ? 1 : 0,
             c->neutral[0], c->neutral[1], c->neutral[2],
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

static void imu_task(void *pvParameters) {
    (void)pvParameters;

    imu_cal_t cal;
    cal_load(&cal);
    cal_share(&cal);

    bool initialized = false;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
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

        // ---- Sensor presence / re-probe ----
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

        // ---- Calibration requests ----
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
                    cal.neutral[0] = roll;
                    cal.neutral[1] = pitch;
                    cal.neutral[2] = yaw;
                    cal_save(&cal);
                    cal_share(&cal);
                    notify_result("neutral", true, "Neutral pose saved");
                } else {
                    notify_result("neutral", false, "No sensor data yet");
                }
                notify_cal_state(&cal);
            } else if (req == REQ_YAW) {
                cal.neutral[2] = yaw;
                cal_share(&cal);
                notify_result("yaw", true, "Yaw zeroed at current heading");
                notify_cal_state(&cal);
            } else if (req == REQ_CLEAR) {
                cal_defaults(&cal);
                cal_clear_nvs();
                cal_share(&cal);
                initialized = false;
                notify_result("clear", true, "All MPU calibration cleared");
                notify_cal_state(&cal);
            }
        }

        // ---- Read sensor ----
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

        // ---- Calibration sample collection ----
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
                        cal.neutral[2] = 0.0f;
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
                    float gx_mean = (float)(sum[0] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float gy_mean = (float)(sum[1] / n) / MPU6050_ACCEL_LSB_PER_G;
                    float gz_mean = (float)(sum[2] / n) / MPU6050_ACCEL_LSB_PER_G;

                    if (!gyro_still || !accel_still) {
                        notify_result(kind, false, "Sensor moved. Keep it still and retry");
                    } else if (fabsf(gx_mean) > FLAT_XY_MAX_G || fabsf(gy_mean) > FLAT_XY_MAX_G ||
                               gz_mean < 0.85f || gz_mean > 1.15f) {
                        notify_result(kind, false, "Place the sensor flat with the chip facing up");
                    } else {
                        cal.accel_bias[0] = gx_mean;
                        cal.accel_bias[1] = gy_mean;
                        cal.accel_bias[2] = gz_mean - 1.0f;
                        cal.accel_ok = 1;
                        cal.neutral[2] = 0.0f;
                        cal_save(&cal);
                        cal_share(&cal);
                        initialized = false;
                        notify_result(kind, true, "Accel bias saved");
                    }
                }
                notify_cal_state(&cal);
                collecting = REQ_NONE;
                s_cal_busy = false;
                last_us = now_us;
            }
            continue;
        }

        // ---- Convert to engineering units and apply calibration ----
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

        float acc_roll  = atan2f(ay, az) * RAD2DEG;
        float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

        if (!initialized) {
            roll = acc_roll;
            pitch = acc_pitch;
            yaw = 0.0f;
            initialized = true;
        } else {
            float r = roll * DEG2RAD;
            float p = pitch * DEG2RAD;
            float cp = cosf(p);
            if (fabsf(cp) < 0.05f) cp = (cp < 0.0f) ? -0.05f : 0.05f;
            float tp = sinf(p) / cp;

            float roll_rate  = gx_f + sinf(r) * tp * gy_f + cosf(r) * tp * gz_f;
            float pitch_rate = cosf(r) * gy_f - sinf(r) * gz_f;
            float yaw_rate   = (sinf(r) * gy_f + cosf(r) * gz_f) / cp;

            roll  += roll_rate * dt;
            pitch += pitch_rate * dt;
            yaw   += yaw_rate * dt;

            // Only trust the accelerometer when the board is close to 1 g (not accelerating)
            float mag = sqrtf(ax * ax + ay * ay + az * az);
            if (mag > 0.8f && mag < 1.2f) {
                roll  += (1.0f - COMP_ALPHA) * wrap180(acc_roll - roll);
                pitch += (1.0f - COMP_ALPHA) * wrap180(acc_pitch - pitch);
            }
            roll = wrap180(roll);
            pitch = wrap180(pitch);
            yaw = wrap180(yaw);
        }

        imu_pose_t out;
        out.roll  = wrap180(roll - cal.neutral[0]);
        out.pitch = wrap180(pitch - cal.neutral[1]);
        out.yaw   = wrap180(yaw - cal.neutral[2]);
        out.ax = ax; out.ay = ay; out.az = az;
        out.gx = gx; out.gy = gy; out.gz = gz;
        out.temp = temp_c;
        publish_pose(&out, now_us);

        // ---- BLE telemetry ----
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
