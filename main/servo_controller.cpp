#include "servo_controller.h"
#include "ble_server.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "SERVO_CTRL";

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY_HZ       50
#define LEDC_PERIOD_US          20000
#define LEDC_MAX_DUTY           16383

#define AUTO_IDLE_TIMEOUT_US    3000000

static const gpio_num_t SERVO_PINS[SERVO_COUNT] = {
    SERVO_1_PIN,
    SERVO_2_PIN,
    SERVO_3_PIN
};

static const ledc_channel_t LEDC_CHANNELS[SERVO_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2
};

static const char *DEFAULT_ACTIONS[ACTION_NUM_SLOTS] = {
    "Home|90,90,90,500,0",
    "Open|90,60,170,400,0",
    "Grab|90,120,20,400,0",
    "Pick & Place|45,130,160,500,200;45,130,25,400,300;45,60,25,500,200;135,60,25,600,200;135,130,25,500,200;135,130,160,400,200;90,90,90,500,0",
    "Wave|90,45,90,400,100;45,45,150,300,150;135,45,30,300,150;45,45,150,300,150;135,45,30,300,150;90,90,90,400,0",
    "Sweep|10,90,90,600,200;170,90,90,800,200;90,90,90,600,0",
    "Rest|90,160,10,600,0",
    "Nod|90,40,90,350,150;90,110,90,350,150;90,40,90,350,150;90,90,90,400,0"
};

static portMUX_TYPE s_servo_mux = portMUX_INITIALIZER_UNLOCKED;

static int  s_angles[SERVO_COUNT] = { 90, 90, 90 };
static int  s_offsets[SERVO_COUNT] = { 0, 0, 0 };
static bool s_enabled[SERVO_COUNT] = { false, false, false };
static bool s_auto_idle = false;
static int64_t s_last_activity_us = 0;

static char s_action_slots[ACTION_NUM_SLOTS][ACTION_MAX_LEN];

static TaskHandle_t s_seq_task_handle = NULL;
static volatile bool s_seq_running = false;
static volatile bool s_seq_looping = false;
static volatile int  s_seq_active_slot = 0;
static volatile bool s_seq_stop_req = false;

static servo_step_t s_active_steps[ACTION_MAX_STEPS];
static size_t       s_active_step_count = 0;

static uint32_t calc_duty(int angle, int offset) {
    int eff = angle + offset;
    if (eff < SERVO_MIN_ANGLE) eff = SERVO_MIN_ANGLE;
    if (eff > SERVO_MAX_ANGLE) eff = SERVO_MAX_ANGLE;

    uint32_t pulse_us = SERVO_MIN_PULSE_US + ((uint32_t)eff * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;

    return (pulse_us * LEDC_MAX_DUTY) / LEDC_PERIOD_US;
}

static void apply_hardware_pwm(int idx, int angle, int offset, bool enable) {
    if (idx < 0 || idx >= SERVO_COUNT) return;

    uint32_t duty = enable ? calc_duty(angle, offset) : 0;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNELS[idx], duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNELS[idx]);
}

static void notify_angles(void) {
    char msg[48];
    snprintf(msg, sizeof(msg), "ANG:%d,%d,%d\n", s_angles[0], s_angles[1], s_angles[2]);
    ble_server_send_notify(msg);
}

static void notify_enable_states(void) {
    char msg[48];
    snprintf(msg, sizeof(msg), "SERVO_EN:%d,%d,%d\n",
             s_enabled[0] ? 1 : 0, s_enabled[1] ? 1 : 0, s_enabled[2] ? 1 : 0);
    ble_server_send_notify(msg);
}

static void notify_action_state(bool running, int slot, bool loop) {
    char msg[64];
    snprintf(msg, sizeof(msg), "ACT_STATE:running=%d,slot=%d,loop=%d\n",
             running ? 1 : 0, slot, loop ? 1 : 0);
    ble_server_send_notify(msg);
}

static void load_nvs_configuration(void) {
    nvs_handle_t h;
    if (nvs_open("servo_cal", NVS_READONLY, &h) == ESP_OK) {
        int32_t o1 = 0, o2 = 0, o3 = 0, idle = 0;
        if (nvs_get_i32(h, "off_0", &o1) == ESP_OK) s_offsets[0] = (int)o1;
        if (nvs_get_i32(h, "off_1", &o2) == ESP_OK) s_offsets[1] = (int)o2;
        if (nvs_get_i32(h, "off_2", &o3) == ESP_OK) s_offsets[2] = (int)o3;
        if (nvs_get_i32(h, "auto_idle", &idle) == ESP_OK) s_auto_idle = (idle != 0);
        nvs_close(h);
    }

    if (nvs_open("arm_acts", NVS_READONLY, &h) == ESP_OK) {
        for (int i = 0; i < ACTION_NUM_SLOTS; i++) {
            char key[16];
            snprintf(key, sizeof(key), "act_%d", i + 1);
            size_t req_len = sizeof(s_action_slots[i]);
            if (nvs_get_str(h, key, s_action_slots[i], &req_len) != ESP_OK || req_len == 0) {
                strncpy(s_action_slots[i], DEFAULT_ACTIONS[i], sizeof(s_action_slots[i]) - 1);
                s_action_slots[i][sizeof(s_action_slots[i]) - 1] = '\0';
            }
        }
        nvs_close(h);
    } else {
        for (int i = 0; i < ACTION_NUM_SLOTS; i++) {
            strncpy(s_action_slots[i], DEFAULT_ACTIONS[i], sizeof(s_action_slots[i]) - 1);
            s_action_slots[i][sizeof(s_action_slots[i]) - 1] = '\0';
        }
    }
}

static size_t parse_sequence_steps(const char *steps_str, servo_step_t *out_steps, size_t max_steps) {
    if (!steps_str || !out_steps || max_steps == 0) return 0;

    size_t count = 0;
    const char *p = steps_str;

    while (*p && count < max_steps) {
        while (*p == ' ' || *p == ';') p++;
        if (!*p) break;

        int s1 = 90, s2 = 90, s3 = 90, move = 400, wait = 0;
        int matched = sscanf(p, "%d,%d,%d,%d,%d", &s1, &s2, &s3, &move, &wait);
        if (matched >= 3) {
            out_steps[count].s1 = (s1 < 0) ? 0 : (s1 > 180 ? 180 : s1);
            out_steps[count].s2 = (s2 < 0) ? 0 : (s2 > 180 ? 180 : s2);
            out_steps[count].s3 = (s3 < 0) ? 0 : (s3 > 180 ? 180 : s3);
            out_steps[count].move_ms = (move < 20) ? 20 : (move > 10000 ? 10000 : move);
            out_steps[count].wait_ms = (matched >= 5 && wait > 0) ? (wait > 30000 ? 30000 : wait) : 0;
            count++;
        }

        const char *semi = strchr(p, ';');
        if (!semi) break;
        p = semi + 1;
    }

    return count;
}

static void interpolate_move(int target_s1, int target_s2, int target_s3, int duration_ms) {
    int start_s1, start_s2, start_s3;
    int off_s1, off_s2, off_s3;

    portENTER_CRITICAL(&s_servo_mux);
    start_s1 = s_angles[0];
    start_s2 = s_angles[1];
    start_s3 = s_angles[2];
    off_s1 = s_offsets[0];
    off_s2 = s_offsets[1];
    off_s3 = s_offsets[2];
    s_enabled[0] = true;
    s_enabled[1] = true;
    s_enabled[2] = true;
    s_last_activity_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_servo_mux);

    if (duration_ms <= 20) {
        portENTER_CRITICAL(&s_servo_mux);
        s_angles[0] = target_s1;
        s_angles[1] = target_s2;
        s_angles[2] = target_s3;
        portEXIT_CRITICAL(&s_servo_mux);

        apply_hardware_pwm(0, target_s1, off_s1, true);
        apply_hardware_pwm(1, target_s2, off_s2, true);
        apply_hardware_pwm(2, target_s3, off_s3, true);
        notify_angles();
        return;
    }

    const int step_interval_ms = 20;
    int total_steps = duration_ms / step_interval_ms;
    if (total_steps < 1) total_steps = 1;

    for (int step = 1; step <= total_steps; step++) {
        if (s_seq_stop_req) break;

        float t = (float)step / (float)total_steps;
        float ease = 0.5f * (1.0f - cosf(t * 3.14159265f));

        int cur_s1 = (int)lroundf((float)start_s1 + ((float)target_s1 - (float)start_s1) * ease);
        int cur_s2 = (int)lroundf((float)start_s2 + ((float)target_s2 - (float)start_s2) * ease);
        int cur_s3 = (int)lroundf((float)start_s3 + ((float)target_s3 - (float)start_s3) * ease);

        portENTER_CRITICAL(&s_servo_mux);
        s_angles[0] = cur_s1;
        s_angles[1] = cur_s2;
        s_angles[2] = cur_s3;
        s_last_activity_us = esp_timer_get_time();
        portEXIT_CRITICAL(&s_servo_mux);

        apply_hardware_pwm(0, cur_s1, off_s1, true);
        apply_hardware_pwm(1, cur_s2, off_s2, true);
        apply_hardware_pwm(2, cur_s3, off_s3, true);

        vTaskDelay(pdMS_TO_TICKS(step_interval_ms));
    }

    if (!s_seq_stop_req) {
        portENTER_CRITICAL(&s_servo_mux);
        s_angles[0] = target_s1;
        s_angles[1] = target_s2;
        s_angles[2] = target_s3;
        portEXIT_CRITICAL(&s_servo_mux);

        apply_hardware_pwm(0, target_s1, off_s1, true);
        apply_hardware_pwm(1, target_s2, off_s2, true);
        apply_hardware_pwm(2, target_s3, off_s3, true);
        notify_angles();
    }
}

static void sequence_runner_task(void *pvParameters) {
    (void)pvParameters;

    do {
        for (size_t i = 0; i < s_active_step_count; i++) {
            if (s_seq_stop_req) break;

            servo_step_t step = s_active_steps[i];
            interpolate_move(step.s1, step.s2, step.s3, step.move_ms);

            if (s_seq_stop_req) break;

            if (step.wait_ms > 0) {
                int waited = 0;
                while (waited < step.wait_ms && !s_seq_stop_req) {
                    int chunk = (step.wait_ms - waited > 20) ? 20 : (step.wait_ms - waited);
                    vTaskDelay(pdMS_TO_TICKS(chunk));
                    waited += chunk;
                }
            }
        }
    } while (s_seq_looping && !s_seq_stop_req);

    s_seq_running = false;
    s_seq_looping = false;
    int prev_slot = s_seq_active_slot;
    s_seq_active_slot = 0;
    s_seq_stop_req = false;

    notify_action_state(false, 0, false);
    ESP_LOGI(TAG, "Sequence execution completed (was slot %d).", prev_slot);

    s_seq_task_handle = NULL;
    vTaskDelete(NULL);
}

static void auto_idle_monitor_task(void *pvParameters) {
    (void)pvParameters;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));

        if (!s_auto_idle || s_seq_running) continue;

        int64_now_us = esp_timer_get_time();
        bool any_enabled = false;
        int64_t last_us = 0;

        portENTER_CRITICAL(&s_servo_mux);
        any_enabled = s_enabled[0] || s_enabled[1] || s_enabled[2];
        last_us = s_last_activity_us;
        portEXIT_CRITICAL(&s_servo_mux);

        if (any_enabled && (now_us - last_us > AUTO_IDLE_TIMEOUT_US)) {
            servo_detach_all();
            ESP_LOGI(TAG, "Auto-idle triggered: Servos detached to protect motors & prevent rail sag.");
        }
    }
}

void servo_controller_init(void) {
    load_nvs_configuration();

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.timer_num        = LEDC_TIMER;
    ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.freq_hz          = LEDC_FREQUENCY_HZ;
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Initialize LEDC channels with 0 duty
    for (int i = 0; i < SERVO_COUNT; i++) {
        s_enabled[i] = false;
        ledc_channel_config_t ledc_ch = {};
        ledc_ch.channel    = LEDC_CHANNELS[i];
        ledc_ch.duty       = 0;
        ledc_ch.gpio_num   = SERVO_PINS[i];
        ledc_ch.speed_mode = LEDC_MODE;
        ledc_ch.intr_type  = LEDC_INTR_DISABLE;
        ledc_ch.hpoint     = 0;
        ledc_ch.timer_sel  = LEDC_TIMER;
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch));
    }

    s_last_activity_us = esp_timer_get_time();

    xTaskCreatePinnedToCore(auto_idle_monitor_task, "auto_idle_task", 3072, NULL, 2, NULL, 1);
    ESP_LOGI(TAG, "Servo controller initialized (Claw=GP1, Arm=GP38, Shoulder=GP39). Servos start DETACHED.");
}

void servo_set_angle(int idx, int angle) {
    if (idx < 0 || idx >= SERVO_COUNT) return;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;

    int off = 0;
    portENTER_CRITICAL(&s_servo_mux);
    s_angles[idx] = angle;
    s_enabled[idx] = true;
    s_last_activity_us = esp_timer_get_time();
    off = s_offsets[idx];
    portEXIT_CRITICAL(&s_servo_mux);

    apply_hardware_pwm(idx, angle, off, true);
}

int servo_get_angle(int idx) {
    if (idx < 0 || idx >= SERVO_COUNT) return 90;
    portENTER_CRITICAL(&s_servo_mux);
    int a = s_angles[idx];
    portEXIT_CRITICAL(&s_servo_mux);
    return a;
}

void servo_set_all(int s1, int s2, int s3) {
    servo_set_angle(0, s1);
    servo_set_angle(1, s2);
    servo_set_angle(2, s3);
    notify_angles();
    notify_enable_states();
}

void servo_move_smooth(int s1, int s2, int s3, int duration_ms) {
    servo_stop_motion();
    interpolate_move(s1, s2, s3, duration_ms);
}

void servo_stop_motion(void) {
    if (s_seq_running) {
        s_seq_stop_req = true;
        for (int i = 0; i < 20 && s_seq_running; i++) {
            vTaskDelay(pdMS_TO_TICKS(15));
        }
    }
}

void servo_detach(int idx) {
    if (idx < 0 || idx >= SERVO_COUNT) return;
    portENTER_CRITICAL(&s_servo_mux);
    s_enabled[idx] = false;
    portEXIT_CRITICAL(&s_servo_mux);

    apply_hardware_pwm(idx, s_angles[idx], s_offsets[idx], false);
    notify_enable_states();
}

void servo_detach_all(void) {
    servo_stop_motion();
    portENTER_CRITICAL(&s_servo_mux);
    s_enabled[0] = false;
    s_enabled[1] = false;
    s_enabled[2] = false;
    portEXIT_CRITICAL(&s_servo_mux);

    for (int i = 0; i < SERVO_COUNT; i++) {
        apply_hardware_pwm(i, s_angles[i], s_offsets[i], false);
    }
    notify_enable_states();
}

void servo_attach_all(void) {
    portENTER_CRITICAL(&s_servo_mux);
    s_enabled[0] = true;
    s_enabled[1] = true;
    s_enabled[2] = true;
    s_last_activity_us = esp_timer_get_time();
    portEXIT_CRITICAL(&s_servo_mux);

    for (int i = 0; i < SERVO_COUNT; i++) {
        apply_hardware_pwm(i, s_angles[i], s_offsets[i], true);
    }
    notify_enable_states();
}

void servo_set_auto_idle(bool enabled) {
    s_auto_idle = enabled;
    s_last_activity_us = esp_timer_get_time();

    nvs_handle_t h;
    if (nvs_open("servo_cal", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "auto_idle", enabled ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

bool servo_get_auto_idle(void) {
    return s_auto_idle;
}

void servo_set_offset(int idx, int offset) {
    if (idx < 0 || idx >= SERVO_COUNT) return;
    if (offset < -45) offset = -45;
    if (offset > 45) offset = 45;

    int angle = 90;
    bool en = false;
    portENTER_CRITICAL(&s_servo_mux);
    s_offsets[idx] = offset;
    s_enabled[idx] = true;
    s_last_activity_us = esp_timer_get_time();
    angle = s_angles[idx];
    en = s_enabled[idx];
    portEXIT_CRITICAL(&s_servo_mux);

    apply_hardware_pwm(idx, angle, offset, en);
}

void servo_save_offsets(int o1, int o2, int o3) {
    portENTER_CRITICAL(&s_servo_mux);
    s_offsets[0] = (o1 < -45) ? -45 : (o1 > 45 ? 45 : o1);
    s_offsets[1] = (o2 < -45) ? -45 : (o2 > 45 ? 45 : o2);
    s_offsets[2] = (o3 < -45) ? -45 : (o3 > 45 ? 45 : o3);
    portEXIT_CRITICAL(&s_servo_mux);

    nvs_handle_t h;
    if (nvs_open("servo_cal", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "off_0", s_offsets[0]);
        nvs_set_i32(h, "off_1", s_offsets[1]);
        nvs_set_i32(h, "off_2", s_offsets[2]);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Offsets saved to NVS: (%d, %d, %d)", s_offsets[0], s_offsets[1], s_offsets[2]);
    }
}

void servo_get_all_angles(int *s1, int *s2, int *s3) {
    portENTER_CRITICAL(&s_servo_mux);
    if (s1) *s1 = s_angles[0];
    if (s2) *s2 = s_angles[1];
    if (s3) *s3 = s_angles[2];
    portEXIT_CRITICAL(&s_servo_mux);
}

void servo_get_all_offsets(int *o1, int *o2, int *o3) {
    portENTER_CRITICAL(&s_servo_mux);
    if (o1) *o1 = s_offsets[0];
    if (o2) *o2 = s_offsets[1];
    if (o3) *o3 = s_offsets[2];
    portEXIT_CRITICAL(&s_servo_mux);
}

void servo_get_all_enabled(bool *e1, bool *e2, bool *e3) {
    portENTER_CRITICAL(&s_servo_mux);
    if (e1) *e1 = s_enabled[0];
    if (e2) *e2 = s_enabled[1];
    if (e3) *e3 = s_enabled[2];
    portEXIT_CRITICAL(&s_servo_mux);
}

bool servo_is_sequence_running(void) {
    return s_seq_running;
}

bool servo_is_sequence_looping(void) {
    return s_seq_looping;
}

int servo_get_active_slot(void) {
    return s_seq_active_slot;
}

bool servo_action_run(int slot, bool loop) {
    if (slot < 1 || slot > ACTION_NUM_SLOTS) return false;

    servo_stop_motion();

    const char *def = s_action_slots[slot - 1];
    const char *pipe = strchr(def, '|');
    const char *steps_str = pipe ? (pipe + 1) : def;

    s_active_step_count = parse_sequence_steps(steps_str, s_active_steps, ACTION_MAX_STEPS);
    if (s_active_step_count == 0) return false;

    s_seq_running = true;
    s_seq_looping = loop;
    s_seq_active_slot = slot;
    s_seq_stop_req = false;

    notify_action_state(true, slot, loop);
    xTaskCreatePinnedToCore(sequence_runner_task, "seq_run_task", 4096, NULL, 5, &s_seq_task_handle, 1);
    return true;
}

bool servo_action_set(int slot, const char *def) {
    if (slot < 1 || slot > ACTION_NUM_SLOTS || !def || strlen(def) == 0) return false;

    strncpy(s_action_slots[slot - 1], def, sizeof(s_action_slots[slot - 1]) - 1);
    s_action_slots[slot - 1][sizeof(s_action_slots[slot - 1]) - 1] = '\0';

    nvs_handle_t h;
    if (nvs_open("arm_acts", NVS_READWRITE, &h) == ESP_OK) {
        char key[16];
        snprintf(key, sizeof(key), "act_%d", slot);
        nvs_set_str(h, key, s_action_slots[slot - 1]);
        nvs_commit(h);
        nvs_close(h);
    }
    return true;
}

const char *servo_action_get(int slot) {
    if (slot < 1 || slot > ACTION_NUM_SLOTS) return "";
    return s_action_slots[slot - 1];
}

void servo_action_reset_defaults(void) {
    nvs_handle_t h;
    bool nvs_ok = (nvs_open("arm_acts", NVS_READWRITE, &h) == ESP_OK);

    for (int i = 0; i < ACTION_NUM_SLOTS; i++) {
        strncpy(s_action_slots[i], DEFAULT_ACTIONS[i], sizeof(s_action_slots[i]) - 1);
        s_action_slots[i][sizeof(s_action_slots[i]) - 1] = '\0';
        if (nvs_ok) {
            char key[16];
            snprintf(key, sizeof(key), "act_%d", i + 1);
            nvs_set_str(h, key, s_action_slots[i]);
        }
    }
    if (nvs_ok) {
        nvs_commit(h);
        nvs_close(h);
    }
}

bool servo_run_sequence_str(const char *seq_str, bool loop) {
    if (!seq_str || strlen(seq_str) == 0) return false;

    servo_stop_motion();

    s_active_step_count = parse_sequence_steps(seq_str, s_active_steps, ACTION_MAX_STEPS);
    if (s_active_step_count == 0) return false;

    s_seq_running = true;
    s_seq_looping = loop;
    s_seq_active_slot = 0;
    s_seq_stop_req = false;

    notify_action_state(true, 0, loop);
    xTaskCreatePinnedToCore(sequence_runner_task, "seq_run_task", 4096, NULL, 5, &s_seq_task_handle, 1);
    return true;
}