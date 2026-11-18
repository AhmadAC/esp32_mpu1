#include "joystick.h"
#include "servo_controller.h"
#include "command_processor.h"
#include "console_serial.h"
#include "ble_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "nvs.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "JOYSTICK_MG996R";

static adc_oneshot_unit_handle_t adc1_handle = NULL;

#define ADC_VRX_CHANNEL     ADC_CHANNEL_1  // GPIO 2
#define ADC_VRY_CHANNEL     ADC_CHANNEL_2  // GPIO 3

#define ADC_EFF_MIN         80.0f
#define ADC_EFF_MAX         4000.0f
#define ADC_MIN_SPAN        250.0f
#define DEADZONE_NORM       0.15f
#define EMA_ALPHA           0.45f
#define JOYSTICK_TICK_MS    20

static volatile int  s_slew_deg_per_sec = JOYSTICK_DEFAULT_SPEED_DPS;
static volatile bool s_invert_axis      = false;

static float read_adc_avg(adc_channel_t channel, int samples) {
    long sum = 0;
    int valid = 0;
    for (int i = 0; i < samples; i++) {
        int val = 0;
        if (adc_oneshot_read(adc1_handle, channel, &val) == ESP_OK) {
            sum += val;
            valid++;
        }
    }
    if (valid == 0) return 2048.0f;
    return (float)sum / (float)valid;
}

static float normalize_axis(float val, float center) {
    float delta = val - center;
    if (delta >= 0.0f) {
        float span = ADC_EFF_MAX - center;
        if (span < ADC_MIN_SPAN) span = ADC_MIN_SPAN;
        float n = delta / span;
        return (n > 1.0f) ? 1.0f : n;
    } else {
        float span = center - ADC_EFF_MIN;
        if (span < ADC_MIN_SPAN) span = ADC_MIN_SPAN;
        float n = delta / span;
        return (n < -1.0f) ? -1.0f : n;
    }
}

void joystick_set_speed(int deg_per_sec) {
    if (deg_per_sec < JOYSTICK_MIN_SPEED_DPS) deg_per_sec = JOYSTICK_MIN_SPEED_DPS;
    if (deg_per_sec > JOYSTICK_MAX_SPEED_DPS) deg_per_sec = JOYSTICK_MAX_SPEED_DPS;
    s_slew_deg_per_sec = deg_per_sec;

    nvs_handle_t handle;
    if (nvs_open("arm_cfg", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, "joy_spd", (int32_t)deg_per_sec);
        nvs_commit(handle);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Joystick slew speed set to %d deg/s", deg_per_sec);
}

int joystick_get_speed(void) {
    return s_slew_deg_per_sec;
}

void joystick_set_invert(bool invert) {
    s_invert_axis = invert;
    nvs_handle_t handle;
    if (nvs_open("arm_cfg", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, "joy_inv", invert ? 1 : 0);
        nvs_commit(handle);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Joystick invert set to %s", invert ? "ON" : "OFF");
}

bool joystick_get_invert(void) {
    return s_invert_axis;
}

void joystick_toggle_pose(void) {
    int cur = servo_get_angle();
    int next = 90;
    if (cur < 45) {
        next = 90;
    } else if (cur < 135) {
        next = 180;
    } else {
        next = 0;
    }
    servo_move_smooth(next, 350);
    char msg[64];
    snprintf(msg, sizeof(msg), ">> [Joystick SW] Step to %d deg\n", next);
    console_print(msg);
    command_notify_status();
}

static void joystick_poll_task(void *pvParameters) {
    (void)pvParameters;

    float accum_x = 0.0f;
    const int CAL_SAMPLES = 32;

    for (int i = 0; i < CAL_SAMPLES; i++) {
        accum_x += read_adc_avg(ADC_VRX_CHANNEL, 4);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    float center_x = accum_x / (float)CAL_SAMPLES;
    float filt_x = center_x;

    ESP_LOGI(TAG, "PS2 Joystick calibrated center: %.1f, Speed: %d deg/s, Invert: %d",
             center_x, s_slew_deg_per_sec, s_invert_axis ? 1 : 0);

    float servo_f = (float)servo_get_angle();

    uint8_t sw_press_count = 0;
    uint8_t sw_release_count = 0;
    bool sw_down = false;
    const uint8_t DEBOUNCE_TICKS = 2; // ~40ms

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_notify_tick = 0;
    bool pending_notify = false;

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(JOYSTICK_TICK_MS));

        // 1. Read & Filter Analog Stick
        float raw_x = read_adc_avg(ADC_VRX_CHANNEL, 6);
        filt_x += EMA_ALPHA * (raw_x - filt_x);

        float nx = normalize_axis(filt_x, center_x);
        if (s_invert_axis) nx = -nx;

        float mag = fabsf(nx);
        if (mag > 1.0f) mag = 1.0f;

        // 2. Read Push Button (Active LOW)
        int sw_level = gpio_get_level(JOYSTICK_SW_PIN);
        if (sw_level == 0) {
            sw_release_count = 0;
            if (sw_press_count < DEBOUNCE_TICKS) {
                sw_press_count++;
                if (sw_press_count >= DEBOUNCE_TICKS && !sw_down) {
                    sw_down = true;
                    joystick_toggle_pose();
                    servo_f = (float)servo_get_angle();
                }
            }
        } else {
            sw_press_count = 0;
            if (sw_down) {
                if (sw_release_count < DEBOUNCE_TICKS) {
                    sw_release_count++;
                    if (sw_release_count >= DEBOUNCE_TICKS) {
                        sw_down = false;
                        sw_release_count = 0;
                    }
                }
            } else {
                sw_release_count = 0;
            }
        }

        // 3. Process Analog Motion
        if (mag > DEADZONE_NORM) {
            float eff_x = (mag - DEADZONE_NORM) / (1.0f - DEADZONE_NORM);
            if (nx < 0.0f) eff_x = -eff_x;

            float slew_per_tick = ((float)s_slew_deg_per_sec * (float)JOYSTICK_TICK_MS) / 1000.0f;
            servo_f += eff_x * slew_per_tick;

            if (servo_f < (float)SERVO_MIN_ANGLE) servo_f = (float)SERVO_MIN_ANGLE;
            if (servo_f > (float)SERVO_MAX_ANGLE) servo_f = (float)SERVO_MAX_ANGLE;

            servo_set_angle((int)lroundf(servo_f));
            pending_notify = true;
        } else {
            // Keep tracking float angle aligned when joystick is centered
            servo_f = (float)servo_get_angle();
        }

        if (pending_notify && (xTaskGetTickCount() - last_notify_tick) >= pdMS_TO_TICKS(100)) {
            command_notify_status();
            last_notify_tick = xTaskGetTickCount();
            pending_notify = false;
        }
    }
}

void joystick_init(void) {
    nvs_handle_t handle;
    if (nvs_open("arm_cfg", NVS_READONLY, &handle) == ESP_OK) {
        int32_t spd = JOYSTICK_DEFAULT_SPEED_DPS;
        if (nvs_get_i32(handle, "joy_spd", &spd) == ESP_OK) {
            if (spd < JOYSTICK_MIN_SPEED_DPS) spd = JOYSTICK_MIN_SPEED_DPS;
            if (spd > JOYSTICK_MAX_SPEED_DPS) spd = JOYSTICK_MAX_SPEED_DPS;
            s_slew_deg_per_sec = (int)spd;
        }
        int32_t inv = 0;
        if (nvs_get_i32(handle, "joy_inv", &inv) == ESP_OK) {
            s_invert_axis = (inv != 0);
        }
        nvs_close(handle);
    }

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
#if defined(ADC_ATTEN_DB_12)
        .atten = ADC_ATTEN_DB_12,
#else
        .atten = ADC_ATTEN_DB_11,
#endif
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_VRX_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_VRY_CHANNEL, &config));

    gpio_config_t btn_cfg = {};
    btn_cfg.mode = GPIO_MODE_INPUT;
    btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    btn_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_cfg.intr_type = GPIO_INTR_DISABLE;
    btn_cfg.pin_bit_mask = (1ULL << JOYSTICK_SW_PIN);
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));

    xTaskCreatePinnedToCore(joystick_poll_task, "joystick_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "PS2 Joystick Initialized: VRX=GPIO2, SW=GPIO4, Speed=%d deg/s, Invert=%d",
             s_slew_deg_per_sec, s_invert_axis ? 1 : 0);
}