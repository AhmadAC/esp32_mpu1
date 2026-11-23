#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#include "node_config.h"
#include "servo_controller.h"
#include "ble_server.h"
#include "console_serial.h"
#include "command_processor.h"
#include "imu.h"
#include "mpu_control.h"
#include "espnow_sync.h"

static const char *TAG = "APP_MAIN";

#if NODE_IS_RECEIVER
// Forward command replies back over ESP-NOW to MPU Sender and local BLE
static void on_espnow_command_reply(const char *reply) {
    if (reply && strlen(reply) > 0) {
        espnow_sync_send_reply(reply);
        ble_server_send_notify(reply);
    }
}

static void on_espnow_command(const char *cmd_str) {
    command_process(cmd_str, on_espnow_command_reply);
}
#endif

extern "C" void app_main(void) {
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);

#if NODE_IS_RECEIVER
    // Initialize default physical servo signal pins cleanly: Claw(GP4), Arm(GP5), Shoulder(GP6), Scissor(GP12)
    const gpio_num_t servo_pins[SERVO_COUNT] = {
        SERVO_1_PIN_DEFAULT,
        SERVO_2_PIN_DEFAULT,
        SERVO_3_PIN_DEFAULT,
        SERVO_4_PIN_DEFAULT
    };
    for (int i = 0; i < SERVO_COUNT; i++) {
        gpio_reset_pin(servo_pins[i]);
        gpio_set_direction(servo_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(servo_pins[i], 0);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(350));

    esp_reset_reason_t rst = esp_reset_reason();
    if (rst == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "WARNING: Previous reset was caused by BROWNOUT (battery sag under load)!");
    }

    ESP_LOGI(TAG, "Booting YD-ESP32-S3 Node [%s] (Reset Reason: %d)...", NODE_ROLE_NAME, (int)rst);

    // 1. Flash Storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Command Processor Subsystem
    command_processor_init();

    // 3. Bluetooth Low Energy Service
    ble_server_init(command_on_ble_received);

#if NODE_IS_RECEIVER
    // 4. Arm Receiver: 4-Servo LEDC PWM Controller (Claw GP4, Arm GP5, Shoulder GP6, Scissor Lift GP12)
    servo_controller_init();

    // 5. Arm Receiver: ESP-NOW Auto-Sync Receiver
    espnow_sync_init(ESPNOW_ROLE_RECEIVER);
    espnow_sync_register_cmd_handler(on_espnow_command);
#endif

#if NODE_IS_SENDER
    // 4. MPU Sender: MPU-6050 IMU on I2C (SDA GP8, SCL GP9)
    imu_init();

    // 5. MPU Sender: Orientation-to-Arm Motion Mapper
    mpu_control_init();

    // 6. MPU Sender: ESP-NOW Auto-Sync Master
    espnow_sync_init(ESPNOW_ROLE_SENDER);
#endif

    // 7. USB-Serial / JTAG Console
    console_serial_init();

    ESP_LOGI(TAG, "Subsystems running for node [%s]. Ready for automatic ESP-NOW pairing.", NODE_ROLE_NAME);
}