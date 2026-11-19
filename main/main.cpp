#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo_controller.h"
#include "ble_server.h"
#include "console_serial.h"
#include "command_processor.h"
#include "imu.h"
#include "mpu_control.h"

static const char *TAG = "APP_MAIN";

extern "C" void app_main(void) {
    // Silence high-frequency NimBLE stack notify logging on console
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    // Clamp servo signal pins LOW before anything else so floating pins never twitch servos
    const gpio_num_t servo_pins[SERVO_COUNT] = { SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN };
    for (int i = 0; i < SERVO_COUNT; i++) {
        gpio_reset_pin(servo_pins[i]);
        gpio_set_direction(servo_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(servo_pins[i], 0);
        gpio_pulldown_en(servo_pins[i]);
    }

    // Allow battery rail and decoupling capacitors to stabilize before RF calibration
    vTaskDelay(pdMS_TO_TICKS(350));

    esp_reset_reason_t rst = esp_reset_reason();
    if (rst == ESP_RST_BROWNOUT) {
        ESP_LOGW(TAG, "WARNING: Previous reset was caused by BROWNOUT (battery voltage sagged under load)!");
    }

    ESP_LOGI(TAG, "Booting YD-ESP32-S3 3-Servo MP90S Arm Controller + MPU-6050 (reset_reason=%d)...", (int)rst);

    // 1. Flash Storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Command Processor Subsystem
    command_processor_init();

    // 3. Bluetooth Low Energy NUS Server ('ESP32S3-Arm')
    ble_server_init(command_on_ble_received);

    // 4. 3-Servo LEDC PWM Controller (starts DETACHED on Claw=GP1, Arm=GP38, Shoulder=GP39)
    servo_controller_init();

    // 5. MPU-6050 IMU (I2C: SDA=GP8, SCL=GP9) with fusion, calibration and BLE telemetry
    imu_init();

    // 6. MPU motion control (roll -> Arm, pitch -> Shoulder, yaw -> Claw), starts OFF
    mpu_control_init();

    // 7. Serial Console (UART0 & Native USB-Serial-JTAG REPL)
    console_serial_init();

    ESP_LOGI(TAG, "All subsystems running. Arm ready for MPU-6050 and BLE control.");
}