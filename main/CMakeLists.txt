idf_component_register(
    SRCS
        "main.cpp"
        "servo_controller.cpp"
        "command_processor.cpp"
        "ble_server.cpp"
        "console_serial.cpp"
        "mpu6050.cpp"
        "imu.cpp"
        "mpu_control.cpp"
        "espnow_sync.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash driver esp_timer bt esp_wifi esp_event esp_netif
)