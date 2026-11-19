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

if(PROJECT_NAME STREQUAL "esp32s3_mpu_sender")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC CONFIG_NODE_ROLE_SENDER=1)
else()
    target_compile_definitions(${COMPONENT_LIB} PUBLIC CONFIG_NODE_ROLE_RECEIVER=1)
endif()