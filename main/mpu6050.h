#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// MPU-6050 wiring on YD-ESP32-S3 (I2C0):
//   VCC -> 3.3V
//   GND -> GND
//   SDA -> GPIO 8
//   SCL -> GPIO 9
//   AD0 -> GND (address 0x68) or 3.3V (address 0x69)
//   INT -> not connected

#define MPU6050_SDA_PIN          GPIO_NUM_8
#define MPU6050_SCL_PIN          GPIO_NUM_9
#define MPU6050_I2C_FREQ_HZ      400000

#define MPU6050_ADDR_LOW         0x68
#define MPU6050_ADDR_HIGH        0x69

// Full-scale ranges configured by mpu6050_init(): +/-2 g and +/-500 deg/s
#define MPU6050_ACCEL_LSB_PER_G  16384.0f
#define MPU6050_GYRO_LSB_PER_DPS 65.5f

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu6050_raw_t;

// Installs the I2C driver (once), probes 0x68 then 0x69 and configures the sensor.
// Safe to call repeatedly: it is used to re-probe after a lost connection.
bool mpu6050_init(void);
bool mpu6050_is_present(void);
uint8_t mpu6050_address(void);
bool mpu6050_read_raw(mpu6050_raw_t *out);

#ifdef __cplusplus
}
#endif
