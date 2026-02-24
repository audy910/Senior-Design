#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "can.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO           21       // I2C SCL
#define I2C_MASTER_SDA_IO           20       // I2C SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// BNO055 Configuration
#define BNO055_ADDR                 0x28
#define BNO055_CHIP_ID_ADDR         0x00
#define BNO055_CHIP_ID              0xA0

// BNO055 Register Addresses
#define BNO055_PAGE_ID_ADDR         0x07
#define BNO055_OPR_MODE_ADDR        0x3D
#define BNO055_PWR_MODE_ADDR        0x3E
#define BNO055_SYS_TRIGGER_ADDR     0x3F

// Data Registers
#define BNO055_ACCEL_DATA_X_LSB     0x08
#define BNO055_GYRO_DATA_X_LSB      0x14
#define BNO055_MAG_DATA_X_LSB       0x0E
#define BNO055_EULER_H_LSB          0x1A
#define BNO055_QUAT_DATA_W_LSB      0x20
#define BNO055_LINEAR_ACCEL_X_LSB   0x28
#define BNO055_GRAVITY_X_LSB        0x2E

// Calibration Status
#define BNO055_CALIB_STAT_ADDR      0x35

// Operation Modes
#define OPERATION_MODE_CONFIG       0x00
#define OPERATION_MODE_NDOF         0x0C    // 9DOF with sensor fusion
#define OPERATION_MODE_IMU          0x08    // 6DOF (no magnetometer)
#define OPERATION_MODE_COMPASS      0x09
#define OPERATION_MODE_M4G          0x0A
#define OPERATION_MODE_NDOF_FMC_OFF 0x0B

// Power Modes
#define POWER_MODE_NORMAL           0x00
#define POWER_MODE_LOWPOWER         0x01
#define POWER_MODE_SUSPEND          0x02

#define STORAGE_NAMESPACE "storage"
#define CALIB_KEY "bno055_cal"
#define BNO055_OFFSET_DATA_ADDR 0x55

esp_err_t save_calibration(uint8_t *data);
esp_err_t load_calibration(uint8_t *data);
esp_err_t i2c_master_init(void);
esp_err_t bno055_write_byte(uint8_t reg_addr, uint8_t data);
esp_err_t bno055_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t bno055_set_mode(uint8_t mode);
esp_err_t bno055_init(void);
esp_err_t bno055_read_euler(float *heading, float *roll, float *pitch);
esp_err_t bno055_read_quaternion(float *w, float *x, float *y, float *z);
esp_err_t bno055_read_accel(float *x, float *y, float *z);
esp_err_t bno055_get_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);
void bno055_read_task(void *pvParameters);
void send_imu_accel(float x, float y, float z, uint8_t cal);
void send_imu_gyro(float x, float y, float z, uint8_t cal);
void send_imu_orientation(float heading, float pitch, float roll, uint8_t cal_sys, uint8_t cal_mag);
void send_imu_quaternion(float w, float x, float y, float z);