#pragma once

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "rover.h"

// CAN configuration for ESP32-S3
#define CAN_TX_GPIO GPIO_NUM_17
#define CAN_RX_GPIO GPIO_NUM_18
#define CAN_BITRATE TWAI_TIMING_CONFIG_500KBITS()

// Update rates (in milliseconds)
#define IMU_UPDATE_RATE_MS 20
#define GPS_UPDATE_RATE_MS 100
#define PROXIMITY_UPDATE_RATE_MS 50

void init_can_bus();
void send_can_message(uint32_t id, uint8_t *data, uint8_t len);

void send_gps_position(double lat, double lon);
void send_gps_velocity(float speed, float course, uint8_t fix_type, uint8_t num_sats, float hdop);
void send_gps_accuracy(float h_acc, float v_acc);
void send_imu_accel(float x, float y, float z, uint8_t cal);
void send_imu_gyro(float x, float y, float z, uint8_t cal);
void send_imu_orientation(float heading, float pitch, float roll, uint8_t cal_sys, uint8_t cal_mag);
void send_imu_quaternion(float w, float x, float y, float z);

// void task_send_imu_data(void *pvParameters);
// void task_send_gps_data(void *pvParameters);
// void simulate_imu_data(float *accel_x, float *accel_y, float *accel_z);
// void simulate_gps_data(double *lat, double *lon, float *speed, float *course);