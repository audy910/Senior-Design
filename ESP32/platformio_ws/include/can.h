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
#define CAN_TX_GPIO GPIO_NUM_1
#define CAN_RX_GPIO GPIO_NUM_2
#define CAN_BITRATE TWAI_TIMING_CONFIG_500KBITS()

// Update rates (in milliseconds)
#define IMU_UPDATE_RATE_MS 20
#define GPS_UPDATE_RATE_MS 100
#define PROXIMITY_UPDATE_RATE_MS 50

void init_can_bus();
void send_can_message(uint32_t id, uint8_t *data, uint8_t len);