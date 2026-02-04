#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "can.h"

#define GNSS_UART          UART_NUM_1
#define GNSS_TX_PIN        GPIO_NUM_15
#define GNSS_RX_PIN        GPIO_NUM_16

#define GNSS_BAUDRATE      460800
#define UART_BUF_SIZE      2048

typedef struct {
    int32_t lon;        // 1e-7 deg
    int32_t lat;        // 1e-7 deg
    int32_t height;     // mm
    int32_t velN;       // mm/s
    int32_t velE;       // mm/s
    int32_t velD;       // mm/s
    uint32_t gSpeed;    // mm/s
    int32_t heading;    // 1e-5 deg (Heading of vehicle)
    int32_t headMot;    // 1e-5 deg (Heading of motion)
    uint32_t hAcc;      // mm
    uint32_t vAcc;      // mm
    uint16_t pDOP;      // 0.01 units
    uint8_t numSV;      // Number of satellites
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
} nav_pvt_t;

typedef enum {
    NAV_QUALITY_INVALID,
    NAV_QUALITY_COARSE,
    NAV_QUALITY_GOOD,
    NAV_QUALITY_EXCELLENT,
    NAV_QUALITY_RTK
} nav_quality_t;

typedef struct {
    bool gnss_ok;
    bool stable;
    bool dead_reckoning;
    nav_quality_t quality;
    float horiz_acc_m;
    float vert_acc_m;
} nav_status_t;


void gnss_uart_init(void);
bool nav_ready(const nav_status_t *s);
nav_status_t interpret_nav_pvt(const nav_pvt_t *pvt);
void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b);
bool ubx_parse_byte(uint8_t byte, nav_pvt_t *out);
void gnss_uart_task(void *arg);

void send_gps_position(double lat, double lon);
void send_gps_velocity(float speed, float course, uint8_t fix_type, uint8_t num_sats, float hdop);
void send_gps_accuracy(float h_acc, float v_acc);