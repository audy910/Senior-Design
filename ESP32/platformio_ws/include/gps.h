#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "can.h"

// UART Configuration
#define GNSS_UART           UART_NUM_1
#define GNSS_TX_PIN         11
#define GNSS_RX_PIN         12
#define GNSS_BAUDRATE       115200  // M100 Mini uses 115200
#define UART_BUF_SIZE       2048

// Navigation PVT structure
typedef struct {
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;        // degrees * 1e7
    int32_t  lat;        // degrees * 1e7
    int32_t  height;     // mm
    uint32_t hAcc;       // mm
    uint32_t vAcc;       // mm
    int32_t  velN;       // mm/s
    int32_t  velE;       // mm/s
    int32_t  velD;       // mm/s
    int32_t  gSpeed;     // mm/s
    int32_t  heading;    // degrees * 1e5
    uint16_t pDOP;       // * 100
    int32_t  headMot;    // degrees * 1e5
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
    float horiz_acc_m;
    float vert_acc_m;
    nav_quality_t quality;
} nav_status_t;

// Function prototypes
void gnss_uart_init(void);
void configure_gps_output(void);
void gnss_uart_task(void *arg);
bool nav_ready(const nav_status_t *s);
nav_status_t interpret_nav_pvt(const nav_pvt_t *pvt);

bool nmea_parse_byte(uint8_t byte, nav_pvt_t *out);

void send_gps_position(double lat, double lon);
void send_gps_velocity(float speed, float course, uint8_t fix_type, uint8_t num_sats, float hdop);
void send_gps_accuracy(float h_acc, float v_acc);