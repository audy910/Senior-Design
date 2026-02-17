#pragma once

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include "esp_rom_sys.h"

#include "can.h"

// Ultrasonic pins
#define TRIG_GPIO_FRONT GPIO_NUM_4
#define ECHO_GPIO_FRONT GPIO_NUM_6
#define TRIG_GPIO_BACK  GPIO_NUM_7
#define ECHO_GPIO_BACK  GPIO_NUM_15

// Line sensor ADC channels
#define ADC_CHAN_LEFT   ADC1_CHANNEL_4
#define ADC_CHAN_CENTER ADC1_CHANNEL_8
#define ADC_CHAN_RIGHT  ADC1_CHANNEL_9

// Thresholds
#define TIMEOUT_US      30000  // 30ms timeout
#define CLIFF_THRESHOLD 10     // ADC reading below this = lifted/cliff

// Sensor IDs
typedef enum {
    SENSOR_FRONT = 0,
    SENSOR_BACK = 1
} ultrasonic_id_t;

// Line sensor data
typedef struct {
    int left;
    int center;
    int right;
} line_sensor_data_t;

// Combined proximity data
typedef struct {
    int front_cm;
    int back_cm;
    line_sensor_data_t line;
    bool front_valid;
    bool back_valid;
    bool cliff_detected;
} proximity_data_t;

// Function declarations
void configure_ultrasonic_pins(int trig_pin, int echo_pin);
void configure_line_sensor_legacy(void);
void ultrasonic_sensors_init_all(void);
int read_ultrasonic_cm(ultrasonic_id_t sensor);
line_sensor_data_t read_line_sensor(void);
bool is_robot_lifted(line_sensor_data_t data);
void read_all_proximity_sensors(proximity_data_t *data);
void send_proximity_sensors(int front_cm, int back_cm, 
                           int line_left, int line_center, int line_right,
                           bool front_valid, bool back_valid, bool cliff_detected);
void proximity_sensors_task(void *arg);