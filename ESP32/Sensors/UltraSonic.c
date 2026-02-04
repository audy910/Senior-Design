#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <rom/ets_sys.h>
#include "driver/adc.h" 

/* PINOUT
    back ultrasonic:   TRIG_GPIO_BACK  = GPIO4,  ECHO_GPIO_BACK  = GPIO6
    front ultrasonic:  TRIG_GPIO_FRONT = GPIO7,  ECHO_GPIO_FRONT = GPIO15
    grayscale sensor:  ADC_CHAN_LEFT   = ADC1_CHANNEL_4 (GPIO5)
                       ADC_CHAN_CENTER = ADC1_CHANNEL_8 (GPIO9)
                       ADC_CHAN_RIGHT  = ADC1_CHANNEL_9 (GPIO10)
*/

// --- ULTRASONIC PINS ---
#define TRIG_GPIO_BACK  GPIO_NUM_4
#define ECHO_GPIO_BACK  GPIO_NUM_6
#define TRIG_GPIO_FRONT GPIO_NUM_7
#define ECHO_GPIO_FRONT GPIO_NUM_15

// --- GRAYSCALE SENSOR PINS  ---
#define ADC_CHAN_LEFT   ADC1_CHANNEL_4
#define ADC_CHAN_CENTER ADC1_CHANNEL_8
#define ADC_CHAN_RIGHT  ADC1_CHANNEL_9

#define TIMEOUT_US 30000 

static const char *TAG = "SENSORS";

typedef enum {
    SENSOR_BACK = 0,
    SENSOR_FRONT = 1
} ultrasonic_id_t;

typedef struct {
    int left;   
    int center; 
    int right;  
} line_sensor_data_t;

/* ================== GPIO CONFIGURATION ================== */
static void configure_ultrasonic_pins(int trig_pin, int echo_pin)
{
    gpio_config_t trig_conf = {
        .pin_bit_mask = 1ULL << trig_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_conf);
    gpio_set_level(trig_pin, 0);

    gpio_config_t echo_conf = {
        .pin_bit_mask = 1ULL << echo_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE, 
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_conf);
}


static void configure_line_sensor_legacy(void)
{
    // 1. Configure the ADC Width 
    // 12 Bit = values 0 to 4095
    adc1_config_width(ADC_WIDTH_BIT_12);

    // 2. Configure Attenuation (Voltage Range)
    adc1_config_channel_atten(ADC_CHAN_LEFT, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC_CHAN_CENTER, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC_CHAN_RIGHT, ADC_ATTEN_DB_12);
}

static void sensors_init_all(void)
{
    configure_ultrasonic_pins(TRIG_GPIO_BACK, ECHO_GPIO_BACK);
    configure_ultrasonic_pins(TRIG_GPIO_FRONT, ECHO_GPIO_FRONT);
    configure_line_sensor_legacy(); 
}

/* ================== READ FUNCTIONS ================== */

static int read_ultrasonic_cm(ultrasonic_id_t sensor)
{
    gpio_num_t trig_pin = (sensor == SENSOR_FRONT) ? TRIG_GPIO_FRONT : TRIG_GPIO_BACK;
    gpio_num_t echo_pin = (sensor == SENSOR_FRONT) ? ECHO_GPIO_FRONT : ECHO_GPIO_BACK;

    // Trigger
    gpio_set_level(trig_pin, 0);
    ets_delay_us(4);
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);

    // Measure
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if (esp_timer_get_time() - start > TIMEOUT_US) return -1;
    }

    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) {
        if (esp_timer_get_time() - pulse_start > TIMEOUT_US) return -1;
    }

    int64_t pulse_us = esp_timer_get_time() - pulse_start;
    float dist = (pulse_us * 0.0343f) / 2.0f;
    return (int)(dist + 0.5f);
}

static line_sensor_data_t read_line_sensor(void)
{
    line_sensor_data_t data;
    // Legacy Read Function
    data.left   = adc1_get_raw(ADC_CHAN_LEFT);
    data.center = adc1_get_raw(ADC_CHAN_CENTER);
    data.right  = adc1_get_raw(ADC_CHAN_RIGHT);
    return data;
}

static bool is_robot_lifted(line_sensor_data_t data)
{
    if (data.left < 10 && data.center < 10 && data.right < 10) {
        return true;
    }
    return false;
}

/* ================== MAIN TASK ================== */

static void sensor_task(void *arg)
{
    sensors_init_all();
    ESP_LOGI(TAG, "Sensors Initialized");

    while (1) {
        // Read Sensors
        int dist_front = read_ultrasonic_cm(SENSOR_FRONT);
        vTaskDelay(pdMS_TO_TICKS(10));
        int dist_back = read_ultrasonic_cm(SENSOR_BACK);
        line_sensor_data_t line = read_line_sensor();

        // Check for cliff
        if (is_robot_lifted(line)) {
            ESP_LOGE(TAG, "!!! STOP !!! No Surface Detected ");
             // TODO: send emergency stop signal to navQ
        } 
        else {
            ESP_LOGI(TAG, "Gray [L:%4d C:%4d R:%4d] | Front: %dcm | Back: %dcm", 
                     line.left, line.center, line.right, dist_front, dist_back);
            // TODO: send metrics to navQ
        }

        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
} 
void app_main(void)
{
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}