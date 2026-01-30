#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <rom/ets_sys.h>

// --- Pin Definitions ---
#define TRIG_GPIO_BACK  GPIO_NUM_4
#define ECHO_GPIO_BACK  GPIO_NUM_6

#define TRIG_GPIO_FRONT GPIO_NUM_7
#define ECHO_GPIO_FRONT GPIO_NUM_15

#define TIMEOUT_US 30000 

static const char *TAG = "ULTRASONIC";

// 1. Create readable names for your sensors
typedef enum {
    SENSOR_BACK = 0,
    SENSOR_FRONT = 1
} sensor_select_t;

/* ---------- GPIO setup ---------- */
// This helper configures one specific pair of pins
static void configure_sensor_pins(int trig_pin, int echo_pin)
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
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // Pull-down is safer for loose wires
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_conf);
}

// Call this once to set up BOTH sensors
static void ultrasonic_init_all(void)
{
    configure_sensor_pins(TRIG_GPIO_BACK, ECHO_GPIO_BACK);
    configure_sensor_pins(TRIG_GPIO_FRONT, ECHO_GPIO_FRONT);
}

/* ---------- Read distance ---------- */
// Now accepts the variable 'sensor' to choose which one to read
static int ultrasonic_read_cm(sensor_select_t sensor)
{
    gpio_num_t trig_pin;
    gpio_num_t echo_pin;

    // 2. Select the correct pins based on the input variable
    if (sensor == SENSOR_FRONT) {
        trig_pin = TRIG_GPIO_FRONT;
        echo_pin = ECHO_GPIO_FRONT;
    } else {
        // Default to BACK
        trig_pin = TRIG_GPIO_BACK;
        echo_pin = ECHO_GPIO_BACK;
    }

    // 3. The Logic (Using the selected pins)
    
    // Prepare Trigger
    gpio_set_level(trig_pin, 0);
    ets_delay_us(4);

    // Send 10us Pulse
    gpio_set_level(trig_pin, 1);
    ets_delay_us(10);
    gpio_set_level(trig_pin, 0);

    // Wait for Echo to Start
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 0) {
        if (esp_timer_get_time() - start > TIMEOUT_US) return -1;
    }

    // Measure Echo Width
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(echo_pin) == 1) {
        if (esp_timer_get_time() - pulse_start > TIMEOUT_US) return -1;
    }

    int64_t pulse_us = esp_timer_get_time() - pulse_start;

    // Calculate Distance
    float dist = (pulse_us * 0.0343f) / 2.0f;
    return (int)(dist + 0.5f);
}

/* ---------- Task ---------- */
static void ultrasonic_task(void *arg)
{
    // Initialize everything once
    ultrasonic_init_all();

    while (1) {
        // 4. Usage Example: Just pass the name!
        int dist_front = ultrasonic_read_cm(SENSOR_FRONT);
        
        // Small delay to prevent sensors interfering with each other
        vTaskDelay(pdMS_TO_TICKS(50)); 
        
        int dist_back = ultrasonic_read_cm(SENSOR_BACK);

        ESP_LOGI(TAG, "FRONT: %d cm | BACK: %d cm", dist_front, dist_back);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ---------- Entry Point ---------- */


void app_main(void)
{
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 5, NULL);
    vTaskDelete(NULL);
}

