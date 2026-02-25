#include "ultrasonic.h"

static const char *TAG = "ULTRASONIC";
#define SPEED_OF_SOUND_CM_US 0.0346f // Speed at ~25°C (346 m/s)
#define ULTRASONIC_SAMPLES 3         // Number of samples to average
void configure_ultrasonic_pins(int trig_pin, int echo_pin)
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

void configure_line_sensor_legacy(void)
{
    // Configure ADC Width - 12 Bit = values 0 to 4095
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure Attenuation (Voltage Range)
    adc1_config_channel_atten(ADC_CHAN_LEFT, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC_CHAN_CENTER, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC_CHAN_RIGHT, ADC_ATTEN_DB_12);
}

void ultrasonic_sensors_init_all(void)
{
    configure_ultrasonic_pins(TRIG_GPIO_BACK, ECHO_GPIO_BACK);
    configure_ultrasonic_pins(TRIG_GPIO_FRONT, ECHO_GPIO_FRONT);
    configure_line_sensor_legacy();
    
    ESP_LOGI(TAG, "Ultrasonic and line sensors initialized");
}


int read_ultrasonic_cm(ultrasonic_id_t sensor)
{
    gpio_num_t trig_pin = (sensor == SENSOR_FRONT) ? TRIG_GPIO_FRONT : TRIG_GPIO_BACK;
    gpio_num_t echo_pin = (sensor == SENSOR_FRONT) ? ECHO_GPIO_FRONT : ECHO_GPIO_BACK;
    
    float total_dist = 0;
    int valid_samples = 0;

    for (int i = 0; i < ULTRASONIC_SAMPLES; i++) {
        // 1. Trigger pulse
        gpio_set_level(trig_pin, 0);
        esp_rom_delay_us(4);
        gpio_set_level(trig_pin, 1);
        esp_rom_delay_us(10);
        gpio_set_level(trig_pin, 0);
        
        // 2. Wait for echo high with timeout
        int64_t start = esp_timer_get_time();
        while (gpio_get_level(echo_pin) == 0) {
            if (esp_timer_get_time() - start > TIMEOUT_US) {
                goto next_sample; // Skip this sample on timeout
            }
        }
        
        // 3. Measure pulse width
        int64_t pulse_start = esp_timer_get_time();
        while (gpio_get_level(echo_pin) == 1) {
            if (esp_timer_get_time() - pulse_start > TIMEOUT_US) {
                goto next_sample;
            }
        }
        
        int64_t pulse_us = esp_timer_get_time() - pulse_start;
        
        // Use a more accurate speed of sound constant
        float dist = (pulse_us * SPEED_OF_SOUND_CM_US) / 2.0f;
        
        // Basic sanity check: HC-SR04 is only reliable between 2cm and 400cm
        if (dist >= 2.0f && dist <= 400.0f) {
            total_dist += dist;
            valid_samples++;
        }

    next_sample:
        // Small "settling" delay between internal samples to avoid interference
        if (i < ULTRASONIC_SAMPLES - 1) {
            esp_rom_delay_us(2000); 
        }
    }

    if (valid_samples == 0) return -1;

    return (int)((total_dist / valid_samples) + 0.5f);
}

line_sensor_data_t read_line_sensor(void)
{
    line_sensor_data_t data;
    
    // Average multiple readings for stability
    int left_sum = 0, center_sum = 0, right_sum = 0;
    for (int i = 0; i < 5; i++) {
        left_sum += adc1_get_raw(ADC_CHAN_LEFT);
        center_sum += adc1_get_raw(ADC_CHAN_CENTER);
        right_sum += adc1_get_raw(ADC_CHAN_RIGHT);
    }
    
    data.left = left_sum / 5;
    data.center = center_sum / 5;
    data.right = right_sum / 5;
    
    return data;
}

bool is_robot_lifted(line_sensor_data_t data)
{
    // All sensors read very low = no surface detected
    if (data.left < CLIFF_THRESHOLD && 
        data.center < CLIFF_THRESHOLD && 
        data.right < CLIFF_THRESHOLD) {
        return true;
    }
    return false;
}

void read_all_proximity_sensors(proximity_data_t *data)
{
    // Read front ultrasonic
    data->front_cm = read_ultrasonic_cm(SENSOR_FRONT);
    data->front_valid = (data->front_cm >= 0);
    
    // Small delay between sensor reads
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Read back ultrasonic
    data->back_cm = read_ultrasonic_cm(SENSOR_BACK);
    data->back_valid = (data->back_cm >= 0);
    
    // Read line sensors
    data->line = read_line_sensor();
    
    // Check for cliff
    data->cliff_detected = is_robot_lifted(data->line);
}

void send_proximity_sensors(int front_cm, int back_cm, 
                           int line_left, int line_center, int line_right,
                           bool front_valid, bool back_valid, bool cliff_detected)
{
    uint8_t data[8] = {0};
    
    // Front distance in mm (16 bits)
    uint16_t front_mm = (front_cm >= 0) ? front_cm * 10 : 0;
    data[0] = (front_mm >> 0) & 0xFF;
    data[1] = (front_mm >> 8) & 0xFF;
    
    // Back distance in mm (16 bits)
    uint16_t back_mm = (back_cm >= 0) ? back_cm * 10 : 0;
    data[2] = (back_mm >> 0) & 0xFF;
    data[3] = (back_mm >> 8) & 0xFF;
    
    // Line sensor center reading scaled to mm
    // Higher ADC value = closer to surface (darker reading)
    // Improved mapping based on ADC range (0-4095 for 12-bit ADC)
    // When robot is lifted/cliff: ADC < CLIFF_THRESHOLD (10)
    // Normal surface: ADC typically 100-4000 depending on surface
    uint16_t cliff_distance;
    if (line_center < CLIFF_THRESHOLD) {
        // Cliff detected - very far from surface
        cliff_distance = 1000;  // 1000mm = 1 meter (max distance)
    } else if (line_center < 50) {
        // Very light surface or partially lifted
        cliff_distance = 500;  // 500mm
    } else if (line_center < 200) {
        // Light surface
        cliff_distance = 100;  // 100mm
    } else {
        // Normal/dark surface - close to ground
        cliff_distance = 50;   // 50mm nominal ground clearance
    }

    data[4] = (cliff_distance >> 0) & 0xFF;
    data[5] = (cliff_distance >> 8) & 0xFF;

    // Cliff detected flag
    data[6] = cliff_detected ? 1 : 0;

    // Valid flags
    data[7] = 0;
    data[7] |= (front_valid ? 1 : 0) << 0;
    data[7] |= (back_valid ? 1 : 0) << 1;
    data[7] |= (1) << 2;  // Cliff sensor is always valid (line sensors always read)
    
    send_can_message(0x107, data, 8);
}

void proximity_sensors_task(void *arg)
{
    proximity_data_t prox_data;
    
    ESP_LOGI("PROXIMITY", "Proximity sensors task started");

    static int count = 0;
    
    while (1) {
        // Read all sensors
        read_all_proximity_sensors(&prox_data);
        count++;
        
        // Check for cliff - EMERGENCY condition
        if (prox_data.cliff_detected) {
            if(count == 20){
                ESP_LOGE("PROXIMITY", "!!! EMERGENCY STOP !!! No Surface Detected!");
                ESP_LOGE("PROXIMITY", "Line Sensor [L:%4d C:%4d R:%4d]", 
                        prox_data.line.left, prox_data.line.center, prox_data.line.right);
            }
            // Send CAN message with cliff warning
            send_proximity_sensors(
                prox_data.front_cm, 
                prox_data.back_cm,
                prox_data.line.left,
                prox_data.line.center,
                prox_data.line.right,
                prox_data.front_valid, 
                prox_data.back_valid, 
                true  // cliff_detected
            );
        } else {
            // Normal operation
            if(count == 20){
                 ESP_LOGI("PROXIMITY", 
                "Line [L:%4d C:%4d R:%4d] | Front: %s%dcm | Back: %s%dcm",
                prox_data.line.left, 
                prox_data.line.center, 
                prox_data.line.right,
                prox_data.front_valid ? "" : "INVALID ",
                prox_data.front_cm >= 0 ? prox_data.front_cm : 0,
                prox_data.back_valid ? "" : "INVALID ",
                prox_data.back_cm >= 0 ? prox_data.back_cm : 0);
            }
            // Send CAN message
            send_proximity_sensors(
                prox_data.front_cm, 
                prox_data.back_cm,
                prox_data.line.left,
                prox_data.line.center,
                prox_data.line.right,
                prox_data.front_valid, 
                prox_data.back_valid, 
                false  // cliff_detected
            );
        }
        
        // Run at 20 Hz (50ms period)
        vTaskDelay(pdMS_TO_TICKS(50));

        if(count == 20){ count = 0;}
    }
}