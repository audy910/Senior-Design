// New Main
#include "gps.h"
#include "imu.h"
#include "can.h"
#include "ultrasonic.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting sensor tasks...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize I2C for BNO055
    i2c_master_init();
    
    // Initialize GNSS UART
    gnss_uart_init();
    ESP_LOGI(TAG, "GNSS initialized");
    
    // Initialize BNO055 sensor
    bno055_init();
    ESP_LOGI(TAG, "BNO055 initialized");
    
    // Initialize Ultrasonic and Line Sensors
    ultrasonic_sensors_init_all();
    ESP_LOGI(TAG, "Proximity sensors initialized");
    
    // Initialize CAN bus
    init_can_bus();
    ESP_LOGI(TAG, "CAN bus initialized");
    
    // Create BNO055 task on Core 0 (20ms = 50 Hz)
    xTaskCreatePinnedToCore(
        bno055_read_task,
        "bno055_task",
        4096,
        NULL,
        5,                      // Priority: Medium
        NULL,
        0                       // Core 0
    );
    
    // Create GNSS task on Core 1 (100ms = 10 Hz)
    xTaskCreatePinnedToCore(
        gnss_uart_task,
        "gnss_uart_task",
        4096,
        NULL,
        10,                     // Priority: Highest (time-sensitive GPS)
        NULL,
        1                       // Core 1
    );
    
    // Create Proximity sensors task on Core 0 (50ms = 20 Hz)
    xTaskCreatePinnedToCore(
        proximity_sensors_task,
        "proximity_sensor_task",
        4096,
        NULL,
        7,                      // Priority: Medium-High (safety critical)
        NULL,
        0                       // Core 0
    );
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "Task Periods: GPS=100ms, IMU=20ms, Proximity=50ms");
}