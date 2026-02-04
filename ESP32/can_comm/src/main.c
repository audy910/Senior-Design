// New Main
#include "gps.h"
#include "imu.h"
#include "can.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting sensor tasks...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize I2C
    i2c_master_init();

    // Initialize GNSS
    gnss_uart_init();

    ESP_LOGI(TAG, "GNSS task started");
    
    // Initialize BNO055 sensor
    bno055_init();

    // Initialize CAN
    init_can_bus();
    
    // // Create BNO055 task on Core 0
    xTaskCreatePinnedToCore(
        bno055_read_task,       
        "bno055_task",          
        4096,                   
        NULL,                   
        5,                      
        NULL,                   
        0                       
    );
    
    // Create GNSS task on Core 1
    xTaskCreatePinnedToCore(
        gnss_uart_task,         
        "gnss_uart_task",       
        4096,                   
        NULL,                   
        10,                     // Priority
        NULL,                   
        1                       
    );
    
    ESP_LOGI(TAG, "Tasks created successfully");
}