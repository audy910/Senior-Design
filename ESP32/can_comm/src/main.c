#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"

// Used in Debug Messages
static const char *TAG = "CAN_SENDER";

#define TX_GPIO_NUM   GPIO_NUM_4
#define RX_GPIO_NUM   GPIO_NUM_5

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Now configure for TWAI
    ESP_LOGI(TAG, "Configuring TWAI driver...");
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        TX_GPIO_NUM, 
        RX_GPIO_NUM, 
        TWAI_MODE_NORMAL
    );
    
    g_config.tx_queue_len = 5;
    g_config.rx_queue_len = 5;
    
    // Enable alerts
    g_config.alerts_enabled = TWAI_ALERT_TX_IDLE | 
                              TWAI_ALERT_TX_SUCCESS | 
                              TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_BUS_ERROR;

    g_config.clkout_io = TWAI_IO_UNUSED;
    g_config.bus_off_io = TWAI_IO_UNUSED;
    g_config.clkout_divider = 0;
    
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_LOGI(TAG, "Installing TWAI driver...");
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Driver install failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Driver installed successfully");

    ESP_LOGI(TAG, "Starting TWAI driver...");
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Driver start failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Driver started in NORMAL mode");
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Check initial status
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        ESP_LOGI(TAG, "Initial State: %d (1=RUNNING, 2=BUS_OFF)", status.state);
        ESP_LOGI(TAG, "TX errors: %d, RX errors: %d", 
                 status.tx_error_counter, status.rx_error_counter);
    }
    
    ESP_LOGI(TAG, "Starting transmission test...");
    
    uint32_t success_count = 0;
    uint32_t fail_count = 0;
    
    while (1) {
        // Check for alerts
        uint32_t alerts;
        if (twai_read_alerts(&alerts, 0) == ESP_OK) {
            if (alerts & TWAI_ALERT_TX_SUCCESS) {
                ESP_LOGI(TAG, "Alert: TX_SUCCESS");
            }
            if (alerts & TWAI_ALERT_TX_FAILED) {
                ESP_LOGW(TAG, "Alert: TX_FAILED");
            }
            if (alerts & TWAI_ALERT_BUS_ERROR) {
                ESP_LOGE(TAG, "Alert: BUS_ERROR");
            }
        }
        
        twai_message_t message = {
            .identifier = 0x123,
            .data_length_code = 8,
            .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
        };
        
        err = twai_transmit(&message, pdMS_TO_TICKS(1000));
        
        if (err == ESP_OK) {
            success_count++;
            ESP_LOGI(TAG, "TX SUCCESS [%lu] - ID: 0x%03lX", 
                     success_count, message.identifier);
        } else {
            fail_count++;
            ESP_LOGE(TAG, "TX FAILED [%lu] - Error: %s", 
                     fail_count, esp_err_to_name(err));
            
            // Get detailed status on failure
            if (twai_get_status_info(&status) == ESP_OK) {
                ESP_LOGE(TAG, "  State: %d, TX_err: %d, RX_err: %d, Queued: %d", 
                         status.state, 
                         status.tx_error_counter,
                         status.rx_error_counter,
                         status.msgs_to_tx);
            }
        }
        
        // Print summary every 10 messages
        if ((success_count + fail_count) % 10 == 0) {
            ESP_LOGI(TAG, "Summary: Success=%lu, Failed=%lu", success_count, fail_count);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}