#include "can.h"

static const char *TAG = "CAN";

void init_can_bus(void)
{
    // Configure TWAI general settings
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    
    // Configure TWAI timing - 500 kbps
    twai_timing_config_t t_config = CAN_BITRATE;
    
    // Configure TWAI filter to accept all messages
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Install TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    
    // Start TWAI driver
    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "CAN Bus initialized successfully at 500 kbps");
}

void send_can_message(uint32_t id, uint8_t *data, uint8_t len)
{
    twai_message_t message;
    message.identifier = id;
    message.data_length_code = len;
    message.flags = TWAI_MSG_FLAG_NONE; // Standard frame, data frame
    memcpy(message.data, data, len);

    // Retry up to 3 times on failure
    const int max_retries = 3;
    esp_err_t err = ESP_FAIL;

    for (int retry = 0; retry < max_retries; retry++) {
        err = twai_transmit(&message, pdMS_TO_TICKS(100));

        if (err == ESP_OK) {
            ESP_LOGD(TAG, "Message sent: ID=0x%03X", id);
            return;  // Success
        }

        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "CAN TX timeout (retry %d/%d): ID=0x%03X", retry + 1, max_retries, id);
        } else if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "CAN bus not ready: %s", esp_err_to_name(err));
            break;  // Don't retry if bus is in bad state
        } else {
            ESP_LOGW(TAG, "CAN TX failed (retry %d/%d): %s", retry + 1, max_retries, esp_err_to_name(err));
        }

        // Short delay before retry
        if (retry < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // All retries failed
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN message dropped after %d retries: ID=0x%03X", max_retries, id);
    }
}