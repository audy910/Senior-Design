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









// // ==================== IMU DATA TASK ====================

// void task_send_imu_data(void *pvParameters)
// {
//     TickType_t last_wake_time = xTaskGetTickCount();
    
//     while (1) {
//         // Read IMU sensor (simulated for this example)
//         float accel_x, accel_y, accel_z;
//         simulate_imu_data(&accel_x, &accel_y, &accel_z);
        
//         // ===== Send IMU Acceleration Data =====
//         struct rover_optimized_imu_data_t imu_msg;
//         imu_msg.imu_time = (uint32_t)(esp_timer_get_time() / 1000); // Convert µs to ms
        
//         // Convert to raw values using DBC scale factors
//         // Scale: 0.01 m/s2 for X and Y, 0.1 m/s2 for Z
//         imu_msg.imu_accel_x = (int16_t)(accel_x / 0.01);
//         imu_msg.imu_accel_y = (int16_t)(accel_y / 0.01);
//         imu_msg.imu_accel_z = (int16_t)(accel_z / 0.1);
        
//         // Pack message using generated function
//         uint8_t data[8];
//         int pack_result = rover_optimized_imu_data_pack(data, &imu_msg, sizeof(data));
        
//         // if (pack_result == 0) {
//         send_can_message(102, data, 8); // Message ID 102 (0x66)
//         ESP_LOGI(TAG, "IMU Accel: X=%.2f, Y=%.2f, Z=%.2f m/s2       Time: %i", 
//                     accel_x, accel_y, accel_z, imu_msg.imu_time);
//         // }
        
//         // ===== Send IMU Orientation Data =====
//         // Simulated heading, pitch, roll
//         static float heading = 0.0;
//         heading += 1.0;
//         if (heading >= 360.0) heading = 0.0;
        
//         float pitch = 5.0;  // degrees
//         float roll = -2.0;  // degrees
        
//         struct rover_optimized_imu_orientation_t orient_msg;
//         orient_msg.imu_orient_time = (uint32_t)(esp_timer_get_time() / 1000);
//         orient_msg.imu_heading = (uint16_t)(heading / 0.01);
//         orient_msg.imu_pitch = (int8_t)(pitch / 0.5);
//         orient_msg.imu_roll = (int8_t)(roll / 1.0);
        
//         rover_optimized_imu_orientation_pack(data, &orient_msg, sizeof(data));
//         ESP_LOGI(TAG, "IMU Orientation: X=%.2f, Y=%.2f, Z=%.2f      Time=%i", 
//                     heading, pitch, roll, orient_msg.imu_orient_time);
//         send_can_message(103, data, 8); // Message ID 103 (0x67)
        
//         // Wait for next cycle
//         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(IMU_UPDATE_RATE_MS));
//     }
// }

// // ==================== GPS DATA TASK ====================

// void task_send_gps_data(void *pvParameters)
// {
//     TickType_t last_wake_time = xTaskGetTickCount();
    
//     while (1) {
//         // Read GPS sensor (simulated)
//         double latitude, longitude;
//         float speed, course;
//         simulate_gps_data(&latitude, &longitude, &speed, &course);
        
//         // ===== Send GPS Position =====
//         struct rover_optimized_gps_position_t gps_pos_msg;
//         gps_pos_msg.gps_time = (uint32_t)(esp_timer_get_time() / 1000);
        
//         // Convert to raw values using DBC scale: 0.0001 degrees
//         gps_pos_msg.gps_latitude = (int32_t)(latitude / 0.0001);
//         gps_pos_msg.gps_longitude = (int32_t)(longitude / 0.0001);
        
//         uint8_t data[8];
//         rover_optimized_gps_position_pack(data, &gps_pos_msg, sizeof(data));
//         send_can_message(100, data, 8); // Message ID 100 (0x64)
        
//         ESP_LOGI(TAG, "GPS Position: Lat=%.6f, Lon=%.6f         Time=%i", latitude, longitude, gps_pos_msg.gps_time);
        
//         // ===== Send GPS Velocity =====
//         struct rover_optimized_gps_velocity_t gps_vel_msg;
//         gps_vel_msg.gps_speed_time = (uint32_t)(esp_timer_get_time() / 1000);
        
//         // Convert to raw values using DBC scale: 0.01
//         gps_vel_msg.gps_speed = (uint16_t)(speed / 0.01);
//         gps_vel_msg.gps_course = (uint16_t)(course / 0.01);
        
//         rover_optimized_gps_velocity_pack(data, &gps_vel_msg, sizeof(data));
//         send_can_message(101, data, 8); // Message ID 101 (0x65)
        
//         ESP_LOGI(TAG, "GPS Velocity: Speed=%.2f m/s, Course=%.1f deg            Time=%i", speed, course, gps_vel_msg.gps_speed_time);
        
//         // Wait for next cycle
//         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(GPS_UPDATE_RATE_MS));
//     }
// }



// void simulate_imu_data(float *accel_x, float *accel_y, float *accel_z)
// {
//     // Simulate some acceleration data
//     static float time = 0.0;
//     time += 0.02; // 20ms increment
    
//     *accel_x = 0.5 * sinf(time);
//     *accel_y = 0.3 * cosf(time);
//     *accel_z = 9.81; // Gravity
// }

// void simulate_gps_data(double *lat, double *lon, float *speed, float *course)
// {
//     // Simulate GPS coordinates (somewhere in California)
//     static double base_lat = 34.0522;  // Los Angeles
//     static double base_lon = -118.2437;
//     static float offset = 0.0;
    
//     offset += 0.0001; // Simulate movement
    
//     *lat = base_lat + offset;
//     *lon = base_lon + offset;
//     *speed = 2.5; // 2.5 m/s
//     *course = 45.0; // Northeast
// }