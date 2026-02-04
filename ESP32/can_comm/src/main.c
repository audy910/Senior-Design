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











// Original CAN Code
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/twai.h"
// #include "driver/gpio.h"

// // Used in Debug Messages
// static const char *TAG = "CAN_SENDER";

// #define TX_GPIO_NUM   GPIO_NUM_4
// #define RX_GPIO_NUM   GPIO_NUM_5

// void app_main(void)
// {
//     vTaskDelay(pdMS_TO_TICKS(2000));
    
//     // Now configure for TWAI
//     ESP_LOGI(TAG, "Configuring TWAI driver...");
    
//     twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
//         TX_GPIO_NUM, 
//         RX_GPIO_NUM, 
//         TWAI_MODE_NORMAL
//     );
    
//     g_config.tx_queue_len = 5;
//     g_config.rx_queue_len = 5;
    
//     // Enable alerts
//     g_config.alerts_enabled = TWAI_ALERT_TX_IDLE | 
//                               TWAI_ALERT_TX_SUCCESS | 
//                               TWAI_ALERT_TX_FAILED |
//                               TWAI_ALERT_ERR_PASS |
//                               TWAI_ALERT_BUS_ERROR;

//     g_config.clkout_io = TWAI_IO_UNUSED;
//     g_config.bus_off_io = TWAI_IO_UNUSED;
//     g_config.clkout_divider = 0;
    
//     twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//     twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

//     ESP_LOGI(TAG, "Installing TWAI driver...");
//     esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Driver install failed: %s", esp_err_to_name(err));
//         return;
//     }
//     ESP_LOGI(TAG, "Driver installed successfully");

//     ESP_LOGI(TAG, "Starting TWAI driver...");
//     err = twai_start();
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Driver start failed: %s", esp_err_to_name(err));
//         return;
//     }
//     ESP_LOGI(TAG, "Driver started in NORMAL mode");
    
//     vTaskDelay(pdMS_TO_TICKS(500));
    
//     // Check initial status
//     twai_status_info_t status;
//     if (twai_get_status_info(&status) == ESP_OK) {
//         ESP_LOGI(TAG, "Initial State: %d (1=RUNNING, 2=BUS_OFF)", status.state);
//         ESP_LOGI(TAG, "TX errors: %d, RX errors: %d", 
//                  status.tx_error_counter, status.rx_error_counter);
//     }
    
//     ESP_LOGI(TAG, "Starting transmission test...");
    
//     uint32_t success_count = 0;
//     uint32_t fail_count = 0;
    
//     while (1) {
//         // Check for alerts
//         uint32_t alerts;
//         if (twai_read_alerts(&alerts, 0) == ESP_OK) {
//             if (alerts & TWAI_ALERT_TX_SUCCESS) {
//                 ESP_LOGI(TAG, "Alert: TX_SUCCESS");
//             }
//             if (alerts & TWAI_ALERT_TX_FAILED) {
//                 ESP_LOGW(TAG, "Alert: TX_FAILED");
//             }
//             if (alerts & TWAI_ALERT_BUS_ERROR) {
//                 ESP_LOGE(TAG, "Alert: BUS_ERROR");
//             }
//         }
        
//         twai_message_t message = {
//             .identifier = 0x123,
//             .data_length_code = 8,
//             .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}
//         };
        
//         err = twai_transmit(&message, pdMS_TO_TICKS(1000));
        
//         if (err == ESP_OK) {
//             success_count++;
//             ESP_LOGI(TAG, "TX SUCCESS [%lu] - ID: 0x%03lX", 
//                      success_count, message.identifier);
//         } else {
//             fail_count++;
//             ESP_LOGE(TAG, "TX FAILED [%lu] - Error: %s", 
//                      fail_count, esp_err_to_name(err));
            
//             // Get detailed status on failure
//             if (twai_get_status_info(&status) == ESP_OK) {
//                 ESP_LOGE(TAG, "  State: %d, TX_err: %d, RX_err: %d, Queued: %d", 
//                          status.state, 
//                          status.tx_error_counter,
//                          status.rx_error_counter,
//                          status.msgs_to_tx);
//             }
//         }
        
//         // Print summary every 10 messages
//         if ((success_count + fail_count) % 10 == 0) {
//             ESP_LOGI(TAG, "Summary: Success=%lu, Failed=%lu", success_count, fail_count);
//         }
        
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }














































// // Most Recent CAN Code
// #include <stdio.h>
// #include <math.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "driver/twai.h"
// #include "driver/gpio.h"
// #include "esp_timer.h"
// #include "esp_log.h"

// // Include generated DBC code
// #include "rover_optimized.h"

// // Logging tag
// static const char *TAG = "ROVER_CAN";

// // CAN configuration for ESP32-S3
// #define CAN_TX_GPIO GPIO_NUM_4
// #define CAN_RX_GPIO GPIO_NUM_5
// #define CAN_BITRATE TWAI_TIMING_CONFIG_500KBITS()

// // Example sensor pins
// #define ULTRASONIC_TRIG GPIO_NUM_5
// #define ULTRASONIC_ECHO GPIO_NUM_6

// // Update rates (in milliseconds)
// #define IMU_UPDATE_RATE_MS 20
// #define GPS_UPDATE_RATE_MS 100
// #define PROXIMITY_UPDATE_RATE_MS 50

// // Function prototypes
// void init_can_bus(void);
// // void init_sensors(void);
// void send_can_message(uint32_t id, uint8_t *data, uint8_t len);
// void task_send_imu_data(void *pvParameters);
// void task_send_gps_data(void *pvParameters);
// // void task_send_proximity_data(void *pvParameters);
// // uint16_t read_ultrasonic_distance(void);
// void simulate_imu_data(float *accel_x, float *accel_y, float *accel_z);
// void simulate_gps_data(double *lat, double *lon, float *speed, float *course);

// void app_main(void)
// {
//     ESP_LOGI(TAG, "Starting Rover CAN Node...");
    
//     // Initialize CAN bus
//     init_can_bus();
    
//     // Initialize sensors
//     // init_sensors();
    
//     // Create tasks for periodic sending
//     xTaskCreate(task_send_imu_data, "IMU_Task", 4096, NULL, 5, NULL);
//     xTaskCreate(task_send_gps_data, "GPS_Task", 4096, NULL, 5, NULL);
//     // xTaskCreate(task_send_proximity_data, "Proximity_Task", 4096, NULL, 5, NULL);
    
//     ESP_LOGI(TAG, "All tasks started!");
// }

// void init_can_bus(void)
// {
//     // Configure TWAI general settings
//     twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
//         CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    
//     // Configure TWAI timing - 500 kbps
//     twai_timing_config_t t_config = CAN_BITRATE;
    
//     // Configure TWAI filter to accept all messages
//     twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
//     // Install TWAI driver
//     esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
//         return;
//     }
    
//     // Start TWAI driver
//     err = twai_start();
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(err));
//         return;
//     }
    
//     ESP_LOGI(TAG, "CAN Bus initialized successfully at 500 kbps");
// }

// // void init_sensors(void)
// // {
// //     // Configure ultrasonic sensor pins
// //     gpio_config_t io_conf = {
// //         .pin_bit_mask = (1ULL << ULTRASONIC_TRIG),
// //         .mode = GPIO_MODE_OUTPUT,
// //         .pull_up_en = GPIO_PULLUP_DISABLE,
// //         .pull_down_en = GPIO_PULLDOWN_DISABLE,
// //         .intr_type = GPIO_INTR_DISABLE,
// //     };
// //     gpio_config(&io_conf);
    
// //     io_conf.pin_bit_mask = (1ULL << ULTRASONIC_ECHO);
// //     io_conf.mode = GPIO_MODE_INPUT;
// //     gpio_config(&io_conf);
    
// //     // Here you would initialize I2C for MPU6050, UART for GPS, etc.
// //     // For this example, we'll simulate sensor data
    
// //     ESP_LOGI(TAG, "Sensors initialized");
// // }

// void send_can_message(uint32_t id, uint8_t *data, uint8_t len)
// {
//     twai_message_t message;
//     message.identifier = id;
//     message.data_length_code = len;
//     message.flags = TWAI_MSG_FLAG_NONE; // Standard frame, data frame
//     memcpy(message.data, data, len);
    
//     esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(100));
//     if (err == ESP_OK) {
//         ESP_LOGD(TAG, "Message sent: ID=0x%03X", id);
//     } else {
//         ESP_LOGW(TAG, "Failed to send message: %s", esp_err_to_name(err));
//     }
// }

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

// // ==================== PROXIMITY SENSOR TASK ====================

// // void task_send_proximity_data(void *pvParameters)
// // {
// //     TickType_t last_wake_time = xTaskGetTickCount();
    
// //     while (1) {
// //         // Read sensors
// //         uint16_t ultrasonic_mm = read_ultrasonic_distance();
// //         uint16_t ir_mm = 1500; // Simulated IR sensor
        
// //         // Create message
// //         struct rover_optimized_proximity_sensors_t prox_msg;
// //         prox_msg.proximity_time = (uint32_t)(esp_timer_get_time() / 1000);
// //         prox_msg.ultra_sonic_front = ultrasonic_mm;
// //         prox_msg.ir_distance = ir_mm;
        
// //         // Pack and send
// //         uint8_t data[8];
// //         rover_optimized_proximity_sensors_pack(data, &prox_msg, sizeof(data));
// //         send_can_message(104, data, 8); // Message ID 104 (0x68)
        
// //         if (ultrasonic_mm < 500) {
// //             ESP_LOGW(TAG, "OBSTACLE DETECTED: %d mm", ultrasonic_mm);
// //         }
        
// //         // Wait for next cycle
// //         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PROXIMITY_UPDATE_RATE_MS));
// //     }
// // }

// // ==================== SENSOR READING FUNCTIONS ====================

// // uint16_t read_ultrasonic_distance(void)
// // {
// //     // Send trigger pulse
// //     gpio_set_level(ULTRASONIC_TRIG, 0);
// //     esp_rom_delay_us(2);
// //     gpio_set_level(ULTRASONIC_TRIG, 1);
// //     esp_rom_delay_us(10);
// //     gpio_set_level(ULTRASONIC_TRIG, 0);
    
// //     // Wait for echo to go high
// //     int64_t start_time = esp_timer_get_time();
// //     while (gpio_get_level(ULTRASONIC_ECHO) == 0) {
// //         if (esp_timer_get_time() - start_time > 30000) { // 30ms timeout
// //             return 65535; // Max value
// //         }
// //     }
    
// //     // Measure pulse width
// //     start_time = esp_timer_get_time();
// //     while (gpio_get_level(ULTRASONIC_ECHO) == 1) {
// //         if (esp_timer_get_time() - start_time > 30000) {
// //             return 65535;
// //         }
// //     }
// //     int64_t duration = esp_timer_get_time() - start_time;
    
// //     // Calculate distance in mm
// //     // Speed of sound = 343 m/s = 0.343 mm/µs
// //     uint16_t distance_mm = (uint16_t)((duration * 0.343) / 2);
    
// //     return distance_mm;
// // }

// // ==================== SIMULATION FUNCTIONS (Replace with real sensors) ====================

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















































































































// BNO055 Code


// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c.h"
// #include "esp_log.h"
// #include "nvs_flash.h"
// #include "nvs.h"

// static const char *TAG = "BNO055";

// // I2C Configuration
// #define I2C_MASTER_SCL_IO           9       // GPIO for I2C SCL
// #define I2C_MASTER_SDA_IO           8       // GPIO for I2C SDA
// #define I2C_MASTER_NUM              I2C_NUM_0
// // #define I2C_MASTER_FREQ_HZ          400000  // 400kHz
// #define I2C_MASTER_FREQ_HZ 100000 // Change from 400kHz to 100kHz (Standard Mode)
// #define I2C_MASTER_TX_BUF_DISABLE   0
// #define I2C_MASTER_RX_BUF_DISABLE   0
// #define I2C_MASTER_TIMEOUT_MS       1000

// // BNO055 Configuration
// #define BNO055_ADDR                 0x28    // Default I2C address (0x29 if ADR pin is high)
// #define BNO055_CHIP_ID_ADDR         0x00
// #define BNO055_CHIP_ID              0xA0

// // BNO055 Register Addresses
// #define BNO055_PAGE_ID_ADDR         0x07
// #define BNO055_OPR_MODE_ADDR        0x3D
// #define BNO055_PWR_MODE_ADDR        0x3E
// #define BNO055_SYS_TRIGGER_ADDR     0x3F

// // Data Registers
// #define BNO055_ACCEL_DATA_X_LSB     0x08
// #define BNO055_GYRO_DATA_X_LSB      0x14
// #define BNO055_MAG_DATA_X_LSB       0x0E
// #define BNO055_EULER_H_LSB          0x1A
// #define BNO055_QUAT_DATA_W_LSB      0x20
// #define BNO055_LINEAR_ACCEL_X_LSB   0x28
// #define BNO055_GRAVITY_X_LSB        0x2E

// // Calibration Status
// #define BNO055_CALIB_STAT_ADDR      0x35

// // Operation Modes
// #define OPERATION_MODE_CONFIG       0x00
// #define OPERATION_MODE_NDOF         0x0C    // 9DOF with sensor fusion
// #define OPERATION_MODE_IMU          0x08    // 6DOF (no magnetometer)
// #define OPERATION_MODE_COMPASS      0x09
// #define OPERATION_MODE_M4G          0x0A
// #define OPERATION_MODE_NDOF_FMC_OFF 0x0B

// // Power Modes
// #define POWER_MODE_NORMAL           0x00
// #define POWER_MODE_LOWPOWER         0x01
// #define POWER_MODE_SUSPEND          0x02

// #define STORAGE_NAMESPACE "storage"
// #define CALIB_KEY "bno055_cal"
// #define BNO055_OFFSET_DATA_ADDR 0x55

// esp_err_t save_calibration(uint8_t *data) {
//     nvs_handle_t my_handle;
//     esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
//     if (err != ESP_OK) return err;

//     err = nvs_set_blob(my_handle, CALIB_KEY, data, 22);
//     if (err == ESP_OK) err = nvs_commit(my_handle);
    
//     nvs_close(my_handle);
//     return err;
// }

// esp_err_t load_calibration(uint8_t *data) {
//     nvs_handle_t my_handle;
//     size_t required_size = 22;
//     esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
//     if (err != ESP_OK) return err;

//     err = nvs_get_blob(my_handle, CALIB_KEY, data, &required_size);
//     nvs_close(my_handle);
//     return err;
// }

// /**
//  * @brief Initialize I2C master
//  */
// static esp_err_t i2c_master_init(void)
// {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_MASTER_SDA_IO,
//         .scl_io_num = I2C_MASTER_SCL_IO,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = I2C_MASTER_FREQ_HZ,
//     };

//     esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
//     if (err != ESP_OK) {
//         return err;
//     }

//     return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
//                              I2C_MASTER_RX_BUF_DISABLE, 
//                              I2C_MASTER_TX_BUF_DISABLE, 0);
// }

// /**
//  * @brief Write a byte to BNO055 register
//  */
// static esp_err_t bno055_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     uint8_t write_buf[2] = {reg_addr, data};
    
//     esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, BNO055_ADDR,
//                                                write_buf, sizeof(write_buf),
//                                                I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to write to register 0x%02X", reg_addr);
//     }
//     return ret;
// }

// /**
//  * @brief Read bytes from BNO055 register
//  */
// static esp_err_t bno055_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, BNO055_ADDR,
//                                                  &reg_addr, 1, data, len,
//                                                  I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to read from register 0x%02X", reg_addr);
//     }
//     return ret;
// }

// /**
//  * @brief Set BNO055 operation mode
//  */
// static esp_err_t bno055_set_mode(uint8_t mode)
// {
//     esp_err_t ret;
    
//     // Set to config mode first
//     ret = bno055_write_byte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
//     if (ret != ESP_OK) return ret;
//     vTaskDelay(pdMS_TO_TICKS(30));  // Required delay after mode change
    
//     // Set desired mode
//     ret = bno055_write_byte(BNO055_OPR_MODE_ADDR, mode);
//     if (ret != ESP_OK) return ret;
//     vTaskDelay(pdMS_TO_TICKS(30));
    
//     return ESP_OK;
// }

// /**
//  * @brief Initialize BNO055 sensor
//  */
// static esp_err_t bno055_init(void)
// {
//     esp_err_t ret;
//     uint8_t chip_id;
//     uint8_t calib_data[22];

//      // Read chip ID
//     ret = bno055_read_bytes(BNO055_CHIP_ID_ADDR, &chip_id, 1);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to read chip ID");
//         return ret;
//     }
    
//     if (chip_id != BNO055_CHIP_ID) {
//         ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BNO055_CHIP_ID);
//         return ESP_ERR_NOT_FOUND;
//     }
    
//     ESP_LOGI(TAG, "BNO055 detected, chip ID: 0x%02X", chip_id);
    
//     // Reset
//     ret = bno055_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20);
//     if (ret != ESP_OK) return ret;
//     vTaskDelay(pdMS_TO_TICKS(650));  // Wait for reset

//     if (load_calibration(calib_data) == ESP_OK) {
//         ESP_LOGI(TAG, "Found saved calibration in NVS. Loading...");
        
//         // Sensor must be in CONFIG mode (which it is after reset)
//         for (int i = 0; i < 22; i++) {
//             bno055_write_byte(BNO055_OFFSET_DATA_ADDR + i, calib_data[i]);
//         }
//         ESP_LOGI(TAG, "Calibration profile applied.");
//     } else {
//         ESP_LOGW(TAG, "No calibration profile found in NVS.");
//     }
    
//     // Set to normal power mode
//     ret = bno055_write_byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
//     if (ret != ESP_OK) return ret;
//     vTaskDelay(pdMS_TO_TICKS(10));
    
//     // Set page 0
//     ret = bno055_write_byte(BNO055_PAGE_ID_ADDR, 0);
//     if (ret != ESP_OK) return ret;
    
//     // Set system trigger
//     ret = bno055_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x00);
//     if (ret != ESP_OK) return ret;
//     vTaskDelay(pdMS_TO_TICKS(10));
    
//     // Set to NDOF mode (9DOF fusion)
//     ret = bno055_set_mode(OPERATION_MODE_NDOF);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to set operation mode");
//         return ret;
//     }
    
//     ESP_LOGI(TAG, "BNO055 initialized successfully in NDOF mode");
//     return ESP_OK;
// }

// /**
//  * @brief Read Euler angles (heading, roll, pitch)
//  */
// static esp_err_t bno055_read_euler(float *heading, float *roll, float *pitch)
// {
//     uint8_t buffer[6];
//     esp_err_t ret = bno055_read_bytes(BNO055_EULER_H_LSB, buffer, 6);
    
//     if (ret == ESP_OK) {
//         // Convert to degrees (1 degree = 16 LSB)
//         int16_t h = (buffer[1] << 8) | buffer[0];
//         int16_t r = (buffer[3] << 8) | buffer[2];
//         int16_t p = (buffer[5] << 8) | buffer[4];
        
//         *heading = h / 16.0f;
//         *roll = r / 16.0f;
//         *pitch = p / 16.0f;
//     }
    
//     return ret;
// }

// /**
//  * @brief Read quaternion data
//  */
// static esp_err_t bno055_read_quaternion(float *w, float *x, float *y, float *z)
// {
//     uint8_t buffer[8];
//     esp_err_t ret = bno055_read_bytes(BNO055_QUAT_DATA_W_LSB, buffer, 8);
    
//     if (ret == ESP_OK) {
//         // Convert to unit quaternion (1 unit = 2^14 LSB)
//         int16_t qw = (buffer[1] << 8) | buffer[0];
//         int16_t qx = (buffer[3] << 8) | buffer[2];
//         int16_t qy = (buffer[5] << 8) | buffer[4];
//         int16_t qz = (buffer[7] << 8) | buffer[6];
        
//         float scale = 1.0f / 16384.0f;
//         *w = qw * scale;
//         *x = qx * scale;
//         *y = qy * scale;
//         *z = qz * scale;
//     }
    
//     return ret;
// }

// /**
//  * @brief Read accelerometer data
//  */
// static esp_err_t bno055_read_accel(float *x, float *y, float *z)
// {
//     uint8_t buffer[6];
//     esp_err_t ret = bno055_read_bytes(BNO055_ACCEL_DATA_X_LSB, buffer, 6);
    
//     if (ret == ESP_OK) {
//         // Convert to m/s^2 (1 m/s^2 = 100 LSB)
//         int16_t ax = (buffer[1] << 8) | buffer[0];
//         int16_t ay = (buffer[3] << 8) | buffer[2];
//         int16_t az = (buffer[5] << 8) | buffer[4];
        
//         *x = ax / 100.0f;
//         *y = ay / 100.0f;
//         *z = az / 100.0f;
//     }
    
//     return ret;
// }

// /**
//  * @brief Read gyroscope data
//  */
// static esp_err_t bno055_read_gyro(float *x, float *y, float *z)
// {
//     uint8_t buffer[6];
//     esp_err_t ret = bno055_read_bytes(BNO055_GYRO_DATA_X_LSB, buffer, 6);
    
//     if (ret == ESP_OK) {
//         // Convert to deg/s (1 deg/s = 16 LSB)
//         int16_t gx = (buffer[1] << 8) | buffer[0];
//         int16_t gy = (buffer[3] << 8) | buffer[2];
//         int16_t gz = (buffer[5] << 8) | buffer[4];
        
//         *x = gx / 16.0f;
//         *y = gy / 16.0f;
//         *z = gz / 16.0f;
//     }
    
//     return ret;
// }

// /**
//  * @brief Read calibration status
//  */
// static esp_err_t bno055_get_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
// {
//     uint8_t cal_status;
//     esp_err_t ret = bno055_read_bytes(BNO055_CALIB_STAT_ADDR, &cal_status, 1);
    
//     if (ret == ESP_OK) {
//         *sys = (cal_status >> 6) & 0x03;
//         *gyro = (cal_status >> 4) & 0x03;
//         *accel = (cal_status >> 2) & 0x03;
//         *mag = cal_status & 0x03;
//     }
    
//     return ret;
// }

// /**
//  * @brief Main application task
//  */
// void app_main(void)
// {
//     ESP_LOGI(TAG, "Starting BNO055 ESP32-S3 example");

//     // Initialize NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);
    
//     // Initialize I2C
//     ESP_ERROR_CHECK(i2c_master_init());
//     ESP_LOGI(TAG, "I2C initialized");
    
//     // Initialize BNO055
//     ret = bno055_init();
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "BNO055 initialization failed");
//         return;
//     }

//     bool already_saved = false;
    
//     // Main loop
//     while (1) {
//         float heading, roll, pitch;
//         float accel_x, accel_y, accel_z;
//         float gyro_x, gyro_y, gyro_z;
//         float quat_w, quat_x, quat_y, quat_z;
//         uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
        
//         // Read Euler angles
//         if (bno055_read_euler(&heading, &roll, &pitch) == ESP_OK) {
//             ESP_LOGI(TAG, "Euler - Heading: %.2f°  Roll: %.2f°  Pitch: %.2f°", 
//                      heading, roll, pitch);
//         }
        
//         // Read accelerometer
//         if (bno055_read_accel(&accel_x, &accel_y, &accel_z) == ESP_OK) {
//             ESP_LOGI(TAG, "Accel - X: %.2f  Y: %.2f  Z: %.2f m/s²", 
//                      accel_x, accel_y, accel_z);
//         }
        
//         // Read gyroscope
//         if (bno055_read_gyro(&gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
//             ESP_LOGI(TAG, "Gyro - X: %.2f  Y: %.2f  Z: %.2f deg/s", 
//                      gyro_x, gyro_y, gyro_z);
//         }
        
//         // Read quaternion
//         if (bno055_read_quaternion(&quat_w, &quat_x, &quat_y, &quat_z) == ESP_OK) {
//             ESP_LOGI(TAG, "Quat - W: %.3f  X: %.3f  Y: %.3f  Z: %.3f", 
//                      quat_w, quat_x, quat_y, quat_z);
//         }

//         if (bno055_get_calibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal) == ESP_OK) {
//             ESP_LOGI(TAG, "Cal - Sys: %d  Gyro: %d  Accel: %d  Mag: %d", sys_cal, gyro_cal, accel_cal, mag_cal);

//             // --- NEW: Auto-save logic ---
//             if (!already_saved && sys_cal == 3 && gyro_cal == 3 && accel_cal == 3 && mag_cal == 3) {
//                 ESP_LOGI(TAG, "Full calibration reached! Saving to NVS...");
                
//                 uint8_t calib_to_save[22];
//                 uint8_t prev_mode = OPERATION_MODE_NDOF; // Current mode

//                 // 1. Switch to CONFIG mode to read offsets
//                 bno055_set_mode(OPERATION_MODE_CONFIG);
                
//                 // 2. Read the 22 bytes
//                 if (bno055_read_bytes(BNO055_OFFSET_DATA_ADDR, calib_to_save, 22) == ESP_OK) {
//                     save_calibration(calib_to_save);
//                     ESP_LOGI(TAG, "Calibration saved successfully.");
//                     already_saved = true;
//                 }

//                 // 3. Switch back to NDOF
//                 bno055_set_mode(prev_mode);
//             }
//         }
        
//         ESP_LOGI(TAG, "---");
//         vTaskDelay(pdMS_TO_TICKS(500));  // Read every 500ms
//     }
// }










































// GPS Code

// NMEA
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "esp_log.h"

// static const char *TAG = "GNSS_DEBUG";

// // Adjust these to match your specific S3 board if necessary
// #define TX_PIN 17
// #define RX_PIN 18
// #define UART_PORT UART_NUM_1

// void rx_task(void *arg) {
//     ESP_LOGI(TAG, "Starting UART RX Task...");
//     uint8_t* data = (uint8_t*) malloc(2048);

//     while (1) {
//         // Read bytes from UART
//         int len = uart_read_bytes(UART_PORT, data, 2047, 100 / portTICK_PERIOD_MS);
        
//         if (len > 0) {
//             data[len] = '\0';
//             // Print raw data just to see IF anything is coming in
//             // Remove this once you see text!
//             printf("Raw data: %s", (char*)data);

//             if (strstr((char*)data, "$GNGGA")) {
//                 ESP_LOGW(TAG, "GNGGA Message Detected!");
//                 // Your parsing logic here...
//             }
//         }
//         // Small delay to prevent watchdog issues
//         vTaskDelay(pdMS_TO_TICKS(10)); 
//     }
// }




// // UBX
// #include <stdio.h>
// #include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include <stdio.h>
// #include <string.h>
// #include <stdbool.h>

// #define GNSS_UART          UART_NUM_1
// #define GNSS_TX_PIN        GPIO_NUM_18
// #define GNSS_RX_PIN        GPIO_NUM_17

// #define GNSS_BAUDRATE      460800
// #define UART_BUF_SIZE      2048

// static const char *TAG = "GNSS";

// typedef struct {
//     int32_t lon;        // 1e-7 deg
//     int32_t lat;        // 1e-7 deg
//     int32_t height;     // mm

//     int32_t velN;       // mm/s
//     int32_t velE;       // mm/s
//     int32_t velD;       // mm/s
//     uint32_t gSpeed;    // mm/s
//     int32_t heading;    // 1e-5 deg

//     uint32_t hAcc;      // mm
//     uint32_t vAcc;      // mm

//     uint8_t fixType;
//     uint8_t flags;
//     uint8_t flags2;
//     uint8_t flags3;
// } nav_pvt_t;

// typedef enum {
//     NAV_QUALITY_INVALID,
//     NAV_QUALITY_COARSE,
//     NAV_QUALITY_GOOD,
//     NAV_QUALITY_EXCELLENT,
//     NAV_QUALITY_RTK
// } nav_quality_t;

// typedef struct {
//     bool gnss_ok;
//     bool stable;
//     bool dead_reckoning;
//     nav_quality_t quality;
//     float horiz_acc_m;
//     float vert_acc_m;
// } nav_status_t;


// static void gnss_uart_init(void)
// {
//     uart_config_t uart_config = {
//         .baud_rate = GNSS_BAUDRATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT
//     };

//     ESP_ERROR_CHECK(uart_driver_install(
//         GNSS_UART,
//         UART_BUF_SIZE,
//         0,
//         0,
//         NULL,
//         0
//     ));

//     ESP_ERROR_CHECK(uart_param_config(GNSS_UART, &uart_config));

//     ESP_ERROR_CHECK(uart_set_pin(
//         GNSS_UART,
//         GNSS_TX_PIN,
//         GNSS_RX_PIN,
//         UART_PIN_NO_CHANGE,
//         UART_PIN_NO_CHANGE
//     ));

//     ESP_LOGI(TAG, "GNSS UART initialized");
// }

// bool nav_ready(const nav_status_t *s)
// {
//     return s->gnss_ok &&
//            s->stable &&
//            !s->dead_reckoning &&
//            s->horiz_acc_m < 2.0f;
// }

// nav_status_t interpret_nav_pvt(const nav_pvt_t *pvt)
// {
//     nav_status_t status = {0};

//     status.horiz_acc_m = pvt->hAcc / 1000.0f;
//     status.vert_acc_m  = pvt->vAcc / 1000.0f;

//     bool gnssFixOK = pvt->flags & (1 << 0);
//     bool confirmed = pvt->flags2 & (1 << 5);

//     bool carrFixed = ((pvt->flags3 >> 6) & 0x03) == 2;

//     status.dead_reckoning = (pvt->fixType == 6); // DR-only
//     status.gnss_ok = gnssFixOK;
//     status.stable  = confirmed;

//     if (!gnssFixOK) {
//         status.quality = NAV_QUALITY_INVALID;
//     }
//     else if (carrFixed) {
//         status.quality = NAV_QUALITY_RTK;
//     }
//     else if (status.horiz_acc_m < 0.5f) {
//         status.quality = NAV_QUALITY_EXCELLENT;
//     }
//     else if (status.horiz_acc_m < 2.0f) {
//         status.quality = NAV_QUALITY_GOOD;
//     }
//     else {
//         status.quality = NAV_QUALITY_COARSE;
//     }

//     return status;
// }

// static void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
// {
//     *ck_a = 0;
//     *ck_b = 0;
//     for (uint16_t i = 0; i < len; i++) {
//         *ck_a += data[i];
//         *ck_b += *ck_a;
//     }
// }

// bool ubx_parse_byte(uint8_t byte, nav_pvt_t *out)
// {
//     static uint8_t buf[128];
//     static uint16_t idx = 0;
//     static uint16_t payload_len = 0;
//     static enum {
//         SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
//     } state = SYNC1;

//     static uint8_t ck_a, ck_b;

//     switch (state) {
//     case SYNC1:
//         if (byte == 0xB5) state = SYNC2;
//         break;

//     case SYNC2:
//         if (byte == 0x62) state = CLASS;
//         else state = SYNC1;
//         break;

//     case CLASS:
//         buf[0] = byte;
//         state = ID;
//         break;

//     case ID:
//         buf[1] = byte;
//         state = LEN1;
//         break;

//     case LEN1:
//         buf[2] = byte;
//         payload_len = byte;
//         state = LEN2;
//         break;

//     case LEN2:
//         buf[3] = byte;
//         payload_len |= (byte << 8);
//         idx = 0;
//         state = PAYLOAD;
//         break;

//     case PAYLOAD:
//         buf[4 + idx] = byte;
//         idx++;
//         if (idx == payload_len) {  // Changed from >= to ==
//             state = CK_A;
//         }
//         break;

//     case CK_A:
//         ck_a = byte;
//         state = CK_B;
//         break;

//     case CK_B:
//         ck_b = byte;

//         uint8_t calc_a, calc_b;
//         ubx_checksum(buf, payload_len + 4, &calc_a, &calc_b);

//         if (calc_a == ck_a && calc_b == ck_b) {
//             // NAV-PVT?
//             if (buf[0] == 0x01 && buf[1] == 0x07 && payload_len == 92) {
//                 uint8_t *p = &buf[4];  // Changed from &buf[2]

//                 out->fixType = p[20];
//                 out->flags   = p[21];
//                 out->flags2  = p[22];
//                 out->flags3  = p[23];

//                 memcpy(&out->lon,     &p[24], 4);
//                 memcpy(&out->lat,     &p[28], 4);
//                 memcpy(&out->height,  &p[32], 4);

//                 memcpy(&out->hAcc,    &p[40], 4);
//                 memcpy(&out->vAcc,    &p[44], 4);

//                 memcpy(&out->velN,    &p[48], 4);
//                 memcpy(&out->velE,    &p[52], 4);
//                 memcpy(&out->velD,    &p[56], 4);
//                 memcpy(&out->gSpeed,  &p[60], 4);
//                 memcpy(&out->heading, &p[64], 4);

//                 state = SYNC1;
//                 return true;
//             }
//         }

//         state = SYNC1;
//         break;
//     }

//     return false;
// }

// static void gnss_uart_task(void *arg)
// {
//     uint8_t rx_buf[256];
//     nav_pvt_t nav_pvt;

//     while (1) {
//         int len = uart_read_bytes(
//             GNSS_UART,
//             rx_buf,
//             sizeof(rx_buf),
//             pdMS_TO_TICKS(100)
//         );

//         // for (int i = 0; i < len; i++) {
//         //     printf("%02X ", rx_buf[i]);
//         // }
//         // printf("\n");

//         for (int i = 0; i < len; i++) {
//             if (ubx_parse_byte(rx_buf[i], &nav_pvt)) {

//                 nav_status_t status = interpret_nav_pvt(&nav_pvt);

//                 if (status.gnss_ok) {
//                     ESP_LOGI(TAG,
//                         "FIX=%d | hAcc=%.2fm | vAcc=%.2fm | READY=%s",
//                         nav_pvt.fixType,
//                         status.horiz_acc_m,
//                         status.vert_acc_m,
//                         nav_ready(&status) ? "YES" : "NO"
//                     );
//                 } else {
//                     ESP_LOGW(TAG, "GNSS fix not valid");
//                 }
//             }
//         }
//     }
// }

// void app_main(void)
// {
//     ESP_LOGI(TAG, "Starting GNSS subsystem");

//     gnss_uart_init();

//     xTaskCreatePinnedToCore(
//         gnss_uart_task,
//         "gnss_uart_task",
//         4096,
//         NULL,
//         10,
//         NULL,
//         1
//     );

//     ESP_LOGI(TAG, "GNSS task started");
// }


