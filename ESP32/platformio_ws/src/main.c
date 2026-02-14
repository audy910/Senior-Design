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
    
    // // Create BNO055 task on Core 0 (20ms = 50 Hz)
    // xTaskCreatePinnedToCore(
    //     bno055_read_task,
    //     "bno055_task",
    //     4096,
    //     NULL,
    //     5,                      // Priority: Medium
    //     NULL,
    //     0                       // Core 0
    // );
    
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
    
    // // Create Proximity sensors task on Core 0 (50ms = 20 Hz)
    // xTaskCreatePinnedToCore(
    //     proximity_sensors_task,
    //     "proximity_sensor_task",
    //     4096,
    //     NULL,
    //     7,                      // Priority: Medium-High (safety critical)
    //     NULL,
    //     0                       // Core 0
    // );
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "Task Periods: GPS=100ms, IMU=20ms, Proximity=50ms");
}














// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "driver/gpio.h"
// #include "esp_log.h"

// static const char *TAG = "NMEA_MONITOR";

// // UART Configuration
// #define GPS_UART_NUM        UART_NUM_1
// #define GPS_TXD_PIN         17  // ESP32 TX -> GPS RX
// #define GPS_RXD_PIN         18  // ESP32 RX -> GPS TX
// #define GPS_BAUD_RATE       115200
// #define GPS_BUF_SIZE        1024

// // Message tracking structure
// typedef struct {
//     char name[8];
//     uint32_t count;
//     uint32_t last_seen_ms;
//     uint32_t interval_ms;
// } nmea_msg_stats_t;

// #define MAX_MSG_TYPES 20
// static nmea_msg_stats_t msg_stats[MAX_MSG_TYPES] = {0};
// static int num_msg_types = 0;

// // Initialize GPS UART
// static void gps_uart_init(void) {
//     const uart_config_t uart_config = {
//         .baud_rate = GPS_BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT,
//     };
    
//     ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
//     ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TXD_PIN, GPS_RXD_PIN, 
//                                   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
//     ESP_LOGI(TAG, "GPS UART initialized at %d baud on RX:%d TX:%d", 
//              GPS_BAUD_RATE, GPS_RXD_PIN, GPS_TXD_PIN);
// }

// // Calculate NMEA checksum
// static uint8_t nmea_checksum(const char *sentence) {
//     uint8_t checksum = 0;
//     for (const char *p = sentence + 1; *p && *p != '*'; p++) {
//         checksum ^= *p;
//     }
//     return checksum;
// }

// // Verify NMEA checksum
// static bool nmea_verify_checksum(const char *sentence) {
//     const char *checksum_pos = strchr(sentence, '*');
//     if (!checksum_pos) return false;
    
//     uint8_t calculated = nmea_checksum(sentence);
//     uint8_t received = (uint8_t)strtol(checksum_pos + 1, NULL, 16);
    
//     return calculated == received;
// }

// // Extract message type from NMEA sentence
// static void extract_msg_type(const char *sentence, char *msg_type, size_t max_len) {
//     // Extract everything between $ and first comma
//     const char *start = sentence;
//     if (*start == '$') start++;
    
//     const char *comma = strchr(start, ',');
//     if (!comma) {
//         strncpy(msg_type, start, max_len - 1);
//         msg_type[max_len - 1] = '\0';
//         return;
//     }
    
//     size_t len = comma - start;
//     if (len >= max_len) len = max_len - 1;
    
//     strncpy(msg_type, start, len);
//     msg_type[len] = '\0';
// }

// // Update message statistics
// static void update_stats(const char *msg_type) {
//     uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
//     // Find existing message type
//     for (int i = 0; i < num_msg_types; i++) {
//         if (strcmp(msg_stats[i].name, msg_type) == 0) {
//             // Update existing
//             msg_stats[i].count++;
            
//             if (msg_stats[i].last_seen_ms > 0) {
//                 uint32_t interval = now_ms - msg_stats[i].last_seen_ms;
//                 // Simple moving average of interval
//                 if (msg_stats[i].interval_ms == 0) {
//                     msg_stats[i].interval_ms = interval;
//                 } else {
//                     msg_stats[i].interval_ms = (msg_stats[i].interval_ms * 3 + interval) / 4;
//                 }
//             }
            
//             msg_stats[i].last_seen_ms = now_ms;
//             return;
//         }
//     }
    
//     // Add new message type
//     if (num_msg_types < MAX_MSG_TYPES) {
//         strncpy(msg_stats[num_msg_types].name, msg_type, sizeof(msg_stats[0].name) - 1);
//         msg_stats[num_msg_types].count = 1;
//         msg_stats[num_msg_types].last_seen_ms = now_ms;
//         msg_stats[num_msg_types].interval_ms = 0;
//         num_msg_types++;
//     }
// }

// // Parse and extract useful data from common NMEA sentences
// static void parse_and_display(const char *sentence) {
//     char msg_type[16];
//     extract_msg_type(sentence, msg_type, sizeof(msg_type));
    
//     // Display based on message type
//     if (strncmp(msg_type, "GPGGA", 5) == 0 || strncmp(msg_type, "GNGGA", 5) == 0) {
//         // Parse GGA: $GPGGA,time,lat,N,lon,W,quality,sats,hdop,alt,M,...
//         char buffer[128];
//         strncpy(buffer, sentence, sizeof(buffer) - 1);
        
//         char *tokens[15];
//         int count = 0;
//         char *token = strtok(buffer, ",");
//         while (token && count < 15) {
//             tokens[count++] = token;
//             token = strtok(NULL, ",");
//         }
        
//         if (count >= 10) {
//             printf("  [GGA] Time:%s Lat:%s%s Lon:%s%s Fix:%s Sats:%s HDOP:%s Alt:%sm\n",
//                    tokens[1], tokens[2], tokens[3], tokens[4], tokens[5],
//                    tokens[6], tokens[7], tokens[8], tokens[9]);
//         }
//     }
//     else if (strncmp(msg_type, "GPRMC", 5) == 0 || strncmp(msg_type, "GNRMC", 5) == 0) {
//         // Parse RMC: $GPRMC,time,status,lat,N,lon,W,speed,course,date,...
//         char buffer[128];
//         strncpy(buffer, sentence, sizeof(buffer) - 1);
        
//         char *tokens[12];
//         int count = 0;
//         char *token = strtok(buffer, ",");
//         while (token && count < 12) {
//             tokens[count++] = token;
//             token = strtok(NULL, ",");
//         }
        
//         if (count >= 9) {
//             printf("  [RMC] Status:%s Speed:%s knots Course:%s° Date:%s\n",
//                    tokens[2], tokens[7], tokens[8], tokens[9]);
//         }
//     }
//     else if (strncmp(msg_type, "GPVTG", 5) == 0 || strncmp(msg_type, "GNVTG", 5) == 0) {
//         // Parse VTG: $GPVTG,course_true,T,course_mag,M,speed_knots,N,speed_kph,K
//         char buffer[128];
//         strncpy(buffer, sentence, sizeof(buffer) - 1);
        
//         char *tokens[10];
//         int count = 0;
//         char *token = strtok(buffer, ",");
//         while (token && count < 10) {
//             tokens[count++] = token;
//             token = strtok(NULL, ",");
//         }
        
//         if (count >= 8) {
//             printf("  [VTG] Course:%s° Speed:%s km/h\n", tokens[1], tokens[7]);
//         }
//     }
//     else if (strncmp(msg_type, "GPGSA", 5) == 0 || strncmp(msg_type, "GNGSA", 5) == 0) {
//         // Parse GSA: satellite IDs and DOP values
//         char buffer[128];
//         strncpy(buffer, sentence, sizeof(buffer) - 1);
        
//         char *tokens[20];
//         int count = 0;
//         char *token = strtok(buffer, ",");
//         while (token && count < 20) {
//             tokens[count++] = token;
//             token = strtok(NULL, ",");
//         }
        
//         if (count >= 17) {
//             printf("  [GSA] Mode:%s Fix:%s PDOP:%s HDOP:%s VDOP:%s\n",
//                    tokens[1], tokens[2], tokens[15], tokens[16], tokens[17]);
//         }
//     }
//     else if (strncmp(msg_type, "GPGSV", 5) == 0 || strncmp(msg_type, "GNGSV", 5) == 0) {
//         // Parse GSV: satellite visibility info
//         char buffer[128];
//         strncpy(buffer, sentence, sizeof(buffer) - 1);
        
//         char *tokens[8];
//         int count = 0;
//         char *token = strtok(buffer, ",");
//         while (token && count < 8) {
//             tokens[count++] = token;
//             token = strtok(NULL, ",");
//         }
        
//         if (count >= 4) {
//             printf("  [GSV] Message %s/%s: %s satellites in view\n",
//                    tokens[2], tokens[1], tokens[3]);
//         }
//     }
// }

// // GPS monitoring task
// static void gps_monitor_task(void *pvParameters) {
//     static char nmea_buffer[256];
//     static int buffer_idx = 0;
    
//     uint8_t data;
//     int len;
//     uint32_t total_sentences = 0;
//     uint32_t valid_sentences = 0;
//     uint32_t invalid_checksums = 0;
    
//     printf("\n=== NMEA GPS Monitor Started ===\n");
//     printf("Listening for NMEA sentences...\n\n");
    
//     while (1) {
//         len = uart_read_bytes(GPS_UART_NUM, &data, 1, pdMS_TO_TICKS(1000));
        
//         if (len > 0) {
//             if (data == '\n') {
//                 nmea_buffer[buffer_idx] = '\0';
                
//                 if (nmea_buffer[0] == '$') {
//                     total_sentences++;
                    
//                     char msg_type[16];
//                     extract_msg_type(nmea_buffer, msg_type, sizeof(msg_type));
                    
//                     // Verify checksum
//                     if (nmea_verify_checksum(nmea_buffer)) {
//                         valid_sentences++;
//                         update_stats(msg_type);
                        
//                         // Display the sentence
//                         printf("[%6lu] %s\n", total_sentences, nmea_buffer);
                        
//                         // Parse and display details
//                         parse_and_display(nmea_buffer);
//                         printf("\n");
//                     } else {
//                         invalid_checksums++;
//                         printf("[%6lu] CHECKSUM ERROR: %s\n\n", total_sentences, nmea_buffer);
//                     }
//                 }
                
//                 buffer_idx = 0;
//             } 
//             else if (data != '\r' && buffer_idx < sizeof(nmea_buffer) - 1) {
//                 nmea_buffer[buffer_idx++] = data;
//             }
//         }
//     }
// }

// // Statistics display task
// static void stats_display_task(void *pvParameters) {
//     vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before first display
    
//     while (1) {
//         printf("\n");
//         printf("╔════════════════════════════════════════════════════════════╗\n");
//         printf("║              NMEA MESSAGE STATISTICS                       ║\n");
//         printf("╠════════════════════════════════════════════════════════════╣\n");
//         printf("║ Message Type │   Count   │  Interval  │  Rate (Hz)       ║\n");
//         printf("╠══════════════╪═══════════╪════════════╪══════════════════╣\n");
        
//         for (int i = 0; i < num_msg_types; i++) {
//             float rate_hz = 0.0f;
//             if (msg_stats[i].interval_ms > 0) {
//                 rate_hz = 1000.0f / msg_stats[i].interval_ms;
//             }
            
//             printf("║ %-12s │ %9lu │ %7lu ms │ %8.2f Hz     ║\n",
//                    msg_stats[i].name,
//                    msg_stats[i].count,
//                    msg_stats[i].interval_ms,
//                    rate_hz);
//         }
        
//         printf("╚════════════════════════════════════════════════════════════╝\n");
//         printf("\n");
        
//         vTaskDelay(pdMS_TO_TICKS(10000)); // Update every 10 seconds
//     }
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "NMEA GPS Monitor Starting...");
    
//     // Initialize GPS UART
//     gps_uart_init();
    
//     // Create GPS monitoring task
//     xTaskCreate(gps_monitor_task, "gps_monitor", 4096, NULL, 5, NULL);
    
//     // Create statistics display task
//     xTaskCreate(stats_display_task, "stats_display", 4096, NULL, 3, NULL);
    
//     ESP_LOGI(TAG, "Monitor tasks started");
//     ESP_LOGI(TAG, "Press Ctrl+] to exit monitor");
// }