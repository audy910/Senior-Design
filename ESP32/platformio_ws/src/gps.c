#include "gps.h"

static const char *TAG = "GNSS";

void gnss_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = GNSS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(
        GNSS_UART,
        UART_BUF_SIZE,
        0,
        0,
        NULL,
        0
    ));

    ESP_ERROR_CHECK(uart_param_config(GNSS_UART, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        GNSS_UART,
        GNSS_TX_PIN,
        GNSS_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    vTaskDelay(pdMS_TO_TICKS(1000));
    configure_gps_output();

    ESP_LOGI(TAG, "GNSS UART initialized at %d baud", GNSS_BAUDRATE);
}

bool nav_ready(const nav_status_t *s)
{
    return s->gnss_ok &&
           s->stable &&
           !s->dead_reckoning &&
           s->horiz_acc_m < 2.0f;
}

nav_status_t interpret_nav_pvt(const nav_pvt_t *pvt)
{
    nav_status_t status = {0};

    status.horiz_acc_m = pvt->hAcc / 1000.0f;
    status.vert_acc_m  = pvt->vAcc / 1000.0f;

    bool gnssFixOK = pvt->flags & (1 << 0);
    bool confirmed = pvt->flags2 & (1 << 5);
    bool carrFixed = ((pvt->flags >> 6) & 0x03) == 2;

    status.dead_reckoning = (pvt->fixType == 6);
    status.gnss_ok = gnssFixOK;
    status.stable  = confirmed;

    if (!gnssFixOK) {
        status.quality = NAV_QUALITY_INVALID;
    }
    else if (carrFixed) {
        status.quality = NAV_QUALITY_RTK;
    }
    else if (status.horiz_acc_m < 0.5f) {
        status.quality = NAV_QUALITY_EXCELLENT;
    }
    else if (status.horiz_acc_m < 2.0f) {
        status.quality = NAV_QUALITY_GOOD;
    }
    else {
        status.quality = NAV_QUALITY_COARSE;
    }

    return status;
}

double nmea_to_degrees(const char *coord, char direction)
{
    if (!coord || strlen(coord) < 4) return 0.0;
    
    double value = atof(coord);
    int degrees = (int)(value / 100);
    double minutes = value - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);
    
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

uint8_t nmea_checksum(const char *sentence)
{
    uint8_t checksum = 0;
    for (const char *p = sentence + 1; *p && *p != '*'; p++) {
        checksum ^= *p;
    }
    return checksum;
}

bool nmea_verify_checksum(const char *sentence)
{
    const char *checksum_pos = strchr(sentence, '*');
    if (!checksum_pos) return false;
    
    uint8_t calculated = nmea_checksum(sentence);
    uint8_t received = (uint8_t)strtol(checksum_pos + 1, NULL, 16);
    
    return calculated == received;
}

// Send UBX configuration to disable unwanted NMEA messages
void configure_gps_output(void)
{
    ESP_LOGI(TAG, "Configuring GPS output messages...");
    
    // Disable GPGSV (satellite info - not needed)
    const uint8_t disable_gpgsv[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 
        0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x02, 0x38
    };
    
    // Disable GNGLL (position - redundant with GGA)
    const uint8_t disable_gngll[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x2A
    };
    
    // Disable GNGSA (DOP and active satellites - can keep one)
    const uint8_t disable_gngsa[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
        0x02, 0x32
    };
    
    // Disable GAGSV, GBGSV, GQGSV (Galileo/BeiDou/QZSS satellite info)
    const uint8_t disable_gagsv[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0xF0, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x03, 0x3C
    };
    
    // Send configuration commands
    uart_write_bytes(GNSS_UART, disable_gpgsv, sizeof(disable_gpgsv));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uart_write_bytes(GNSS_UART, disable_gngll, sizeof(disable_gngll));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uart_write_bytes(GNSS_UART, disable_gngsa, sizeof(disable_gngsa));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uart_write_bytes(GNSS_UART, disable_gagsv, sizeof(disable_gagsv));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "GPS configuration sent");
}

// Fixed GNVTG parser
static bool parse_gnvtg(const char *sentence, nav_pvt_t *pvt)
{
    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    char *token = strtok(buffer, ",");
    int field = 0;
    
    while (token != NULL && field < 10) {
        switch(field) {
            case 1: // Track angle (True) in degrees
                if (strlen(token) > 0) {
                    pvt->headMot = (int32_t)(atof(token) * 100000); // degrees to 1e-5
                    pvt->heading = pvt->headMot;
                }
                break;
            case 7: // Speed in km/h
                if (strlen(token) > 0) {
                    float speed_kph = atof(token);
                    pvt->gSpeed = (int32_t)(speed_kph * 277.778); // km/h to mm/s
                }
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }
    
    return true;
}

// Updated parse function for GNGGA
static bool parse_gngga(const char *sentence, nav_pvt_t *pvt)
{
    char buffer[128];
    strncpy(buffer, sentence, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    
    char *token = strtok(buffer, ",");
    int field = 0;
    
    char lat_str[16] = {0}, lat_dir = 0;
    char lon_str[16] = {0}, lon_dir = 0;
    uint8_t fix_quality = 0;
    
    while (token != NULL && field < 15) {
        switch(field) {
            case 2: // Latitude
                strncpy(lat_str, token, sizeof(lat_str) - 1);
                break;
            case 3: // N/S
                lat_dir = token[0];
                break;
            case 4: // Longitude
                strncpy(lon_str, token, sizeof(lon_str) - 1);
                break;
            case 5: // E/W
                lon_dir = token[0];
                break;
            case 6: // Fix quality
                fix_quality = atoi(token);
                break;
            case 7: // Number of satellites
                pvt->numSV = atoi(token);
                break;
            case 8: // HDOP
                pvt->pDOP = (uint16_t)(atof(token) * 100);
                break;
            case 9: // Altitude
                pvt->height = (int32_t)(atof(token) * 1000); // m to mm
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }
    
    if (lat_str[0] && lon_str[0]) {
        double lat = nmea_to_degrees(lat_str, lat_dir);
        double lon = nmea_to_degrees(lon_str, lon_dir);
        
        pvt->lat = (int32_t)(lat * 10000000.0);
        pvt->lon = (int32_t)(lon * 10000000.0);
        
        // Map fix quality: 0=no fix, 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
        if (fix_quality == 0) {
            pvt->fixType = 0;
            pvt->flags = 0;
        } else if (fix_quality == 1) {
            pvt->fixType = 3; // 3D fix
            pvt->flags = 0x01; // gnssFixOK
        } else if (fix_quality >= 2) {
            pvt->fixType = 3;
            pvt->flags = 0x01 | (2 << 6); // gnssFixOK + differential
        }
        
        pvt->flags2 = (1 << 5); // confirmed
        
        // Accuracy estimates based on fix quality and HDOP
        float hdop = pvt->pDOP / 100.0f;
        if (fix_quality == 0) {
            pvt->hAcc = 100000; // 100m
            pvt->vAcc = 100000;
        } else if (fix_quality == 1) {
            pvt->hAcc = (uint32_t)(hdop * 5000); // HDOP * 5m
            pvt->vAcc = (uint32_t)(hdop * 8000); // HDOP * 8m
        } else {
            pvt->hAcc = (uint32_t)(hdop * 1000); // HDOP * 1m for DGPS
            pvt->vAcc = (uint32_t)(hdop * 2000);
        }
        
        return true;
    }
    
    return false;
}

// Main NMEA parser
bool nmea_parse_byte(uint8_t byte, nav_pvt_t *out)
{
    static char nmea_buffer[128];
    static uint8_t idx = 0;
    static nav_pvt_t cached_pvt = {0};
    
    if (byte == '\n') {
        nmea_buffer[idx] = '\0';
        
        if (nmea_verify_checksum(nmea_buffer)) {
            
            // Parse GNGGA - position data (10Hz trigger)
            if (strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
                if (parse_gngga(nmea_buffer, &cached_pvt)) {
                    memcpy(out, &cached_pvt, sizeof(nav_pvt_t));
                    idx = 0;
                    return true; // Trigger update on GGA
                }
            }
            // Parse GNVTG - velocity data (updates cache)
            else if (strncmp(nmea_buffer, "$GNVTG", 6) == 0) {
                parse_gnvtg(nmea_buffer, &cached_pvt);
                // Don't trigger - just update velocity in cache
            }
        }
        
        idx = 0;
    } 
    else if (byte != '\r' && idx < sizeof(nmea_buffer) - 1) {
        nmea_buffer[idx++] = byte;
    }
    
    return false;
}

void send_gps_position(double lat, double lon)
{
    uint8_t data[8];
    
    int32_t lat_scaled = (int32_t)(lat / 0.0000001);
    data[0] = (lat_scaled >> 0) & 0xFF;
    data[1] = (lat_scaled >> 8) & 0xFF;
    data[2] = (lat_scaled >> 16) & 0xFF;
    data[3] = (lat_scaled >> 24) & 0xFF;
    
    int32_t lon_scaled = (int32_t)(lon / 0.0000001);
    data[4] = (lon_scaled >> 0) & 0xFF;
    data[5] = (lon_scaled >> 8) & 0xFF;
    data[6] = (lon_scaled >> 16) & 0xFF;
    data[7] = (lon_scaled >> 24) & 0xFF;
    
    send_can_message(0x100, data, 8);
}

void send_gps_velocity(float speed, float course, uint8_t fix_type, uint8_t num_sats, float hdop)
{
    uint8_t data[8];
    
    uint32_t speed_scaled = (uint32_t)(speed / 0.001);
    data[0] = (speed_scaled >> 0) & 0xFF;
    data[1] = (speed_scaled >> 8) & 0xFF;
    data[2] = (speed_scaled >> 16) & 0xFF;
    
    uint16_t course_scaled = (uint16_t)(course / 0.01);
    data[3] = (course_scaled >> 0) & 0xFF;
    data[4] = (course_scaled >> 8) & 0xFF;
    
    data[5] = fix_type;
    data[6] = num_sats;
    data[7] = (uint8_t)(hdop / 0.1);
    
    send_can_message(0x101, data, 8);
}

void send_gps_accuracy(float h_acc, float v_acc)
{
    uint8_t data[8];
    
    uint32_t h_acc_scaled = (uint32_t)(h_acc / 0.001);
    data[0] = (h_acc_scaled >> 0) & 0xFF;
    data[1] = (h_acc_scaled >> 8) & 0xFF;
    data[2] = (h_acc_scaled >> 16) & 0xFF;
    data[3] = (h_acc_scaled >> 24) & 0xFF;
    
    uint32_t v_acc_scaled = (uint32_t)(v_acc / 0.001);
    data[4] = (v_acc_scaled >> 0) & 0xFF;
    data[5] = (v_acc_scaled >> 8) & 0xFF;
    data[6] = (v_acc_scaled >> 16) & 0xFF;
    data[7] = (v_acc_scaled >> 24) & 0xFF;
    
    send_can_message(0x102, data, 8);
}

void gnss_uart_task(void *arg)
{
    uint8_t rx_buf[256];
    nav_pvt_t nav_pvt = {0};
    
    ESP_LOGI(TAG, "GNSS task started (Protocol: NMEA)");
    
    while (1) {
        int len = uart_read_bytes(GNSS_UART, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        
        for (int i = 0; i < len; i++) {
            bool parsed = false;
            
            parsed = nmea_parse_byte(rx_buf[i], &nav_pvt);
            
            if (parsed) {
                nav_status_t status = interpret_nav_pvt(&nav_pvt);
                
                if (status.gnss_ok) {
                    double lat = nav_pvt.lat / 10000000.0;
                    double lon = nav_pvt.lon / 10000000.0;
                    float speed = nav_pvt.gSpeed / 1000.0f;
                    float course = nav_pvt.headMot / 100000.0f;
                    
                    send_gps_position(lat, lon);
                    send_gps_velocity(speed, course, nav_pvt.fixType, 
                                     nav_pvt.numSV, nav_pvt.pDOP / 100.0f);
                    send_gps_accuracy(status.horiz_acc_m, status.vert_acc_m);
                    
                    ESP_LOGI(TAG, "GPS: Lat=%.7f Lon=%.7f Fix=%d Sats=%d Acc=%.2fm", 
                             lat, lon, nav_pvt.fixType, nav_pvt.numSV, status.horiz_acc_m);
                }
            }
        }
    }
}