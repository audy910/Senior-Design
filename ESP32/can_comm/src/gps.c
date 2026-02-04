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

    ESP_LOGI(TAG, "GNSS UART initialized");
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

    status.dead_reckoning = (pvt->fixType == 6); // DR-only
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

void ubx_checksum(const uint8_t *data, uint16_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (uint16_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

bool ubx_parse_byte(uint8_t byte, nav_pvt_t *out)
{
    static uint8_t buf[128];
    static uint16_t idx = 0;
    static uint16_t payload_len = 0;
    static enum {
        SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B
    } state = SYNC1;

    static uint8_t ck_a, ck_b;

    switch (state) {
    case SYNC1:
        if (byte == 0xB5) state = SYNC2;
        break;

    case SYNC2:
        if (byte == 0x62) state = CLASS;
        else state = SYNC1;
        break;

    case CLASS:
        buf[0] = byte;
        state = ID;
        break;

    case ID:
        buf[1] = byte;
        state = LEN1;
        break;

    case LEN1:
        buf[2] = byte;
        payload_len = byte;
        state = LEN2;
        break;

    case LEN2:
        buf[3] = byte;
        payload_len |= (byte << 8);
        idx = 0;
        state = PAYLOAD;
        break;

    case PAYLOAD:
        buf[4 + idx] = byte;
        idx++;
        if (idx == payload_len) {
            state = CK_A;
        }
        break;

    case CK_A:
        ck_a = byte;
        state = CK_B;
        break;

    case CK_B:
        ck_b = byte;

        uint8_t calc_a, calc_b;
        ubx_checksum(buf, payload_len + 4, &calc_a, &calc_b);

        if (calc_a == ck_a && calc_b == ck_b) {
            // NAV-PVT?
            if (buf[0] == 0x01 && buf[1] == 0x07 && payload_len == 92) {
                uint8_t *p = &buf[4]; 

                out->fixType = p[20];
                out->flags   = p[21];
                out->flags2  = p[22];
                out->numSV   = p[23];

                memcpy(&out->lon,     &p[24], 4);
                memcpy(&out->lat,     &p[28], 4);
                memcpy(&out->height,  &p[32], 4);
                memcpy(&out->hAcc,    &p[40], 4);
                memcpy(&out->vAcc,    &p[44], 4);
                memcpy(&out->velN,    &p[48], 4);
                memcpy(&out->velE,    &p[52], 4);
                memcpy(&out->velD,    &p[56], 4);
                memcpy(&out->gSpeed,  &p[60], 4);
                memcpy(&out->heading, &p[64], 4);
                memcpy(&out->pDOP,    &p[76], 2);
                memcpy(&out->headMot, &p[84], 4);

                state = SYNC1;
                return true;
            }
        }

        state = SYNC1;
        break;
    }

    return false;
}


void send_gps_position(double lat, double lon)
{
    uint8_t data[8];
    
    // Latitude (32 bits signed, scale 0.0000001)
    int32_t lat_scaled = (int32_t)(lat / 0.0000001);
    data[0] = (lat_scaled >> 0) & 0xFF;
    data[1] = (lat_scaled >> 8) & 0xFF;
    data[2] = (lat_scaled >> 16) & 0xFF;
    data[3] = (lat_scaled >> 24) & 0xFF;
    
    // Longitude (32 bits signed, scale 0.0000001)
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
    
    // Speed (24 bits, scale 0.001)
    uint32_t speed_scaled = (uint32_t)(speed / 0.001);
    data[0] = (speed_scaled >> 0) & 0xFF;
    data[1] = (speed_scaled >> 8) & 0xFF;
    data[2] = (speed_scaled >> 16) & 0xFF;
    
    // Course (16 bits, scale 0.01)
    uint16_t course_scaled = (uint16_t)(course / 0.01);
    data[3] = (course_scaled >> 0) & 0xFF;
    data[4] = (course_scaled >> 8) & 0xFF;
    
    // Fix type (8 bits)
    data[5] = fix_type;
    
    // Number of satellites (8 bits)
    data[6] = num_sats;
    
    // HDOP (8 bits, scale 0.1)
    data[7] = (uint8_t)(hdop / 0.1);
    
    send_can_message(0x101, data, 8);
}

void send_gps_accuracy(float h_acc, float v_acc)
{
    uint8_t data[8];
    
    // Horizontal accuracy (32 bits, scale 0.001)
    uint32_t h_acc_scaled = (uint32_t)(h_acc / 0.001);
    data[0] = (h_acc_scaled >> 0) & 0xFF;
    data[1] = (h_acc_scaled >> 8) & 0xFF;
    data[2] = (h_acc_scaled >> 16) & 0xFF;
    data[3] = (h_acc_scaled >> 24) & 0xFF;
    
    // Vertical accuracy (32 bits, scale 0.001)
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
    nav_pvt_t nav_pvt;
    
    while (1) {
        int len = uart_read_bytes(GNSS_UART, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        
        for (int i = 0; i < len; i++) {
            if (ubx_parse_byte(rx_buf[i], &nav_pvt)) {
                nav_status_t status = interpret_nav_pvt(&nav_pvt);
                
                if (status.gnss_ok) {
                    double lat = nav_pvt.lat / 10000000.0;
                    double lon = nav_pvt.lon / 10000000.0;
                    float speed = nav_pvt.gSpeed / 1000.0f;
                    float course = nav_pvt.headMot / 100000.0f;
                    
                    // Send GPS data
                    send_gps_position(lat, lon);
                    send_gps_velocity(speed, course, nav_pvt.fixType, 
                                     nav_pvt.numSV, nav_pvt.pDOP / 100.0f);
                    send_gps_accuracy(status.horiz_acc_m, status.vert_acc_m);
                    
                    ESP_LOGI("GNSS", "GPS sent - Fix:%d Sats:%d", 
                             nav_pvt.fixType, nav_pvt.numSV);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}