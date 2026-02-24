#include "imu.h"

static const char *TAG = "BNO055";

esp_err_t save_calibration(uint8_t *data) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(my_handle, CALIB_KEY, data, 22);
    if (err == ESP_OK) err = nvs_commit(my_handle);
    
    nvs_close(my_handle);
    return err;
}

esp_err_t load_calibration(uint8_t *data) {
    nvs_handle_t my_handle;
    size_t required_size = 22;
    esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;

    err = nvs_get_blob(my_handle, CALIB_KEY, data, &required_size);
    nvs_close(my_handle);
    return err;
}

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                             I2C_MASTER_RX_BUF_DISABLE, 
                             I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t bno055_write_byte(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, BNO055_ADDR,
                                               write_buf, sizeof(write_buf),
                                               I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to register 0x%02X", reg_addr);
    }
    return ret;
}

esp_err_t bno055_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, BNO055_ADDR,
                                                 &reg_addr, 1, data, len,
                                                 I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from register 0x%02X", reg_addr);
    }
    return ret;
}

esp_err_t bno055_set_mode(uint8_t mode)
{
    esp_err_t ret;
    
    // Set to config mode first
    ret = bno055_write_byte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(30));  // Required delay after mode change
    
    // Set desired mode
    ret = bno055_write_byte(BNO055_OPR_MODE_ADDR, mode);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(30));
    
    return ESP_OK;
}

esp_err_t bno055_init(void)
{
    esp_err_t ret;
    uint8_t chip_id;
    uint8_t calib_data[22];

     // Read chip ID
    ret = bno055_read_bytes(BNO055_CHIP_ID_ADDR, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }
    
    if (chip_id != BNO055_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BNO055_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "BNO055 detected, chip ID: 0x%02X", chip_id);
    
    // Reset
    ret = bno055_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(650));  // Wait for reset

    if (load_calibration(calib_data) == ESP_OK) {
        ESP_LOGI(TAG, "Found saved calibration in NVS. Loading...");

        // Validate calibration data (simple checksum: sum of all bytes should be non-zero)
        uint32_t checksum = 0;
        for (int i = 0; i < 22; i++) {
            checksum += calib_data[i];
        }

        if (checksum == 0) {
            ESP_LOGW(TAG, "Calibration data appears invalid (all zeros), skipping load");
        } else {
            // Write calibration data with error checking - all or nothing approach
            bool write_success = true;
            for (int i = 0; i < 22; i++) {
                ret = bno055_write_byte(BNO055_OFFSET_DATA_ADDR + i, calib_data[i]);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to write calibration byte %d", i);
                    write_success = false;
                    break;
                }
            }

            if (write_success) {
                ESP_LOGI(TAG, "Calibration profile applied successfully.");
            } else {
                ESP_LOGW(TAG, "Calibration load incomplete, IMU will self-calibrate");
                // Don't use partial calibration - let IMU start fresh
            }
        }
    } else {
        ESP_LOGW(TAG, "No calibration profile found in NVS.");
    }
    
    // Set to normal power mode
    ret = bno055_write_byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set page 0
    ret = bno055_write_byte(BNO055_PAGE_ID_ADDR, 0);
    if (ret != ESP_OK) return ret;
    
    // Set system trigger
    ret = bno055_write_byte(BNO055_SYS_TRIGGER_ADDR, 0x00);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Set to NDOF mode (9DOF fusion)
    ret = bno055_set_mode(OPERATION_MODE_NDOF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set operation mode");
        return ret;
    }
    
    ESP_LOGI(TAG, "BNO055 initialized successfully in NDOF mode");
    return ESP_OK;
}

esp_err_t bno055_read_euler(float *heading, float *roll, float *pitch)
{
    uint8_t buffer[6];
    esp_err_t ret = bno055_read_bytes(BNO055_EULER_H_LSB, buffer, 6);
    
    if (ret == ESP_OK) {
        // Convert to degrees (1 degree = 16 LSB)
        int16_t h = (buffer[1] << 8) | buffer[0];
        int16_t r = (buffer[3] << 8) | buffer[2];
        int16_t p = (buffer[5] << 8) | buffer[4];
        
        *heading = h / 16.0f;
        *roll = r / 16.0f;
        *pitch = p / 16.0f;
    }
    
    return ret;
}

esp_err_t bno055_read_quaternion(float *w, float *x, float *y, float *z)
{
    uint8_t buffer[8];
    esp_err_t ret = bno055_read_bytes(BNO055_QUAT_DATA_W_LSB, buffer, 8);
    
    if (ret == ESP_OK) {
        // Convert to unit quaternion (1 unit = 2^14 LSB)
        int16_t qw = (buffer[1] << 8) | buffer[0];
        int16_t qx = (buffer[3] << 8) | buffer[2];
        int16_t qy = (buffer[5] << 8) | buffer[4];
        int16_t qz = (buffer[7] << 8) | buffer[6];
        
        float scale = 1.0f / 16384.0f;
        *w = qw * scale;
        *x = qx * scale;
        *y = qy * scale;
        *z = qz * scale;
    }
    
    return ret;
}

esp_err_t bno055_read_accel(float *x, float *y, float *z)
{
    uint8_t buffer[6];
    esp_err_t ret = bno055_read_bytes(BNO055_ACCEL_DATA_X_LSB, buffer, 6);
    
    if (ret == ESP_OK) {
        // Convert to m/s^2 (1 m/s^2 = 100 LSB)
        int16_t ax = (buffer[1] << 8) | buffer[0];
        int16_t ay = (buffer[3] << 8) | buffer[2];
        int16_t az = (buffer[5] << 8) | buffer[4];
        
        *x = ax / 100.0f;
        *y = ay / 100.0f;
        *z = az / 100.0f;
    }
    
    return ret;
}

esp_err_t bno055_read_gyro(float *x, float *y, float *z)
{
    uint8_t buffer[6];
    esp_err_t ret = bno055_read_bytes(BNO055_GYRO_DATA_X_LSB, buffer, 6);
    
    if (ret == ESP_OK) {
        // Convert to deg/s (1 deg/s = 16 LSB)
        int16_t gx = (buffer[1] << 8) | buffer[0];
        int16_t gy = (buffer[3] << 8) | buffer[2];
        int16_t gz = (buffer[5] << 8) | buffer[4];
        
        *x = gx / 16.0f;
        *y = gy / 16.0f;
        *z = gz / 16.0f;
    }
    
    return ret;
}

esp_err_t bno055_get_calibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag)
{
    uint8_t cal_status;
    esp_err_t ret = bno055_read_bytes(BNO055_CALIB_STAT_ADDR, &cal_status, 1);
    
    if (ret == ESP_OK) {
        *sys = (cal_status >> 6) & 0x03;
        *gyro = (cal_status >> 4) & 0x03;
        *accel = (cal_status >> 2) & 0x03;
        *mag = cal_status & 0x03;
    }
    
    return ret;
}

void send_imu_accel(float x, float y, float z, uint8_t cal)
{
    uint8_t data[8];
    
    // AccelX (16 bits signed, scale 0.001)
    int16_t accel_x = (int16_t)(x / 0.001);
    data[0] = (accel_x >> 0) & 0xFF;
    data[1] = (accel_x >> 8) & 0xFF;
    
    // AccelY (16 bits signed, scale 0.001)
    int16_t accel_y = (int16_t)(y / 0.001);
    data[2] = (accel_y >> 0) & 0xFF;
    data[3] = (accel_y >> 8) & 0xFF;
    
    // AccelZ (16 bits signed, scale 0.001)
    int16_t accel_z = (int16_t)(z / 0.001);
    data[4] = (accel_z >> 0) & 0xFF;
    data[5] = (accel_z >> 8) & 0xFF;
    
    // Calibration status (8 bits)
    data[6] = cal;
    
    data[7] = 0;  // Unused
    
    send_can_message(0x103, data, 8);
}

void send_imu_gyro(float x, float y, float z, uint8_t cal)
{
    uint8_t data[8];
    
    // GyroX (16 bits signed, scale 0.01)
    int16_t gyro_x = (int16_t)(x / 0.01);
    data[0] = (gyro_x >> 0) & 0xFF;
    data[1] = (gyro_x >> 8) & 0xFF;
    
    // GyroY (16 bits signed, scale 0.01)
    int16_t gyro_y = (int16_t)(y / 0.01);
    data[2] = (gyro_y >> 0) & 0xFF;
    data[3] = (gyro_y >> 8) & 0xFF;
    
    // GyroZ (16 bits signed, scale 0.01)
    int16_t gyro_z = (int16_t)(z / 0.01);
    data[4] = (gyro_z >> 0) & 0xFF;
    data[5] = (gyro_z >> 8) & 0xFF;
    
    // Calibration status (8 bits)
    data[6] = cal;
    
    data[7] = 0;  // Unused
    
    send_can_message(0x104, data, 8);
}

void send_imu_orientation(float heading, float pitch, float roll, uint8_t cal_sys, uint8_t cal_mag)
{
    uint8_t data[8];
    
    // Heading (16 bits, scale 0.01)
    uint16_t heading_scaled = (uint16_t)(heading / 0.01);
    data[0] = (heading_scaled >> 0) & 0xFF;
    data[1] = (heading_scaled >> 8) & 0xFF;
    
    // Pitch (16 bits signed, scale 0.01)
    int16_t pitch_scaled = (int16_t)(pitch / 0.01);
    data[2] = (pitch_scaled >> 0) & 0xFF;
    data[3] = (pitch_scaled >> 8) & 0xFF;
    
    // Roll (16 bits signed, scale 0.01)
    int16_t roll_scaled = (int16_t)(roll / 0.01);
    data[4] = (roll_scaled >> 0) & 0xFF;
    data[5] = (roll_scaled >> 8) & 0xFF;
    
    // System calibration (8 bits)
    data[6] = cal_sys;
    
    // Magnetometer calibration (8 bits)
    data[7] = cal_mag;
    
    send_can_message(0x105, data, 8);
}

void send_imu_quaternion(float w, float x, float y, float z)
{
    uint8_t data[8];
    
    // QuatW (16 bits signed, scale 0.0001)
    int16_t quat_w = (int16_t)(w / 0.0001);
    data[0] = (quat_w >> 0) & 0xFF;
    data[1] = (quat_w >> 8) & 0xFF;
    
    // QuatX (16 bits signed, scale 0.0001)
    int16_t quat_x = (int16_t)(x / 0.0001);
    data[2] = (quat_x >> 0) & 0xFF;
    data[3] = (quat_x >> 8) & 0xFF;
    
    // QuatY (16 bits signed, scale 0.0001)
    int16_t quat_y = (int16_t)(y / 0.0001);
    data[4] = (quat_y >> 0) & 0xFF;
    data[5] = (quat_y >> 8) & 0xFF;
    
    // QuatZ (16 bits signed, scale 0.0001)
    int16_t quat_z = (int16_t)(z / 0.0001);
    data[6] = (quat_z >> 0) & 0xFF;
    data[7] = (quat_z >> 8) & 0xFF;
    
    send_can_message(0x106, data, 8);
}

void bno055_read_task(void *arg)
{
    static bool already_saved = false;
    
    while (1) {
        float heading, roll, pitch;
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float quat_w, quat_x, quat_y, quat_z;
        uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
        static int count = 0;
        
        // Read all sensor data
        if (bno055_read_accel(&accel_x, &accel_y, &accel_z) == ESP_OK &&
            bno055_read_gyro(&gyro_x, &gyro_y, &gyro_z) == ESP_OK &&
            bno055_read_euler(&heading, &roll, &pitch) == ESP_OK &&
            bno055_read_quaternion(&quat_w, &quat_x, &quat_y, &quat_z) == ESP_OK &&
            bno055_get_calibration(&sys_cal, &gyro_cal, &accel_cal, &mag_cal) == ESP_OK) {
            
            // Send IMU data
            send_imu_accel(accel_x, accel_y, accel_z, accel_cal);
            send_imu_gyro(gyro_x, gyro_y, gyro_z, gyro_cal);
            send_imu_orientation(heading, pitch, roll, sys_cal, mag_cal);
            send_imu_quaternion(quat_w, quat_x, quat_y, quat_z);
            
            count++;
            if(count == 50){
                ESP_LOGI("BNO055", "IMU sent - Cal: S:%d G:%d A:%d M:%d", 
                            sys_cal, gyro_cal, accel_cal, mag_cal);
                count = 0;
            }
            
            // Auto-save logic
            if (!already_saved && sys_cal == 3 && gyro_cal == 3 && accel_cal == 3 && mag_cal == 3) {
                ESP_LOGI(TAG, "Full calibration reached! Saving to NVS...");

                uint8_t calib_to_save[22];

                // Switch to CONFIG mode
                bno055_set_mode(OPERATION_MODE_CONFIG);
                vTaskDelay(pdMS_TO_TICKS(25));  // Give time to switch modes

                // Read calibration offsets
                if (bno055_read_bytes(BNO055_OFFSET_DATA_ADDR, calib_to_save, 22) == ESP_OK) {
                    // Validate data before saving (ensure it's not all zeros)
                    uint32_t checksum = 0;
                    for (int i = 0; i < 22; i++) {
                        checksum += calib_to_save[i];
                    }

                    if (checksum == 0) {
                        ESP_LOGW(TAG, "Calibration data invalid (all zeros), not saving");
                    } else if (save_calibration(calib_to_save) == ESP_OK) {
                        ESP_LOGI(TAG, "Calibration saved successfully.");
                        already_saved = true;
                    } else {
                        ESP_LOGE(TAG, "Failed to save calibration to NVS");
                    }
                } else {
                    ESP_LOGE(TAG, "Failed to read calibration data");
                }

                // Switch back to NDOF
                bno055_set_mode(OPERATION_MODE_NDOF);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}