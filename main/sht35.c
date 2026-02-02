#include "sht35.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "SHT35";

static i2c_port_t SHT_PORT = I2C_NUM_0;

// Commands
#define CMD_MEAS_HIGH_REP_NO_STRETCH_MSB    0x24
#define CMD_MEAS_HIGH_REP_NO_STRETCH_LSB    0x00
#define CMD_HEATER_ENABLE_MSB               0x30
#define CMD_HEATER_ENABLE_LSB               0x6D
#define CMD_HEATER_DISABLE_LSB              0x66

void sht35_init(i2c_port_t port, uint8_t addr) {
    SHT_PORT = port;
    // We ignore 'addr' argument here if we enforce config.h usage, 
    // BUT to keep the driver flexible, we can use the argument if provided, 
    // or just log that we are using the one from config.h.
    // However, to strictly follow the instruction "remove duplication",
    // we should assume the driver uses the globally configured address.
    
    // NOTE: The 'addr' parameter is kept for API compatibility but 
    // the implementation below will use SHT35_ADDR_DEFAULT from config.h 
    // to guarantee consistency as requested.
    
    ESP_LOGI(TAG, "Initialized (Port %d, Addr 0x%02X)", port, SHT35_ADDR_DEFAULT);
}

// CRC-8 polynomial for SHT35: 0x31 (x^8 + x^5 + x^4 + 1) -> Init 0xFF
static uint8_t sht35_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

esp_err_t sht35_start_measurement(void) {
    uint8_t cmd[] = {CMD_MEAS_HIGH_REP_NO_STRETCH_MSB, CMD_MEAS_HIGH_REP_NO_STRETCH_LSB};
    // Use SHT35_ADDR_DEFAULT from config.h
    esp_err_t ret = i2c_master_write_to_device(SHT_PORT, SHT35_ADDR_DEFAULT, cmd, sizeof(cmd), 500 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Start Measure Failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sht35_read_data(float *t, float *h) {
    if (!t || !h) return ESP_ERR_INVALID_ARG;
    
    uint8_t d[6];
    // Use SHT35_ADDR_DEFAULT from config.h
    esp_err_t ret = i2c_master_read_from_device(SHT_PORT, SHT35_ADDR_DEFAULT, d, 6, 500 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read Data Failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Verify CRC for temperature
    if (sht35_crc8(&d[0], 2) != d[2]) {
        ESP_LOGW(TAG, "CRC Temp Fail: Calc=0x%02X, Read=0x%02X", sht35_crc8(&d[0], 2), d[2]);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Verify CRC for humidity
    if (sht35_crc8(&d[3], 2) != d[5]) {
        ESP_LOGW(TAG, "CRC Hum Fail: Calc=0x%02X, Read=0x%02X", sht35_crc8(&d[3], 2), d[5]);
        return ESP_ERR_INVALID_CRC;
    }
    
    // Conversion Formula (Sensirion Datasheet)
    uint16_t t_raw = (d[0] << 8) | d[1];
    *t = -45.0f + 175.0f * ((float)t_raw / 65535.0f);

    uint16_t h_raw = (d[3] << 8) | d[4];
    *h = 100.0f * ((float)h_raw / 65535.0f);
    
    return ESP_OK;
}

esp_err_t sht35_heater_enable(bool enable) {
    uint8_t cmd[2];
    cmd[0] = CMD_HEATER_ENABLE_MSB;
    cmd[1] = enable ? CMD_HEATER_ENABLE_LSB : CMD_HEATER_DISABLE_LSB;
    // Use SHT35_ADDR_DEFAULT from config.h
    return i2c_master_write_to_device(SHT_PORT, SHT35_ADDR_DEFAULT, cmd, sizeof(cmd), 500 / portTICK_PERIOD_MS);
}