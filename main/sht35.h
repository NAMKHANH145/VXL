#ifndef SHT35_H
#define SHT35_H

#include "esp_err.h"
#include "driver/i2c.h"
#include "config.h"

// Note: SHT35_ADDR_DEFAULT is now defined in config.h

/**
 * @brief Initialize the SHT35 driver configuration
 * 
 * @param port I2C port number
 * @param addr I2C address (usually 0x44 or 0x45)
 */
void sht35_init(i2c_port_t port, uint8_t addr);

/**
 * @brief Send the measurement command (High Repeatability, No Clock Stretching)
 *        Wait approx 15-20ms before reading.
 * 
 * @return ESP_OK on success
 */
esp_err_t sht35_start_measurement(void);

/**
 * @brief Read temperature and humidity data
 *        Call this after sht35_start_measurement and a delay.
 * 
 * @param t Pointer to float for Temperature
 * @param h Pointer to float for Humidity
 * @return ESP_OK on success, ESP_ERR_INVALID_CRC on data corruption
 */
esp_err_t sht35_read_data(float *t, float *h);

/**
 * @brief Enable or Disable the internal heater
 * 
 * @param enable true to turn on, false to turn off
 * @return ESP_OK on success
 */
esp_err_t sht35_heater_enable(bool enable);

#endif