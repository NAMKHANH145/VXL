#ifndef DS18B20_H
#define DS18B20_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "config.h" 

/**
 * @brief Initialize the DS18B20 GPIO.
 * Configures the pin as Open-Drain Output with internal Pull-up.
 * 
 * @param gpio_num The GPIO pin connected to the sensor data line.
 */
void ds18b20_init(gpio_num_t gpio_num);

/**
 * @brief Send the 'Convert T' (0x44) command to all sensors on the bus.
 * This starts the temperature conversion process.
 * 
 * @return ESP_OK on success (presence pulse detected), ESP_ERR_NOT_FOUND otherwise.
 */
esp_err_t ds18b20_trigger_conversion(void);

/**
 * @brief Read temperature from the sensor.
 * Assumes a single sensor on the bus or reads the first one responding.
 * 
 * @param temp Pointer to float where temperature (Celsius) will be stored.
 * @return 
 *    - ESP_OK: Success
 *    - ESP_ERR_NOT_FOUND: Sensor not responding
 *    - ESP_ERR_INVALID_CRC: Data integrity check failed
 *    - ESP_ERR_INVALID_RESPONSE: Value out of valid range (-55 to +125)
 */
esp_err_t ds18b20_read_temp(float *temp);

#endif // DS18B20_H
