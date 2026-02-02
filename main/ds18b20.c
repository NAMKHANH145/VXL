#include "ds18b20.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// Current configured pin
static gpio_num_t s_ds18b20_pin = PIN_DS18B20; 
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// ================= INTERNAL 1-WIRE PRIMITIVES =================

/**
 * @brief Calculate CRC8 for 1-Wire data.
 * Polynomial: x^8 + x^5 + x^4 + 1
 */
static uint8_t calc_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

/**
 * @brief Reset the 1-Wire bus and check for presence pulse.
 */
static bool ow_reset(void) {
    bool presence;
    
    // 1. Drive Low for 480us (Reset Pulse)
    gpio_set_level(s_ds18b20_pin, 0);
    esp_rom_delay_us(480);
    
    // 2. Release line (High-Z) and wait
    portENTER_CRITICAL(&spinlock);
    gpio_set_level(s_ds18b20_pin, 1);
    portEXIT_CRITICAL(&spinlock);
    
    // 3. Wait 70us for DS18B20 to pull low
    esp_rom_delay_us(70);
    
    // 4. Sample the line
    portENTER_CRITICAL(&spinlock);
    presence = !gpio_get_level(s_ds18b20_pin);
    portEXIT_CRITICAL(&spinlock);
    
    // 5. Wait for remainder of timeslot (480 - 70 = 410)
    esp_rom_delay_us(410);
    
    return presence;
}

/**
 * @brief Write a single bit to the bus.
 */
static void ow_wr_bit(int b) {
    portENTER_CRITICAL(&spinlock);
    
    gpio_set_level(s_ds18b20_pin, 0); // Pull Low
    // Write 1: Low for 6us, then Release
    // Write 0: Low for 60us, then Release
    esp_rom_delay_us(b ? 6 : 60); 
    
    gpio_set_level(s_ds18b20_pin, 1); // Release
    // Wait for slot to finish
    esp_rom_delay_us(b ? 64 : 10);
    
    portEXIT_CRITICAL(&spinlock);
}

/**
 * @brief Read a single bit from the bus.
 */
static int ow_rd_bit(void) {
    int b;
    portENTER_CRITICAL(&spinlock);
    
    gpio_set_level(s_ds18b20_pin, 0); // Pull Low
    esp_rom_delay_us(6);              // Hold for 6us
    gpio_set_level(s_ds18b20_pin, 1); // Release
    esp_rom_delay_us(9);              // Wait for stable sample window
    
    b = gpio_get_level(s_ds18b20_pin); // Sample
    
    portEXIT_CRITICAL(&spinlock);
    
    esp_rom_delay_us(55); // Complete the timeslot
    return b;
}

static void ow_wr_byte(uint8_t d) {
    for (int i = 0; i < 8; i++) {
        ow_wr_bit(d & 1);
        d >>= 1;
    }
}

static uint8_t ow_rd_byte(void) {
    uint8_t d = 0;
    for (int i = 0; i < 8; i++) {
        if (ow_rd_bit()) d |= (1 << i);
    }
    return d;
}

// ================= PUBLIC API =================

void ds18b20_init(gpio_num_t gpio_num) {
    s_ds18b20_pin = gpio_num;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_ds18b20_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD, // Open Drain is critical for 1-Wire
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Internal Pull-up helps signal stability
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Ensure idle high
    gpio_set_level(s_ds18b20_pin, 1);
}

esp_err_t ds18b20_trigger_conversion(void) {
    for (int attempt = 0; attempt < 3; attempt++) {
        if (ow_reset()) {
            ow_wr_byte(0xCC); // Skip ROM (Address all devices)
            ow_wr_byte(0x44); // Convert T
            return ESP_OK;
        }
        esp_rom_delay_us(2000);
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t ds18b20_read_scratchpad(uint8_t data[9]) {
    if (!ow_reset()) return ESP_ERR_NOT_FOUND;
    
    ow_wr_byte(0xCC); // Skip ROM
    ow_wr_byte(0xBE); // Read Scratchpad
    
    for (int i = 0; i < 9; i++) {
        data[i] = ow_rd_byte();
    }
    
    // Verify CRC (Last byte is CRC)
    if (calc_crc8(data, 8) != data[8]) {
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

esp_err_t ds18b20_read_temp(float *temp) {
    if (!temp) return ESP_ERR_INVALID_ARG;

    uint8_t data[9];
    esp_err_t ret = ESP_FAIL;
    
    // Attempt read up to 3 times
    for (int attempt = 0; attempt < 3; attempt++) {
        ret = ds18b20_read_scratchpad(data);
        if (ret == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield before retry
    }
    if (ret != ESP_OK) return ret;

    // Convert: 12-bit resolution (0.0625 C per bit)
    int16_t raw = (data[1] << 8) | data[0];
    float t = (float)raw / 16.0f;

    // 85.0C is the power-on reset value of the register
    // If we read exactly 85.0C without a successful conversion, it's likely invalid.
    if (t > 125.0f || t < -55.0f || fabsf(t - 85.0f) < 0.001f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *temp = t;
    return ESP_OK;
}
