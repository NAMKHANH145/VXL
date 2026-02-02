#include "lcd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "LCD";

// Internal state to track backlight (Persistent)
static uint8_t lcd_backlight_state = LCD_BACKLIGHT;

/*
 * EXPLANATION OF 4-BIT I2C LCD (PCF8574):
 * 
 * The PCF8574 is an 8-bit I/O expander. We connect it to the 16-pin LCD.
 * Pin Mapping usually is:
 * P0 -> RS (Register Select)
 * P1 -> RW (Read/Write)
 * P2 -> EN (Enable)
 * P3 -> Backlight
 * P4-P7 -> D4-D7 (Data High Nibble)
 * 
 * Since we only have 4 data lines (D4-D7) connected, we use 4-bit mode.
 * To send a full byte (8 bits), we send it in two parts (nibbles):
 * 1. High Nibble (Bits 4-7)
 * 2. Low Nibble (Bits 0-3)
 * 
 * For EACH nibble, we must toggle the Enable (EN) pin:
 * 1. Set Data bits + EN High
 * 2. Wait >450ns
 * 3. Set Data bits + EN Low (Latch data)
 */

static esp_err_t lcd_write_nibble(uint8_t nibble, uint8_t mode)
{
    // Prepare the byte to send to PCF8574:
    // Nibble (D4-D7) | Backlight status | Control Mode (RS/RW)
    // Note: 'nibble' arg here is expected to be already shifted or just the upper 4 bits mask
    
    // 1. Data Setup (Enable HIGH)
    uint8_t data_en = (nibble & 0xF0) | lcd_backlight_state | mode | EN;
    
    // 2. Data Latch (Enable LOW)
    uint8_t data_dis = (nibble & 0xF0) | lcd_backlight_state | mode;
    
    uint8_t tx_data[2] = { data_en, data_dis };
    
    esp_err_t err = ESP_FAIL;
    
    // I2C Write with Retry Logic
    for (int i = 0; i < 3; i++) {
        err = i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, tx_data, 2, 100 / portTICK_PERIOD_MS);
        if (err == ESP_OK) break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C Error sending nibble 0x%02X: %s", nibble, esp_err_to_name(err));
    }
    return err;
}

static esp_err_t lcd_send_byte(uint8_t data, uint8_t mode)
{
    // Send High Nibble
    esp_err_t err = lcd_write_nibble(data & 0xF0, mode);
    if (err != ESP_OK) return err;
    
    // Send Low Nibble (Shifted to upper position)
    err = lcd_write_nibble((data << 4) & 0xF0, mode);
    return err;
}

esp_err_t lcd_send_cmd(uint8_t cmd) {
    return lcd_send_byte(cmd, 0); // RS = 0 (Command)
}

esp_err_t lcd_send_data(uint8_t data) {
    return lcd_send_byte(data, RS); // RS = 1 (Data)
}

void lcd_clear(void) {
    lcd_send_cmd(LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(10)); // Clear takes ~2ms, wait longer to be safe
}

void lcd_put_cur(int row, int col) {
    // DDRAM Addresses:
    // Line 1: 0x00 - 0x27
    // Line 2: 0x40 - 0x67
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_send_cmd(LCD_SETDDRAMADDR | addr);
}

void lcd_backlight_on(void) {
    lcd_backlight_state = LCD_BACKLIGHT;
    // We send a dummy byte just to update the backlight bit on the PCF8574
    // We can just write the state directly
    uint8_t data = lcd_backlight_state;
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, &data, 1, 100 / portTICK_PERIOD_MS);
}

void lcd_backlight_off(void) {
    lcd_backlight_state = LCD_NOBACKLIGHT;
    uint8_t data = lcd_backlight_state;
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, &data, 1, 100 / portTICK_PERIOD_MS);
}

esp_err_t lcd_init(void)
{
    esp_err_t err;
    ESP_LOGI(TAG, "Init LCD at 0x%02X...", LCD_ADDR);

    vTaskDelay(pdMS_TO_TICKS(100)); // Power-up delay

    // Magic Sequence to Initialize 4-bit mode reliably
    // 1. Try 8-bit mode 3 times
    err = lcd_write_nibble(0x30, 0); if(err) return err;
    vTaskDelay(pdMS_TO_TICKS(5));
    err = lcd_write_nibble(0x30, 0); if(err) return err;
    vTaskDelay(pdMS_TO_TICKS(1));
    err = lcd_write_nibble(0x30, 0); if(err) return err;
    vTaskDelay(pdMS_TO_TICKS(1));

    // 2. Set to 4-bit mode
    err = lcd_write_nibble(0x20, 0); if(err) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Configure Function Set: 4-bit, 2 Line, 5x8
    lcd_send_cmd(LCD_FUNCTIONSET | LCD_2LINE);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 4. Display OFF
    lcd_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYOFF);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 5. Clear
    lcd_clear();

    // 6. Entry Mode
    lcd_send_cmd(LCD_ENTRYMODESET | LCD_ENTRY_INC);
    vTaskDelay(pdMS_TO_TICKS(1));

    // 7. Display ON
    lcd_send_cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    
    ESP_LOGI(TAG, "LCD Init Done");
    return ESP_OK;
}

void lcd_send_string(char *str) {
    while (*str) lcd_send_data((uint8_t)(*str++));
}

void lcd_send_int(int number) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d", number);
    lcd_send_string(buffer);
}

void lcd_send_float(float number, int precision) {
    char buffer[16];
    char fmt[8];
    snprintf(fmt, sizeof(fmt), "%%.%df", precision);
    snprintf(buffer, sizeof(buffer), fmt, number);
    lcd_send_string(buffer);
}