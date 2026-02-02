#ifndef LCD_H
#define LCD_H

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>
#include "config.h" 

// NOTE: I2C Pins and Address are now defined in config.h
// LCD_ADDR, I2C_MASTER_NUM etc.

// HD44780 Instruction Set
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_CURSORSHIFT     0x10
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

// Entry Mode Flags
#define LCD_ENTRY_INC       0x02     // Increment cursor (Left to Right)
#define LCD_ENTRY_DEC       0x00     // Decrement cursor
#define LCD_ENTRY_SHIFT     0x01     // Shift display
#define LCD_ENTRY_NOSHIFT   0x00

// Display Control Flags
#define LCD_DISPLAYON       0x04
#define LCD_DISPLAYOFF      0x00
#define LCD_CURSORON        0x02
#define LCD_CURSOROFF       0x00
#define LCD_BLINKON         0x01
#define LCD_BLINKOFF        0x00

// Function Set Flags
#define LCD_4BITMODE        0x00
#define LCD_2LINE           0x08
#define LCD_5x8DOTS         0x00

// Backlight Control (PCF8574 specific)
#define LCD_BACKLIGHT       0x08
#define LCD_NOBACKLIGHT     0x00

// Control Bits
#define EN 0b00000100  // Enable Bit
#define RW 0b00000010  // Read/Write Bit
#define RS 0b00000001  // Register Select Bit

// Public API
/**
 * @brief Initialize the LCD (PCF8574 I2C Adapter).
 * Must be called AFTER i2c_driver_install() in main.
 */
esp_err_t lcd_init(void);

/**
 * @brief Send a Command byte to the LCD (RS=0).
 */
esp_err_t lcd_send_cmd(uint8_t cmd);

/**
 * @brief Send Data byte to the LCD (RS=1).
 * Use this to print characters.
 */
esp_err_t lcd_send_data(uint8_t data);

// Helper Functions
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_send_string(char *str);
void lcd_send_int(int number);
void lcd_send_float(float number, int precision);
void lcd_backlight_on(void);
void lcd_backlight_off(void);

#endif // LCD_H
