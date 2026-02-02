#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/ledc.h" // Added for PWM_RES type

// ================= HARDWARE CONFIGURATION =================

// --- I2C BUS (LCD & SHT35) ---
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          20000  // Reduced to 20kHz for extreme noise immunity

// --- LCD ---
#define LCD_ADDR                    0x27

// --- SHT35 ---
#define SHT35_ADDR_DEFAULT          0x44

// --- DS18B20 (1-Wire) ---
#define PIN_DS18B20                 GPIO_NUM_4

// --- PWM / MOSFETS ---
#define PIN_PELTIER                 GPIO_NUM_13
#define PIN_PUMP                    GPIO_NUM_17
#define PIN_FAN_BLOWER              GPIO_NUM_14
#define PIN_FAN_CASE                GPIO_NUM_26
#define PIN_FAN_CIRC                GPIO_NUM_27

// --- PWM SETTINGS ---
#define PWM_FREQ            1000        // 1kHz Frequency
#define PWM_RES             LEDC_TIMER_10_BIT // 10-bit Resolution (0-1023)
#define PWM_MAX_DUTY        1023

// --- LEDC CHANNEL MAPPING ---
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_TIMER              LEDC_TIMER_0
#define PWM_CH_BLOWER           LEDC_CHANNEL_0
#define PWM_CH_CASE             LEDC_CHANNEL_1
#define PWM_CH_PELTIER          LEDC_CHANNEL_2
#define PWM_CH_CIRC             LEDC_CHANNEL_3

// --- BUTTONS ---
#define PIN_BTN_MODE                GPIO_NUM_32
#define PIN_BTN_UP                  GPIO_NUM_33
#define PIN_BTN_DOWN                GPIO_NUM_25

// Button wiring: if your button connects one side to GND and the other to the GPIO
// (active-low), enable the internal pull-up. Set to 0 if your buttons connect to VCC
// and require an internal pull-down.
#ifndef BUTTON_USE_INTERNAL_PULLUP
#define BUTTON_USE_INTERNAL_PULLUP 1
#endif
// --- SD CARD (SPI) ---
#define PIN_SD_CS                   5
#define PIN_SD_MOSI                 23
#define PIN_SD_MISO                 19
#define PIN_SD_CLK                  18

// ================= SYSTEM CONSTANTS =================
#define SHT_SAMPLE_INTERVAL_MS      250   // Read SHT35 every 250ms (4Hz)
#define MA_WINDOW_SIZE              20    // 20 samples * 250ms = 5 seconds moving average
#define LOG_INTERVAL_MS             5000  // Log data every 5 seconds
#define FLUSH_INTERVAL_MS           900000 // Flush to SD card every 15 minutes (15 * 60 * 1000)

// ================= LOGIC CONSTANTS (V3.1) =================
#define DEFAULT_TARGET_HUM          60.0f
#define HYSTERESIS_HUM              2.0f   // (+- 2%)

// Thermostat & Protection Thresholds
#define TEMP_WATER_HEATER_BOOST     45.0f  // Duoi 45: Boost, Tren 45: Maintain
#define TEMP_WATER_COOLER_MIN       30.0f  // Map Min
#define TEMP_WATER_COOLER_MAX       50.0f  // Map Max
#define TEMP_WATER_EMERGENCY        70.0f  // 70C -> Emergency

// Timings
#define DEFROST_DURATION_MS         120000  // 2 Minutes
#define DEFROST_TRIGGER_MS          2700000 // 45 Minutes
#define SENSOR_READ_DELAY_MS        800

// Button Timing Constants
#define DUAL_PRESS_THRESHOLD_CYCLES 60  // 60 * 50ms = 3s
#define EMERGENCY_RESET_CYCLES      100 // 100 * 50ms = 5s

// Error Thresholds
#define SENSOR_ERROR_THRESHOLD      50  // Increased to allow more recovery time
#define SENSOR_ERROR_LOG_THRESHOLD  30  // Log warning when error count exceeds this

// ================= GLOBAL SHARED DATA =================
typedef struct {
    float t_air;
    float h_air;
    float t_water;
    bool air_ok;
    bool water_ok;
    uint32_t error_count;
} SensorData_t;

typedef struct {
    float target_hum;
    int mode; // 0: EDIT, 1: LOCK
} SystemSetpoints_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_HUMIDIFY,   // Tang am (Suc khi nong)
    STATE_DEHUMIDIFY, // Giam am (Ngung tu)
    STATE_EMERGENCY   // Loi qua nhiet/cam bien
} SystemState_t;

// Global Variables
extern SensorData_t g_sensors;
extern SystemSetpoints_t g_setpoints;
extern SystemState_t g_current_state; // Shared State for Logger
extern SemaphoreHandle_t g_mutex_sensors;
extern SemaphoreHandle_t g_mutex_setpoints;
extern SemaphoreHandle_t g_mutex_state;
extern const char *STATE_NAMES[];

#endif // CONFIG_H