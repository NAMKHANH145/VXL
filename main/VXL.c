#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_task_wdt.h"

#define TAG "VXL_FINAL"

// ================= PINS (ĐỊNH NGHĨA CHÂN KẾT NỐI) =================
// --- ACTUATORS (THIẾT BỊ ĐIỀU KHIỂN - QUA MOSFET) ---
#define PIN_PELTIER         GPIO_NUM_13 // Module MOSFET 1 -> Sò nóng lạnh
#define PIN_PUMP            GPIO_NUM_17 // Module MOSFET 2 -> Máy bơm khí
#define PIN_FAN_BLOWER      GPIO_NUM_14 // Module MOSFET 3 -> Quạt sên (PWM)
#define PIN_FAN_CASE        GPIO_NUM_26 // Module MOSFET 4 -> Quạt tản vỏ (PWM)
#define PIN_FAN_CIRC        GPIO_NUM_27 // Module MOSFET 5 -> Quạt đối lưu (Luôn bật)

// --- SENSORS (CẢM BIẾN) ---
#define PIN_DS18B20         GPIO_NUM_4  // Cảm biến nhiệt nước (Dây vàng)

// --- I2C BUS (MÀN HÌNH LCD & CẢM BIẾN SHT35) ---
#define PIN_I2C_SDA         GPIO_NUM_21 // Dây SDA
#define PIN_I2C_SCL         GPIO_NUM_22 // Dây SCL

// --- SPI BUS (MODULE THẺ NHỚ MICRO SD) ---
#define PIN_SD_CS           GPIO_NUM_5  // Chân CS
#define PIN_SD_MOSI         GPIO_NUM_23 // Chân MOSI
#define PIN_SD_MISO         GPIO_NUM_19 // Chân MISO
#define PIN_SD_CLK          GPIO_NUM_18 // Chân CLK (SCK)

// --- BUTTONS (NÚT NHẤN - KÍCH ÂM) ---
#define PIN_BTN_MODE        GPIO_NUM_32 // Nút Chế độ (MODE)
#define PIN_BTN_UP          GPIO_NUM_33 // Nút Tăng (UP)
#define PIN_BTN_DOWN        GPIO_NUM_25 // Nút Giảm (DOWN)

// ================= SETTINGS =================
#define PWM_FREQ            5000
#define PWM_RES             LEDC_TIMER_10_BIT
#define PWM_CH_BLOWER       LEDC_CHANNEL_0
#define PWM_CH_CASE         LEDC_CHANNEL_1

#define FILTER_SAMPLES      10
#define AFTER_COOL_TIME_MS  45000 // 45s
#define SOFT_START_STEP_MS  20    // 2s total for 100 steps
#define LOG_INTERVAL_MS     10000 // Log every 10s

// Thresholds
#define TEMP_WATER_WARMUP   40.0f
#define TEMP_WATER_AUX_ON   60.0f
#define TEMP_WATER_CRITICAL 70.0f

// ================= GLOBALS =================
typedef enum {
    STATE_IDLE = 0,
    STATE_HUMIDIFY,
    STATE_DEHUMIDIFY,
    STATE_AFTER_COOL, // New: Run Cooling fan after DEH
    STATE_PROTECT_ERR
} SystemState_t;

typedef struct {
    float buf[FILTER_SAMPLES];
    int idx;
    float sum;
    bool full;
} MovingAvg_t;

typedef struct {
    float t_air;
    float h_air;
    bool  env_valid;
    float t_water;
    bool  water_valid;
    
    // Settings
    float target_hum;
    float hysteresis;

    SystemState_t current_state;
    bool sd_mounted;
    uint32_t uptime_s;
    
    // UI
    bool edit_mode;
    int  edit_item; // 0: Target, 1: Hysteresis
} SharedData_t;

static SharedData_t sys = {
    .target_hum = 75.0f,
    .hysteresis = 3.0f,
    .current_state = STATE_IDLE,
    .sd_mounted = false
};
static SemaphoreHandle_t mutex = NULL;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// I2C Addresses
static uint8_t lcd_addr = 0x27;
static uint8_t sht_addr = 0x44;

// Actuator Target Duties (for Soft Start)
static int tgt_duty_blower = 0;
static int tgt_duty_case = 0;
static int cur_duty_blower = 0;
static int cur_duty_case = 0;

// ================= HELPERS =================
void filter_add(MovingAvg_t *f, float val) {
    if (val != val) return; // NaN check
    if (!f->full && f->idx < FILTER_SAMPLES) {
        f->buf[f->idx++] = val;
        f->sum += val;
        if (f->idx >= FILTER_SAMPLES) f->full = true;
    } else {
        f->idx %= FILTER_SAMPLES;
        f->sum -= f->buf[f->idx];
        f->buf[f->idx] = val;
        f->sum += val;
        f->idx++;
    }
}
float filter_get(MovingAvg_t *f) {
    if (f->idx == 0 && !f->full) return 0;
    int count = f->full ? FILTER_SAMPLES : f->idx;
    return f->sum / count;
}

void log_to_sd(float t, float h, float w, int st) {
    if(!sys.sd_mounted) return;
    FILE *f = fopen("/sdcard/log.csv", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open log file");
        return;
    }
    // Format: Uptime(ms), T_Air, H_Air, T_Water, State
    fprintf(f, "%lu,%.1f,%.1f,%.1f,%d\n", (unsigned long)(esp_timer_get_time()/1000), t, h, w, st);
    fclose(f);
}

// ================= DRIVERS =================
// --- GPIO & PWM ---
void init_hw(void) {
    // Outputs
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL<<PIN_PELTIER)|(1ULL<<PIN_PUMP)|(1ULL<<PIN_FAN_CIRC),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en=0, .pull_down_en=0
    };
    gpio_config(&out_conf);
    gpio_set_level(PIN_FAN_CIRC, 1); // Always ON

    // Inputs (Buttons)
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL<<PIN_BTN_MODE)|(1ULL<<PIN_BTN_UP)|(1ULL<<PIN_BTN_DOWN),
        .mode = GPIO_MODE_INPUT, .pull_up_en=1, .pull_down_en=0
    };
    gpio_config(&in_conf);

    // PWM
    ledc_timer_config_t timer = {
        .speed_mode=LEDC_LOW_SPEED_MODE, .timer_num=LEDC_TIMER_0,
        .duty_resolution=PWM_RES, .freq_hz=PWM_FREQ, .clk_cfg=LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);
    
    ledc_channel_config_t ch0 = { .gpio_num=PIN_FAN_BLOWER, .speed_mode=LEDC_LOW_SPEED_MODE, .channel=PWM_CH_BLOWER, .timer_sel=LEDC_TIMER_0, .duty=0 };
    ledc_channel_config(&ch0);
    ledc_channel_config_t ch1 = { .gpio_num=PIN_FAN_CASE, .speed_mode=LEDC_LOW_SPEED_MODE, .channel=PWM_CH_CASE, .timer_sel=LEDC_TIMER_0, .duty=0 };
    ledc_channel_config(&ch1);
}

void set_pwm(int ch, int duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

// --- OneWire DS18B20 ---
void ow_delay(int us) { esp_rom_delay_us(us); }
bool ow_reset(void) {
    portENTER_CRITICAL(&spinlock);
    gpio_set_direction(PIN_DS18B20, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DS18B20, 0);
    portEXIT_CRITICAL(&spinlock);
    ow_delay(480);
    portENTER_CRITICAL(&spinlock);
    gpio_set_direction(PIN_DS18B20, GPIO_MODE_INPUT);
    portEXIT_CRITICAL(&spinlock);
    ow_delay(70);
    int p = gpio_get_level(PIN_DS18B20);
    ow_delay(410);
    return !p;
}
void ow_wr_bit(int b) {
    portENTER_CRITICAL(&spinlock);
    gpio_set_direction(PIN_DS18B20, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DS18B20, 0);
    ow_delay(b ? 6 : 60);
    gpio_set_direction(PIN_DS18B20, GPIO_MODE_INPUT);
    ow_delay(b ? 64 : 10);
    portEXIT_CRITICAL(&spinlock);
}
int ow_rd_bit(void) {
    portENTER_CRITICAL(&spinlock);
    gpio_set_direction(PIN_DS18B20, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DS18B20, 0);
    ow_delay(6);
    gpio_set_direction(PIN_DS18B20, GPIO_MODE_INPUT);
    ow_delay(9);
    int b = gpio_get_level(PIN_DS18B20);
    portEXIT_CRITICAL(&spinlock);
    ow_delay(55);
    return b;
}
void ow_wr_byte(uint8_t d) { for(int i=0;i<8;i++) { ow_wr_bit(d&1); d>>=1; } }
uint8_t ow_rd_byte(void) { uint8_t d=0; for(int i=0;i<8;i++) { if(ow_rd_bit()) d|=(1<<i); } return d; }
esp_err_t ds18b20_read(float *t) {
    if(!ow_reset()) return ESP_FAIL;
    ow_wr_byte(0xCC); ow_wr_byte(0x44);
    // Don't wait here, assume 1s cycle time
    if(!ow_reset()) return ESP_FAIL;
    ow_wr_byte(0xCC); ow_wr_byte(0xBE);
    uint8_t l=ow_rd_byte(), h=ow_rd_byte();
    int16_t r = (h<<8)|l;
    *t = r/16.0;
    return ESP_OK;
}

// --- I2C ---
void i2c_init_bus(void) {
    i2c_config_t c = { .mode=I2C_MODE_MASTER, .sda_io_num=PIN_I2C_SDA, .scl_io_num=PIN_I2C_SCL, .sda_pullup_en=1, .scl_pullup_en=1, .master.clk_speed=100000 };
    i2c_param_config(0, &c); i2c_driver_install(0, c.mode, 0,0,0);
}
esp_err_t sht_read(float *t, float *h) {
    uint8_t cmd[]={0x24,0x00};
    i2c_master_write_to_device(0, sht_addr, cmd, 2, 100);
    vTaskDelay(20/portTICK_PERIOD_MS);
    uint8_t d[6];
    if(i2c_master_read_from_device(0, sht_addr, d, 6, 100)!=ESP_OK) return ESP_FAIL;
    *t = -45 + 175*((d[0]<<8)|d[1])/65535.0;
    *h = 100*((d[3]<<8)|d[4])/65535.0;
    return ESP_OK;
}

// --- LCD ---
void lcd_nib(uint8_t n, int rs) {
    uint8_t d = (n&0xF0)|(rs?1:0)|0x08;
    uint8_t p[] = {d|4, d};
    i2c_master_write_to_device(0, lcd_addr, p, 2, 100);
}
void lcd_byte(uint8_t b, int rs) { lcd_nib(b,rs); lcd_nib(b<<4,rs); }
void lcd_cmd(uint8_t c) { lcd_byte(c,0); }
void lcd_str(const char *s) { while(*s) lcd_byte(*s++,1); }
void lcd_init_dev(void) {
    vTaskDelay(50/portTICK_PERIOD_MS); lcd_nib(0x30,0); vTaskDelay(5/portTICK_PERIOD_MS);
    lcd_nib(0x30,0); lcd_nib(0x30,0); lcd_nib(0x20,0);
    lcd_cmd(0x28); lcd_cmd(0x0C); lcd_cmd(0x06); lcd_cmd(0x01); vTaskDelay(2/portTICK_PERIOD_MS);
}
void lcd_pos(int r, int c) { lcd_cmd((r?0xC0:0x80)|c); }

// --- SD ---
void sd_init(void) {
    esp_vfs_fat_sdmmc_mount_config_t mc = {.format_if_mount_failed=1, .max_files=5};
    sdmmc_card_t *card;
    sdmmc_host_t h = SDSPI_HOST_DEFAULT();
    // Reduce freq to 5MHz for stability with jumper wires
    h.max_freq_khz = 5000; 
    
    spi_bus_config_t b = {.mosi_io_num=PIN_SD_MOSI, .miso_io_num=PIN_SD_MISO, .sclk_io_num=PIN_SD_CLK, .quadwp_io_num=-1, .quadhd_io_num=-1};
    spi_bus_initialize(h.slot, &b, SDSPI_DEFAULT_DMA);
    sdspi_device_config_t s = SDSPI_DEVICE_CONFIG_DEFAULT();
    s.gpio_cs = PIN_SD_CS; s.host_id = h.slot;
    sys.sd_mounted = (esp_vfs_fat_sdspi_mount("/sdcard", &h, &s, &mc, &card)==ESP_OK);
    if(sys.sd_mounted) {
        // Write header if new file
        FILE *f = fopen("/sdcard/log.csv", "r");
        if(!f) {
            f = fopen("/sdcard/log.csv", "w");
            if(f) { fprintf(f, "TimeMS,TempAir,HumAir,TempWater,State\n"); fclose(f); }
        } else fclose(f);
    }
}

// ================= TASKS =================

// --- Soft Start PWM Task ---
void task_soft_start(void *arg) {
    while(1) {
        // Ramp Blower
        if(cur_duty_blower < tgt_duty_blower) cur_duty_blower += 10;
        else if(cur_duty_blower > tgt_duty_blower) cur_duty_blower = tgt_duty_blower; // Instant Cut check below
        
        // Ramp Cooling
        if(cur_duty_case < tgt_duty_case) cur_duty_case += 10;
        else if(cur_duty_case > tgt_duty_case) cur_duty_case -= 10;

        if(cur_duty_blower > 1023) cur_duty_blower = 1023;
        if(cur_duty_case > 1023) cur_duty_case = 1023;

        set_pwm(PWM_CH_BLOWER, cur_duty_blower);
        set_pwm(PWM_CH_CASE, cur_duty_case);

        vTaskDelay(SOFT_START_STEP_MS / portTICK_PERIOD_MS);
    }
}

void task_control(void *arg) {
    esp_task_wdt_config_t wdt_conf = {
        .timeout_ms = 5000,
        .idle_core_mask = (1 << 0) | (1 << 1),    
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_conf);
    esp_task_wdt_add(NULL);
    
    MovingAvg_t favg_t = {0}, favg_h = {0};
    uint32_t after_cool_start = 0;
    uint32_t last_log_time = 0;

    // NOTE: Defrost logic from report requires cold-side sensor which is missing in BOM.
    // We only have one DS18B20 for Hot Water. Defrost is omitted for now.

    while(1) {
        esp_task_wdt_reset();
        
        // Read & Filter
        float t=0, h=0, w=0;
        bool e_ok = (sht_read(&t, &h) == ESP_OK);
        bool w_ok = (ds18b20_read(&w) == ESP_OK);
        
        if(e_ok) { filter_add(&favg_t, t); filter_add(&favg_h, h); }
        float fil_t = filter_get(&favg_t);
        float fil_h = filter_get(&favg_h);

        // Update Shared
        xSemaphoreTake(mutex, portMAX_DELAY);
        sys.t_air = fil_t; sys.h_air = fil_h; sys.env_valid = e_ok;
        sys.t_water = w; sys.water_valid = w_ok;
        float tgt = sys.target_hum;
        float hys = sys.hysteresis;
        xSemaphoreGive(mutex);

        // Logging
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_log_time > LOG_INTERVAL_MS) {
            log_to_sd(fil_t, fil_h, w, sys.current_state);
            last_log_time = now;
        }

        // Logic
        SystemState_t next = sys.current_state;
        bool ovr_heat = (w_ok && w > TEMP_WATER_CRITICAL);
        bool aux_cool = (w_ok && w > TEMP_WATER_AUX_ON); // > 60 C

        if(ovr_heat) next = STATE_PROTECT_ERR;
        else if(sys.current_state == STATE_PROTECT_ERR && w < 60) next = STATE_IDLE; // Recovery

        if(next != STATE_PROTECT_ERR) {
            if(sys.current_state == STATE_IDLE || sys.current_state == STATE_AFTER_COOL) {
                if(fil_h < tgt - hys) next = STATE_HUMIDIFY;
                else if(fil_h > tgt + hys) next = STATE_DEHUMIDIFY;
                else if(sys.current_state == STATE_AFTER_COOL) {
                    if(now - after_cool_start > AFTER_COOL_TIME_MS) 
                        next = STATE_IDLE;
                }
            }
            else if(sys.current_state == STATE_HUMIDIFY) {
                if(fil_h >= tgt) next = STATE_IDLE;
            }
            else if(sys.current_state == STATE_DEHUMIDIFY) {
                if(fil_h <= tgt) {
                    next = STATE_AFTER_COOL;
                    after_cool_start = now;
                }
            }
        }

        // Outputs
        bool pelt=0, pump=0;
        int duty_b=0, duty_c=0;

        switch(next) {
            case STATE_HUMIDIFY:
                pelt = 1;
                // Pump ON only if Warm-up finished (>= 40C)
                pump = (w_ok && w >= TEMP_WATER_WARMUP);
                break;
            case STATE_DEHUMIDIFY:
                pelt = 1; 
                duty_b = 1023; // Blower ON Max
                break;
            case STATE_AFTER_COOL:
                duty_b = 800; // Run blower to dry heatsink
                break;
            case STATE_PROTECT_ERR:
                duty_b = 0; 
                break;
            default: // IDLE
                break;
        }

        // Case Fan Logic (Always protect if hot, regardless of state)
        if (aux_cool) {
            duty_c = 1023; // > 60 C -> Turn on Case Fan
        } else {
            duty_c = 0;
        }

        // Instant Cut Logic for Blower (Anti-Soakback)
        if(sys.current_state == STATE_DEHUMIDIFY && next != STATE_DEHUMIDIFY) {
            // Force current duty to 0 immediately
            cur_duty_blower = 0; 
            tgt_duty_blower = 0;
            set_pwm(PWM_CH_BLOWER, 0); // Hardware write
        } else {
            tgt_duty_blower = duty_b;
        }
        
        tgt_duty_case = duty_c;
        
        gpio_set_level(PIN_PELTIER, pelt);
        gpio_set_level(PIN_PUMP, pump);

        xSemaphoreTake(mutex, portMAX_DELAY);
        sys.current_state = next;
        xSemaphoreGive(mutex);

        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void task_ui(void *arg) {
    lcd_init_dev();
    char buf[17];
    const char *sname[] = {"IDL", "HUM", "DEH", "AFT", "ERR"};
    
    // Button States
    int btn_mode_last=1, btn_up_last=1, btn_down_last=1;
    uint32_t last_btn_time = 0;
    int blink_sd = 0;

    while(1) {
        // Input Handling
        int bm = gpio_get_level(PIN_BTN_MODE);
        int bu = gpio_get_level(PIN_BTN_UP);
        int bd = gpio_get_level(PIN_BTN_DOWN);
        uint32_t now = xTaskGetTickCount()*portTICK_PERIOD_MS;

        if(now - last_btn_time > 200) { // Debounce
            if(!bm && btn_mode_last) {
                sys.edit_mode = !sys.edit_mode;
                sys.edit_item = 0;
                last_btn_time = now;
            }
            if(sys.edit_mode) {
                if(!bu && btn_up_last) {
                    if(sys.edit_item==0) sys.target_hum += 1.0;
                    else sys.hysteresis += 0.5;
                    last_btn_time = now;
                }
                if(!bd && btn_down_last) {
                    if(sys.edit_item==0) sys.target_hum -= 1.0;
                    else sys.hysteresis -= 0.5;
                    last_btn_time = now;
                }
            }
        }
        btn_mode_last=bm; btn_up_last=bu; btn_down_last=bd;

        // Display
        xSemaphoreTake(mutex, portMAX_DELAY);
        float t = sys.t_air, h = sys.h_air, w = sys.t_water;
        int st = sys.current_state;
        float tg = sys.target_hum;
        bool em = sys.edit_mode;
        bool sdm = sys.sd_mounted;
        xSemaphoreGive(mutex);

        if(em) {
            snprintf(buf, 17, "SETTING MODE  ");
            lcd_pos(0,0); lcd_str(buf);
            snprintf(buf, 17, "Tgt:%.0f Hys:%.1f", tg, sys.hysteresis);
            lcd_pos(1,0); lcd_str(buf);
        } else {
            snprintf(buf, 17, "T:%.1f H:%.1f %s", t, h, sname[st]);
            lcd_pos(0,0); lcd_str(buf);
            snprintf(buf, 17, "W:%.1f S:%.0f  %c", w, tg, (sdm && blink_sd)?'*':' ');
            lcd_pos(1,0); lcd_str(buf);
        }
        blink_sd = !blink_sd;
        
        vTaskDelay(500/portTICK_PERIOD_MS); // 500ms refresh for blinking
    }
}

void app_main(void) {
    mutex = xSemaphoreCreateMutex();
    init_hw();
    i2c_init_bus();
    sd_init();

    xTaskCreate(task_soft_start, "soft", 2048, NULL, 5, NULL);
    xTaskCreatePinnedToCore(task_control, "ctrl", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_ui, "ui", 4096, NULL, 3, NULL, 0);

    ESP_LOGI(TAG, "RUNNING");
}
