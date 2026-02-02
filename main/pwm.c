#include "pwm.h"
#include "esp_log.h"
#include "config.h"

static void config_pwm_channel(ledc_channel_t ch, int pin) {
    ledc_channel_config_t cfg = {
        .gpio_num   = pin,
        .speed_mode = LEDC_MODE,
        .channel    = ch,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&cfg);
}

void mosfet_init(void) {
    // 1. Configure Pump (GPIO Switch)
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << PIN_PUMP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);
    gpio_set_level(PIN_PUMP, 0); // Default OFF

    // 2. Configure LEDC Timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .timer_num  = LEDC_TIMER,
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // 3. Configure Channels
    config_pwm_channel(PWM_CH_BLOWER, PIN_FAN_BLOWER);
    config_pwm_channel(PWM_CH_CASE,   PIN_FAN_CASE);
    config_pwm_channel(PWM_CH_PELTIER, PIN_PELTIER);
    config_pwm_channel(PWM_CH_CIRC,   PIN_FAN_CIRC);

    // 4. Install Fade Service
    ledc_fade_func_install(0);
    
    ESP_LOGI("PWM", "Init Complete (Freq: %d Hz)", PWM_FREQ);
}

void mosfet_set_pump(bool on) {
    gpio_set_level(PIN_PUMP, on ? 1 : 0);
}

// Helper for Fade vs Instant Set
static void set_pwm_duty(ledc_channel_t ch, uint32_t duty, int fade_ms) {
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    
    if (fade_ms > 0) {
        esp_err_t err = ledc_set_fade_with_time(LEDC_MODE, ch, duty, fade_ms);
        if (err == ESP_OK) {
            ledc_fade_start(LEDC_MODE, ch, LEDC_FADE_NO_WAIT);
        } else {
             // Fallback to instant set if fade fails (e.g. fade not installed or busy)
             ledc_set_duty(LEDC_MODE, ch, duty);
             ledc_update_duty(LEDC_MODE, ch);
        }
    } else {
        ledc_set_duty(LEDC_MODE, ch, duty);
        ledc_update_duty(LEDC_MODE, ch);
    }
}

void pwm_set_peltier(uint32_t duty, int fade_ms) { set_pwm_duty(PWM_CH_PELTIER, duty, fade_ms); }
void pwm_set_blower(uint32_t duty, int fade_ms) { set_pwm_duty(PWM_CH_BLOWER, duty, fade_ms); }
void pwm_set_case(uint32_t duty, int fade_ms) { set_pwm_duty(PWM_CH_CASE, duty, fade_ms); }
void pwm_set_circ(uint32_t duty, int fade_ms) { set_pwm_duty(PWM_CH_CIRC, duty, fade_ms); }

void pwm_stop_all(void) {
    // Immediate cut off
    mosfet_set_pump(false);
    
    // Set all duties to 0 instantly (no fade)
    ledc_set_duty(LEDC_MODE, PWM_CH_PELTIER, 0);
    ledc_update_duty(LEDC_MODE, PWM_CH_PELTIER);

    ledc_set_duty(LEDC_MODE, PWM_CH_BLOWER, 0);
    ledc_update_duty(LEDC_MODE, PWM_CH_BLOWER);

    ledc_set_duty(LEDC_MODE, PWM_CH_CASE, 0);
    ledc_update_duty(LEDC_MODE, PWM_CH_CASE);

    ledc_set_duty(LEDC_MODE, PWM_CH_CIRC, 0);
    ledc_update_duty(LEDC_MODE, PWM_CH_CIRC);
    
    ESP_LOGW("PWM", "ALL OUTPUTS STOPPED (EMI/SAFETY)");
}