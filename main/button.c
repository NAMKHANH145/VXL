#include "button.h"
#include "esp_timer.h"
#include "esp_log.h"

// Configuration Analysis:
// - Buttons are Active LOW (Connect to GND when pressed).
// - Internal Pull-up is enabled.
// - Debounce Logic: 50ms hold time required to confirm a press.

#define BTN_DEBOUNCE_MS 50 

typedef struct {
    gpio_num_t pin;
    bool is_held;            // Current stable state (True = Held Down)
    int64_t last_event_time; // Timestamp of last level change
} btn_state_t;

// State tracking for the 3 system buttons
static btn_state_t buttons[] = {
    { .pin = PIN_BTN_MODE, .is_held = false, .last_event_time = 0 },
    { .pin = PIN_BTN_UP,   .is_held = false, .last_event_time = 0 },
    { .pin = PIN_BTN_DOWN, .is_held = false, .last_event_time = 0 }
};

static btn_state_t* get_btn_state(gpio_num_t pin) {
    for (int i = 0; i < sizeof(buttons)/sizeof(buttons[0]); i++) {
        if (buttons[i].pin == pin) return &buttons[i];
    }
    return NULL;
}

void button_init(gpio_num_t pin) {
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_conf);
    
    // Log only once to avoid clutter, or check specific pin
    if (pin == PIN_BTN_DOWN) {
        ESP_LOGI("BTN", "Buttons Initialized (Active Low, Pull-up)");
    }
}

bool button_is_pressed(gpio_num_t pin) {
    btn_state_t* btn = get_btn_state(pin);
    if (!btn) return false;

    int level = gpio_get_level(pin); // 0 = Pressed, 1 = Released
    int64_t now = esp_timer_get_time() / 1000; // ms
    bool should_update_time = false;

    if (level == 0) { 
        // Button is physically DOWN
        if (!btn->is_held) {
            // It was UP previously. Check if it has been stable long enough?
            // Actually, for "First Press", we usually wait for stable contact.
            
            // Here: We detect the transition.
            if (now - btn->last_event_time >= BTN_DEBOUNCE_MS) {
                // CONFIRMED PRESS (Falling Edge + Debounce)
                btn->is_held = true;       
                should_update_time = true;
                return true; // Return TRUE once per press
            }
        }
    } else {
        // Button is physically UP
        if (btn->is_held) {
            // It was DOWN previously.
             if (now - btn->last_event_time >= BTN_DEBOUNCE_MS) {
                // CONFIRMED RELEASE (Rising Edge + Debounce)
                btn->is_held = false;      
                should_update_time = true;
            }
        }
    }

    if (should_update_time) {
        btn->last_event_time = now;
    }

    return false;
}

bool button_is_down(gpio_num_t pin) {
    return (gpio_get_level(pin) == 0);
}