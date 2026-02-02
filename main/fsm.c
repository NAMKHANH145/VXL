#include "fsm.h"
#include "pwm.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FSM";

// Internal Timers & Flags
static uint32_t s_dehum_start_tick = 0;
static uint32_t s_humidify_start_tick = 0;

// Timeouts
#define MAX_RUNTIME_MS (2 * 60 * 60 * 1000) // 2 Hours

void fsm_init(void) {
    s_dehum_start_tick = 0;
    s_humidify_start_tick = 0;
    ESP_LOGI(TAG, "FSM Initialized (No Defrost Logic, No Test Mode)");
}

void fsm_run_cycle(const SensorData_t *sensors, 
                   const SystemSetpoints_t *setpoints, 
                   SystemState_t *state,
                   bool emergency_reset_cmd) 
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    SystemState_t current_state = *state;

    // ====================================================
    // 1. GLOBAL SAFETY OVERRIDE
    // ====================================================
    if (current_state != STATE_EMERGENCY) {
        bool critical_error = false;
        
        // Check Sensor Faults
        if (sensors->error_count > SENSOR_ERROR_THRESHOLD) critical_error = true;

        // Check Water Overheat (> 70C)
        if (sensors->water_ok && sensors->t_water > TEMP_WATER_EMERGENCY) critical_error = true;

        if (critical_error) {
            current_state = STATE_EMERGENCY;
            ESP_LOGE(TAG, "CRITICAL FAULT -> EMERGENCY (err=%lu, wat=%.1f)", 
                     sensors->error_count, sensors->t_water);
        }
    }

    // ====================================================
    // 2. STATE TRANSITIONS
    // ====================================================
    if (current_state == STATE_EMERGENCY) {
        if (emergency_reset_cmd) {
            if (sensors->water_ok && sensors->t_water < 55.0f) {
                current_state = STATE_IDLE;
                ESP_LOGW(TAG, "EMERGENCY RESET BY USER");
            }
        }
    }
    else {
        // --- NORMAL OPERATION LOGIC ---
        if (current_state == STATE_IDLE) {
            s_humidify_start_tick = 0;
            s_dehum_start_tick = 0;

            if (sensors->air_ok) {
                // Hysteresis Entry: Only start if diff > 2%
                if (sensors->h_air < setpoints->target_hum - HYSTERESIS_HUM) {
                    current_state = STATE_HUMIDIFY;
                    s_humidify_start_tick = now;
                }
                else if (sensors->h_air > setpoints->target_hum + HYSTERESIS_HUM) {
                    current_state = STATE_DEHUMIDIFY;
                    s_dehum_start_tick = now;
                }
            }
        } 
        else if (current_state == STATE_HUMIDIFY) {
            // Exit: Run UNTIL target reached
            if (sensors->h_air >= setpoints->target_hum) {
                current_state = STATE_IDLE;
            }
            else if (now - s_humidify_start_tick > MAX_RUNTIME_MS) {
                ESP_LOGE(TAG, "TIMEOUT: Humidify > 2H");
                current_state = STATE_IDLE; 
            }
        } 
        else if (current_state == STATE_DEHUMIDIFY) {
            // Exit: Run UNTIL target reached
            if (sensors->h_air <= setpoints->target_hum) {
                current_state = STATE_IDLE;
            }
            else if (now - s_dehum_start_tick > MAX_RUNTIME_MS) {
                 ESP_LOGE(TAG, "TIMEOUT: Dehumidify > 2H");
                 current_state = STATE_IDLE;
            }
        }
    }

    // Update Output State
    *state = current_state;

    // ====================================================
    // 3. ACTUATOR CONTROL (OUTPUTS)
    // ====================================================
    // 0-1023 Scale:
    // 20% = 205 | 40% = 410 | 50% = 512 | 70% = 716 | 80% = 819 | 100% = 1023
    
    int peltier_duty = 0;
    int blower_duty = 0;
    int case_fan_duty = 0;
    int circ_fan_duty = 0;
    bool pump_on = false;

    switch (current_state) {
        case STATE_IDLE:
            circ_fan_duty = 716; // 50%
            
            // Case Fan Logic for Idle
            // "Khi nước mát 40%, nóng 100%"
            // Dùng ngưỡng 45 độ C làm mốc nóng
            if (sensors->water_ok && sensors->t_water > 45.0f) {
                case_fan_duty = 1023; // 100%
            } else {
                case_fan_duty = 716; // 70%
            }
            break;

        case STATE_HUMIDIFY:
            pump_on = true;
            circ_fan_duty = 512; // 50%
            blower_duty = 0;
            
            if (sensors->water_ok) {
                // Logic moi: Om nhiet toi da
                if (sensors->t_water < 60.0f) {
                    // PHASE 1: BOOST (< 60C)
                    peltier_duty = 1023; // 100%
                    case_fan_duty = 0;   // FAN OFF -> Max Heat
                } else {
                    // PHASE 2: MAINTAIN (>= 60C)
                    peltier_duty = 716;  // ~70% (Giam cong suat Peltier)
                    case_fan_duty = 205; // 20% (Thoi nhe de tranh qua nhiet cuc bo)
                }
            } else {
                peltier_duty = 0; // Safety
                case_fan_duty = 716; // 70%
            }
            break;

        case STATE_DEHUMIDIFY:
            pump_on = false;
            circ_fan_duty = 819; // 80%
            
            // "Peltier 70%, Blower 100%, Case Auto 100%"
            peltier_duty = 716;  // 70%
            blower_duty = 1023;  // 100%
            case_fan_duty = 1023;// 100%
            break;

        case STATE_EMERGENCY:
            peltier_duty = 0;
            pump_on = false;
            case_fan_duty = 1023; // COOL DOWN
            blower_duty = 1023;
            circ_fan_duty = 0;
            break;
    }

    // Apply Outputs
    pwm_set_peltier(peltier_duty, 1000); 
    pwm_set_blower(blower_duty, 500);
    pwm_set_case(case_fan_duty, 500);
    pwm_set_circ(circ_fan_duty, 500);
    mosfet_set_pump(pump_on);

    // --- LOG ---
    static int fsm_log_div = 0;
    if (++fsm_log_div >= 20) { 
        ESP_LOGI(TAG, "CTRL | St:%d | Pel:%4d | Blo:%4d | Cas:%4d | Cir:%4d | Pmp:%d", 
                 current_state, peltier_duty, blower_duty, case_fan_duty, circ_fan_duty, pump_on);
        fsm_log_div = 0;
    }
}