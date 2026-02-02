#ifndef PWM_H
#define PWM_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "driver/ledc.h"

// Note: PWM Configuration Constants (PWM_FREQ, PWM_RES, PWM_MAX_DUTY) 
// are now defined in config.h

/**
 * @brief Initialize the MOSFET Control System.
 * Configures:
 * - Pump: GPIO Output (ON/OFF)
 * - Peltier, Blower, Case, Circ Fans: PWM Channels
 */
void mosfet_init(void);

/**
 * @brief Set Pump state (ON/OFF).
 * @param on True to turn pump ON, False for OFF.
 */
void mosfet_set_pump(bool on);

// --- PWM SETTERS (Duty Cycle: 0-1023) ---

/**
 * @brief Control Peltier Module Power.
 * @param duty Duty cycle (0-1023).
 * @param fade_ms Fade time in milliseconds (0 for instant).
 */
void pwm_set_peltier(uint32_t duty, int fade_ms);

/**
 * @brief Control Blower Fan (Cold Side).
 * @param duty Duty cycle (0-1023).
 * @param fade_ms Fade time in milliseconds.
 */
void pwm_set_blower(uint32_t duty, int fade_ms);

/**
 * @brief Control Case Fan (Water/Hot Side).
 * @param duty Duty cycle (0-1023).
 * @param fade_ms Fade time in milliseconds.
 */
void pwm_set_case(uint32_t duty, int fade_ms);

/**
 * @brief Control Circulation Fan (Chamber Air).
 * @param duty Duty cycle (0-1023).
 * @param fade_ms Fade time in milliseconds.
 */
void pwm_set_circ(uint32_t duty, int fade_ms);

/**
 * @brief STOP ALL outputs instantly.
 * Used for Safety, Emergency Stop, or Setup Mode entry.
 */
void pwm_stop_all(void);

#endif // PWM_H