#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "config.h" // Import Pin Definitions from System Config

/**
 * @brief Initialize a specific button pin.
 * Configures as Input with Pull-up.
 * @param pin GPIO number
 */
void button_init(gpio_num_t pin);

/**
 * @brief Check if a button is pressed with software debouncing.
 * 
 * @param pin GPIO number to check
 * @return true if button is pressed (and debounced), false otherwise.
 */
bool button_is_pressed(gpio_num_t pin);

/**
 * @brief Check if a button is physically held down (ignoring edge/debounce logic).
 * Useful for detecting long simultaneous presses.
 * @param pin GPIO number
 * @return true if button is currently low (active).
 */
bool button_is_down(gpio_num_t pin);

#endif // BUTTON_H