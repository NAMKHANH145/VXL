#ifndef FSM_H
#define FSM_H

#include "config.h"

/**
 * @brief Initialize FSM internal variables (timers, default states).
 */
void fsm_init(void);

/**
 * @brief Run one cycle of the Control Logic (Finite State Machine).
 * This function is NON-BLOCKING and should be called every 100ms.
 * 
 * @param sensors Pointer to VALIDATED sensor data (Input).
 * @param setpoints Pointer to current system setpoints (Input).
 * @param state Pointer to the current System State (Input/Output - Reference).
 * @param emergency_reset_cmd Boolean flag to force exit Emergency state (Input).
 */
void fsm_run_cycle(const SensorData_t *sensors, 
                   const SystemSetpoints_t *setpoints, 
                   SystemState_t *state,
                   bool emergency_reset_cmd);

#endif // FSM_H
