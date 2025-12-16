/**
 * @file
 * @brief Drives the span actuators.
 */

#pragma once

#include <stdint.h>

#include "errors.h"

int span_init(void);

enum span_position {
    SPAN_POSITION_UNKNOWN = 0,
    SPAN_POSITION_EXTENDED = 1,
    SPAN_POSITION_RETRACTED = 2,
};

enum span_position span_get_position(void);

enum span_state {
    SPAN_STATE_HALT = 0,
    SPAN_STATE_EXTEND = 1,
    SPAN_STATE_RETRACT = 2,
    SPAN_STATE_DONE = 3,
    SPAN_STATE_ERROR = 4,
};

enum span_state span_get_state(void);

/*
 * The poll functions perform an action incrementally while keeping the actuators synchronized.
 * 
 * @retval 0 If the action is done.
 * @retval 1 If the action is still in progress.
 * @retval SPAN_* A specific error from the span component.
 * @return -errno If a generic error occurred.
 */

/**
 * @brief Stop the span actuators and put the motor drivers to sleep.
 */
int span_poll_sleep(void);

/**
 * @brief Wake the motor drivers from sleep and keep the outputs disabled.
 *
 * Because the lift and span drivers share the same sleep signal, this state can be used to keep
 * the span drivers on standby while the lift moves.
 */
int span_poll_standby(void);

/**
 * @brief Extend all spans until all of the actuators stall at the endstop.
 */
int span_poll_extend(void);

/**
 * @brief Retract all spans until all of the actuators stall at the endstop.
 */
int span_poll_retract(void);

/**
 * @brief Jog specific actuators in a particular direction under manual control.
 * 
 * @param extend Whether to extend the actuators or retract them.
 * @param actuator_set Combine BIT(0), BIT(1), BIT(2), or BIT(3) to address specific actuators.
 */
int span_poll_jog(bool extend, unsigned actuator_set);
