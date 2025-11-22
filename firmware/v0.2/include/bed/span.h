/**
 * @file
 * @brief Drives the span actuators.
 */

#pragma once

#include <stdint.h>

int span_init(void);

enum span_action {
    /** @brief Stop the span actuators. */
    SPAN_ACTION_STOP = 0,
    /** @brief Extend all spans until all of the motors stall at the assumed limit position. */
    SPAN_ACTION_EXTEND = 1,
    /** @brief Retract all spans until all of the motors stall at the assumed limit position. */
    SPAN_ACTION_RETRACT = 2,
};

/**
 * @brief Perform an action incrementally while keeping the motors synchronized.
 * 
 * @retval 0 If the action is done.
 * @retval 1 If the action is still in progress.
 * @return -errno If an error occurred.
 */
int span_poll(enum span_action action);
