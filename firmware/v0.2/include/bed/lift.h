/**
 * @file
 * @brief Drives the lift actuators.
 */

#pragma once

#include <stdint.h>

enum lift_position {
    LIFT_POSITION_UNKNOWN = 0,
    LIFT_POSITION_LOWER_LIMIT = 1,
    LIFT_POSITION_BELOW_SAFE_ZONE = 2,
    LIFT_POSITION_IN_SAFE_ZONE = 3,
    LIFT_POSITION_ABOVE_SAFE_ZONE = 4,
    LIFT_POSITION_UPPER_LIMIT = 5,
};

enum lift_action {
    /** @brief Stop the lift. */
    LIFT_ACTION_STOP = 0,
    /** @brief Raise the lift until the upper limit position is achieved. */
    LIFT_ACTION_RAISE = 1,
    /** @brief Lower the lift until the lower limit position is achieved. */
    LIFT_ACTION_LOWER = 2,
};

int lift_init(void);

/**
 * @brief Perform an action incrementally while keeping the motors synchronized.
 * 
 * @retval 0 If the action is done.
 * @retval 1 If the action is still in progress.
 * @return -errno If an error occurred.
 */
int lift_poll(enum lift_action action);

enum lift_position lift_get_position(void);
