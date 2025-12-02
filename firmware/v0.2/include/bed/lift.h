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

int lift_init(void);
enum lift_position lift_get_position(void);

/*
 * The poll functions perform an action incrementally while keeping the actuators synchronized.
 * 
 * @retval 0 If the action is done.
 * @retval 1 If the action is still in progress.
 * @return -errno If an error occurred.
 */

/**
 * @brief Maintain current position.
 */
int lift_poll_standby(void);

/**
 * @brief Raise the lift until the upper limit position is achieved.
 */
int lift_poll_raise(void);

/**
 * @brief Lower the lift until the lower limit position is achieved.
 */
int lift_poll_lower(void);
