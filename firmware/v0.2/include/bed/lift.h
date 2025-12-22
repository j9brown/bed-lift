/**
 * @file
 * @brief Drives the lift actuators.
 */

#pragma once

#include <stdint.h>

#include "errors.h"

int lift_init(void);

enum lift_position {
    LIFT_POSITION_UNKNOWN = 0,
    LIFT_POSITION_LOWER_LIMIT = 1,
    LIFT_POSITION_BELOW_SAFE_ZONE = 2,
    LIFT_POSITION_IN_SAFE_ZONE = 3,
    LIFT_POSITION_ABOVE_SAFE_ZONE = 4,
    LIFT_POSITION_ABOVE_CEILING = 5,
    LIFT_POSITION_UPPER_LIMIT = 6,
};

enum lift_position lift_get_position(void);

enum lift_state {
    LIFT_STATE_HALT = 0,
    LIFT_STATE_RAISE = 1,
    LIFT_STATE_LOWER = 2,
    LIFT_STATE_DONE = 3,
    LIFT_STATE_ERROR = 4,
};

enum lift_state lift_get_state(void);

/*
 * The poll functions perform an action incrementally while keeping the actuators synchronized.
 * 
 * @retval 0 If the action is done.
 * @retval 1 If the action is still in progress.
 * @retval LIFT_* A specific error from the lift component.
 * @return -errno If a generic error occurred.
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

/** @brief The type of lift movement to perform. */
enum lift_move {
    LIFT_MOVE_TANDEM = 0,
    LIFT_MOVE_INDEPENDENT_BOTH = 1,
    LIFT_MOVE_INDEPENDENT_1 = 2,
    LIFT_MOVE_INDEPENDENT_2 = 3,
};

/**
 * @brief Jog specific actuators in a particular direction under manual control.
 * 
 * @param move The type of movement to perform.
 * @param raise Whether to raise the actuators or lower them.
 */
int lift_poll_jog(enum lift_move move, bool raise);
