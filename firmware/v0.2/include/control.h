/**
 * @file
 * @brief Reads the control button state.
 */

#pragma once

#include <stdint.h>

enum control_action {
    CONTROL_ACTION_RELEASE = 0,
    CONTROL_ACTION_PRESS_UP = 1,
    CONTROL_ACTION_PRESS_DOWN = 2,
    CONTROL_ACTION_PRESS_MODE = 3,
    CONTROL_ACTION_HOLD_UP = 4,
    CONTROL_ACTION_HOLD_DOWN = 5,
    CONTROL_ACTION_HOLD_MODE = 6,
};

/**
 * @brief Gets the current action.
 */
enum control_action control_get_action_current(void);

/**
 * @brief Initialize the control driver.
 */
int control_init(void);
