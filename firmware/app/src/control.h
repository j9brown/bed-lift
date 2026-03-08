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
    CONTROL_ACTION_HOLD_UP_THEN_PRESS_MODE = 7,
    CONTROL_ACTION_HOLD_DOWN_THEN_PRESS_MODE = 8,
    CONTROL_ACTION_LONG_HOLD_MODE = 9,
};

/**
 * @brief Get the action determined by the buttons.
 */
enum control_action control_get_action(void);

/**
 * @brief Initialize the control driver.
 */
int control_init(void);
