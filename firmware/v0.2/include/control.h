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
    CONTROL_ACTION_HOLD_UP = 3,
    CONTROL_ACTION_HOLD_DOWN = 4,
};

/**
 * @brief The callback to invoke when the control action changes.
 * Runs on the system work queue.
 */
extern void control_action_callback(enum control_action action);

/**
 * @brief Initialize the control driver.
 */
int control_init(void);
