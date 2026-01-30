/**
 * @file
 * @brief Implements the external monitoring interface.
 */

#pragma once

#include <stdint.h>

/**
 * @brief Initialize monitoring.
 */
int monitor_init(void);

struct __packed monitor_status {
    unsigned bed_state : 2;
    unsigned bed_pose_current : 2;
    unsigned bed_pose_target : 2;
    unsigned lift_position : 3;
    unsigned lift_state : 3;
    unsigned span_position : 2;
    unsigned span_state : 3;
    unsigned : 4;
    bool control_inhibited : 1;
    bool control_active : 1;
    bool control_error : 1;
    unsigned last_error : 8;
};
_Static_assert(sizeof(struct monitor_status) == 4, "");

struct __packed monitor_setting {
    bool control_inhibit : 1;
    unsigned : 31;
};
_Static_assert(sizeof(struct monitor_setting) == 4, "");

void monitor_set_status(struct monitor_status value);

struct monitor_setting monitor_get_setting(void);
