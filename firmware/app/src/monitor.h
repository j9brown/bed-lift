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

struct __packed monitor_lift_debug {
    int lift1_duty : 12; // q11 fraction
    int lift2_duty : 12; // q11 fraction
    unsigned limit_state : 2;
    unsigned : 6;
};
_Static_assert(sizeof(struct monitor_lift_debug) == 4, "");

struct __packed monitor_span_debug {
    unsigned speed : 24; // microsteps per second
    bool extend : 1;
    unsigned limit_state : 4;
    unsigned : 3;
};
_Static_assert(sizeof(struct monitor_span_debug) == 4, "");

struct __packed monitor_span_test {
    unsigned speed_mm_s : 8;
    unsigned accel_mm_s2 : 8;
    unsigned stall_threshold : 12; // if non-zero, override the default
    bool test : 1; // if true, enable test mode
    unsigned : 3;
};
_Static_assert(sizeof(struct monitor_span_test) == 4, "");

void monitor_set_status(struct monitor_status value);
void monitor_set_lift_debug(struct monitor_lift_debug value);
void monitor_set_span_debug(struct monitor_span_debug value);

struct monitor_setting monitor_get_setting(void);
struct monitor_span_test monitor_get_span_test(void);
