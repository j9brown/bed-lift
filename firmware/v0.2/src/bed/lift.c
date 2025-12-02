#include <errno.h>

#include "bed/lift.h"

int lift_init(void) {
    return 0;
}

enum lift_position lift_get_position(void) {
    //return LIFT_POSITION_UNKNOWN;
    return LIFT_POSITION_IN_SAFE_ZONE;
}

int lift_poll_standby(void) {
    return 0;
}

int lift_poll_raise(void) {
    return 0;
}

int lift_poll_lower(void) {
    return 0;
}
