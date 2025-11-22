#include <errno.h>

#include "bed/lift.h"

int lift_init(void) {
    return 0;
}

int lift_poll(enum lift_action action) {
    if (action == LIFT_ACTION_STOP) {
        return 0;
    }
    return -ENOTSUP;
}

enum lift_position lift_get_position(void) {
    //return LIFT_POSITION_UNKNOWN;
    return LIFT_POSITION_IN_SAFE_ZONE;
}
