#include <errno.h>
#include <zephyr/sys/__assert.h>

#include "bed/bed.h"
#include "bed/lift.h"
#include "bed/span.h"

static enum bed_pose bed_current_pose;
static enum bed_pose bed_target_pose;
static enum bed_state bed_state;
static unsigned bed_progress;

int bed_init(void) {
    int err;
    if ((err = lift_init())) {
        return err;
    }
    if ((err = span_init())) {
        return err;
    }
    return 0;
}

unsigned bed_get_progress(void) {
    return bed_progress;
}

enum bed_state bed_get_state(void) {
    return bed_state;
}

enum bed_pose bed_get_current_pose(void) {
    return bed_current_pose;
}

enum bed_pose bed_get_target_pose(void) {
    return bed_target_pose;
}

int bed_poll_stop(void) {
    bed_target_pose = BED_POSE_UNKNOWN;
    if (bed_state == BED_STATE_MOVING) {
        int lift_err = lift_poll(LIFT_ACTION_STOP);
        int span_err = span_poll(SPAN_ACTION_STOP);
        if (lift_err > 0 || span_err > 0) {
            return 1;
        }
        if (lift_err == 0 && span_err == 0) {
            bed_state = BED_STATE_ABORTED;
        } else if (lift_err < 0 || span_err < 0) {
            bed_state = BED_STATE_ERROR;
        }
    }
    return bed_state == BED_STATE_ERROR ? -EIO : 0;
}

int bed_poll_pose(enum bed_pose next_pose) {
    __ASSERT(next_pose != BED_POSE_UNKNOWN, "Target pose must not be unknown");

    if (next_pose != bed_target_pose) {
        bed_state = BED_STATE_MOVING;
        bed_current_pose = BED_POSE_UNKNOWN;
        bed_target_pose = next_pose;
        bed_progress = 0;
    }
    switch (next_pose) {
        case BED_POSE_SLEEP:
            span_poll(SPAN_ACTION_EXTEND);
            break;
        case BED_POSE_LOUNGE:
            span_poll(SPAN_ACTION_RETRACT);
            break;
        default:
            break;
    }
    bed_progress += 1;

    // if finished, set current pose and state to done
    return 1;
}
