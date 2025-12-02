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
        bed_state = BED_STATE_ABORTED;
    }
    int lift_err = lift_poll_standby();
    int span_err = span_poll_sleep(); // span must sleep after lift action
    if (lift_err > 0 || span_err > 0) {
        return 1;
    }
    if (lift_err < 0 || span_err < 0) {
        bed_state = BED_STATE_ERROR;
        return -ECANCELED;
    }
    return 0;
}

int bed_poll_pose(enum bed_pose pose) {
    __ASSERT(pose != BED_POSE_UNKNOWN, "Target pose must not be unknown");

    if (bed_state != BED_STATE_MOVING || pose != bed_target_pose) {
        bed_state = BED_STATE_MOVING;
        bed_current_pose = BED_POSE_UNKNOWN;
        bed_target_pose = pose;
        bed_progress = 0;
    }

    int lift_err;
    int span_err;
    bool maybe_done = false;
    switch (pose) {
        case BED_POSE_SLEEP:
            switch (lift_get_position()) {
                case LIFT_POSITION_LOWER_LIMIT:
                    switch (span_get_position()) {
                        case SPAN_POSITION_EXTENDED:
                            // Achieved sleep pose
                            lift_err = lift_poll_standby();
                            span_err = span_poll_sleep(); // span must sleep after lift action
                            maybe_done = true;
                            break;
                        default:
                            // Raise to safe zone to extend span
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_raise();
                            break;
                    }
                    break;
                case LIFT_POSITION_BELOW_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_EXTENDED:
                            // Proceed to lower the lift
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_lower();
                            break;
                        default:
                            // Raise to safe zone to extend span
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_raise();
                            break;
                    }
                    break;
                case LIFT_POSITION_IN_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_EXTENDED:
                            // Proceed to lower the lift
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_lower();
                            break;
                        default:
                            // Extend the span before lowering
                            span_err = span_poll_extend(); // span must wake before lift action
                            lift_err = lift_poll_standby();
                            break;
                    }
                    break;
                case LIFT_POSITION_ABOVE_SAFE_ZONE:
                case LIFT_POSITION_UPPER_LIMIT:
                    // Proceed to lower the lift
                    span_err = span_poll_standby(); // span must wake before lift action
                    lift_err = lift_poll_lower();
                    break;
                default:
                    // Unknown position
                    lift_err = -ECANCELED;
                    span_err = -ECANCELED;
                    break;
            }
            break;
        case BED_POSE_LOUNGE:
            switch (lift_get_position()) {
                case LIFT_POSITION_LOWER_LIMIT:
                case LIFT_POSITION_BELOW_SAFE_ZONE:
                    // Proceed to raise the lift
                    span_err = span_poll_standby(); // span must wake before lift action
                    lift_err = lift_poll_raise();
                    break;
                case LIFT_POSITION_IN_SAFE_ZONE:
                case LIFT_POSITION_ABOVE_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Proceed to raise the lift
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_raise();
                            break;
                        default:
                            // Retract the span before raising
                            span_err = span_poll_retract(); // span must wake before lift action
                            lift_err = lift_poll_standby();
                            break;
                    }
                    break;
                case LIFT_POSITION_UPPER_LIMIT:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Achieved lounge pose
                            lift_err = lift_poll_standby();
                            span_err = span_poll_sleep(); // span must sleep after lift action
                            maybe_done = true;
                            break;
                        default:
                            // Retract the span before completion
                            span_err = span_poll_retract(); // span must wake before lift action
                            lift_err = lift_poll_standby();
                            break;
                    }
                    break;
                default:
                    // Unknown position
                    lift_err = -ECANCELED;
                    span_err = -ECANCELED;
                    break;
            }
            break;
        default:
            // Unknown pose
            lift_err = -ECANCELED;
            span_err = -ECANCELED;
            break;
    }

    if (lift_err < 0 || span_err < 0) {
        bed_state = BED_STATE_ERROR;
        return -ECANCELED;
    }

    bed_progress += 1;
    if (lift_err != 0 || span_err != 0 || !maybe_done) {
        return 1; // movement still in progress
    }

    bed_current_pose = pose;
    bed_state = BED_STATE_DONE;
    return 0;
}
