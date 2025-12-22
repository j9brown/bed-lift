#include <errno.h>
#include <zephyr/sys/__assert.h>

#include "bed/bed.h"
#include "bed/lift.h"
#include "bed/span.h"

// If true, allows the span and lift to move together while in the
// safe zone, otherwise the span movement is required to complete first.
#define BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE 1

static enum bed_pose bed_current_pose;
static enum bed_pose bed_target_pose;
static enum bed_state bed_state;
static unsigned bed_progress;
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
static bool bed_wait_for_span_before_lowering_in_safe_zone;
#endif

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
    if (lift_err < 0) {
        bed_state = BED_STATE_ERROR;
        return lift_err;
    }
    if (span_err < 0) {
        bed_state = BED_STATE_ERROR;
        return span_err;
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
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
        bed_wait_for_span_before_lowering_in_safe_zone = false;
#endif
    }

    int lift_err, span_err;
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
                            bed_progress = 100;
                            maybe_done = true;
                            break;
                        default:
                            // Raise to safe zone to extend span
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_raise();
                            bed_progress = 0;
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            bed_wait_for_span_before_lowering_in_safe_zone = true;
#endif
                            break;
                    }
                    break;
                case LIFT_POSITION_BELOW_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_EXTENDED:
                            // Proceed to lower the lift
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_lower();
                            bed_progress = 80;
                            break;
                        default:
                            // Raise to safe zone to extend span
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_raise();
                            bed_progress = 20;
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            bed_wait_for_span_before_lowering_in_safe_zone = true;
#endif
                            break;
                    }
                    break;
                case LIFT_POSITION_IN_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_EXTENDED:
                            // Proceed to lower the lift
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_lower();
                            bed_progress = 60;
                            break;
                        default:
                            // Extend the span before lowering or move both together
                            span_err = span_poll_extend(); // span must wake before lift action
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            if (bed_wait_for_span_before_lowering_in_safe_zone) {
                                // Handle edge case where the lift approached the safe zone from below
                                // so the span must completely extend before lowering again.
                                lift_err = lift_poll_standby();
                            } else {
                                lift_err = lift_poll_lower();
                            }
#else
                            lift_err = lift_poll_standby();
#endif
                            bed_progress = 40;
                            break;
                    }
                    break;
                case LIFT_POSITION_ABOVE_SAFE_ZONE:
                    // Proceed to lower the lift
                    span_err = span_poll_standby(); // span must wake before lift action
                    lift_err = lift_poll_lower();
                    bed_progress = 20;
                    break;
                case LIFT_POSITION_ABOVE_CEILING:
                case LIFT_POSITION_UPPER_LIMIT:
                    // Proceed to lower the lift
                    span_err = span_poll_standby(); // span must wake before lift action
                    lift_err = lift_poll_lower();
                    bed_progress = 0;
                    break;
                default:
                    // Unknown position
                    lift_err = BED_ERROR_BAD_STATE;
                    span_err = BED_ERROR_BAD_STATE;
                    break;
            }
            break;
        case BED_POSE_LOUNGE:
            switch (lift_get_position()) {
                case LIFT_POSITION_LOWER_LIMIT:
                    // Proceed to raise the lift
                    span_err = span_poll_standby(); // span must wake before lift action
                    lift_err = lift_poll_raise();
                    bed_progress = 0;
                    break;
                case LIFT_POSITION_BELOW_SAFE_ZONE:
                    // Proceed to raise the lift
                    span_err = span_poll_standby(); // span must wake before lift action
                    lift_err = lift_poll_raise();
                    bed_progress = 20;
                    break;
                case LIFT_POSITION_IN_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Proceed to raise the lift
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_raise();
                            bed_progress = 60;
                            break;
                        default:
                            // Retract the span before raising or move both together
                            span_err = span_poll_retract(); // span must wake before lift action
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            lift_err = lift_poll_raise();
#else
                            lift_err = lift_poll_standby();
#endif
                            bed_progress = 40;
                            break;
                    }
                    break;
                case LIFT_POSITION_ABOVE_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Proceed to raise the lift
                            span_err = span_poll_standby(); // span must wake before lift action
                            lift_err = lift_poll_raise();
                            bed_progress = 80;
                            break;
                        default:
                            // Retract the span before raising
                            span_err = span_poll_retract(); // span must wake before lift action
                            lift_err = lift_poll_standby();
                            bed_progress = 60;
                            break;
                    }
                    break;
                case LIFT_POSITION_ABOVE_CEILING:
                case LIFT_POSITION_UPPER_LIMIT:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Achieved lounge pose
                            lift_err = lift_poll_standby();
                            span_err = span_poll_sleep(); // span must sleep after lift action
                            bed_progress = 100;
                            maybe_done = true;
                            break;
                        default:
                            // Retract the span before completion
                            span_err = span_poll_retract(); // span must wake before lift action
                            lift_err = lift_poll_standby();
                            bed_progress = 80;
                            break;
                    }
                    break;
                default:
                    // Unknown position
                    lift_err = BED_ERROR_BAD_STATE;
                    span_err = BED_ERROR_BAD_STATE;
                    break;
            }
            break;
        default:
            // Unknown pose
            lift_err = BED_ERROR_BAD_STATE;
            span_err = BED_ERROR_BAD_STATE;
            break;
    }

    if (lift_err < 0) {
        bed_state = BED_STATE_ERROR;
        return lift_err;
    }
    if (span_err < 0) {
        bed_state = BED_STATE_ERROR;
        return span_err;
    }

    if (lift_err != 0 || span_err != 0 || !maybe_done) {
        return 1; // movement still in progress
    }

    bed_current_pose = pose;
    bed_state = BED_STATE_DONE;
    bed_progress = 100;
    return 0;
}
