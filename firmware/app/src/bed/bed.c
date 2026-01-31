#include <errno.h>
#include <zephyr/drivers/gpio.h>
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
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
static bool bed_was_extending_span_while_lowering_in_safe_zone;
static bool bed_finish_extending_span_before_lowering_in_safe_zone;
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
    int lift_err = lift_poll_sleep();
    int span_err = span_poll_sleep();
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
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
        bed_was_extending_span_while_lowering_in_safe_zone = false;
        bed_finish_extending_span_before_lowering_in_safe_zone = false;
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
                            lift_err = lift_poll_sleep();
                            span_err = span_poll_sleep();
                            maybe_done = true;
                            break;
                        default:
                            // Raise to safe zone to extend span
                            span_err = span_poll_sleep();
                            lift_err = lift_poll_raise();
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            bed_finish_extending_span_before_lowering_in_safe_zone = true;
#endif
                            break;
                    }
                    break;
                case LIFT_POSITION_BELOW_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_EXTENDED:
                            // Proceed to lower the lift
                            span_err = span_poll_sleep();
                            lift_err = lift_poll_lower();
                            break;
                        default:
                            // Raise to safe zone to extend span
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            if (bed_was_extending_span_while_lowering_in_safe_zone) {
                                lift_err = lift_poll_sleep();
                                span_err = span_poll_extend();
                            } else {
                                bed_finish_extending_span_before_lowering_in_safe_zone = true;
                                span_err = span_poll_sleep();
                                lift_err = lift_poll_raise();
                            }
#else
                            span_err = span_poll_sleep();
                            lift_err = lift_poll_raise();
#endif
                            break;
                    }
                    break;
                case LIFT_POSITION_IN_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_EXTENDED:
                            // Proceed to lower the lift
                            span_err = span_poll_sleep();
                            lift_err = lift_poll_lower();
                            break;
                        default:
                            // Extend the span before lowering or move both together
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            if (bed_finish_extending_span_before_lowering_in_safe_zone) {
                                // Handle edge case where the lift approached the safe zone from below
                                // so the span must completely extend before lowering again.
                                lift_err = lift_poll_sleep();
                            } else {
                                lift_err = lift_poll_lower();
                                bed_was_extending_span_while_lowering_in_safe_zone = true;
                            }
#else
                            lift_err = lift_poll_sleep();
#endif
                            span_err = span_poll_extend();
                            break;
                    }
                    break;
                case LIFT_POSITION_ABOVE_SAFE_ZONE:
                    // Proceed to lower the lift
                    span_err = span_poll_sleep();
                    lift_err = lift_poll_lower();
                    break;
                case LIFT_POSITION_ABOVE_CEILING:
                case LIFT_POSITION_UPPER_LIMIT:
                    // Proceed to lower the lift
                    span_err = span_poll_sleep();
                    lift_err = lift_poll_lower();
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
                    span_err = span_poll_sleep();
                    lift_err = lift_poll_raise();
                    break;
                case LIFT_POSITION_BELOW_SAFE_ZONE:
                    // Proceed to raise the lift
                    span_err = span_poll_sleep();
                    lift_err = lift_poll_raise();
                    break;
                case LIFT_POSITION_IN_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Proceed to raise the lift
                            span_err = span_poll_sleep();
                            lift_err = lift_poll_raise();
                            break;
                        default:
                            // Retract the span before raising or move both together
                            span_err = span_poll_retract();
#if BED_MOVE_SPAN_AND_LIFT_TOGETHER_IN_SAFE_ZONE
                            lift_err = lift_poll_raise();
#else
                            lift_err = lift_poll_sleep();
#endif
                            break;
                    }
                    break;
                case LIFT_POSITION_ABOVE_SAFE_ZONE:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Proceed to raise the lift
                            span_err = span_poll_sleep();
                            lift_err = lift_poll_raise();
                            break;
                        default:
                            // Retract the span before raising
                            lift_err = lift_poll_sleep();
                            span_err = span_poll_retract();
                            break;
                    }
                    break;
                case LIFT_POSITION_ABOVE_CEILING:
                case LIFT_POSITION_UPPER_LIMIT:
                    switch (span_get_position()) {
                        case SPAN_POSITION_RETRACTED:
                            // Achieved lounge pose
                            lift_err = lift_poll_sleep();
                            span_err = span_poll_sleep();
                            maybe_done = true;
                            break;
                        default:
                            // Retract the span before completion
                            lift_err = lift_poll_sleep();
                            span_err = span_poll_retract();
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
    return 0;
}

void bed_prepare_for_manual_action(void) {
    bed_current_pose = BED_POSE_UNKNOWN;
    bed_target_pose = BED_POSE_UNKNOWN;
    bed_state = BED_STATE_ABORTED;
}
