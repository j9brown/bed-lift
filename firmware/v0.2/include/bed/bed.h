/**
 * @file
 * @brief Drives the complete bed assembly including the lift and span actuators.
 */

#pragma once

#include <stdint.h>

enum bed_pose {
    BED_POSE_UNKNOWN = 0,
    BED_POSE_SLEEP = 1,
    BED_POSE_LOUNGE = 2,
};

enum bed_state {
    /** Bed is not moving and has not achieved a target pose. */
    BED_STATE_ABORTED = 0,
    /** Bed is moving towards the target pose. */
    BED_STATE_MOVING = 1,
    /** Bed has achieve the target pose. */
    BED_STATE_DONE = 2,
    /** Bed failed to achieve the target pose. */
    BED_STATE_ERROR = 3,
};

int bed_init(void);

unsigned bed_get_progress(void);
enum bed_state bed_get_state(void);
enum bed_pose bed_get_current_pose(void);
enum bed_pose bed_get_target_pose(void);

/**
 * @brief Perform an action incrementally while keeping the motors synchronized.
 * 
 * @retval 0 If the action is done.
 * @retval 1 If the action is still in progress.
 * @return -errno If an error occurred.
 */
int bed_poll_stop(void);
int bed_poll_pose(enum bed_pose next_pose);
