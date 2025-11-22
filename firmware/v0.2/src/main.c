#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "bed/bed.h"
#include "control.h"
#include "indicator.h"

static atomic_t control_action_current;

void control_action_callback(enum control_action action) {
    atomic_set(&control_action_current, action);
}

void loop(void) {
    const enum control_action control_action = atomic_get(&control_action_current);
    switch (control_action) {
        case CONTROL_ACTION_PRESS_UP:
            bed_poll_stop();
            break;
        case CONTROL_ACTION_HOLD_UP:
            bed_poll_pose(BED_POSE_LOUNGE);
            break;
        case CONTROL_ACTION_PRESS_DOWN:
            bed_poll_stop();
            break;
        case CONTROL_ACTION_HOLD_DOWN:
            bed_poll_pose(BED_POSE_SLEEP);
            break;
        default:
            bed_poll_stop();
            break;
    }

    // TODO: turn the indicators off shortly after the controls have been released
    switch (bed_get_state()) {
        case BED_STATE_ABORTED:
            status_indicator_on(INDICATOR_HUE_YELLOW);
            break;
        case BED_STATE_MOVING:
            switch (bed_get_target_pose()) {
                case BED_POSE_LOUNGE:
                    status_indicator_on(INDICATOR_HUE_MAGENTA + bed_get_progress() * 10);
                    break;
                case BED_POSE_SLEEP:
                    status_indicator_on(INDICATOR_HUE_BLUE - bed_get_progress() * 10);
                    break;
                default:
                    __ASSERT(false, "");
                    break;
            }
            break;
        case BED_STATE_ERROR:
            status_indicator_on(INDICATOR_HUE_RED);
            break;
        case BED_STATE_DONE:
            status_indicator_on(INDICATOR_HUE_GREEN);
            break;
    }
}

int main(void) {
    // TODO: Set up a blink code for reporting errors
    int err;
    if ((err = control_init())) {
        status_indicator_on(INDICATOR_HUE_YELLOW);
        return err;
    }
    if ((err = bed_init())) {
        status_indicator_on(INDICATOR_HUE_CYAN);
        return err;
    }

    while (true) {
        loop();
        k_msleep(1);
    }
    return 0;
}
