#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/atomic.h>

#include "bed/bed.h"
#include "control.h"
#include "indicator.h"

#define WATCHDOG_TIMEOUT_MS (10000)
static const struct device *watchdog_dev = DEVICE_DT_GET(DT_NODELABEL(iwdg));

static atomic_t control_action_current;

void control_action_callback(enum control_action action) {
    atomic_set(&control_action_current, action);
}

int setup(void) {
    int err;
    struct wdt_timeout_cfg watchdog_cfg = {
        .window = {
            .min = 0,
            .max = WATCHDOG_TIMEOUT_MS,
        },
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };
    if ((err = wdt_install_timeout(watchdog_dev, &watchdog_cfg))) {
        return err;
    }
    if ((err = wdt_setup(watchdog_dev, WDT_OPT_PAUSE_HALTED_BY_DBG))) {
        return err;
    }

    // TODO: Set up a blink code for reporting errors
    if ((err = control_init())) {
        status_indicator_on(INDICATOR_HUE_YELLOW);
        return err;
    }
    if ((err = bed_init())) {
        status_indicator_on(INDICATOR_HUE_CYAN);
        return err;
    }
    return 0;
}

void loop(void) {
    static bool latched_error;

    const enum control_action control_action = atomic_get(&control_action_current);
    if (control_action == CONTROL_ACTION_RELEASE) {
        latched_error = false;
    }
    if (latched_error) {
        bed_poll_stop();
    } else {
        int err = 0;
        switch (control_action) {
            case CONTROL_ACTION_PRESS_UP:
                err = bed_poll_stop();
                break;
            case CONTROL_ACTION_HOLD_UP:
                err = bed_poll_pose(BED_POSE_LOUNGE);
                break;
            case CONTROL_ACTION_PRESS_DOWN:
                err = bed_poll_stop();
                break;
            case CONTROL_ACTION_HOLD_DOWN:
                err = bed_poll_pose(BED_POSE_SLEEP);
                break;
            default:
                err = bed_poll_stop();
                break;
        }
        if (err < 0) {
            latched_error = true;
        }
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
            switch (bed_get_current_pose()) {
                case BED_POSE_LOUNGE:
                    status_indicator_on(INDICATOR_HUE_GREEN);
                    break;
                case BED_POSE_SLEEP:
                    status_indicator_on(INDICATOR_HUE_CYAN);
                    break;
                default:
                    __ASSERT(false, "");
                    break;
            }
            break;
    }
}

int main(void) {
    int err;
    if ((err = setup())) {
        return err;
    }

    while (true) {
        if ((err = wdt_feed(watchdog_dev, 0))) {
            return err;
        }
        loop();
        k_msleep(10);
    }
    return 0;
}
