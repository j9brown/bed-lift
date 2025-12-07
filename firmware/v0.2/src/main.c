#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/sys/atomic.h>

#include "bed/bed.h"
#include "bed/lift.h"
#include "bed/span.h"
#include "control.h"
#include "indicator.h"

#define WATCHDOG_TIMEOUT_MS (10000)
static const struct device *watchdog_dev = DEVICE_DT_GET(DT_NODELABEL(iwdg));

// Timeout for the indicator to go dark after completing an activity.
#define ACTIVITY_TIMEOUT_MS (1500)

static atomic_t control_action_current;

void control_action_callback(enum control_action action) {
    atomic_set(&control_action_current, action);
}

enum menu {
    MENU_MAIN = 0,
    MENU_JOG = 1,
};
enum menu menu;

INDICATOR_PATTERN_LOOP(indicator_pattern_error,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_RED, 2, 2));

INDICATOR_PATTERN_LOOP(indicator_pattern_aborted,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 2, 2));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s0000_l11,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_GREEN, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s0000_l10,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_GREEN, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_GREEN, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s0000_l01,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_GREEN, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_GREEN, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_GREEN, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s1111_l00,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s1100_l00,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s0011_l00,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s1000_l00,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s0100_l00,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s0010_l00,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 6));

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_s0001_l00,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 3, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 2),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 6));

struct jog_spec {
    unsigned span;
    unsigned lift;
    const indicator_pattern_t *indicator_pattern;
};
static const struct jog_spec jog_specs[] = {
    { .span = 0, .lift = BIT(0) | BIT(1), .indicator_pattern = indicator_pattern_jog_s0000_l11 },
    { .span = 0, .lift = BIT(0), .indicator_pattern = indicator_pattern_jog_s0000_l10 },
    { .span = 0, .lift = BIT(1), .indicator_pattern = indicator_pattern_jog_s0000_l01},
    { .span = BIT(0) | BIT(1) | BIT(2) | BIT(3), .lift = 0, .indicator_pattern = indicator_pattern_jog_s1111_l00 },
    { .span = BIT(0) | BIT(1), .lift = 0, .indicator_pattern = indicator_pattern_jog_s1100_l00 },
    { .span = BIT(2) | BIT(3), .lift = 0, .indicator_pattern = indicator_pattern_jog_s0011_l00},
    { .span = BIT(0), .lift = 0, .indicator_pattern = indicator_pattern_jog_s1000_l00 },
    { .span = BIT(1), .lift = 0, .indicator_pattern = indicator_pattern_jog_s0100_l00 },
    { .span = BIT(2), .lift = 0, .indicator_pattern = indicator_pattern_jog_s0010_l00 },
    { .span = BIT(3), .lift = 0, .indicator_pattern = indicator_pattern_jog_s0001_l00 },
};
static unsigned jog_index;

static int setup(void) {
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
    if ((err = control_init())) {
        return err;
    }
    if ((err = bed_init())) {
        return err;
    }
    return 0;
}

static int do_rest(void) {
    return bed_poll_stop();
}

static int do_click_up(void) {
    return 0;
}

static int do_click_down(void) {
    return 0;
}

static int do_click_mode(void) {
    jog_index += 1;
    if (jog_index >= ARRAY_SIZE(jog_specs)) {
        jog_index = 0;
    }
    return 0;
}

static int do_hold_up(void) {
    switch (menu) {
        case MENU_MAIN:
            return bed_poll_pose(BED_POSE_LOUNGE);
        case MENU_JOG:
            return span_poll_jog(false, jog_specs[jog_index].span);
        default:
            return 0;
    }
}

static int do_hold_down(void) {
    switch (menu) {
        case MENU_MAIN:
            return bed_poll_pose(BED_POSE_SLEEP);
        case MENU_JOG:
            return span_poll_jog(true, jog_specs[jog_index].span);
        default:
            return 0;
    }
}

static int do_long_press_mode(void) {
    switch (menu) {
        case MENU_MAIN:
            menu = MENU_JOG;
            jog_index = 0;
            return 0;
        case MENU_JOG:
            menu = MENU_MAIN;
            return 0;
        default:
            return 0;
    }
}

static void show_status(void) {
    switch (menu) {
        case MENU_JOG:
            indicator_pattern(jog_specs[jog_index].indicator_pattern);
            break;
        case MENU_MAIN:
        default:
            switch (bed_get_state()) {
                case BED_STATE_ERROR:
                    indicator_pattern(indicator_pattern_error);
                    break;
                case BED_STATE_ABORTED:
                    indicator_pattern(indicator_pattern_aborted);
                    break;
                case BED_STATE_MOVING:
                    switch (bed_get_target_pose()) {
                        case BED_POSE_LOUNGE:
                            indicator_on(INDICATOR_HUE_CYAN - bed_get_progress() * (INDICATOR_HUE_CYAN - INDICATOR_HUE_GREEN) / 100);
                            break;
                        case BED_POSE_SLEEP:
                            indicator_on(INDICATOR_HUE_BLUE + bed_get_progress() * (INDICATOR_HUE_MAGENTA - INDICATOR_HUE_BLUE) / 100);
                            break;
                        default:
                            __ASSERT(false, "");
                            break;
                    }
                    break;
                case BED_STATE_DONE:
                    switch (bed_get_current_pose()) {
                        case BED_POSE_LOUNGE:
                            indicator_on(INDICATOR_HUE_GREEN);
                            break;
                        case BED_POSE_SLEEP:
                            indicator_on(INDICATOR_HUE_MAGENTA);
                            break;
                        default:
                            __ASSERT(false, "");
                            break;
                    }
                    break;
            }
            break;
    }
}

static void loop(void) {
    static bool action_error;
    static enum control_action action_pending;
    static int64_t last_action_time;

    const enum control_action action_current = atomic_get(&control_action_current);
    if (action_current == CONTROL_ACTION_RELEASE) {
        action_error = false;
    }
    int err = 0; // 0 means done (ready to rest), 1 means in progress, < 0 means error
    bool did_action = false;
    if (!action_error) {
        switch (action_current) {
            case CONTROL_ACTION_PRESS_UP:
            case CONTROL_ACTION_PRESS_DOWN:
            case CONTROL_ACTION_PRESS_MODE:
                action_pending = action_current;
                break;
            case CONTROL_ACTION_HOLD_UP:
                err = do_hold_up();
                action_pending = CONTROL_ACTION_RELEASE;
                did_action = true;
                break;
            case CONTROL_ACTION_HOLD_DOWN:
                err = do_hold_down();
                action_pending = CONTROL_ACTION_RELEASE;
                did_action = true;
                break;
            case CONTROL_ACTION_HOLD_MODE:
                if (action_pending == CONTROL_ACTION_PRESS_MODE) {
                    err = do_long_press_mode();
                    action_pending = CONTROL_ACTION_HOLD_MODE;
                    did_action = true;
                }
                break;
            case CONTROL_ACTION_RELEASE:
            default:
                switch (action_pending) {
                    case CONTROL_ACTION_PRESS_UP:
                        err = do_click_up();
                        did_action = true;
                        break;
                    case CONTROL_ACTION_PRESS_DOWN:
                        err = do_click_down();
                        did_action = true;
                        break;
                    case CONTROL_ACTION_PRESS_MODE:
                        err = do_click_mode();
                        did_action = true;
                        break;
                    default:
                        break;
                }
                action_pending = CONTROL_ACTION_RELEASE;
                break;
        }
    }
    if (did_action) {
        last_action_time = k_uptime_get();
    }
    if (err == 0) {
        err = do_rest();
    }
    if (err < 0) {
        action_error = true;
    }
    if (action_error) {
        indicator_pattern(indicator_pattern_error);
    } else if (k_uptime_get() - last_action_time < ACTIVITY_TIMEOUT_MS
            || menu != MENU_MAIN) {
        show_status();
    } else {
        indicator_pattern(NULL);
    }
}

int main(void) {
    int err;
    if ((err = setup())) {
        indicator_on(INDICATOR_HUE_RED);
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
