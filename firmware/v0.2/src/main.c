#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#include "bed/bed.h"
#include "bed/lift.h"
#include "bed/span.h"
#include "control.h"
#include "indicator.h"
#include "monitor.h"

#define WATCHDOG_TIMEOUT_MS (10000)
static const struct device *watchdog_dev = DEVICE_DT_GET(DT_NODELABEL(iwdg));

// Timeout for the indicator to go dark after completing an activity.
#define ACTIVITY_TIMEOUT_MS (1500)

/*
 * Error codes.
 */

INDICATOR_PATTERN_LOOP(indicator_pattern_aborted, 0,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 2, 2, 1));

INDICATOR_PATTERN_LOOP(indicator_pattern_generic_error, 0,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_RED, 2, 2, 1));

INDICATOR_PATTERN_LOOP(indicator_pattern_control_error, 4,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_RED, 5, 3, 1),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_MAGENTA, 1, 3, INDICATOR_PATTERN_PARAM_CYCLES));

INDICATOR_PATTERN_LOOP(indicator_pattern_bed_error, 4,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_RED, 5, 3, 1),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 1, 3, INDICATOR_PATTERN_PARAM_CYCLES));

INDICATOR_PATTERN_LOOP(indicator_pattern_lift_error, 4,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_RED, 5, 3, 1),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 3, INDICATOR_PATTERN_PARAM_CYCLES));

INDICATOR_PATTERN_LOOP(indicator_pattern_span_error, 4,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_RED, 5, 3, 1),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 3, INDICATOR_PATTERN_PARAM_CYCLES));

static void show_error(int err) {
    switch (ERROR_CATEGORY(err)) {
        case ERROR_CATEGORY(CONTROL_ERROR_FIRST_):
            indicator_pattern(indicator_pattern_control_error, ERROR_INDEX(err));
            break;
        case ERROR_CATEGORY(BED_ERROR_FIRST_):
            indicator_pattern(indicator_pattern_bed_error, ERROR_INDEX(err));
            break;
        case ERROR_CATEGORY(LIFT_ERROR_FIRST_):
            indicator_pattern(indicator_pattern_lift_error, ERROR_INDEX(err));
            break;
        case ERROR_CATEGORY(SPAN_ERROR_FIRST_):
            indicator_pattern(indicator_pattern_span_error, ERROR_INDEX(err));
            break;
        default:
            indicator_pattern(indicator_pattern_generic_error, 0);
            break;
    }
}

/*
 * Jog specifications.
 */

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_lift, 4,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 5, 3, 1),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_CYAN, 1, 3, INDICATOR_PATTERN_PARAM_CYCLES));

struct jog_lift_spec {
    enum lift_move move;
    unsigned indicator_pattern_param;
};
static const struct jog_lift_spec jog_lift_specs[] = {
    { .move = LIFT_MOVE_TANDEM, .indicator_pattern_param = 1,  },
    { .move = LIFT_MOVE_INDEPENDENT_BOTH, .indicator_pattern_param = 2 },
    { .move = LIFT_MOVE_INDEPENDENT_1, .indicator_pattern_param = 3 },
    { .move = LIFT_MOVE_INDEPENDENT_2, .indicator_pattern_param = 4 },
};

INDICATOR_PATTERN_LOOP(indicator_pattern_jog_span, 4,
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_AMBER, 5, 3, 1),
    INDICATOR_PATTERN_ENTRY(INDICATOR_HUE_BLUE, 1, 3, INDICATOR_PATTERN_PARAM_CYCLES));

struct jog_span_spec {
    enum span_move move;
    unsigned actuator_set;
    unsigned indicator_pattern_param;
};
static const struct jog_span_spec jog_span_specs[] = {
    { .move = SPAN_MOVE_HOME, .actuator_set = SPAN_ACTUATOR_SET_ALL, .indicator_pattern_param = 1 },
    { .move = SPAN_MOVE_HOME, .actuator_set = SPAN_ACTUATOR_SET_01, .indicator_pattern_param = 2 },
    { .move = SPAN_MOVE_HOME, .actuator_set = SPAN_ACTUATOR_SET_23, .indicator_pattern_param = 3 },
    { .move = SPAN_MOVE_JOG, .actuator_set = BIT(0), .indicator_pattern_param = 4 },
    { .move = SPAN_MOVE_JOG, .actuator_set = BIT(1), .indicator_pattern_param = 5 },
    { .move = SPAN_MOVE_JOG, .actuator_set = BIT(2), .indicator_pattern_param = 6 },
    { .move = SPAN_MOVE_JOG, .actuator_set = BIT(3), .indicator_pattern_param = 7 },
};

/*
 * Menus.
 */

enum menu {
    MENU_MAIN = 0,
    MENU_JOG_LIFT = 1,
    MENU_JOG_SPAN = 2,
};
static enum menu menu;
static unsigned menu_mode;
static const unsigned menu_mode_count[] = { 0, ARRAY_SIZE(jog_lift_specs), ARRAY_SIZE(jog_span_specs) };

/*
 * Control interface.
 */

static int do_rest(void) {
    return bed_poll_stop();
}

static int do_click_action(bool up) {
    return 0;
}

static int do_click_mode(void) {
    menu_mode += 1;
    if (menu_mode >= menu_mode_count[menu]) {
        menu_mode = 0;
    }
    return 0;
}

static int do_hold_action(bool up, struct monitor_setting setting) {
    if (setting.control_inhibit) {
        return CONTROL_ERROR_INHIBITED;
    }
    switch (menu) {
        case MENU_MAIN:
            return bed_poll_pose(up ? BED_POSE_LOUNGE : BED_POSE_SLEEP);
        case MENU_JOG_LIFT: {
            int span_err = span_poll_standby();
            int lift_err = lift_poll_jog(jog_lift_specs[menu_mode].move, up);
            return span_err ? span_err : lift_err;
        }
        case MENU_JOG_SPAN: {
            int span_err = span_poll_jog(jog_span_specs[menu_mode].move, up, jog_span_specs[menu_mode].actuator_set);
            int lift_err = lift_poll_standby();
            return span_err ? span_err : lift_err;
        }
        default:
            return 0;
    }
}

static int do_long_press_mode(void) {
    menu += 1;
    menu_mode = 0;
    if (menu >= ARRAY_SIZE(menu_mode_count)) {
        menu = MENU_MAIN;
    }
    return 0;
}

static void show_status(void) {
    switch (menu) {
        case MENU_JOG_LIFT:
            indicator_pattern(indicator_pattern_jog_lift, 
                    jog_lift_specs[menu_mode].indicator_pattern_param);
            break;
        case MENU_JOG_SPAN:
            indicator_pattern(indicator_pattern_jog_span,
                    jog_span_specs[menu_mode].indicator_pattern_param);
            break;
        case MENU_MAIN:
        default:
            switch (bed_get_state()) {
                case BED_STATE_ERROR:
                    indicator_pattern(indicator_pattern_generic_error, 0);
                    break;
                case BED_STATE_ABORTED:
                    indicator_pattern(indicator_pattern_aborted, 0);
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
    if ((err = monitor_init())) {
        return err;
    }
    if ((err = bed_init())) {
        return err;
    }
    return 0;
}

static void loop(void) {
    static int action_error;
    static bool action_complete;
    static enum control_action action_pending;
    static int64_t last_action_time;
    static int last_error;

    const struct monitor_setting setting = monitor_get_setting();
    const enum control_action action_current = control_get_action_current();
    if (action_current == CONTROL_ACTION_RELEASE) {
        action_error = 0;
        action_complete = false;
    }
    int err = 0; // 0 means done (ready to rest), 1 means in progress, < 0 means error
    bool did_action = false;
    if (!action_error && !action_complete) {
        switch (action_current) {
            case CONTROL_ACTION_PRESS_UP:
            case CONTROL_ACTION_PRESS_DOWN:
            case CONTROL_ACTION_PRESS_MODE:
                action_pending = action_current;
                break;
            case CONTROL_ACTION_HOLD_UP:
                err = do_hold_action(true, setting);
                action_pending = CONTROL_ACTION_RELEASE;
                did_action = true;
                break;
            case CONTROL_ACTION_HOLD_DOWN:
                err = do_hold_action(false, setting);
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
                        err = do_click_action(true);
                        did_action = true;
                        break;
                    case CONTROL_ACTION_PRESS_DOWN:
                        err = do_click_action(false);
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
        if (err == 0) {
            action_complete = true;
        }
    }
    if (!did_action || action_complete) {
        err = do_rest();
    }
    if (err < 0) {
        action_error = err;
    }
    if (action_error) {
        show_error(action_error);
        last_error = action_error;
    } else if (k_uptime_get() - last_action_time < ACTIVITY_TIMEOUT_MS
            || menu != MENU_MAIN) {
        show_status();
    } else {
        indicator_pattern(NULL, 0);
    }
    struct monitor_status status = {
        .bed_state = bed_get_state(),
        .bed_pose_current = bed_get_current_pose(),
        .bed_pose_target = bed_get_target_pose(),
        .lift_position = lift_get_position(),
        .lift_state = lift_get_state(),
        .span_position = span_get_position(),
        .span_state = span_get_state(),
        .control_inhibited = setting.control_inhibit,
        .control_active = did_action,
        .control_error = !!action_error,
        .last_error = -last_error,
    };
    monitor_set_status(status);
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
