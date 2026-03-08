#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bed/bed.h"
#include "bed/lift.h"
#include "bed/span.h"
#include "control.h"
#include "indicator.h"
#include "melody.h"
#include "monitor.h"
#include "power.h"

LOG_MODULE_REGISTER(app);

#define WATCHDOG_TIMEOUT_MS (1000)
static const struct device *watchdog_dev = DEVICE_DT_GET(DT_NODELABEL(iwdg));

// Timeout for the indicator to go dark after completing an activity.
#define ACTIVITY_TIMEOUT_MS (1500)

// Timeout to revert to the main menu after completing an activity.
#define MENU_TIMEOUT_MS (15000)

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
 * Melodies.
 */

MELODY(melody_error_forbidden,
    MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(E6, 50));

MELODY(melody_error_alert,
    MELODY_NOTE(E6, 500), MELODY_REST(250), MELODY_NOTE(E6, 500), MELODY_REST(250), MELODY_NOTE(E6, 500));

MELODY(melody_pose_lounge,
    MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(G6, 250));

MELODY(melody_pose_sleep,
    MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(C6, 250));

MELODY(melody_menu_mode,
    MELODY_NOTE(E6, 50));

MELODY(melody_menu_main,
    MELODY_NOTE(E6, 50), MELODY_REST(25), MELODY_NOTE(F6, 25), MELODY_REST(25),
    MELODY_NOTE(E6, 25), MELODY_REST(25), MELODY_NOTE(D6, 25), MELODY_REST(25),
    MELODY_NOTE(E6, 50));

MELODY(melody_menu_jog_lift,
    MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(F6, 50), MELODY_REST(50), MELODY_NOTE(G6, 50));

MELODY(melody_menu_jog_span,
    MELODY_NOTE(E6, 50), MELODY_REST(50), MELODY_NOTE(D6, 50), MELODY_REST(50), MELODY_NOTE(C6, 50));

static void melody_play_error(int err) {
    switch (err) {
        case CONTROL_ERROR_INHIBITED:
            melody_play(melody_error_forbidden);
            break;
        default:
            melody_play(melody_error_alert);
            break;
    }
}

static void melody_play_pose(void) {
    switch (bed_get_current_pose()) {
        case BED_POSE_LOUNGE:
            melody_play(melody_pose_lounge);
            break;
        case BED_POSE_SLEEP:
            melody_play(melody_pose_sleep);
            break;
        default:
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

struct menu_spec {
    const unsigned mode_count;
    const melody_t *melody;
};
static const struct menu_spec menu_specs[] = {
    { 0, melody_menu_main },
    { ARRAY_SIZE(jog_lift_specs), melody_menu_jog_lift },
    { ARRAY_SIZE(jog_span_specs), melody_menu_jog_span },
};

static enum menu menu;
static unsigned menu_mode;

/*
 * Control interface.
 */

static int do_rest(void) {
    return bed_poll_stop();
}

static void do_goto_menu(enum menu new_menu) {
    menu_mode = 0;
    if (menu != new_menu) {
        menu = new_menu;
        melody_play(menu_specs[menu].melody);
    }
}

static void do_next_menu(void) {
    do_goto_menu(menu + 1 >= ARRAY_SIZE(menu_specs) ? MENU_MAIN : menu + 1);
}

static void do_next_menu_mode(void) {
    const unsigned count = menu_specs[menu].mode_count;
    if (count) {
        menu_mode += 1;
        if (menu_mode >= count) {
            menu_mode = 0;
        }
        melody_play(melody_menu_mode);
    }
}

static int do_move(bool up, struct monitor_setting setting) {
    if (setting.control_inhibit) {
        return CONTROL_ERROR_INHIBITED;
    }
    switch (menu) {
        case MENU_MAIN:
            return bed_poll_pose(up ? BED_POSE_LOUNGE : BED_POSE_SLEEP);
        case MENU_JOG_LIFT: {
            bed_prepare_for_manual_action();
            int span_err = span_poll_sleep();
            int lift_err = lift_poll_jog(jog_lift_specs[menu_mode].move, up);
            return span_err ? span_err : lift_err;
        }
        case MENU_JOG_SPAN: {
            bed_prepare_for_manual_action();
            int span_err = span_poll_jog(jog_span_specs[menu_mode].move, up, jog_span_specs[menu_mode].actuator_set);
            int lift_err = lift_poll_sleep();
            return span_err ? span_err : lift_err;
        }
        default:
            return 0;
    }
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
                            indicator_on(k_uptime_get() << 5);
                            break;
                        case BED_POSE_SLEEP:
                            indicator_on(-k_uptime_get() << 5);
                            break;
                        default:
                            __ASSERT(false, "");
                            break;
                    }
                    break;
                case BED_STATE_DONE:
                    indicator_on(INDICATOR_HUE_GREEN);
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
    if ((err = melody_init())) {
        return err;
    }
    if ((err = control_init())) {
        return err;
    }
    if ((err = monitor_init())) {
        return err;
    }
    if ((err = power_init())) {
        return err;
    }
    if ((err = bed_init())) {
        return err;
    }
    return 0;
}

enum command {
    COMMAND_NONE = 0,
    COMMAND_NEXT_MENU = 1,
    COMMAND_NEXT_MENU_MODE = 2,
    COMMAND_MOVE_UP = 3,
    COMMAND_MOVE_DOWN = 4,
    COMMAND_AUTO_MOVE_UP = 5,
    COMMAND_AUTO_MOVE_DOWN = 6,
};

static void loop(void) {
    static int last_error;

    static enum command command_previous;
    static bool command_issued;   // true if the command has been issued, result indicates most recent status
    static int command_result;    // 0 means done (ready to rest), 1 means in progress (issue again), < 0 means error (abort)
    static int64_t command_time;  // time when the command was last issued
    static enum control_action control_action_previous;

    // Determine which command to perform based on control inputs.
    enum control_action control_action_next = control_get_action();
    enum command command_next;
    switch (control_action_next) {
        case CONTROL_ACTION_HOLD_UP:
            command_next = COMMAND_MOVE_UP;
            break;
        case CONTROL_ACTION_HOLD_UP_THEN_PRESS_MODE:
            command_next = COMMAND_AUTO_MOVE_UP;
            break;
        case CONTROL_ACTION_HOLD_DOWN:
            command_next = COMMAND_MOVE_DOWN;
            break;
        case CONTROL_ACTION_HOLD_DOWN_THEN_PRESS_MODE:
            command_next = COMMAND_AUTO_MOVE_DOWN;
            break;
        case CONTROL_ACTION_HOLD_MODE:
            command_next = menu != MENU_MAIN ? COMMAND_NEXT_MENU : COMMAND_NONE;
            break;
        case CONTROL_ACTION_LONG_HOLD_MODE:
            command_next = menu == MENU_MAIN ? COMMAND_NEXT_MENU : COMMAND_NONE;
            break;
        case CONTROL_ACTION_RELEASE:
            switch (control_action_previous) {
                case CONTROL_ACTION_PRESS_MODE:
                    command_next = COMMAND_NEXT_MENU_MODE;
                    break;
                default:
                    if ((command_previous == COMMAND_AUTO_MOVE_UP || command_previous == COMMAND_AUTO_MOVE_DOWN)
                            && command_issued && command_result == 1) {
                        command_next = command_previous;
                    } else {
                        command_next = COMMAND_NONE;
                    }
                    break;
            }
            break;
        default:
            command_next = COMMAND_NONE;
            break;
    }
    control_action_previous = control_action_next;
    command_previous = command_next;
    LOG_DBG("control %d, command %d, menu %d, menu mode %d", control_action_next, command_next, menu, menu_mode);

    // Invoke the command.
    const struct monitor_setting setting = monitor_get_setting();
    bool did_move = false;
    if (command_next == COMMAND_NONE) {
        command_issued = false;
        command_result = 0;
    } else if (!command_issued || command_result == 1) {
        command_issued = true;
        command_time = k_uptime_get();
        command_result = 0;
        switch (command_next) {
            case COMMAND_MOVE_UP:
            case COMMAND_AUTO_MOVE_UP:
                command_result = do_move(/*up*/ true, setting);
                did_move = true;
                break;
            case COMMAND_MOVE_DOWN:
            case COMMAND_AUTO_MOVE_DOWN:
                command_result = do_move(/*up*/ false, setting);
                did_move = true;
                break;
            case COMMAND_NEXT_MENU:
                do_next_menu();
                break;
            case COMMAND_NEXT_MENU_MODE:
                do_next_menu_mode();
                break;
            default:
                break;
        }
    }
    if (!command_issued || command_result <= 0) {
        int err = do_rest();
        if (!command_result) {
            command_result = err;
        }
    }
    if (command_result < 0) {
        melody_play_error(command_result);
        last_error = command_result;
        LOG_ERR("Error %d from command %d", command_result, command_next);
    } else if (did_move && menu == MENU_MAIN && command_result == 0) {
        melody_play_pose();
    }

    // Update menu.
    if (menu != MENU_MAIN && k_uptime_get() - command_time >= MENU_TIMEOUT_MS) {
        do_goto_menu(MENU_MAIN);
    }
    if (menu != MENU_MAIN) {
        // Enable 5V power so the span hall sensors operate while on the test menu.
        power_5v_request(POWER_DEMAND_TEST);
    } else {
        power_5v_release(POWER_DEMAND_TEST);
    }

    // Update indicators.
    if (command_result < 0) {
        show_error(command_result);
    } else if (k_uptime_get() - command_time < ACTIVITY_TIMEOUT_MS
            || menu != MENU_MAIN) {
        show_status();
    } else {
        indicator_pattern(NULL, 0);
    }

    // Update monitor status.
    struct monitor_status status = {
        .bed_state = bed_get_state(),
        .bed_pose_current = bed_get_current_pose(),
        .bed_pose_target = bed_get_target_pose(),
        .lift_position = lift_get_position(),
        .lift_state = lift_get_state(),
        .span_position = span_get_position(),
        .span_state = span_get_state(),
        .control_inhibited = setting.control_inhibit,
        .control_active = command_issued,
        .control_error = !!command_result,
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
