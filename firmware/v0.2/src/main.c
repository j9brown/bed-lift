#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "indicator.h"
#include "control.h"

static atomic_t control_action_current;

void control_action_callback(enum control_action action) {
    atomic_set(&control_action_current, action);
}

int main(void) {
    int err;
    if ((err = control_init())) {
        return err;
    }

    uint32_t hue = 0;
    while (true) {
        enum control_action action = atomic_get(&control_action_current);
        switch (action) {
            case CONTROL_ACTION_PRESS_UP:
                hue = INDICATOR_HUE_YELLOW;
                break;
            case CONTROL_ACTION_HOLD_UP:
                hue = INDICATOR_HUE_RED;
                break;
            case CONTROL_ACTION_PRESS_DOWN:
                hue = INDICATOR_HUE_CYAN;
                break;
            case CONTROL_ACTION_HOLD_DOWN:
                hue = INDICATOR_HUE_BLUE;
                break;
            default:
                hue += 1;
                break;
        }
        status_indicator_on(hue);

        k_msleep(1);
    }

    return 0;
}