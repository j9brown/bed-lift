#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

#include "control.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static struct gpio_dt_spec control_up_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, control_up_gpios);
static struct gpio_dt_spec control_down_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, control_down_gpios);
static struct gpio_callback control_gpio_callback;
static atomic_t control_action_pending;

static void control_work_handler(struct k_work* work);
K_WORK_DELAYABLE_DEFINE(control_work, control_work_handler);

static void control_gpio_callback_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    gpio_port_value_t value = 0;
    gpio_port_get(port, &value);
    bool control_up = value & BIT(control_up_gpio.pin);
    bool control_down = value & BIT(control_down_gpio.pin);
    enum control_action action;
    if (control_up && !control_down) {
        action = CONTROL_ACTION_PRESS_UP;
    } else if (control_down && !control_up) {
        action = CONTROL_ACTION_PRESS_DOWN;
    } else if (control_up && control_down) {
        action = CONTROL_ACTION_PRESS_MODE;
    } else {
        action = CONTROL_ACTION_RELEASE;
    }
    atomic_set(&control_action_pending, action);
    k_work_reschedule(&control_work, K_MSEC(10));
}

static void control_work_handler(struct k_work* work) {
    enum control_action action = atomic_get(&control_action_pending);
    bool reschedule_for_hold;
    switch (action) {
        case CONTROL_ACTION_PRESS_UP:
            reschedule_for_hold = atomic_cas(&control_action_pending, action, CONTROL_ACTION_HOLD_UP);
            break;
        case CONTROL_ACTION_PRESS_DOWN:
            reschedule_for_hold = atomic_cas(&control_action_pending, action, CONTROL_ACTION_HOLD_DOWN);
            break;
        case CONTROL_ACTION_PRESS_MODE:
            reschedule_for_hold = atomic_cas(&control_action_pending, action, CONTROL_ACTION_HOLD_MODE);
            break;
        default:
            reschedule_for_hold = false;
            break;
    }
    if (reschedule_for_hold) {
        k_work_reschedule(&control_work, K_MSEC(500));
    }

    control_action_callback(action);
}

int control_init(void) {
    int err;
    const struct device *port = control_up_gpio.port;
    if (!device_is_ready(port)) {
        return -ENODEV;
    }
    if (port != control_down_gpio.port) {
        return -EIO;
    }
    gpio_init_callback(&control_gpio_callback, control_gpio_callback_handler,
        BIT(control_up_gpio.pin) | BIT(control_down_gpio.pin));

    if ((err = gpio_pin_configure_dt(&control_up_gpio, GPIO_INPUT))) {
        return err;
    }
    if ((err = gpio_pin_configure_dt(&control_down_gpio, GPIO_INPUT))) {
        return err;
    }
    if ((err = gpio_pin_interrupt_configure_dt(&control_up_gpio, GPIO_INT_EDGE_BOTH))) {
        return err;
    }
    if ((err = gpio_pin_interrupt_configure_dt(&control_down_gpio, GPIO_INT_EDGE_BOTH))) {
        return err;
    }
    if ((err = gpio_add_callback(port, &control_gpio_callback))) {
        return err;
    }
    return 0;
}
