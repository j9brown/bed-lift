#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

#include "power.h"

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec power_5v_en_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, power_5v_en_gpios);

static struct k_spinlock power_lock;
static unsigned power_demands; // guarded by power_lock

int power_init(void) {
    int err;
    if (!device_is_ready(power_5v_en_gpio.port)) {
        return -ENODEV;
    }
    if ((err = gpio_pin_configure_dt(&power_5v_en_gpio, GPIO_OUTPUT_INACTIVE))) {
        return err;
    }
    return 0;
}

void power_5v_request(unsigned demand) {
    K_SPINLOCK(&power_lock) {
        unsigned old = power_demands;
        power_demands |= demand;
        if (!old && power_demands) {
            gpio_pin_set_dt(&power_5v_en_gpio, true);
        }
    }
}

void power_5v_release(unsigned demand) {
    K_SPINLOCK(&power_lock) {
        unsigned old = power_demands;
        power_demands &= ~demand;
        if (old && !power_demands) {
            gpio_pin_set_dt(&power_5v_en_gpio, false);
        }
    }
}
