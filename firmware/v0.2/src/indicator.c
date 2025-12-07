#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "indicator.h"

static const struct pwm_dt_spec red_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(indicator_status), 0);
static const struct pwm_dt_spec green_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(indicator_status), 1);
static const struct pwm_dt_spec blue_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(indicator_status), 2);

K_SEM_DEFINE(indicator_sem, 1, 1);

static const indicator_pattern_t *indicator_pattern_current;
static unsigned indicator_pattern_index;
static bool indicator_pattern_on;

static void indicator_pattern_handler(struct k_work* work);
K_WORK_DELAYABLE_DEFINE(indicator_pattern_work, indicator_pattern_handler);

// Brightness scale factors: 0 (off), 64 (full bright)
#define INDICATOR_RED_SCALE (64)
#define INDICATOR_GREEN_SCALE (8)
#define INDICATOR_BLUE_SCALE (12)
#define INDICATOR_SCALE_BITS (6)

static void indicator_pwm_set_off_l() {
    pwm_set_pulse_dt(&red_pwm, 0);
    pwm_set_pulse_dt(&green_pwm, 0);
    pwm_set_pulse_dt(&blue_pwm, 0);
}

static void indicator_pwm_set_on_l(unsigned hue) {
    unsigned r, g, b;
    hue = ((hue & 0xffff) * 1536 + 0x8000) >> 16;
    if (hue < 512) {
        if (hue < 256) {
            g = hue;
            r = 256;
        } else {
            r = 512 - hue;
            g = 256;
        }
        b = 0;
    } else if (hue < 1024) {
        if (hue < 768) {
            b = hue - 512;
            g = 256;
        } else {
            g = 1024 - hue;
            b = 256;
        }
        r = 0;
    } else {
        if (hue < 1280) {
            r = hue - 1024;
            b = 256;
        } else {
            b = 1536 - hue;
            r = 256;
        }
        g = 0;
    }

    pwm_set_pulse_dt(&red_pwm, (r * red_pwm.period * INDICATOR_RED_SCALE) >> (8 + INDICATOR_SCALE_BITS));
    pwm_set_pulse_dt(&green_pwm, (g * green_pwm.period * INDICATOR_GREEN_SCALE) >> (8 + INDICATOR_SCALE_BITS));
    pwm_set_pulse_dt(&blue_pwm, (b * blue_pwm.period * INDICATOR_BLUE_SCALE) >> (8 + INDICATOR_SCALE_BITS));
}

void indicator_off() {
    k_sem_take(&indicator_sem, K_FOREVER);
    indicator_pattern_current = NULL;
    indicator_pattern_on = false;
    indicator_pwm_set_off_l();
    k_sem_give(&indicator_sem);
}

void indicator_on(unsigned hue) {
    k_sem_take(&indicator_sem, K_FOREVER);
    indicator_pattern_current = NULL;
    indicator_pattern_on = false;
    indicator_pwm_set_on_l(hue);
    k_sem_give(&indicator_sem);
}

void indicator_pattern_advance_l() {
    for (;;) {
        const struct indicator_pattern_entry *entry = &indicator_pattern_current[indicator_pattern_index];
        if (!indicator_pattern_on) {
            if (entry->on_time) {
                indicator_pattern_on = true;
                indicator_pwm_set_on_l(entry->hue);
                k_work_reschedule(&indicator_pattern_work, K_MSEC(entry->on_time * 100));
                break; // wait for next cycle
            }
            if (entry->off_time) {
                break; // end pattern
            }
            indicator_pattern_index = 0; // repeat pattern
        } else {
            indicator_pattern_index += 1;
            indicator_pattern_on = false;
            if (entry->off_time) {
                indicator_pwm_set_off_l();
                k_work_reschedule(&indicator_pattern_work, K_MSEC(entry->off_time * 100));
                break; // wait for next cycle
            }
        }
    }
}

static void indicator_pattern_handler(struct k_work* work) {
    k_sem_take(&indicator_sem, K_FOREVER);
    if (indicator_pattern_current) {
        indicator_pattern_advance_l();
    } else if (indicator_pattern_on) {
        indicator_pwm_set_off_l();
    }
    k_sem_give(&indicator_sem);
}

void indicator_pattern(const indicator_pattern_t *pattern) {
    k_sem_take(&indicator_sem, K_FOREVER);
    if (indicator_pattern_current != pattern) {
        indicator_pattern_current = pattern;
        if (pattern) {
            indicator_pattern_index = 0;
            indicator_pattern_on = false;
            indicator_pattern_advance_l();
        }
    } else if (!indicator_pattern_on) {
        indicator_pwm_set_off_l();
    }
    k_sem_give(&indicator_sem);
}