#include <zephyr/drivers/pwm.h>

#include "indicator.h"

static const struct pwm_dt_spec red_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(indicator_status), 0);
static const struct pwm_dt_spec green_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(indicator_status), 1);
static const struct pwm_dt_spec blue_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(indicator_status), 2);

void status_indicator_off() {
    pwm_set_pulse_dt(&red_pwm, 0);
    pwm_set_pulse_dt(&green_pwm, 0);
    pwm_set_pulse_dt(&blue_pwm, 0);
}

void status_indicator_on(unsigned hue) {
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

    pwm_set_pulse_dt(&red_pwm, (r * red_pwm.period) >> 8);
    pwm_set_pulse_dt(&green_pwm, (g * green_pwm.period) >> 8);
    pwm_set_pulse_dt(&blue_pwm, (b * blue_pwm.period) >> 8);
}
