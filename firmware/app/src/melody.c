#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "melody.h"

static const struct pwm_dt_spec sound_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(sound), 0);

K_SEM_DEFINE(melody_sem, 1, 1);

static const melody_t *melody_current;
static unsigned melody_index;
static uint64_t melody_cycles_per_second;

static void melody_handler(struct k_work* work);
K_WORK_DELAYABLE_DEFINE(melody_work, melody_handler);

static void melody_set_sound_l(unsigned freq) {
    if (freq) {
        unsigned period = melody_cycles_per_second / freq;
        pwm_set_cycles(sound_pwm.dev, sound_pwm.channel, period, period / 2, sound_pwm.flags);
    } else {
        pwm_set_cycles(sound_pwm.dev, sound_pwm.channel, 65535, 0, sound_pwm.flags);
    }
}

static void melody_advance_l(void) {
    const struct melody_tone *tone = &melody_current[melody_index];
    if (tone->duration) {
        melody_set_sound_l(tone->freq);
        k_work_reschedule(&melody_work, K_MSEC(tone->duration));
        melody_index += 1;
    } else {
        melody_set_sound_l(0);
        melody_current = NULL;
    }
}

static void melody_handler(struct k_work* work) {
    k_sem_take(&melody_sem, K_FOREVER);
    if (melody_current) {
        melody_advance_l();
    }
    k_sem_give(&melody_sem);
}

int melody_init(void) {
    return pwm_get_cycles_per_sec(sound_pwm.dev, sound_pwm.channel, &melody_cycles_per_second);
}

void melody_stop(void) {
    k_sem_take(&melody_sem, K_FOREVER);
    melody_current = NULL;
    melody_set_sound_l(0);
    k_sem_give(&melody_sem);
}

void melody_play(const melody_t *melody) {
    k_sem_take(&melody_sem, K_FOREVER);
    if (melody_current != melody) {
        melody_current = melody;
        melody_index = 0;
        melody_advance_l();
    }
    k_sem_give(&melody_sem);
}
