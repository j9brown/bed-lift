#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/drv8434s.h>

#include "bed/span.h"

#define NUM_STEPPERS (4)

static const struct device *span_step_driver_dev = DEVICE_DT_GET(DT_NODELABEL(span_step_driver));
static const struct device *span_step_counter_dev = DEVICE_DT_GET(DT_NODELABEL(span_step_counter));

static void step_loop(void *, void *, void *);
//K_KERNEL_THREAD_DEFINE(step_tid, K_THREAD_STACK, step_loop, K_PRIO_COOP(0), K_ESSENTIAL, 0);

static void step_loop(void *, void *, void *) {

}

static bool started;

int span_init(void) {
    return 0;
}

int span_poll(enum span_action action) {
    int err;
    if (action == SPAN_ACTION_STOP) {
        if (started) {
            if ((err = drv8434s_stop(span_step_driver_dev))) {
                return err;
            }
            started = false;
        }
    } else {
        uint8_t status[NUM_STEPPERS];
        struct drv8434s_options options = {
            .status_buf = status,
            .clear_fault = false,
        };

        if (!started) {
            if ((err = drv8434s_start(span_step_driver_dev, NUM_STEPPERS))) {
                return err;
            }
            started = true;
            if ((err = drv8434s_set_output_enable(span_step_driver_dev, &options, BIT64(0)))) {
                return err;
            }
        }

        uint8_t step_requests[NUM_STEPPERS];
        for (unsigned i = 0; i < NUM_STEPPERS; i++) {
            step_requests[i] = drv8434s_make_step_request(i == 0, action == SPAN_ACTION_EXTEND, DRV8434S_MICROSTEP_2);
        }
        if ((err = drv8434s_step(span_step_driver_dev, &options, step_requests))) {
            return err;
        }
    }
    return 0;
}
