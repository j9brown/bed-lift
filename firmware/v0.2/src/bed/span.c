/**
 * @file Component that extends and retracts the wing spans of the bed.
 *
 * The bed frame physically couples two pairs of linear actuators driven by stepper motors. Both actuators in
 * a pair move at the same speed in the same direction until on or both actuators stall or are stopped.
 * To prevent damage to the frame in case of malfunction, when one actuator in a pair stalls, it is commanded
 * to hold position while its sibling performs a limited number of additional steps towards its end stop.
 * The component stops all actuators and reports an error if both actuators in a pair do not achieve their
 * end stop together or when faults other than a stall occur.
 *
 * Both pairs of actuators accelerate and travel at the same speed so they share velocity and acceleration
 * vectors. The step loop blocks until released by a timer tick then it checks the actuator status report
 * from the previous iteration and asynchronously issues a new batch of steps for all of the actuators.
 * Note that there are two timer ticks of latency between issuing a batch of steps and observing the
 * resulting status report. The latency is small in practice and it is a conscious trade-off to halve
 * the number of context switches for the step loop thread compared to synchronously issuing batches
 * of steps and checking the status immediately thereafter.
 * 
 * In case of emergencies, actuators can also be controlled individually or in groups to bring them back
 * into synchronization.
 */

#include <errno.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/drv8434s.h>

#include "bed/span.h"

// When set to 1, ignore open-load faults when some actuators are not connected
#define SPAN_DEBUG_IGNORE_OL_FAULT 1

// When set to 1, accelerate to full speed immediately
#define SPAN_DEBUG_INFINITE_ACCEL 0

#define SPAN_NUM_ACTUATORS (4)
#define SPAN_ALL_ACTUATOR_BITS (BIT(0) | BIT(1) | BIT(2) | BIT(3))

#define SPAN_MICROSTEP_DIVISOR 16
#define SPAN_MICROSTEP_MODE _CONCAT(DRV8434S_MICROSTEP_, SPAN_MICROSTEP_DIVISOR)

#define SPAN_MM_PER_STEP_F (0.04)
#define SPAN_MICROSTEPS_PER_MM_F (SPAN_MICROSTEP_DIVISOR * 1. / SPAN_MM_PER_STEP_F)
#define SPAN_TO_MICROSTEPS_(mm) (int)(mm * SPAN_MICROSTEPS_PER_MM_F)

#define SPAN_MAX_SPEED_MM_PER_S_F (8.)
#define SPAN_COAST_SPEED_MM_PER_S_F (SPAN_MAX_SPEED_MM_PER_S_F * 1.)
#define SPAN_JOG_SPEED_MM_PER_S_F (SPAN_MAX_SPEED_MM_PER_S_F * 0.25)
#define SPAN_ACCEL_MM_PER_S2_F (5.)

// Note: If the maximum number of microsteps is too high, the microstep loop may
// starve other threads if the CPU can't keep up with the context switches.
#define SPAN_MAX_MICROSTEPS_PER_S_F (SPAN_MICROSTEPS_PER_MM_F * SPAN_MAX_SPEED_MM_PER_S_F)
#define SPAN_MICROSTEP_TICK_PERIOD_US (int)(1000000. / SPAN_MAX_MICROSTEPS_PER_S_F)

#define SPAN_WATCHDOG_TIMEOUT_US (20000)
#define SPAN_WATCHDOG_TIMEOUT_TICKS (int)((SPAN_WATCHDOG_TIMEOUT_US + SPAN_MICROSTEP_TICK_PERIOD_US - 1) / SPAN_MICROSTEP_TICK_PERIOD_US)

#define SPAN_FRACTION_DENOM (1 << 30)

#define SPAN_TO_SPEED_FRACTION_(mm_per_s) (int)(SPAN_FRACTION_DENOM * 1. * mm_per_s / SPAN_MAX_SPEED_MM_PER_S_F)
#define SPAN_JOG_SPEED_FRACTION SPAN_TO_SPEED_FRACTION_(SPAN_JOG_SPEED_MM_PER_S_F)
#define SPAN_COAST_SPEED_FRACTION SPAN_TO_SPEED_FRACTION_(SPAN_COAST_SPEED_MM_PER_S_F)

#define SPAN_TO_ACCEL_FRACTION_(mm_per_s2) (int)(SPAN_FRACTION_DENOM * 1. * mm_per_s2 / SPAN_MAX_SPEED_MM_PER_S_F / SPAN_MAX_MICROSTEPS_PER_S_F)
#define SPAN_ACCEL_FRACTION SPAN_TO_ACCEL_FRACTION_(SPAN_ACCEL_MM_PER_S2_F)

// The maximum distance that the actuators are expected to travel from one endstop to the other.
// Constrains how long the actuators can run before an error is reported in case of failure.
#define SPAN_MAX_TRAVEL_MM_F (250.)
#define SPAN_MAX_TRAVEL_MICROSTEPS SPAN_TO_MICROSTEPS_(SPAN_MAX_TRAVEL_MM_F)

// The distance to move the actuators after each jog command.
#define SPAN_JOG_MM_F (0.5)
#define SPAN_JOG_MICROSTEPS SPAN_TO_MICROSTEPS_(SPAN_JOG_MM_F)

// The distance to move the actuators back from their endstops to remove strain on the lead screw.
#define SPAN_RELIEF_MM_F (3.)
#define SPAN_RELIEF_MICROSTEPS SPAN_TO_MICROSTEPS_(SPAN_RELIEF_MM_F)

// The distance that one actuator is allowed to continue traveling after its peer in a pair
// has stalled (presumably at an endstop). Constrains how far the actuators can become desynchronized.
#define SPAN_MAX_DESYNC_MM_F (3.)
#define SPAN_MAX_DESYNC_MICROSTEPS SPAN_TO_MICROSTEPS_(SPAN_MAX_DESYNC_MM_F)

static const struct device *span_stepper_dev = DEVICE_DT_GET(DT_NODELABEL(span_stepper));
static const struct device *span_counter_dev = DEVICE_DT_GET(DT_NODELABEL(span_counter));

// This semaphore guards the global state of this component and access to the device drivers.
K_SEM_DEFINE(span_state_sem, 1, 1);

// This semaphore blocks the step loop until the next timer tick occurs.
K_SEM_DEFINE(span_tick_sem, 0, 1);

// Decremented by one each time the step loop handles a tick of the timer.
// At zero, the loop stops the actuators and reports an error.
// Behaves as a watchdog to ensure that the main loop is always in control.
static unsigned span_ticks_remaining;

// Decremented by one after each microstep is issued.
// When this counter reaches zero, the loop transitions to the done state.
static unsigned span_microsteps_remaining;

// Fractional microstep position, velocity and acceleration.
// The step loop updates these values on each tick as follows:
// - If span_microstep_velocity_actual is less than span_microstep_velocity_target, add
//   span_microstep_accel to span_microstep_velocity_actual and clamp the result to
//   span_microstep_velocity_target.
// - Similarly, if span_microstep_velocity_actual is greater than span_microstep_velocity_target,
//   subtract span_microstep_accel from span_microstep_velocity_actual and clamp the
//   result to span_microstep_velocity_target.
// - Add span_microstep_velocity_actual to span_microstep_position_fraction, when the fraction
//   wraps at +/- SPAN_FRACTION_DENOM issue one microstep to each active actuator.
static int span_microstep_position_fraction; // range: +/- SPAN_FRACTION_DENOM
static int span_microstep_velocity_actual;   // range: +/- SPAN_FRACTION_DENOM
static int span_microstep_velocity_target;   // range: +/- SPAN_FRACTION_DENOM
static int span_microstep_accel;             // range: 0 to SPAN_FRACTION_DENOM

// The number of times each actuator has lost steps due to a stall or other fault.
// When this counter exceeds SPAN_MAX_DESYNC_MICROSTEPS, the actuator's peer is considered desynchronized.
static unsigned span_microsteps_lost[SPAN_NUM_ACTUATORS];

// State of each actuator. The loop only drives actuators that are in one of the RUN_* states.
enum span_microstep_actuator_state {
    SPAN_MICROSTEP_ACTUATOR_RUN_PAIR = 2,        // Run actuator and keep synchronized with its peer
    SPAN_MICROSTEP_ACTUATOR_RUN_INDEPENDENT = 1, // Run actuator independently of its peer
    SPAN_MICROSTEP_ACTUATOR_HALT = 0,            // Halt actuator
    SPAN_MICROSTEP_ACTUATOR_STALL = -1,          // The actuator stalled
    SPAN_MICROSTEP_ACTUATOR_FAULT = -2,          // The actuator faulted
    SPAN_MICROSTEP_ACTUATOR_DESYNC = -3,         // The actuator lost synchronization with its peer
};
static enum span_microstep_actuator_state span_microstep_actuator_state[SPAN_NUM_ACTUATORS];

// State of the microstep loop. The loop only drives the actuators when in the RUN state.
enum span_microstep_loop_state {
    SPAN_MICROSTEP_LOOP_RUN = 1,      // Run actuators
    SPAN_MICROSTEP_LOOP_HALT = 0,     // Halt actuators
    SPAN_MICROSTEP_LOOP_TIMEOUT = -1, // The ticks left counter reached zero while the loop was running
    SPAN_MICROSTEP_LOOP_ERROR = -2,   // An error occurred while issuing steps to the driver
    SPAN_MICROSTEP_LOOP_FAULT = -3,   // One or more actuators reported a fault
    SPAN_MICROSTEP_LOOP_DESYNC = -4,  // One or more actuators lost synchronization with its peer
    SPAN_MICROSTEP_LOOP_DONE = -5,    // All actuators are halted, stalled, or zero microsteps remaining
};
static enum span_microstep_loop_state span_microstep_loop_state;

static void span_counter_tick(const struct device *dev, void *user_data) {
    k_sem_give(&span_tick_sem);
}

static void span_microstep_complete(const struct device *dev, int result, void *user_data) {
    int *result_ptr = user_data;
    *result_ptr = result;
    k_sem_give(&span_state_sem);
}

static void span_microstep_loop(void *, void *, void *) {
    uint8_t pending_step_status[SPAN_NUM_ACTUATORS] = {0};
    struct drv8434s_options pending_step_options = {
        .status_buf = pending_step_status,
        .clear_fault = false,
    };
    const int STEP_NOT_PENDING = 1;
    int pending_step_result = STEP_NOT_PENDING;
    uint8_t pending_step_requests[SPAN_NUM_ACTUATORS];
    bool have_steps;
    bool have_fault;
    int err;

    for (;;) {
        // Wait for the timer callback to release the semaphore; it is not released by this loop.
        k_sem_take(&span_tick_sem, K_FOREVER);

        // Check for errors and timeouts.
        k_sem_take(&span_state_sem, K_FOREVER);
        const int last_step_result = pending_step_result;
        pending_step_result = STEP_NOT_PENDING;
        if (span_microstep_loop_state <= SPAN_MICROSTEP_LOOP_HALT) {
            goto give_state_sem_and_continue;
        }
        if (last_step_result < 0) {
            span_microstep_loop_state = SPAN_MICROSTEP_LOOP_ERROR;
            goto give_state_sem_and_continue;
        }
        if (span_ticks_remaining == 0) {
            span_microstep_loop_state = SPAN_MICROSTEP_LOOP_TIMEOUT;
            goto give_state_sem_and_continue;
        }
        span_ticks_remaining -= 1;

        // Check for faults.
        have_fault = false;
        for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
            uint8_t status = pending_step_status[i];
            if (status) {
#if SPAN_DEBUG_IGNORE_OL_FAULT
                if (status == DRV8434S_STATUS_OL) {
                    continue;
                }
#endif
                if (pending_step_requests[i] & DRV8434S_STEP_REQUEST_STEP) {
                    span_microsteps_lost[i] += 1;
                }
                if (status != DRV8434S_STATUS_STL) {
                    span_microstep_actuator_state[i] = SPAN_MICROSTEP_ACTUATOR_FAULT;
                    have_fault = true;
                } else if (span_microstep_actuator_state[i] > SPAN_MICROSTEP_ACTUATOR_HALT) {
                    span_microstep_actuator_state[i] = SPAN_MICROSTEP_ACTUATOR_STALL;
                }
            }
        }
        if (have_fault) {
            span_microstep_loop_state = SPAN_MICROSTEP_LOOP_FAULT;
            goto give_state_sem_and_continue;
        }

        // Update motion vectors and remaining step count.
        if (span_microsteps_remaining == 0) {
            span_microstep_loop_state = SPAN_MICROSTEP_LOOP_DONE;
            goto give_state_sem_and_continue;
        }
#if SPAN_DEBUG_INFINITE_ACCEL
        bool dir = span_microstep_velocity_target > 0;
#else
        if (span_microstep_velocity_actual < span_microstep_velocity_target) {
            span_microstep_velocity_actual += span_microstep_accel;
            if (span_microstep_velocity_actual > span_microstep_velocity_target) {
                span_microstep_velocity_actual = span_microstep_velocity_target;
            }
        } else if (span_microstep_velocity_actual > span_microstep_velocity_target) {
            span_microstep_velocity_actual -= span_microstep_accel;
            if (span_microstep_velocity_actual < span_microstep_velocity_target) {
                span_microstep_velocity_actual = span_microstep_velocity_target;
            }
        }
        span_microstep_position_fraction += span_microstep_velocity_actual;
        bool dir;
        if (span_microstep_position_fraction >= SPAN_FRACTION_DENOM) {
            span_microstep_position_fraction -= SPAN_FRACTION_DENOM;
            dir = true;
        } else if (span_microstep_position_fraction <= -SPAN_FRACTION_DENOM) {
            span_microstep_position_fraction += SPAN_FRACTION_DENOM;
            dir = false;
        } else {
            // Not ready to take the next step yet.
            goto give_state_sem_and_continue;
        }
#endif
        span_microsteps_remaining -= 1;

        // Prepare step requests and ensure actuator pair synchronization.
        have_steps = false;
        for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
            bool step = false;
            switch (span_microstep_actuator_state[i]) {
                case SPAN_MICROSTEP_ACTUATOR_RUN_INDEPENDENT:
                    step = true;
                    break;
                case SPAN_MICROSTEP_ACTUATOR_RUN_PAIR:
                    if (span_microstep_actuator_state[i ^ 1] == SPAN_MICROSTEP_ACTUATOR_RUN_PAIR &&
                            span_microsteps_lost[i ^ 1] < SPAN_MAX_DESYNC_MICROSTEPS) {
                        step = true;
                        break;
                    }
                    span_microstep_actuator_state[i] = SPAN_MICROSTEP_ACTUATOR_DESYNC;
                    // fall-through
                case SPAN_MICROSTEP_ACTUATOR_DESYNC:
                    span_microstep_loop_state = SPAN_MICROSTEP_LOOP_DESYNC;
                    goto give_state_sem_and_continue;
                case SPAN_MICROSTEP_ACTUATOR_HALT:
                case SPAN_MICROSTEP_ACTUATOR_STALL:
                    break;
                case SPAN_MICROSTEP_ACTUATOR_FAULT:
                default:
                    span_microstep_loop_state = SPAN_MICROSTEP_LOOP_FAULT;
                    goto give_state_sem_and_continue;
            }
            pending_step_requests[i] = drv8434s_make_step_request(step, dir, SPAN_MICROSTEP_MODE);
            have_steps |= step;
        }
        if (!have_steps) {
            span_microstep_loop_state = SPAN_MICROSTEP_LOOP_DONE;
            goto give_state_sem_and_continue;
        }

        // Issue step requests.
#if 1
        if ((err = drv8434s_step_async(span_stepper_dev, &pending_step_options, pending_step_requests,
                span_microstep_complete, &pending_step_result))) {
            span_microstep_loop_state = SPAN_MICROSTEP_LOOP_ERROR;
            goto give_state_sem_and_continue;
        }
#else
        if ((err = drv8434s_step(span_stepper_dev, &pending_step_options, pending_step_requests))) {
            pending_step_result = err;
            span_microstep_loop_state = SPAN_MICROSTEP_LOOP_ERROR;
            goto give_state_sem_and_continue;
        }
        pending_step_result = 0;
        k_sem_give(&span_state_sem);
#endif

        // Await result on next tick. Keep holding the state semaphore until the step complete callback runs.
        continue;

        // Release the state semaphore when no step was issued or an error occurred.
    give_state_sem_and_continue:
        k_sem_give(&span_state_sem);
    }
}
#define SPAN_THREAD_STACK_SIZE 1024
K_KERNEL_THREAD_DEFINE(span_microstep_tid, SPAN_THREAD_STACK_SIZE, span_microstep_loop, NULL, NULL, NULL, K_PRIO_COOP(0), K_ESSENTIAL, 0);

static void span_microstep_init_l(unsigned microsteps_remaining, int velocity_target, unsigned accel, unsigned actuator_mask,
        enum span_microstep_actuator_state actuator_state, enum span_microstep_loop_state loop_state) {
    span_ticks_remaining = 0;
    span_microsteps_remaining = microsteps_remaining;
    span_microstep_position_fraction = 0;
    span_microstep_velocity_actual = 0;
    span_microstep_velocity_target = velocity_target;
    span_microstep_accel = accel;
    for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
        span_microsteps_lost[i] = 0;
        span_microstep_actuator_state[i] = IS_BIT_SET(actuator_mask, i) ? actuator_state : SPAN_MICROSTEP_ACTUATOR_HALT;
    }
    span_microstep_loop_state = loop_state;
}

static void span_microstep_halt_l(void) {
    span_microstep_init_l(0, 0, 0, 0, SPAN_MICROSTEP_ACTUATOR_HALT, SPAN_MICROSTEP_LOOP_HALT);
}

static void span_microstep_run_l(unsigned microsteps_remaining, int velocity_target, unsigned accel, unsigned actuator_mask,
        enum span_microstep_actuator_state actuator_state) {
    span_microstep_init_l(microsteps_remaining, velocity_target, accel, actuator_mask, actuator_state, SPAN_MICROSTEP_LOOP_RUN);
}

static void span_microstep_feed_l(void) {
    span_ticks_remaining = SPAN_WATCHDOG_TIMEOUT_TICKS;
}

static int span_prepare_l(bool start, unsigned actuator_mask, bool clear_fault) {
    static bool actual_start;
    static unsigned actual_actuator_mask;

    int err = 0;
    if (start) {
        if (!actual_start) {
            if ((err = drv8434s_start(span_stepper_dev, SPAN_NUM_ACTUATORS))) {
                return err;
            }
            counter_start(span_counter_dev); // cannot fail
            actual_start = true;
        }
        if (actual_actuator_mask != actuator_mask || clear_fault) {
            struct drv8434s_options options = {
                .status_buf = NULL,
                .clear_fault = clear_fault,
            };
            if ((err = drv8434s_set_output_enable(span_stepper_dev, &options, actuator_mask))) {
                return err;
            }
            actual_actuator_mask = actuator_mask;
        }
    } else {
        if (actual_start) {
            counter_stop(span_counter_dev); // cannot fail
            if ((err = drv8434s_stop(span_stepper_dev))) {
                return err;
            }
            actual_start = false;
        }
    }
    return err;
}

enum span_action_state {
    SPAN_ACTION_ABORT = -1,
    SPAN_ACTION_SLEEP_DONE = 0,
    SPAN_ACTION_STANDBY_DONE = 1,
    SPAN_ACTION_HOME_TRAVEL = 2,
    SPAN_ACTION_HOME_RELIEF = 3,
    SPAN_ACTION_HOME_DONE = 4,
    SPAN_ACTION_JOG_TRAVEL = 5,
};
static enum span_action_state span_action_state; // not guarded by semaphore

static enum span_position span_position; // not guarded by semaphore

int span_init(void) {
    struct counter_top_cfg cfg = {
        .ticks = counter_us_to_ticks(span_counter_dev, SPAN_MICROSTEP_TICK_PERIOD_US),
        .callback = span_counter_tick,
        .user_data = NULL,
        .flags = 0,
    };
    int err;
    if ((err = counter_set_top_value(span_counter_dev, &cfg))) {
        return err;
    }
    span_action_state = SPAN_ACTION_SLEEP_DONE;
    span_position = SPAN_POSITION_UNKNOWN;
    return 0;
}

enum span_position span_get_position(void) {
    return span_position;
}

int span_poll_sleep(void) {
    if (span_action_state == SPAN_ACTION_SLEEP_DONE) {
        return 0;
    }

    int err;
    k_sem_take(&span_state_sem, K_FOREVER);

    span_microstep_halt_l();
    if ((err = span_prepare_l(false, 0, true))) {
        span_action_state = SPAN_ACTION_ABORT;
        goto give_state_sem_and_return;
    }
    span_action_state = SPAN_ACTION_SLEEP_DONE;

give_state_sem_and_return:
    k_sem_give(&span_state_sem);
    return err;
}

int span_poll_standby(void) {
    if (span_action_state == SPAN_ACTION_STANDBY_DONE) {
        return 0;
    }

    int err;
    k_sem_take(&span_state_sem, K_FOREVER);

    span_microstep_halt_l();
    if ((err = span_prepare_l(true, 0, true))) {
        span_action_state = SPAN_ACTION_ABORT;
        goto give_state_sem_and_return;
    }
    span_action_state = SPAN_ACTION_STANDBY_DONE;

give_state_sem_and_return:
    k_sem_give(&span_state_sem);
    return err;
}

static int span_poll_home(bool extend) {
    static bool actual_extend;

    if (actual_extend != extend) {
        span_action_state = SPAN_ACTION_ABORT;
        actual_extend = extend;
    }
    if (span_action_state == SPAN_ACTION_HOME_DONE) {
        return 0;
    }
    span_position = SPAN_POSITION_UNKNOWN;

    int err;
    k_sem_take(&span_state_sem, K_FOREVER);

    if (span_action_state != SPAN_ACTION_HOME_TRAVEL && span_action_state != SPAN_ACTION_HOME_RELIEF) {
        span_microstep_run_l(SPAN_MAX_TRAVEL_MICROSTEPS,
            extend ? SPAN_COAST_SPEED_FRACTION : -SPAN_COAST_SPEED_FRACTION, SPAN_ACCEL_FRACTION,
            SPAN_ALL_ACTUATOR_BITS, SPAN_MICROSTEP_ACTUATOR_RUN_PAIR);
        if ((err = span_prepare_l(true, SPAN_ALL_ACTUATOR_BITS, true))) {
            span_microstep_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_HOME_TRAVEL;
    }
    if (span_action_state == SPAN_ACTION_HOME_TRAVEL) {
        if (span_microstep_loop_state == SPAN_MICROSTEP_LOOP_RUN) {
            span_microstep_feed_l();
            err = 1; // in progress
            goto give_state_sem_and_return;
        }
        if (span_microstep_loop_state != SPAN_MICROSTEP_LOOP_DONE) {
            err = -ECANCELED; // failed
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
            if (span_microstep_actuator_state[i] != SPAN_MICROSTEP_ACTUATOR_STALL) {
                err = -ECANCELED; // did not stall at endstop as expected
                span_action_state = SPAN_ACTION_ABORT;
                goto give_state_sem_and_return;
            }
        }
        span_microstep_run_l(SPAN_RELIEF_MICROSTEPS,
            extend ? -SPAN_JOG_SPEED_FRACTION : SPAN_JOG_SPEED_FRACTION, SPAN_ACCEL_FRACTION,
            SPAN_ALL_ACTUATOR_BITS, SPAN_MICROSTEP_ACTUATOR_RUN_PAIR);
        if ((err = span_prepare_l(true, SPAN_ALL_ACTUATOR_BITS, true))) {
            span_microstep_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_HOME_RELIEF;
    }
    if (span_action_state == SPAN_ACTION_HOME_RELIEF) {
        if (span_microstep_loop_state == SPAN_MICROSTEP_LOOP_RUN) {
            span_microstep_feed_l();
            err = 1; // in progress
            goto give_state_sem_and_return;
        }
        if (span_microstep_loop_state != SPAN_MICROSTEP_LOOP_DONE) {
            err = -ECANCELED; // failed
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_microstep_halt_l();
        if ((err = span_prepare_l(false, 0, true))) {
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_HOME_DONE;
        span_position = extend ? SPAN_POSITION_EXTENDED : SPAN_POSITION_RETRACTED;
    }
    err = 0; // done

give_state_sem_and_return:
    k_sem_give(&span_state_sem);
    return err;
}

int span_poll_extend(void) {
    return span_poll_home(true);
}

int span_poll_retract(void) {
    return span_poll_home(false);
}

int span_poll_jog(bool extend, unsigned actuator_mask) {
    static bool actual_extend;
    static unsigned actual_actuator_mask;

    if (actual_extend != extend || actual_actuator_mask != actuator_mask) {
        span_action_state = SPAN_ACTION_ABORT;
        actual_extend = extend;
        actual_actuator_mask = actuator_mask;
    }
    span_position = SPAN_POSITION_UNKNOWN;

    int err;
    k_sem_take(&span_state_sem, K_FOREVER);

    if (span_action_state != SPAN_ACTION_JOG_TRAVEL) {
        span_microstep_run_l(SPAN_JOG_MICROSTEPS,
            extend ? SPAN_JOG_SPEED_FRACTION : -SPAN_JOG_SPEED_FRACTION, SPAN_ACCEL_FRACTION,
            actuator_mask, SPAN_MICROSTEP_ACTUATOR_RUN_INDEPENDENT);
        if ((err = span_prepare_l(true, actuator_mask, true))) {
            span_microstep_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_JOG_TRAVEL;
    }
    span_microstep_feed_l();
    err = 1; // in progress

give_state_sem_and_return:
    k_sem_give(&span_state_sem);
    return err;
}
