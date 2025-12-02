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
 * Both pairs of actuators accelerate and travel at the same speed so they are all driven by the same timer.
 * The step loop blocks until released by a timer tick then it checks the actuator status report
 * from the previous iteration and asynchronously issues a new batch of steps for all of the actuators.
 * Note that there are two timer ticks of latency between issuing a batch of steps and observing the
 * resulting status report. The latency is small in practice and it is a conscious trade-off to halve
 * the number of context switches for the step loop thread compared to synchronously issuing batches
 * of steps and checking the status immediately thereafter.
 * 
 * In case of emergencies, actuators can also be controlled individually or in groups to bring them back
 * into synchronization.
 * 
 * This implementation appears to work best with SPI async and interrupts enabled. Using DMA results in
 * twice as many context switches, unfortunately.
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/drv8434s.h>

#include "bed/span.h"

// When set to 1, ignore open-load faults when some actuators are not connected
#define SPAN_DEBUG_IGNORE_OL_FAULT 1

// When set to 1, enables stall learning
// Results can only be observed with the debugger because logging is disabled
#define SPAN_DEBUG_STALL_LEARN 0

#define SPAN_NUM_ACTUATORS (4)
#define SPAN_ALL_ACTUATORS_SET (BIT(0) | BIT(1) | BIT(2) | BIT(3))

// The maximum tick frequency is limited by context switch speed; if it's too fast
// then the step loop may starve other threads. This value is empirically determined.
#define SPAN_MAX_TICK_FREQUENCY (3000)

// The minimum tick frequency is limited by the 16 bit timer resolution and the
// prescale factor in the device tree.
#define SPAN_MIN_TICK_FREQUENCY (100)

// The microstep divisor for intermediate calculations involving fractions of steps.
#define SPAN_MICROSTEP_DIVISOR 256

// Linear actuator travel parameters.
#define SPAN_MM_PER_STEP (0.04)
#define SPAN_MICROSTEPS_PER_MM (SPAN_MICROSTEP_DIVISOR * 1. / SPAN_MM_PER_STEP)
#define SPAN_MM_TO_MICROSTEPS(mm) (int)(SPAN_MICROSTEPS_PER_MM * mm)

// Speed and acceleration factors.
#define SPAN_INITIAL_SPEED_MICROSTEPS_PER_S (SPAN_MM_TO_MICROSTEPS(1.))
#define SPAN_COAST_SPEED_MICROSTEPS_PER_S (SPAN_MM_TO_MICROSTEPS(25.))
#define SPAN_JOG_SPEED_MICROSTEPS_PER_S (SPAN_MM_TO_MICROSTEPS(4.))
#define SPAN_ACCEL_MICROSTEPS_PER_S2 (SPAN_MM_TO_MICROSTEPS(10.))

// Step loop watchdog timeout.
#define SPAN_WATCHDOG_TIMEOUT_US (20000)

// The maximum distance that the actuators are expected to travel from one endstop to the other.
// Constrains how long the actuators can run before an error is reported in case of failure.
#define SPAN_MAX_TRAVEL_MICROSTEPS SPAN_MM_TO_MICROSTEPS(250.)

// The distance to move the actuators after each jog command.
#define SPAN_JOG_MICROSTEPS SPAN_MM_TO_MICROSTEPS(0.5)

// The distance to move the actuators back from their endstops to remove strain on the lead screw.
#define SPAN_RELIEF_MICROSTEPS SPAN_MM_TO_MICROSTEPS(3.)

// The distance that one actuator is allowed to continue traveling after its peer in a pair
// has stalled (presumably at an endstop). Constrains how far the actuators can become desynchronized.
#define SPAN_MAX_DESYNC_MICROSTEPS SPAN_MM_TO_MICROSTEPS(3.)

static const struct device *span_stepper_dev = DEVICE_DT_GET(DT_NODELABEL(span_stepper));
static const struct device *span_counter_dev = DEVICE_DT_GET(DT_NODELABEL(span_counter));

// This semaphore guards the global state of this component and access to the device drivers.
K_SEM_DEFINE(span_state_sem, 1, 1);

// This semaphore blocks the step loop until the next timer tick occurs.
K_SEM_DEFINE(span_tick_sem, 0, 1);

// The counter frequency.
static unsigned span_counter_frequency;

// The step direction.
static bool span_loop_dir;

// The number of microsteps per second for which the loop has been configured.
// Set to 0 when the timer is disabled.
static unsigned span_loop_speed;

// The microstep mode for each step taken in the loop.
static enum drv8434s_microstep_mode span_loop_microstep_mode;

// The number of microsteps each iteration of the loop counts for.
static unsigned span_loop_microstep_increment;

// Decremented as microsteps are issued.
// When this counter reaches zero, the loop transitions to the done state.
static unsigned span_loop_microsteps_remaining;

// Decremented by one each time the step loop handles a tick of the timer.
// At zero, the loop stops the actuators and reports an error.
// Behaves as a watchdog to ensure that the main loop is still in control.
static unsigned span_loop_ticks_remaining;

// Reload value for span_loop_ticks_remaining.
static unsigned span_loop_ticks_reload;

// State of the microstep loop. The loop only drives the actuators when in the RUN state.
enum span_loop_state {
    SPAN_LOOP_RUN = 1,      // Run actuators
    SPAN_LOOP_HALT = 0,     // Halt actuators
    SPAN_LOOP_TIMEOUT = -1, // The ticks left counter reached zero while the loop was running
    SPAN_LOOP_ERROR = -2,   // An error occurred while issuing steps to the driver
    SPAN_LOOP_FAULT = -3,   // One or more actuators reported a fault
    SPAN_LOOP_DESYNC = -4,  // One or more actuators lost synchronization with its peer
    SPAN_LOOP_DONE = -5,    // All actuators are halted, stalled, or zero microsteps remaining
};
static enum span_loop_state span_loop_state;

// State of each actuator. The loop only drives actuators that are in one of the RUN_* states.
enum span_actuator_state {
    SPAN_ACTUATOR_RUN_PAIR = 2,        // Run actuator and keep synchronized with its peer
    SPAN_ACTUATOR_RUN_INDEPENDENT = 1, // Run actuator independently of its peer
    SPAN_ACTUATOR_HALT = 0,            // Halt actuator
    SPAN_ACTUATOR_STALL = -1,          // The actuator stalled
    SPAN_ACTUATOR_FAULT = -2,          // The actuator faulted
    SPAN_ACTUATOR_DESYNC = -3,         // The actuator lost synchronization with its peer
};
static enum span_actuator_state span_actuator_state[SPAN_NUM_ACTUATORS];

// The number of microsteps each actuator has lost steps due to a stall or other fault.
// When this counter exceeds SPAN_MAX_DESYNC_MICROSTEPS, the actuator's peer is considered desynchronized.
static unsigned span_actuator_microsteps_lost[SPAN_NUM_ACTUATORS];

// The control input velocity in microsteps per second. Sign indicates stepping direction.
static int span_control_velocity_actual;
static int span_control_velocity_target;

// The control input acceleration in microsteps per second squared.
static unsigned span_control_accel;

// Reference time for applying acceleration since the last update.
static int64_t span_control_reftime;

static void span_counter_tick(const struct device *dev, void *user_data) {
    k_sem_give(&span_tick_sem);
}

#ifdef CONFIG_SPI_ASYNC
static void span_microstep_complete(const struct device *dev, int result, void *user_data) {
    int *result_ptr = user_data;
    *result_ptr = result;
    k_sem_give(&span_state_sem);
}
#endif

static void span_loop(void *, void *, void *) {
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
        if (span_loop_state <= SPAN_LOOP_HALT) {
            goto give_state_sem_and_continue;
        }
        if (last_step_result < 0) {
            span_loop_state = SPAN_LOOP_ERROR;
            goto give_state_sem_and_continue;
        }
        if (span_loop_ticks_remaining == 0) {
            span_loop_state = SPAN_LOOP_TIMEOUT;
            goto give_state_sem_and_continue;
        }
        span_loop_ticks_remaining -= 1;

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
                    span_actuator_microsteps_lost[i] += span_loop_microstep_increment;
                }
                if (status != DRV8434S_STATUS_STL) {
                    span_actuator_state[i] = SPAN_ACTUATOR_FAULT;
                    have_fault = true;
                } else if (span_actuator_state[i] > SPAN_ACTUATOR_HALT) {
                    span_actuator_state[i] = SPAN_ACTUATOR_STALL;
                }
            }
        }
        if (have_fault) {
            span_loop_state = SPAN_LOOP_FAULT;
            goto give_state_sem_and_continue;
        }

        // Nothing to do if the speed is zero.
        if (!span_loop_speed) {
            goto give_state_sem_and_continue;
        }

        // Update remaining step count.
        if (span_loop_microsteps_remaining < span_loop_microstep_increment) {
            if (span_loop_microsteps_remaining == 0) {
                span_loop_state = SPAN_LOOP_DONE;
                goto give_state_sem_and_continue;
            }
            span_loop_microsteps_remaining = 0;
        } else {
            span_loop_microsteps_remaining -= span_loop_microstep_increment;
        }

        // Prepare step requests and ensure actuator pair synchronization.
        have_steps = false;
        for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
            bool step = false;
            switch (span_actuator_state[i]) {
                case SPAN_ACTUATOR_RUN_INDEPENDENT:
                    step = true;
                    break;
                case SPAN_ACTUATOR_RUN_PAIR:
                    if (span_actuator_state[i ^ 1] == SPAN_ACTUATOR_RUN_PAIR &&
                            span_actuator_microsteps_lost[i ^ 1] < SPAN_MAX_DESYNC_MICROSTEPS) {
                        step = true;
                        break;
                    }
                    span_actuator_state[i] = SPAN_ACTUATOR_DESYNC;
                    __fallthrough;
                case SPAN_ACTUATOR_DESYNC:
                    span_loop_state = SPAN_LOOP_DESYNC;
                    goto give_state_sem_and_continue;
                case SPAN_ACTUATOR_HALT:
                case SPAN_ACTUATOR_STALL:
                    break;
                case SPAN_ACTUATOR_FAULT:
                default:
                    span_loop_state = SPAN_LOOP_FAULT;
                    goto give_state_sem_and_continue;
            }
            pending_step_requests[i] = drv8434s_make_step_request(step, span_loop_dir, span_loop_microstep_mode);
            have_steps |= step;
        }
        if (!have_steps) {
            span_loop_state = SPAN_LOOP_DONE;
            goto give_state_sem_and_continue;
        }

        // Issue step requests.
#ifdef CONFIG_SPI_ASYNC
        if ((err = drv8434s_step_async(span_stepper_dev, &pending_step_options, pending_step_requests,
                span_microstep_complete, &pending_step_result))) {
            span_loop_state = SPAN_LOOP_ERROR;
            goto give_state_sem_and_continue;
        }
        // Await result on next tick. Keep holding the state semaphore until the step complete callback runs.
        continue;
#else
        if ((err = drv8434s_step(span_stepper_dev, &pending_step_options, pending_step_requests))) {
            pending_step_result = err;
            span_loop_state = SPAN_LOOP_ERROR;
            goto give_state_sem_and_continue;
        }
        pending_step_result = 0;
#endif

        // Release the state semaphore when not awaiting an async callback.
    give_state_sem_and_continue:
        k_sem_give(&span_state_sem);
    }
}
#define SPAN_THREAD_STACK_SIZE 1024
K_KERNEL_THREAD_DEFINE(span_loop_tid, SPAN_THREAD_STACK_SIZE, span_loop, NULL, NULL, NULL, K_PRIO_COOP(0), K_ESSENTIAL, 0);

static void span_loop_update_l(void) {
    int time_delta = k_uptime_delta(&span_control_reftime);
    if (span_control_velocity_actual < span_control_velocity_target) {
        span_control_velocity_actual += time_delta * span_control_accel / 1000;
        if (span_control_velocity_actual > span_control_velocity_target) {
            span_control_velocity_actual = span_control_velocity_target;
        }
    } else if (span_control_velocity_actual > span_control_velocity_target) {
        span_control_velocity_actual -= time_delta * span_control_accel / 1000;
        if (span_control_velocity_actual < span_control_velocity_target) {
            span_control_velocity_actual = span_control_velocity_target;
        }
    }

#if SPAN_DEBUG_STALL_LEARN
    if (span_control_velocity_actual) {
        struct drv8434s_options options = {
            .status_buf = NULL,
            .clear_fault = false,
        };
        uint64_t learned_set = 0;
        uint16_t stall_th_buf[SPAN_NUM_ACTUATORS];
        int err = drv8434s_get_stall_learn_result(span_stepper_dev, &options, &learned_set, stall_th_buf);
        if (!err && learned_set) {
            static volatile bool x;
            x = true;
        }
    }
#endif

    unsigned speed = abs(span_control_velocity_actual);
    if (speed >= SPAN_MIN_TICK_FREQUENCY) {
        if (span_loop_speed != speed) {
            unsigned scale = 0;
            unsigned frequency = speed;
            while (frequency > SPAN_MAX_TICK_FREQUENCY) {
                frequency >>= 1;
                scale += 1;
            }
            span_loop_speed = speed;
            span_loop_microstep_mode = _CONCAT(DRV8434S_MICROSTEP_, SPAN_MICROSTEP_DIVISOR) - scale;
            if (span_loop_microstep_mode < DRV8434S_MICROSTEP_2) {
                // TODO: Report an error in case the speed is unachievable.
                span_loop_microstep_mode = DRV8434S_MICROSTEP_1;
            }
            span_loop_microstep_increment = (1 << scale);
            span_loop_ticks_reload = (SPAN_WATCHDOG_TIMEOUT_US * frequency / 1000000) + 1;
            struct counter_top_cfg cfg = {
                .ticks = span_counter_frequency / frequency,
                .callback = span_counter_tick,
                .user_data = NULL,
                .flags = COUNTER_TOP_CFG_DONT_RESET | COUNTER_TOP_CFG_RESET_WHEN_LATE,
            };
            int err = counter_set_top_value(span_counter_dev, &cfg);
            if (err == -EINVAL) {
                // The frequency is too low
                counter_stop(span_counter_dev); // never fails
            }
            counter_start(span_counter_dev); // never fails
        }
        span_loop_dir = span_control_velocity_actual > 0;
        span_loop_ticks_remaining = span_loop_ticks_reload;
    } else {
        span_loop_speed = 0;
        span_loop_ticks_remaining = 1;
        counter_stop(span_counter_dev); // never fails
    }
}

static void span_loop_init_l(unsigned microsteps_remaining, int velocity_actual, int velocity_target, unsigned accel,
        unsigned actuator_set, enum span_actuator_state actuator_state, enum span_loop_state loop_state) {
    span_loop_microsteps_remaining = microsteps_remaining;
    span_control_velocity_actual = velocity_actual;
    span_control_velocity_target = velocity_target;
    span_control_accel = accel;
    for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
        span_actuator_microsteps_lost[i] = 0;
        span_actuator_state[i] = IS_BIT_SET(actuator_set, i) ? actuator_state : SPAN_ACTUATOR_HALT;
    }
    span_loop_state = loop_state;
    span_control_reftime = k_uptime_get();
    span_loop_update_l();
}

static void span_loop_halt_l(void) {
    span_loop_init_l(0, 0, 0, 0, 0, SPAN_ACTUATOR_HALT, SPAN_LOOP_HALT);
}

static void span_loop_run_l(unsigned microsteps_remaining, int velocity_actual, int velocity_target, unsigned accel,
        unsigned actuator_set, enum span_actuator_state actuator_state) {
    span_loop_init_l(microsteps_remaining, velocity_actual, velocity_target, accel, actuator_set, actuator_state, SPAN_LOOP_RUN);
}

static int span_prepare_actuators_l(bool start, unsigned actuator_set, bool clear_fault) {
    static bool actual_start;
    static unsigned actual_actuator_set;

    int err = 0;
    if (start) {
        if (!actual_start) {
            if ((err = drv8434s_start(span_stepper_dev, SPAN_NUM_ACTUATORS))) {
                return err;
            }
            actual_start = true;
        }
        if (actual_actuator_set != actuator_set || clear_fault) {
            struct drv8434s_options options = {
                .status_buf = NULL,
                .clear_fault = clear_fault,
            };
            if ((err = drv8434s_set_output_enable(span_stepper_dev, &options, actuator_set))) {
                return err;
            }
#if SPAN_DEBUG_STALL_LEARN
            if ((err = drv8434s_set_stall_learn_mode(span_stepper_dev, &options, true))) {
                return err;
            }
#endif
            actual_actuator_set = actuator_set;
        }
    } else {
        if (actual_start) {
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
    span_counter_frequency = counter_get_frequency(span_counter_dev);
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

    span_loop_halt_l();
    if ((err = span_prepare_actuators_l(false, 0, true))) {
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

    span_loop_halt_l();
    if ((err = span_prepare_actuators_l(true, 0, true))) {
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
        span_loop_run_l(SPAN_MAX_TRAVEL_MICROSTEPS,
            extend ? SPAN_INITIAL_SPEED_MICROSTEPS_PER_S : -SPAN_INITIAL_SPEED_MICROSTEPS_PER_S,
            extend ? SPAN_COAST_SPEED_MICROSTEPS_PER_S : -SPAN_COAST_SPEED_MICROSTEPS_PER_S,
            SPAN_ACCEL_MICROSTEPS_PER_S2,
            SPAN_ALL_ACTUATORS_SET, SPAN_ACTUATOR_RUN_PAIR);
        if ((err = span_prepare_actuators_l(true, SPAN_ALL_ACTUATORS_SET, true))) {
            span_loop_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_HOME_TRAVEL;
    }
    if (span_action_state == SPAN_ACTION_HOME_TRAVEL) {
        if (span_loop_state == SPAN_LOOP_RUN) {
            span_loop_update_l();
            err = 1; // in progress
            goto give_state_sem_and_return;
        }
        if (span_loop_state != SPAN_LOOP_DONE) {
            err = -ECANCELED; // failed
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
            if (span_actuator_state[i] != SPAN_ACTUATOR_STALL) {
                err = -ECANCELED; // did not stall at endstop as expected
                span_action_state = SPAN_ACTION_ABORT;
                goto give_state_sem_and_return;
            }
        }
        span_loop_run_l(SPAN_RELIEF_MICROSTEPS,
            extend ? -SPAN_INITIAL_SPEED_MICROSTEPS_PER_S : SPAN_INITIAL_SPEED_MICROSTEPS_PER_S,
            extend ? -SPAN_JOG_SPEED_MICROSTEPS_PER_S : SPAN_JOG_SPEED_MICROSTEPS_PER_S,
            SPAN_ACCEL_MICROSTEPS_PER_S2,
            SPAN_ALL_ACTUATORS_SET, SPAN_ACTUATOR_RUN_PAIR);
        if ((err = span_prepare_actuators_l(true, SPAN_ALL_ACTUATORS_SET, true))) {
            span_loop_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_HOME_RELIEF;
    }
    if (span_action_state == SPAN_ACTION_HOME_RELIEF) {
        if (span_loop_state == SPAN_LOOP_RUN) {
            span_loop_update_l();
            err = 1; // in progress
            goto give_state_sem_and_return;
        }
        if (span_loop_state != SPAN_LOOP_DONE) {
            err = -ECANCELED; // failed
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_loop_halt_l();
        if ((err = span_prepare_actuators_l(false, 0, true))) {
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

int span_poll_jog(bool extend, unsigned actuator_set) {
    static bool actual_extend;
    static unsigned actual_actuator_set;

    if (actual_extend != extend || actual_actuator_set != actuator_set) {
        span_action_state = SPAN_ACTION_ABORT;
        actual_extend = extend;
        actual_actuator_set = actuator_set;
    }
    span_position = SPAN_POSITION_UNKNOWN;

    int err;
    k_sem_take(&span_state_sem, K_FOREVER);

    if (span_action_state != SPAN_ACTION_JOG_TRAVEL) {
        span_loop_run_l(SPAN_JOG_MICROSTEPS,
            extend ? SPAN_INITIAL_SPEED_MICROSTEPS_PER_S : -SPAN_INITIAL_SPEED_MICROSTEPS_PER_S,
            extend ? SPAN_JOG_SPEED_MICROSTEPS_PER_S : -SPAN_JOG_SPEED_MICROSTEPS_PER_S,
            SPAN_ACCEL_MICROSTEPS_PER_S2,
            actuator_set, SPAN_ACTUATOR_RUN_INDEPENDENT);
        if ((err = span_prepare_actuators_l(true, actuator_set, true))) {
            span_loop_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_JOG_TRAVEL;
    }
    span_loop_update_l();
    err = 1; // in progress

give_state_sem_and_return:
    k_sem_give(&span_state_sem);
    return err;
}
