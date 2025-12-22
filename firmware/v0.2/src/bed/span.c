/**
 * @file Component to extend and retract the wing spans of the bed.
 *
 * The bed wings each physically couple one pairs of linear actuators driven by stepper motors. Both actuators
 * in a pair move at the same speed in the same direction until on or both actuators stall or are stopped.
 * To prevent damage to the frame in case of malfunction, when one actuator in a pair stalls, it is commanded
 * to hold position while its sibling performs a limited number of additional steps towards its end stop.
 * The component stops all actuators and reports an error if both actuators in a pair do not achieve their
 * end stop together or when faults other than a stall occur.
 *
 * Both pairs of actuators accelerate and travel at the same speed so they are all driven by the same step
 * pulse. In case of emergencies, actuators can also be jogged individually or in groups to bring them
 * back into synchronization while the remaining actuator outputs are disabled.
 *
 * The main thread calls the span_poll_* functions to incrementally update the state of the actuators and
 * monitor their progress. A watchdog halts movement automatically if the main thread stops polling.
 *
 * The step thread runs at cooperative priority to update the velocity vector; set the step rate, direction,
 * and microstep mode; and handle faults. Stall detection requires precise and consistent timing of steps so
 * the step pulses are sent to the DRV8434S STEP pin with PWM in hardware instead of over SPI. Using PWM
 * allows for high frequency stepping with minimal jitter which improves acoustics but it also comes with
 * the risk of runaway motor operation in case of certain software failures so an ISR counts steps and stops
 * PWM when the steps remaining counter is exhausted.
 *
 * The step thread blocks until released by a timer tick or fault interrupt. It issues SPI step requests
 * asynchronously to reduce context switch overhead.
 *
 * A previous version of this module sent step pulses via SPI instead of with a PWM GPIO but it was limited
 * to about 3000 steps per second due to context switch overhead and there was too much jitter for DRV8434S
 * stall detection to work.
 */

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32c0xx_hal.h>
#include <stm32c0xx_ll_tim.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/drv8434s.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "bed/span.h"

#define SPAN_NUM_ACTUATORS (4)

// Disables the actuators whose corresponding bits are set to prevent, prevents them
// from receiving step inputs, ignores faults, and operates their tandem peer in
// independent mode.  Can be used to work around missing or non-functional actuators.
#define SPAN_DEBUG_DISABLED_ACTUATORS (BIT(0))

// When set to 1, enables stall learning
// Results can only be observed with the debugger because logging is disabled.
#define SPAN_DEBUG_STALL_LEARN 0
#define SPAN_DEBUG_STALL_LEARN_ACTUATOR_SET (BIT(0) | BIT(1))

// When set to 1, configures the stall threshold to 0.
#define SPAN_DEBUG_STALL_DETECTION_DISABLED 0

// Establishes an unit of length equal to the distance traversed theoretically
// by one microstep with the given divisor. Note that the step loop may issue
// requests with multiples of this microstep size when moving at higher speeds.
#define SPAN_TRAVEL_MICROSTEP_DIVISOR 256

// Linear actuator travel parameters.
#define SPAN_MM_PER_STEP (0.04)
#define SPAN_TRAVEL_PER_MM (SPAN_TRAVEL_MICROSTEP_DIVISOR * 1. / SPAN_MM_PER_STEP)
#define SPAN_MM_TO_TRAVEL(mm) (int)(SPAN_TRAVEL_PER_MM * mm)

// The full distance that the actuators are expected to travel from one endstop to the other.
#define SPAN_FULL_TRAVEL (SPAN_MM_TO_TRAVEL(235.))

// The distance to move the actuators back from their endstops to remove strain on the lead screw.
#define SPAN_RELIEF_TRAVEL (SPAN_MM_TO_TRAVEL(3.))

// Additional travel to make up for inaccuracies and partial stalls.
#define SPAN_EXTRA_TRAVEL (SPAN_MM_TO_TRAVEL(15.))

// The distance that one actuator is allowed to continue traveling after its peer in a pair
// has stalled (presumably at an endstop). Constrains how far the actuators can become desynchronized.
// Transient stalls can also happen while in motion if the frame binds up slightly.
// For now, we rely on the user to recover from those situations.
#define SPAN_MAX_DESYNC_TRAVEL (SPAN_MM_TO_TRAVEL(10.))

// Describes one continuous movement and parameters for ramping the speed.
// The movement starts at a slow speed for the slow start distance, accelerates to a rapid speed,
// then decelerates to a slow speed for the slow end distance.
struct span_move_spec {
    // The total distance to travel (unless a stall is encountered).
    unsigned travel_total;

    // The distance to travel for slow start and stop.
    // Marks when the acceleration or deceleration begins; not when it ends.
    unsigned travel_slow_start;
    unsigned travel_slow_stop;

    // The speed and acceleration targets.
    unsigned speed_slow;   // travel units per second
    unsigned speed_rapid;  // travel units per second
    unsigned accel;        // travel units per second per second
};

#define SPAN_HOME_SLOW_START_TRAVEL (SPAN_MM_TO_TRAVEL(5.))
#define SPAN_HOME_SLOW_STOP_TRAVEL (SPAN_MM_TO_TRAVEL(10.))
#define SPAN_HOME_SPEED_SLOW (SPAN_MM_TO_TRAVEL(8.))
#define SPAN_HOME_SPEED_RAPID (SPAN_MM_TO_TRAVEL(30.))
#define SPAN_HOME_ACCEL (SPAN_MM_TO_TRAVEL(20.))

#define SPAN_DISTANCE(velocity_delta, accel) (int)(((int64_t)(velocity_delta) * (velocity_delta)) / (accel) / 2)

/*
static const struct span_move_spec span_move_spec_home = {
    .travel_total = SPAN_FULL_TRAVEL + SPAN_EXTRA_TRAVEL,
    .travel_slow_start = SPAN_HOME_SLOW_START_TRAVEL,
    .travel_slow_stop = SPAN_HOME_SLOW_STOP_TRAVEL + SPAN_EXTRA_TRAVEL + 
            SPAN_DISTANCE(SPAN_HOME_SPEED_RAPID - SPAN_HOME_SPEED_SLOW, SPAN_HOME_ACCEL),
    .speed_slow = SPAN_HOME_SPEED_SLOW,
    .speed_rapid = SPAN_HOME_SPEED_RAPID,
    .accel = SPAN_HOME_ACCEL,
};
*/
static const struct span_move_spec span_move_spec_home = {
    .travel_total = SPAN_FULL_TRAVEL + SPAN_EXTRA_TRAVEL,
    .travel_slow_start = 0,
    .travel_slow_stop = 0,
    .speed_slow = SPAN_MM_TO_TRAVEL(20.),
    .speed_rapid = SPAN_MM_TO_TRAVEL(20.),
    .accel = 0,
};

#define SPAN_RELIEF_SPEED (SPAN_MM_TO_TRAVEL(10.))

static const struct span_move_spec span_move_spec_relief = {
    .travel_total = SPAN_MM_TO_TRAVEL(4.),
    .travel_slow_start = 0,
    .travel_slow_stop = 0,
    .speed_slow = SPAN_RELIEF_SPEED,
    .speed_rapid = SPAN_RELIEF_SPEED,
    .accel = 0,
};

#define SPAN_JOG_SPEED (SPAN_MM_TO_TRAVEL(5.))

static const struct span_move_spec span_move_spec_jog = {
    .travel_total = SPAN_FULL_TRAVEL + SPAN_EXTRA_TRAVEL,
    .travel_slow_start = 0,
    .travel_slow_stop = 0,
    .speed_slow = SPAN_JOG_SPEED,
    .speed_rapid = SPAN_JOG_SPEED,
    .accel = 0,
};

// Gets the stall threshold based on the speed in microsteps per second.
// This value is sensitive to step speed and also to motor current limit.
// At 25 mm/s, stall learning yields a recommended threshold of 2000 but that value is clearly wrong.
//
// To test, set the initial speed to some value and the acceleration to zero.
// If the motor stalls prematurely, then lower the threshold.
// If the motor makes a lot of noise when reaching the end stop, then raise the threshold.
// If the span makes a metallic sound when starting and stopping, check the screw tension on the drive arms.
static inline uint16_t span_stall_threshold(unsigned speed) {
    if (SPAN_DEBUG_STALL_DETECTION_DISABLED) {
        return 0;
    }
    if (speed >= SPAN_MM_TO_TRAVEL(40.)) {
        return 1000;
    }
    if (speed >= SPAN_MM_TO_TRAVEL(30.)) {
        return 500;
    }
    if (speed >= SPAN_MM_TO_TRAVEL(20.)) {
        return 180;
    }
    if (speed >= SPAN_MM_TO_TRAVEL(15.)) {
        return 150;
    }
    if (speed >= SPAN_MM_TO_TRAVEL(10.)) {
        return 100;
    }
    if (speed >= SPAN_MM_TO_TRAVEL(5.)) {
        return 80;
    }
    if (speed >= SPAN_MM_TO_TRAVEL(3.)) {
        return 40;
    }
    return 30;
}

// This semaphore guards the global state of this component and access to the device drivers.
// Functions that require holding this semaphore have the `_l` suffix.
K_SEM_DEFINE(span_state_sem, 1, 1);

/*
 * STEP SEQUENCER: Generates a limited number of step pulses at a specified rate.
 */

static const struct device *span_stepper_dev = DEVICE_DT_GET(DT_NODELABEL(span_stepper));

#define SPAN_STEP_TIMER_NUMBER 17
#define SPAN_STEP_TIMER_NODE DT_NODELABEL(_CONCAT(timers, SPAN_STEP_TIMER_NUMBER))
#define SPAN_STEP_TIMER_CHANNEL LL_TIM_CHANNEL_CH1
static TIM_TypeDef * const span_step_timer = _CONCAT(TIM, SPAN_STEP_TIMER_NUMBER);
static const struct pwm_dt_spec span_step_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(span_step), 0);

// The maximum and minimum step rate in pulses per second.
// The maximum is limited by interrupt processing overhead (the DRV8434S allows up to 500 kHz step).
// The minimum is limited by the 16 bit timer resolution and prescale factor.
#define SPAN_STEP_RATE_MAX (18000)
#define SPAN_STEP_RATE_MIN (100)

// The minimum step duration in the DRV8434S datasheet is 1 us.
#define SPAN_STEP_PULSE_US (10)

_Static_assert(SPAN_NUM_ACTUATORS == 4);
union span_step_requests {
    uint8_t elem[SPAN_NUM_ACTUATORS];
    uint32_t packed;
};

struct span_step_data {
    uint64_t clock_rate;   // in cycles per second
    uint32_t pulse_width;  // in clock cycles

    unsigned step_rate;        // in pulses per second
    atomic_val_t step_count;   // number of steps to perform
    atomic_t steps_remaining;  // decrements down from step_count after each step, stops at 0
    union span_step_requests step_requests;  // requests sent to DRV8434S over SPI

    unsigned travel_per_step;   // distance traveled per step
    unsigned travel_total;      // distance traveled since the total was last reset
    unsigned travel_remaining;  // distance remaining to travel in travel units, populates steps_remaining
};
static struct span_step_data span_step_data;

// Note: This timer ISR has elevated interrupt priority. Be quick.
static void span_step_timer_isr(void *user_data) {
    if (LL_TIM_IsActiveFlag_UPDATE(span_step_timer)) {
        LL_TIM_ClearFlag_UPDATE(span_step_timer);
        if (LL_TIM_CC_IsEnabledChannel(span_step_timer, SPAN_STEP_TIMER_CHANNEL)) {
            atomic_val_t value = atomic_dec(&span_step_data.steps_remaining);
            if (value <= 1) {
                LL_TIM_CC_DisableChannel(span_step_timer, SPAN_STEP_TIMER_CHANNEL);
            }
        }
    }
}

static int span_step_timer_init_l(void) {
    // Precompute the pulse width.
    int err;
    if ((err = pwm_get_cycles_per_sec(span_step_pwm.dev, span_step_pwm.channel,
            &span_step_data.clock_rate))) {
        return err;
    }
    span_step_data.pulse_width = span_step_data.clock_rate * SPAN_STEP_PULSE_US / USEC_PER_SEC;

    // Prevent the motor from stepping while the debugger is stopped at a breakpoint.
    _CONCAT(__HAL_DBGMCU_FREEZE_TIM, SPAN_STEP_TIMER_NUMBER)();

    // Set up ISR to count pulses.
    IRQ_CONNECT(DT_IRQN(SPAN_STEP_TIMER_NODE),
        DT_IRQ(SPAN_STEP_TIMER_NODE, priority),
        span_step_timer_isr, NULL, 0);
    irq_enable(DT_IRQN(SPAN_STEP_TIMER_NODE));
    LL_TIM_EnableIT_UPDATE(span_step_timer);
    return 0;
}

static inline int span_step_timer_start_l() {
    __ASSERT(span_step_data.step_rate >= SPAN_MIN_STEP_RATE && span_step_data.step_rate <= SPAN_MAX_STEP_RATE, "");
    return pwm_set_cycles(span_step_pwm.dev, span_step_pwm.channel,
            span_step_data.clock_rate / span_step_data.step_rate, span_step_data.pulse_width, span_step_pwm.flags);
}

static inline int span_step_timer_stop_l(void) {
    return pwm_set_cycles(span_step_pwm.dev, span_step_pwm.channel, 0, 0, span_step_pwm.flags);
}

static void span_step_update_travel_l(void) {
    atomic_val_t steps_remaining = atomic_get(&span_step_data.steps_remaining);
    atomic_val_t steps_traveled = span_step_data.step_count - steps_remaining;
    span_step_data.step_count = steps_remaining;
    if (steps_traveled > 0) {
        unsigned travel_distance = steps_traveled * span_step_data.travel_per_step;
        span_step_data.travel_total += travel_distance;
        if (span_step_data.travel_remaining > travel_distance) {
            span_step_data.travel_remaining -= travel_distance;
        } else {
            span_step_data.travel_remaining = 0;
        }
    }
}

static void span_step_set_travel_l(unsigned travel_total, unsigned travel_remaining) {
    span_step_data.travel_total = travel_total;
    span_step_data.travel_remaining = travel_remaining;
}

static void span_step_update_steps_remaining_l(unsigned step_rate, unsigned travel_per_step) {
    LL_TIM_DisableIT_UPDATE(span_step_timer);
    span_step_update_travel_l();
    span_step_data.step_rate = step_rate;
    span_step_data.travel_per_step = travel_per_step; // after getting travel remaining
    span_step_data.steps_remaining = span_step_data.travel_remaining / travel_per_step;
    atomic_set(&span_step_data.step_count, span_step_data.steps_remaining);
    LL_TIM_EnableIT_UPDATE(span_step_timer);
}

static void span_step_async_complete_l(const struct device *dev, int result, void *user_data) {
    if (!result) {
        result = span_step_timer_start_l();
    }
    *(int *)user_data = result;
    k_sem_give(&span_state_sem);
}

// Updates the remaining step count and step rate.
// If the step requests have changed, asynchronously sends a message to DRV8434S and returns 1.
// Returns 0 when the call completes synchronously.
static int span_step_async_l(union span_step_requests step_requests, unsigned step_rate,
        unsigned travel_per_step, struct drv8434s_options *options, int *async_result) {
    if (step_rate < SPAN_STEP_RATE_MIN) {
        step_rate = 0;
    }

    int err;
    if (span_step_data.step_requests.packed != step_requests.packed) {
        if ((err = span_step_timer_stop_l())) {
            return err;
        }

        span_step_update_steps_remaining_l(step_rate, travel_per_step);

        if ((err = drv8434s_step_async(span_stepper_dev, options, step_requests.elem,
                span_step_async_complete_l, async_result))) {
            return err;
        }
        span_step_data.step_requests.packed = step_requests.packed;
        return 1;
    }

    span_step_update_steps_remaining_l(step_rate, travel_per_step);
    if ((err = span_step_timer_start_l())) {
        return err;
    }
    *async_result = 0;
    return 0;
}

// Reset the step sequencer after an error or when going to sleep.
static int span_step_halt_l() {
    span_step_data.step_requests.packed = 0;
    return span_step_timer_stop_l();
}

/*
 * STEP LOOP: Updates step velocity and handles faults.
 */

static const struct device *span_loop_tick_dev = DEVICE_DT_GET(DT_NODELABEL(span_loop_tick));

// This semaphore blocks the step loop until the next tick occurs.
K_SEM_DEFINE(span_loop_tick_sem, 0, 1);

// Step loop watchdog timeout to ensure the main loop remains in control of the movement.
#define SPAN_LOOP_TIMEOUT_US (20000)

// The frequency at which the loop ticks to update the velocity vector.
#define SPAN_LOOP_TICK_FREQUENCY (100)
#define SPAN_LOOP_TICK_PERIOD_US (USEC_PER_SEC / SPAN_LOOP_TICK_FREQUENCY)

// The motor stall condition is acknowledged when the stall fault remains active for a
// minimum duration of time in case it resolves itself after intermittent binding of
// one of the actuators in a pair.
#define SPAN_STALL_DURATION_US (100000)
#define SPAN_STALL_DURATION_TICKS (SPAN_STALL_DURATION_US / SPAN_LOOP_TICK_PERIOD_US)

enum span_loop_state {
    SPAN_LOOP_RUN = 1,                  // Run actuators
    SPAN_LOOP_HALT = 0,                 // Halt actuators
    SPAN_LOOP_ERROR = -1,               // An error occurred as indicated by loop_error
    SPAN_LOOP_DONE_TRAVEL = -2,         // Zero travel remaining
    SPAN_LOOP_DONE_HALT_OR_STALL = -3,  // All actuators have halted or stalled
};

enum span_actuator_state {
    SPAN_ACTUATOR_RUN_TANDEM = 2,      // Run actuator and keep synchronized with its peer
    SPAN_ACTUATOR_RUN_INDEPENDENT = 1, // Run actuator independently of its peer
    SPAN_ACTUATOR_HALT = 0,            // Halt actuator
    SPAN_ACTUATOR_STALL = -1,          // The actuator stalled
    SPAN_ACTUATOR_FAULT = -2,          // The actuator faulted
    SPAN_ACTUATOR_DESYNC = -3,         // The actuator lost synchronization with its peer
    SPAN_ACTUATOR_DISABLED = -4,       // The actuator is disabled
};

struct span_actuator_data {
    // State of the actuator. The loop only drives actuators that are in one of the RUN_* states.
    enum span_actuator_state state;

    // Incremented each time a stall fault is reported for the actuator to filter glitches.
    // When it exceeds SPAN_STALL_DURATION_TICKS, the actuator is deemed to have stalled for each.
    unsigned stall_ticks;

    // The travel total (position) of the actuator at the moment it encountered a stall.
    unsigned travel_total_at_stall;
};

struct span_loop_data {
    // State of the control loop. The loop only drives the actuators when in the RUN state.
    enum span_loop_state loop_state;

    // The SPAN_ERROR_* error associated with the SPAN_LOOP_ERROR state.
    int loop_error;

    // Decremented by one each time the step loop handles a tick of the timer.
    // At zero, the loop stops the actuators and reports an error.
    // Behaves as a watchdog to ensure that the main loop is still in control.
    unsigned ticks_remaining;

    // The movement being performed, or NULL if stopped.
    const struct span_move_spec *move_spec;

    // Whether the span is being extended.
    bool extend;

    // The current speed.
    unsigned speed;

    // Set to true to clear faults when restarting the loop after a stall.
    bool restart_after_stall;

    // State of each actuator.
    struct span_actuator_data actuator[SPAN_NUM_ACTUATORS];

#if SPAN_DEBUG_STALL_LEARN
    bool stall_learn_pending;
#endif
};
static struct span_loop_data span_loop_data;

static void span_loop_tick_handler(const struct device *dev, void *user_data) {
    k_sem_give(&span_loop_tick_sem);
}

static void span_loop(void *, void *, void *) {
    struct drv8434s_options options = {
        .status_buf = NULL,
        .clear_fault = false,
    };
    uint8_t faults[SPAN_NUM_ACTUATORS] = {0};
    const int STEP_NOT_PENDING = 1;
    int pending_step_result = STEP_NOT_PENDING;
    union span_step_requests pending_step_requests = {0};
    int err;

    for (;;) {
        // Wait for a tick; this semaphore is not released by this loop.
        k_sem_take(&span_loop_tick_sem, K_FOREVER);

        // Check step result from the last iteration.
        k_sem_take(&span_state_sem, K_FOREVER);
        const int last_step_result = pending_step_result;
        pending_step_result = STEP_NOT_PENDING;
        if (span_loop_data.loop_state <= SPAN_LOOP_HALT) {
            goto halt_and_continue;
        }
        if (last_step_result < 0) {
            span_loop_data.loop_state = SPAN_LOOP_ERROR;
            span_loop_data.loop_error = SPAN_ERROR_DRIVER;
            goto halt_and_continue;
        }
        span_step_update_travel_l();
        unsigned travel_total = span_step_data.travel_total;

        // Check for faults.
        bool clear_transient_fault = false;
        if (drv8434s_has_fault(span_stepper_dev)) {
            options.clear_fault = span_loop_data.restart_after_stall;
            if ((err = drv8434s_get_fault_status(span_stepper_dev, &options, faults))) {
                span_loop_data.loop_state = SPAN_LOOP_ERROR;
                span_loop_data.loop_error = SPAN_ERROR_DRIVER;
                goto halt_and_continue;
            }
            bool have_fault = false;
            for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
                if (span_loop_data.actuator[i].state == SPAN_ACTUATOR_DISABLED) {
                    continue;
                }
                uint8_t fault = faults[i];
                if (fault) {
                    if (fault == (DRV8434S_EX_STATUS_FAULT | DRV8434S_STATUS_STL)) {
                        if (!span_loop_data.restart_after_stall &&
                                span_loop_data.actuator[i].state > SPAN_ACTUATOR_HALT) {
                            span_loop_data.actuator[i].stall_ticks += 1;
                            if (span_loop_data.actuator[i].stall_ticks >= SPAN_STALL_DURATION_TICKS) {
                                span_loop_data.actuator[i].state = SPAN_ACTUATOR_STALL;
                                span_loop_data.actuator[i].travel_total_at_stall = travel_total;
                            } else {
                                clear_transient_fault = true;
                            }
                        }
                    } else {
                        span_loop_data.actuator[i].state = SPAN_ACTUATOR_FAULT;
                        have_fault = true;
                    }
                }
            }
            if (have_fault) {
                span_loop_data.loop_state = SPAN_LOOP_ERROR;
                span_loop_data.loop_error = SPAN_ERROR_FAULT;
                goto halt_and_continue;
            }
        } else {
            span_loop_data.restart_after_stall = false;
        }

        // Check for timeout.
        if (span_loop_data.ticks_remaining == 0) {
            span_loop_data.loop_state = SPAN_LOOP_ERROR;
            span_loop_data.loop_error = SPAN_ERROR_TIMEOUT;
            goto halt_and_continue;
        }
        span_loop_data.ticks_remaining -= 1;

        // Check for end of travel.
        if (span_step_data.travel_remaining == 0) {
            span_loop_data.loop_state = SPAN_LOOP_DONE_TRAVEL;
            goto halt_and_continue;
        }

        // Skip movement if waiting for the stall to clear.
        if (span_loop_data.restart_after_stall) {
            goto give_state_sem_and_continue;
        }

        // Update the speed.
        unsigned speed_target = 0;
        unsigned accel = 0;
        const struct span_move_spec *move_spec = span_loop_data.move_spec;
        if (move_spec) {
            accel = move_spec->accel;
            if (travel_total >= move_spec->travel_slow_start &&
                    travel_total + move_spec->travel_slow_stop <= move_spec->travel_total) {
                speed_target = move_spec->speed_rapid;
            } else {
                speed_target = move_spec->speed_slow;
            }
        }
        if (accel) {
            if (span_loop_data.speed < speed_target) {
                unsigned speed_delta = (uint64_t)SPAN_LOOP_TICK_PERIOD_US * accel / USEC_PER_SEC;
                span_loop_data.speed += MIN(speed_delta, speed_target - span_loop_data.speed);
            } else if (span_loop_data.speed > speed_target) {
                unsigned speed_delta = (uint64_t)SPAN_LOOP_TICK_PERIOD_US * accel / USEC_PER_SEC;
                span_loop_data.speed -= MIN(speed_delta, span_loop_data.speed - speed_target);
            }
        } else {
            span_loop_data.speed = speed_target;
        }

#if SPAN_DEBUG_STALL_LEARN
        if (span_loop_data.speed != speed_target || speed_target == 0) {
            span_loop_data.stall_learn_pending = false;
        } else if (!span_loop_data.stall_learn_pending) {
            span_loop_data.stall_learn_pending = true;
            drv8434s_set_stall_learn_active(span_stepper_dev, &options, SPAN_DEBUG_STALL_LEARN_ACTUATOR_SET);
        } else {
            uint64_t stall_learn_active_set = 0;
            uint64_t stall_learn_ok_set = 0;
            uint16_t stall_th_buf[SPAN_NUM_ACTUATORS];
            uint16_t trq_count_buf[SPAN_NUM_ACTUATORS];
            drv8434s_get_stall_learn_status(span_stepper_dev, &options, &stall_learn_active_set, &stall_learn_ok_set, stall_th_buf);
            drv8434s_get_trq_count(span_stepper_dev, &options, trq_count_buf);
            if (stall_learn_active_set != SPAN_DEBUG_STALL_LEARN_ACTUATOR_SET) {
                span_loop_data.stall_learn_pending = false;
                if (stall_learn_active_set) {
                    drv8434s_set_stall_learn_active(span_stepper_dev, &options, SPAN_DEBUG_STALL_LEARN_ACTUATOR_SET);
                }
            }
        }
#endif

        // Update the speed and microstep mode.
        const unsigned speed = span_loop_data.speed;
        unsigned step_scale = 0;
        unsigned step_rate = speed;
        unsigned travel_per_step;
        while (step_rate > SPAN_STEP_RATE_MAX) {
            step_rate >>= 1;
            step_scale += 1;
        }
        enum drv8434s_microstep_mode microstep_mode =
                _CONCAT(DRV8434S_MICROSTEP_, SPAN_TRAVEL_MICROSTEP_DIVISOR) - step_scale;
        if (microstep_mode < DRV8434S_MICROSTEP_2) {
            // TODO: Report an error in case the speed is unachievable.
            microstep_mode = DRV8434S_MICROSTEP_1;
        }
        travel_per_step = 1 << step_scale;

        // Prepare step requests and ensure actuator pair synchronization.
        bool have_steps = false;
        for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
            bool step = false;
            switch (span_loop_data.actuator[i].state) {
                case SPAN_ACTUATOR_RUN_INDEPENDENT:
                    step = true;
                    break;
                case SPAN_ACTUATOR_RUN_TANDEM: {
                    unsigned peer = i ^ 1;
                    enum span_actuator_state peer_state = span_loop_data.actuator[peer].state;
                    if (peer_state == SPAN_ACTUATOR_RUN_TANDEM || peer_state == SPAN_ACTUATOR_DISABLED ||
                            (peer_state == SPAN_ACTUATOR_STALL &&
                            travel_total - span_loop_data.actuator[peer].travel_total_at_stall < SPAN_MAX_DESYNC_TRAVEL)) {
                        step = true;
                        break;
                    }
                    span_loop_data.actuator[i].state = SPAN_ACTUATOR_DESYNC;
                    __fallthrough;
                }
                case SPAN_ACTUATOR_DESYNC:
                    span_loop_data.loop_state = SPAN_LOOP_ERROR;
                    span_loop_data.loop_error = SPAN_ERROR_DESYNC;
                    goto give_state_sem_and_continue;
                case SPAN_ACTUATOR_HALT:
                case SPAN_ACTUATOR_STALL:
                case SPAN_ACTUATOR_DISABLED:
                    break;
                case SPAN_ACTUATOR_FAULT:
                default:
                    span_loop_data.loop_state = SPAN_LOOP_ERROR;
                    span_loop_data.loop_error = SPAN_ERROR_FAULT;
                    goto give_state_sem_and_continue;
            }
            pending_step_requests.elem[i] = drv8434s_make_step_request(
                    step ? DRV8434S_STEP_PIN : DRV8434S_STEP_DISABLE,
                    span_loop_data.extend ? DRV8434S_DIR_LOW : DRV8434S_DIR_HIGH, microstep_mode);
            have_steps |= step;
        }
        if (!have_steps) {
            span_loop_data.loop_state = SPAN_LOOP_DONE_HALT_OR_STALL;
            goto halt_and_continue;
        }

        // Update the stall threshold if needed and issue the step requests.
        options.clear_fault = false;
        err = drv8434s_set_stall_threshold(span_stepper_dev, &options, span_stall_threshold(speed));
        if (!err) {
            options.clear_fault = clear_transient_fault;
            err = span_step_async_l(pending_step_requests, step_rate, travel_per_step, &options, &pending_step_result);
        }
        if (err < 0) {
            span_loop_data.loop_state = SPAN_LOOP_ERROR;
            span_loop_data.loop_error = SPAN_ERROR_DRIVER;
            goto halt_and_continue;
        }
        if (err == 1) {
            // Await result on next iteration.
            continue;
        }
        goto give_state_sem_and_continue;

        // Halt the step pulses, typically after an error.
    halt_and_continue:
        span_step_halt_l();

        // Release the state semaphore when not awaiting an async callback.
    give_state_sem_and_continue:
        k_sem_give(&span_state_sem);
    }
}
#define SPAN_THREAD_STACK_SIZE 1024
K_KERNEL_THREAD_DEFINE(span_loop_tid, SPAN_THREAD_STACK_SIZE, span_loop, NULL, NULL, NULL, K_PRIO_COOP(0), K_ESSENTIAL, 0);

static void span_loop_init_l(const struct span_move_spec *move_spec, bool extend,
        unsigned actuator_set, enum span_actuator_state actuator_state, enum span_loop_state loop_state) {
    span_step_set_travel_l(0, move_spec->travel_total);
    span_loop_data.move_spec = move_spec;
    span_loop_data.extend = extend;
    span_loop_data.speed = move_spec->speed_slow;
    for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
        span_loop_data.actuator[i].state = 
                IS_BIT_SET(SPAN_DEBUG_DISABLED_ACTUATORS, i) ? SPAN_ACTUATOR_DISABLED :
                IS_BIT_SET(actuator_set, i) ? actuator_state : SPAN_ACTUATOR_HALT;
        span_loop_data.actuator[i].stall_ticks = 0;
        span_loop_data.actuator[i].travel_total_at_stall = 0;
    }
    span_loop_data.loop_state = loop_state;
    span_loop_data.loop_error = 0;
    span_loop_data.restart_after_stall = false;
}

static inline void span_loop_halt_l(void) {
    span_loop_init_l(NULL, false, 0, SPAN_ACTUATOR_HALT, SPAN_LOOP_HALT);
}

static inline void span_loop_run_l(const struct span_move_spec *move_spec, bool extend,
        unsigned actuator_set, enum span_actuator_state actuator_state) {
    span_loop_init_l(move_spec, extend, actuator_set, actuator_state, SPAN_LOOP_RUN);
}

static void span_loop_feed_l(void) {
    span_loop_data.ticks_remaining = (SPAN_LOOP_TIMEOUT_US + SPAN_LOOP_TICK_PERIOD_US - 1) / SPAN_LOOP_TICK_PERIOD_US;
}

static int span_loop_await_done_l(void) {
    if (span_loop_data.loop_state > SPAN_LOOP_HALT) {
        span_loop_feed_l();
        return 1; // in progress
    }
    if (span_loop_data.loop_state != SPAN_LOOP_DONE_TRAVEL && span_loop_data.loop_state != SPAN_LOOP_DONE_HALT_OR_STALL) {
        return span_loop_data.loop_error;
    }
    return 0; // done
}

static int span_loop_prepare_actuators_l(bool start, unsigned actuator_set, bool clear_fault) {
    static bool actual_start;
    static unsigned actual_actuator_set;

    int err;
    if (start) {
        if (!actual_start) {
            if ((err = drv8434s_start(span_stepper_dev, SPAN_NUM_ACTUATORS))) {
                return SPAN_ERROR_DRIVER;
            }
            actual_start = true;
        }
        if (actual_actuator_set != actuator_set || clear_fault) {
            struct drv8434s_options options = {
                .status_buf = NULL,
                .clear_fault = clear_fault,
            };
            if ((err = drv8434s_set_output_enable(span_stepper_dev, &options, actuator_set))) {
                return SPAN_ERROR_DRIVER;
            }
            actual_actuator_set = actuator_set;
        }
        counter_start(span_loop_tick_dev); // never fails
    } else {
        span_step_halt_l();
        counter_stop(span_loop_tick_dev); // never fails
        if (actual_start) {
            if ((err = drv8434s_stop(span_stepper_dev))) {
                return SPAN_ERROR_DRIVER;
            }
            actual_start = false;
        }
    }
    return 0;
}

static bool span_loop_check_states_l(int expected_loop_state, int expected_actuator_state) {
    if (span_loop_data.loop_state != expected_loop_state) {
        return false;
    }
    for (unsigned i = 0; i < SPAN_NUM_ACTUATORS; i++) {
        if (span_loop_data.actuator[i].state != expected_actuator_state &&
                span_loop_data.actuator[i].state != SPAN_ACTUATOR_DISABLED) {
            return false;
        }
    }
    return true;
}

/*
 * CONTROLLER ENTRY POINTS
 */

enum span_action_state {
    SPAN_ACTION_ABORT = -1,
    SPAN_ACTION_SLEEP_DONE = 0,
    SPAN_ACTION_STANDBY_DONE = 1,
    SPAN_ACTION_HOME_TRAVEL = 2,
    SPAN_ACTION_HOME_RELIEF = 3,
    SPAN_ACTION_HOME_DONE = 4,
    SPAN_ACTION_JOG_TRAVEL = 5,
    SPAN_ACTION_JOG_DONE = 6,
};
static enum span_action_state span_action_state; // not guarded by semaphore

static enum span_position span_position; // not guarded by semaphore

int span_init(void) {
    int err;
    if ((err = span_step_timer_init_l())) {
        return err;
    }

    struct counter_top_cfg cfg = {
        .ticks = counter_get_frequency(span_loop_tick_dev) / SPAN_LOOP_TICK_FREQUENCY,
        .callback = span_loop_tick_handler,
        .user_data = NULL,
        .flags = 0,
    };
    if ((err = counter_set_top_value(span_loop_tick_dev, &cfg))) {
        return err;
    }

    span_action_state = SPAN_ACTION_SLEEP_DONE;
    span_position = SPAN_POSITION_UNKNOWN;
    return 0;
}

enum span_position span_get_position(void) {
    return span_position;
}

enum span_state span_get_state(void) {
    enum span_state result;
    k_sem_take(&span_state_sem, K_FOREVER);

    switch (span_loop_data.loop_state) {
        case SPAN_LOOP_HALT:
            result = SPAN_STATE_HALT;
            break;
        case SPAN_LOOP_RUN:
            if (span_loop_data.move_spec) {
                result = span_loop_data.extend ? SPAN_STATE_EXTEND : SPAN_STATE_RETRACT;
            } else {
                result = SPAN_STATE_HALT;
            }
            break;
        case SPAN_LOOP_DONE_TRAVEL:
        case SPAN_LOOP_DONE_HALT_OR_STALL:
            result = SPAN_STATE_DONE;
            break;
        default:
            result = SPAN_STATE_ERROR;
            break;
    }

    k_sem_give(&span_state_sem);
    return result;
}

int span_poll_sleep(void) {
    if (span_action_state == SPAN_ACTION_SLEEP_DONE) {
        return 0;
    }

    int err;
    k_sem_take(&span_state_sem, K_FOREVER);

    span_loop_halt_l();
    if ((err = span_loop_prepare_actuators_l(false, 0, true))) {
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
    if ((err = span_loop_prepare_actuators_l(true, 0, true))) {
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
        span_loop_run_l(&span_move_spec_home, extend, SPAN_ACTUATOR_SET_ALL, SPAN_ACTUATOR_RUN_TANDEM);
        if ((err = span_loop_prepare_actuators_l(true, SPAN_ACTUATOR_SET_ALL, true))) {
            span_loop_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_HOME_TRAVEL;
    }
    if (span_action_state == SPAN_ACTION_HOME_TRAVEL) {
        if ((err = span_loop_await_done_l())) {
            if (err < 0) {
                span_action_state = SPAN_ACTION_ABORT;
            }
            goto give_state_sem_and_return;
        }
        if (!span_loop_check_states_l(SPAN_LOOP_DONE_HALT_OR_STALL, SPAN_ACTUATOR_STALL)) {
            span_action_state = SPAN_ACTION_ABORT;
            err = SPAN_ERROR_NOT_HOME;
            goto give_state_sem_and_return;
        }
        span_loop_run_l(&span_move_spec_relief, !extend, SPAN_ACTUATOR_SET_ALL, SPAN_ACTUATOR_RUN_TANDEM);
        span_loop_data.restart_after_stall = true;
        span_action_state = SPAN_ACTION_HOME_RELIEF;
    }
    if (span_action_state == SPAN_ACTION_HOME_RELIEF) {
        if ((err = span_loop_await_done_l())) {
            if (err < 0) {
                span_action_state = SPAN_ACTION_ABORT;
            }
            goto give_state_sem_and_return;
        }
        if (!span_loop_check_states_l(SPAN_LOOP_DONE_TRAVEL, SPAN_ACTUATOR_RUN_TANDEM)) {
            span_action_state = SPAN_ACTION_ABORT;
            err = SPAN_ERROR_NOT_TRAVEL;
            goto give_state_sem_and_return;
        }
        span_loop_halt_l();
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

static const struct span_move_spec *span_poll_jog_move_spec(enum span_move move) {
    if (move == SPAN_MOVE_JOG) {
        return &span_move_spec_jog;
    }
    return &span_move_spec_home;
}

static enum span_actuator_state span_poll_jog_actuator_state(unsigned actuator_set) {
    if (actuator_set == SPAN_ACTUATOR_SET_ALL || actuator_set == SPAN_ACTUATOR_SET_01 ||
            actuator_set == SPAN_ACTUATOR_SET_23) {
        return SPAN_ACTUATOR_RUN_TANDEM;
    }
    return SPAN_ACTUATOR_RUN_INDEPENDENT;
}

int span_poll_jog(enum span_move move, bool extend, unsigned actuator_set) {
    static bool actual_extend;
    static unsigned actual_actuator_set;

    if (actual_extend != extend || actual_actuator_set != actuator_set) {
        span_action_state = SPAN_ACTION_ABORT;
        actual_extend = extend;
        actual_actuator_set = actuator_set;
    }
    if (span_action_state == SPAN_ACTION_JOG_DONE) {
        return 0;
    }
    span_position = SPAN_POSITION_UNKNOWN;

    int err;
    k_sem_take(&span_state_sem, K_FOREVER);

    if (span_action_state != SPAN_ACTION_JOG_TRAVEL) {
        span_loop_run_l(span_poll_jog_move_spec(move), extend, actuator_set, span_poll_jog_actuator_state(actuator_set));
        if ((err = span_loop_prepare_actuators_l(true, actuator_set, true))) {
            span_loop_halt_l();
            span_action_state = SPAN_ACTION_ABORT;
            goto give_state_sem_and_return;
        }
        span_action_state = SPAN_ACTION_JOG_TRAVEL;
    }
    if (span_action_state == SPAN_ACTION_JOG_TRAVEL) {
        if ((err = span_loop_await_done_l())) {
            if (err < 0) {
                span_action_state = SPAN_ACTION_ABORT;
            }
            goto give_state_sem_and_return;
        }
        span_loop_halt_l();
        span_action_state = SPAN_ACTION_JOG_DONE;
    }
    err = 0; // done

give_state_sem_and_return:
    k_sem_give(&span_state_sem);
    return err;
}
