/**
 * @file Component to raise and lower the bed platform.
 *
 * The bed platform physically couples a pair of linear actuators with lead screws driven
 * by brushed DC motors and position feedback from (noisy) hall sensor with quadrature encoding.
 * To prevent damage to the frame, the movement of both actuators must be synchronized continously.
 * The actuators have built-in limit switches that can be used to home the actuators at the
 * end of travel by detecting when they have become stationary while powered.
 *
 * The lift assembly also has two microswitches that encode four position states along its travel
 * which is used to coordinate the movement of the wing spans.
 *
 * The main thread calls the lift_poll_* functions to incrementally update the state of the actuators and
 * monitor their progress. A watchdog halts movement automatically if the main thread stops polling.
 *
 * The limit switches and hall sensors are sampled by an interrupt and debounced by recording the cycle time
 * when the inputs most recently changed. This mechanism also updates each actuator's assumed position
 * and determined whether it is stationary.
 *
 * On each tick of a timer, an interrupt checks for faults, compares the position of the actuators, and
 * determines new PWM control inputs for the actuators. It uses a PID controller to maintain alignment.
 */

#include <errno.h>
#include <stdlib.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>

#include "bed/lift.h"
#include "fixed_point.h"

// If defined, pretend the lift limit switches are reporting the specified state.
// Useful for testing the span actuators while the limit switches have been disconnected.
//#define LIFT_DEBUG_ASSUME_LIMIT_STATE LIFT_LIMIT_BELOW_SAFE_ZONE
//#define LIFT_DEBUG_ASSUME_LIMIT_STATE LIFT_LIMIT_IN_SAFE_ZONE
//#define LIFT_DEBUG_ASSUME_LIMIT_STATE LIFT_LIMIT_ABOVE_SAFE_ZONE
//#define LIFT_DEBUG_ASSUME_LIMIT_STATE LIFT_LIMIT_ABOVE_CEILING

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

// Spin lock for the limit and hall sensor state.
// Functions that require holding this spinlock have the `_l` suffix.
struct k_spinlock lift_lock;

/*
 * LIFT LIMIT SWITCHES
 */

// Lift limit switch debounce interval.
#define LIFT_LIMIT_DEBOUNCE_MS (50)
#define LIFT_LIMIT_DEBOUNCE_CYCLES (uint32_t)((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * LIFT_LIMIT_DEBOUNCE_MS / 1000)

static const struct gpio_dt_spec lift_limit_a_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lift_limit_a_gpios);
static const struct gpio_dt_spec lift_limit_b_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lift_limit_b_gpios);

enum lift_limit_state {
    LIFT_LIMIT_BELOW_SAFE_ZONE = 0b11,
    LIFT_LIMIT_IN_SAFE_ZONE = 0b01,
    LIFT_LIMIT_ABOVE_SAFE_ZONE = 0b00,
    LIFT_LIMIT_ABOVE_CEILING = 0b10,
};

struct lift_limit_data {
    // These variables are guarded by lift_lock
    enum lift_limit_state raw_state;
    uint32_t raw_change_cycle;
    enum lift_limit_state state;
};
static struct lift_limit_data lift_limit_data;

// This function must be called on every timer tick (before the cycle counter rolls over).
static void lift_limit_update_l(uint32_t cycle) {
    gpio_port_value_t value = 0;
    gpio_port_get(lift_limit_a_gpio.port, &value);
    enum lift_limit_state state =
            ((value & BIT(lift_limit_a_gpio.pin)) ? 0b10 : 0) |
            ((value & BIT(lift_limit_b_gpio.pin)) ? 0b01 : 0);
    if (state != lift_limit_data.raw_state) {
        lift_limit_data.raw_state = state;
        lift_limit_data.raw_change_cycle = cycle;
    } else if (state != lift_limit_data.state &&
            cycle - lift_limit_data.raw_change_cycle >= LIFT_LIMIT_DEBOUNCE_CYCLES) {
        lift_limit_data.state = state;
    }
#ifdef LIFT_DEBUG_ASSUME_LIMIT_STATE
    lift_limit_data.state = LIFT_DEBUG_ASSUME_LIMIT_STATE;
#endif
}

/*
 * LIFT HALL SENSORS
 */

// Lift stationary time.
// The actuator is determined to be stationary when there is no change in position within a
// certain time. This condition generally indicates that the actuator is not powered, stalled,
// or has reached one of its internal limits.
#define LIFT_HALL_STATIONARY_MS (500)
#define LIFT_HALL_STATIONARY_CYCLES (uint32_t)((uint64_t)CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC * LIFT_HALL_STATIONARY_MS / 1000)

#define LIFT_HALL_PULSES_PER_INCH (660)

static const struct gpio_dt_spec lift1_hall1_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lift1_hall1_gpios);
static const struct gpio_dt_spec lift1_hall2_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lift1_hall2_gpios);
static const struct gpio_dt_spec lift2_hall1_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lift2_hall1_gpios);
static const struct gpio_dt_spec lift2_hall2_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lift2_hall2_gpios);

struct lift_hall_data {
    struct gpio_callback gpio_callback;

    // These variables are only written by the ISR
    unsigned isr_bits;
    atomic_t isr_position;

    // These variables are guarded by lift_lock
    int raw_position;
    uint32_t stationary_change_cycle;
    bool stationary_while_powered;
    int origin;
};
static struct lift_hall_data lift1_hall_data;
static struct lift_hall_data lift2_hall_data;

static int8_t LIFT_HALL_QUADRATURE[] = {
    0, 1, -1, 0 /*error*/,
    -1, 0, 0 /*error*/, 1,
    1, 0 /*error*/, 0, -1,
    0 /*error*/, -1, 1, 0,
};

// Note: This GPIO handler shares an ISR with elevated interrupt priority. Be quick.
static void lift1_hall_gpio_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    gpio_port_value_t value = 0;
    gpio_port_get(port, &value);
    unsigned bits =
            ((value & BIT(lift1_hall1_gpio.pin)) ? 0b10 : 0) |
            ((value & BIT(lift1_hall2_gpio.pin)) ? 0b01 : 0);
    atomic_add(&lift1_hall_data.isr_position, LIFT_HALL_QUADRATURE[(lift1_hall_data.isr_bits << 2) | bits]);
    lift1_hall_data.isr_bits = bits;
}

// Note: This GPIO handler shares an ISR with elevated interrupt priority. Be quick.
static void lift2_hall_gpio_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins) {
    gpio_port_value_t value = 0;
    gpio_port_get(port, &value);
    unsigned bits =
            ((value & BIT(lift2_hall1_gpio.pin)) ? 0b10 : 0) |
            ((value & BIT(lift2_hall2_gpio.pin)) ? 0b01 : 0);
    atomic_add(&lift2_hall_data.isr_position, LIFT_HALL_QUADRATURE[(lift2_hall_data.isr_bits << 2) | bits]);
    lift2_hall_data.isr_bits = bits;
}

static inline void lift_hall_reset_origin_l(void) {
    lift1_hall_data.origin = lift1_hall_data.raw_position;
    lift2_hall_data.origin = lift2_hall_data.raw_position;
}

// This function must be called on every timer tick (before the cycle counter rolls over).
static void lift_hall_update_l(struct lift_hall_data *data, uint32_t cycle, bool powered) {
    int position = atomic_get(&data->isr_position);
    if (position != data->raw_position) {
        data->raw_position = position;
        data->stationary_change_cycle = cycle;
        data->stationary_while_powered = false;
    } else if (!powered) {
        data->stationary_change_cycle = cycle;
        data->stationary_while_powered = false;
    } else if (!data->stationary_while_powered && cycle - data->stationary_change_cycle >= LIFT_HALL_STATIONARY_CYCLES) {
        data->stationary_while_powered = true;
    }
}

/*
 * LIFT MOTOR CONTROLLER
 */

static const struct gpio_dt_spec lift_fault_gpio = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, lift_fault_gpios);
static const struct pwm_dt_spec lift1_in1_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(lift1_in1), 0);
static const struct pwm_dt_spec lift1_in2_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(lift1_in2), 0);
static const struct pwm_dt_spec lift2_in1_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(lift2_in1), 0);
static const struct pwm_dt_spec lift2_in2_pwm = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(lift2_in2), 0);

static const struct device *lift_loop_tick_dev = DEVICE_DT_GET(DT_NODELABEL(lift_loop_tick));

// Lift loop watchdog timeout to ensure the main loop remains in control of the movement.
#define LIFT_LOOP_TIMEOUT_US (20000)

// The frequency at which the loop ticks to update the velocity vector.
#define LIFT_LOOP_TICK_FREQUENCY (100)
#define LIFT_LOOP_TICK_PERIOD_US (USEC_PER_SEC / LIFT_LOOP_TICK_FREQUENCY)

// The maximum positional error to allow while running in tandem.
#define LIFT_LOOP_MAX_TANDEM_ERROR (LIFT_HALL_PULSES_PER_INCH / 3)

// Terms for the PID controller. Adjusts balance factor per step of misalignment.
// The parameters are expressed in recursive form.
// https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Pseudocode
// Oscillation occurs with KP >= 0.08
#define LIFT_LOOP_KP (0.03) /* proportional gain */
#define LIFT_LOOP_KI (0.1) /* integral gain */
#define LIFT_LOOP_KD (0.) /* derivative gain */
#define LIFT_LOOP_DT (1. / LIFT_LOOP_TICK_FREQUENCY)
#define LIFT_LOOP_PID_A0 Q15_CONST(LIFT_LOOP_KP + LIFT_LOOP_KI * LIFT_LOOP_DT + LIFT_LOOP_KD / LIFT_LOOP_DT)
#define LIFT_LOOP_PID_A1 Q15_CONST(-LIFT_LOOP_KP - 2 * LIFT_LOOP_KD / LIFT_LOOP_DT)
#define LIFT_LOOP_PID_A2 Q15_CONST(LIFT_LOOP_KD / LIFT_LOOP_DT)

enum lift_loop_state {
    LIFT_LOOP_RUN = 1,     // Run actuator movement
    LIFT_LOOP_HALT = 0,    // Halt actuators
    LIFT_LOOP_ERROR = -1,  // An error occurred as indicated by loop_error
    LIFT_LOOP_DONE = -2,   // All actuators stopped at end of travel
};

struct lift_loop_data {
    // State of the control loop. The loop only drives the actuators when in the RUN state.
    enum lift_loop_state loop_state;

    // The error associated with the LIFT_LOOP_ERROR state.
    int loop_error;

    // The type of movement to perform.
    enum lift_move move;

    // Decremented by one each time the lift loop handles a tick of the timer.
    // At zero, the loop stops the actuators and reports an error.
    // Behaves as a watchdog to ensure that the main loop is still in control.
    unsigned ticks_remaining;

    // The direction to run the actuators.
    bool raise;

    // The position of the lift taking into account the limit switch state and whether the
    // lift actuators both stopped at end-of-travel.
    enum lift_position position;

    // PWM pulse period for the motor inputs in timer clock cycles.
    uint32_t pulse_period_cycles;

    // Most recent duty cycle of each actuator. The sign indicates the direction of travel.
    // Saturates at +/- Q15_ONE.
    q15_t lift1_duty;
    q15_t lift2_duty;

    // The output of the control loop.
    // When 0, both actuators receive the same duty cycle.
    // When positive, lift1 receives more duty cycle, lift2 receives less duty cycle.
    // When negative, lift1 receives less duty cycle, lift1 receives more duty cycle.
    // The control balance increases when lift1 falls behind lift2 in the direction of travel
    // and decreases when lift1 gets ahead of lift2 in the direction of travel.
    // Saturates at +/- Q15_ONE.
    q15_t control_balance;

    // The last 3 positional error terms.
    // Positive when lift1 falls behind lift2 in the direction of travel.
    // Negative when lift1 gets ahead of lift2 in the direction of travel.
    int control_error[3];
};
static struct lift_loop_data lift_loop_data;

static int lift_has_fault(void) {
	return gpio_pin_get_dt(&lift_fault_gpio);
}

static void lift_loop_tick_handler(const struct device *dev, void *user_data) {
    uint32_t cycle = k_cycle_get_32();
    q15_t lift1_duty, lift2_duty;
    K_SPINLOCK(&lift_lock) {
        // Update the current state of the lift.
        lift_limit_update_l(cycle);
        lift_hall_update_l(&lift1_hall_data, cycle, !!lift_loop_data.lift1_duty);
        lift_hall_update_l(&lift2_hall_data, cycle, !!lift_loop_data.lift2_duty);

        // Update the overall position of the lift based on the limit switches. Assume that if an actuator is
        // stationary while commanded to move in the direction of the limit then the actuator's internal limit
        // switch has triggered.
        bool stall = lift1_hall_data.stationary_while_powered && lift2_hall_data.stationary_while_powered;
        switch (lift_limit_data.state) {
            case LIFT_LIMIT_BELOW_SAFE_ZONE:
                if (lift_loop_data.lift1_duty < 0 && lift_loop_data.lift2_duty < 0 && stall) {
                    lift_loop_data.position = LIFT_POSITION_LOWER_LIMIT;
                    if (lift_loop_data.loop_state > LIFT_LOOP_HALT && !lift_loop_data.raise) {
                        lift_hall_reset_origin_l();
                        lift_loop_data.loop_state = LIFT_LOOP_DONE;
                        goto halt_and_exit_spinlock;
                    }
                    stall = false;
                } else if (lift_loop_data.position != LIFT_POSITION_LOWER_LIMIT ||
                        lift_loop_data.lift1_duty || lift_loop_data.lift2_duty) {
                    lift_loop_data.position = LIFT_POSITION_BELOW_SAFE_ZONE;
                }
                break;
            case LIFT_LIMIT_IN_SAFE_ZONE:
                lift_loop_data.position = LIFT_POSITION_IN_SAFE_ZONE;
                break;
            case LIFT_LIMIT_ABOVE_SAFE_ZONE:
                lift_loop_data.position = LIFT_POSITION_ABOVE_SAFE_ZONE;
                break;
            case LIFT_LIMIT_ABOVE_CEILING:
                if (lift_loop_data.lift1_duty > 0 && lift_loop_data.lift2_duty > 0 && stall) {
                    lift_loop_data.position = LIFT_POSITION_UPPER_LIMIT;
                    if (lift_loop_data.loop_state > LIFT_LOOP_HALT && lift_loop_data.raise) {
                        lift_hall_reset_origin_l();
                        lift_loop_data.loop_state = LIFT_LOOP_DONE;
                        goto halt_and_exit_spinlock;
                    }
                    stall = false;
                } else if (lift_loop_data.position != LIFT_POSITION_UPPER_LIMIT ||
                        lift_loop_data.lift1_duty || lift_loop_data.lift2_duty) {
                    lift_loop_data.position = LIFT_POSITION_ABOVE_CEILING;
                }
                break;
            default:
                lift_loop_data.position = LIFT_POSITION_UNKNOWN;
                break;
        }

        // Check for faults, timeouts, and limits.
         if (lift_loop_data.loop_state <= LIFT_LOOP_HALT) {
            goto halt_and_exit_spinlock;
        }
        if (lift_has_fault()) {
            lift_loop_data.loop_state = LIFT_LOOP_ERROR;
            lift_loop_data.loop_error = LIFT_ERROR_FAULT;
            goto halt_and_exit_spinlock;
        }
        if (lift_loop_data.ticks_remaining == 0) {
            lift_loop_data.loop_state = LIFT_LOOP_ERROR;
            lift_loop_data.loop_error = LIFT_ERROR_TIMEOUT;
            goto halt_and_exit_spinlock;
        }
        if (stall) {
            lift_loop_data.loop_state = LIFT_LOOP_ERROR;
            lift_loop_data.loop_error = LIFT_ERROR_STALL;
            goto halt_and_exit_spinlock;
        }

        // Determine the motor duty cycles.
        const q15_t full_duty = Q15_ONE; // todo: maybe ramp up to full duty
        const int sign = lift_loop_data.raise ? 1 : -1;
        switch (lift_loop_data.move) {
            case LIFT_MOVE_INDEPENDENT_BOTH:
                lift_loop_data.lift1_duty = full_duty * sign;
                lift_loop_data.lift2_duty = full_duty * sign;
                break;
            case LIFT_MOVE_INDEPENDENT_1:
                lift_loop_data.lift1_duty = full_duty * sign;
                lift_loop_data.lift2_duty = 0;
                break;
            case LIFT_MOVE_INDEPENDENT_2:
                lift_loop_data.lift1_duty = 0;
                lift_loop_data.lift2_duty = full_duty * sign;
                break;
            case LIFT_MOVE_TANDEM: {
                const int lift1_position = lift1_hall_data.raw_position - lift1_hall_data.origin;
                const int lift2_position = lift2_hall_data.raw_position - lift2_hall_data.origin;
                const int delta = lift2_position - lift1_position;
                if (abs(delta) > LIFT_LOOP_MAX_TANDEM_ERROR) {
                    // While it might be possible to recover from a large positional error by only
                    // driving one of the actuators for some time, the presence of a large discrepancy
                    // suggests prior measurement error (the sensor position estimate might be wrong)
                    // so it's safer to force the user to jog the actuators back into alignment.
                    lift_loop_data.loop_state = LIFT_LOOP_ERROR;
                    lift_loop_data.loop_error = LIFT_ERROR_DESYNC;
                    goto halt_and_exit_spinlock;
                }
                lift_loop_data.control_error[2] = lift_loop_data.control_error[1];
                lift_loop_data.control_error[1] = lift_loop_data.control_error[0];
                lift_loop_data.control_error[0] = delta * sign;
                const q15_t extreme_balance = Q15_ONE * 15 / 16; // ensure resulting duty is always non-zero
                lift_loop_data.control_balance = q15_clamp(lift_loop_data.control_balance +
                        LIFT_LOOP_PID_A0 * lift_loop_data.control_error[0] +
                        LIFT_LOOP_PID_A1 * lift_loop_data.control_error[1] +
                        LIFT_LOOP_PID_A2 * lift_loop_data.control_error[2], -extreme_balance, extreme_balance);
                const q15_t reduced_duty = q15_mul_q15(full_duty, lift_loop_data.control_balance);
                if (reduced_duty >= 0) {
                    lift_loop_data.lift1_duty = full_duty * sign;
                    lift_loop_data.lift2_duty = (full_duty - reduced_duty) * sign;
                } else {
                    lift_loop_data.lift1_duty = (full_duty + reduced_duty) * sign;
                    lift_loop_data.lift2_duty = full_duty * sign;
                }
                break;
            }
        }
        goto exit_spinlock;

    halt_and_exit_spinlock:
        lift_loop_data.lift1_duty = 0;
        lift_loop_data.lift2_duty = 0;
    
    exit_spinlock:
        lift1_duty = lift_loop_data.lift1_duty;
        lift2_duty = lift_loop_data.lift2_duty;
    }

    // Update the motor PWM.
    // Use slow decay ("brake") for the off duty mode to let current recirculate through the
    // motor windings.  If use used fast decay ("coast") then the energy would be shunted back
    // to the power supply and raise the voltage at the VM terminal.  This also has the effect
    // of stopping the motor more quickly at zero duty cycle.
    // Note that the PWM polarity is inverted by the devicetree overlay to facilitate use of
    // slow decay mode.
    const struct device *lift_in_dev = lift1_in1_pwm.dev;
    const uint32_t lift_in_pulse_period_cycles = lift_loop_data.pulse_period_cycles;
    int err = 0;
    if (lift1_duty <= 0) {
        err |= pwm_set_cycles(lift_in_dev, lift1_in1_pwm.channel, lift_in_pulse_period_cycles,
                q15_to_int(-lift1_duty * lift_in_pulse_period_cycles), lift1_in1_pwm.flags);
        err |= pwm_set_cycles(lift_in_dev, lift1_in2_pwm.channel, lift_in_pulse_period_cycles,
                0, lift1_in2_pwm.flags);
    } else {
        err |= pwm_set_cycles(lift_in_dev, lift1_in1_pwm.channel, lift_in_pulse_period_cycles,
                0, lift1_in1_pwm.flags);
        err |= pwm_set_cycles(lift_in_dev, lift1_in2_pwm.channel, lift_in_pulse_period_cycles,
                q15_to_int(lift1_duty * lift_in_pulse_period_cycles), lift1_in2_pwm.flags);
    }
    if (lift2_duty <= 0) {
        err |= pwm_set_cycles(lift_in_dev, lift2_in1_pwm.channel, lift_in_pulse_period_cycles,
                q15_to_int(-lift2_duty * lift_in_pulse_period_cycles), lift2_in1_pwm.flags);
        err |= pwm_set_cycles(lift_in_dev, lift2_in2_pwm.channel, lift_in_pulse_period_cycles,
                0, lift2_in2_pwm.flags);
    } else {
        err |= pwm_set_cycles(lift_in_dev, lift2_in1_pwm.channel, lift_in_pulse_period_cycles,
                0, lift2_in1_pwm.flags);
        err |= pwm_set_cycles(lift_in_dev, lift2_in2_pwm.channel, lift_in_pulse_period_cycles,
                q15_to_int(lift2_duty * lift_in_pulse_period_cycles), lift2_in2_pwm.flags);
    }
    if (err) {
        K_SPINLOCK(&lift_lock) {
            if (lift_loop_data.loop_state > LIFT_LOOP_HALT) {
                lift_loop_data.loop_state = LIFT_LOOP_ERROR;
                lift_loop_data.loop_error = LIFT_ERROR_DRIVER;
            }
        }
    }
}

static void lift_loop_init_l(enum lift_loop_state loop_state, enum lift_move move, bool raise) {
    lift_loop_data.loop_state = loop_state;
    lift_loop_data.loop_error = 0;
    lift_loop_data.move = move;
    lift_loop_data.raise = raise;
    lift_loop_data.lift1_duty = 0;
    lift_loop_data.lift2_duty = 0;
    lift_loop_data.control_balance = 0;
    lift_loop_data.control_error[0] = 0;
    lift_loop_data.control_error[1] = 0;
    lift_loop_data.control_error[2] = 0;
}

static void lift_loop_halt_l(void) {
    lift_loop_init_l(LIFT_LOOP_HALT, LIFT_MOVE_INDEPENDENT_BOTH, false);
}

static void lift_loop_feed_l(void) {
    lift_loop_data.ticks_remaining = (LIFT_LOOP_TIMEOUT_US + LIFT_LOOP_TICK_PERIOD_US - 1) / LIFT_LOOP_TICK_PERIOD_US;
}

/*
 * CONTROLLER ENTRY POINTS
 */

enum lift_action_state {
    LIFT_ACTION_ABORT = -1,
    LIFT_ACTION_STANDBY_DONE = 0,
    LIFT_ACTION_MOVE_TRAVEL = 2,
    LIFT_ACTION_MOVE_DONE = 3,
};
static enum lift_action_state lift_action_state; // not guarded by semaphore

int lift_init(void) {
    int err;
    const struct device *lift_fault_port = lift_fault_gpio.port;
    const struct device *lift_limit_port = lift_limit_a_gpio.port;
    const struct device *lift1_hall_port = lift1_hall1_gpio.port;
    const struct device *lift2_hall_port = lift2_hall1_gpio.port;
    const struct device *lift_in_dev = lift1_in1_pwm.dev;
    const uint32_t lift_in_period_ns = lift1_in1_pwm.period;
    if (!device_is_ready(lift_fault_port) ||
            !device_is_ready(lift_limit_port) ||
            !device_is_ready(lift1_hall_port) ||
            !device_is_ready(lift2_hall_port) ||
            !device_is_ready(lift_in_dev)) {
        return -ENODEV;
    }
    if (lift_limit_port != lift_limit_a_gpio.port ||
            lift1_hall_port != lift1_hall2_gpio.port ||
            lift2_hall_port != lift2_hall2_gpio.port ||
            lift1_in2_pwm.dev != lift_in_dev || lift1_in2_pwm.period != lift_in_period_ns ||
            lift2_in1_pwm.dev != lift_in_dev || lift2_in1_pwm.period != lift_in_period_ns ||
            lift2_in2_pwm.dev != lift_in_dev || lift2_in2_pwm.period != lift_in_period_ns) {
        return -EIO;
    }

    if ((err = gpio_pin_configure_dt(&lift_limit_a_gpio, GPIO_INPUT)) ||
            (err = gpio_pin_configure_dt(&lift_limit_b_gpio, GPIO_INPUT)) ||
            (err = gpio_pin_configure_dt(&lift1_hall1_gpio, GPIO_INPUT)) ||
            (err = gpio_pin_configure_dt(&lift1_hall2_gpio, GPIO_INPUT)) ||
            (err = gpio_pin_configure_dt(&lift2_hall1_gpio, GPIO_INPUT)) ||
            (err = gpio_pin_configure_dt(&lift2_hall2_gpio, GPIO_INPUT))) {
        return err;
    }
    if ((err = gpio_pin_interrupt_configure_dt(&lift1_hall1_gpio, GPIO_INT_EDGE_BOTH)) ||
            (err = gpio_pin_interrupt_configure_dt(&lift1_hall2_gpio, GPIO_INT_EDGE_BOTH)) ||
            (err = gpio_pin_interrupt_configure_dt(&lift2_hall1_gpio, GPIO_INT_EDGE_BOTH)) ||
            (err = gpio_pin_interrupt_configure_dt(&lift2_hall2_gpio, GPIO_INT_EDGE_BOTH))) {
        return err;
    }
    gpio_init_callback(&lift1_hall_data.gpio_callback, lift1_hall_gpio_handler,
        BIT(lift1_hall1_gpio.pin) | BIT(lift1_hall2_gpio.pin));
    gpio_init_callback(&lift2_hall_data.gpio_callback, lift2_hall_gpio_handler,
        BIT(lift2_hall1_gpio.pin) | BIT(lift2_hall2_gpio.pin));
    lift1_hall_gpio_handler(lift1_hall_port, &lift1_hall_data.gpio_callback,
            lift1_hall_data.gpio_callback.pin_mask);
    lift2_hall_gpio_handler(lift2_hall_port, &lift2_hall_data.gpio_callback,
            lift2_hall_data.gpio_callback.pin_mask);
    if ((err = gpio_add_callback(lift1_hall_port, &lift1_hall_data.gpio_callback)) ||
            (err = gpio_add_callback(lift2_hall_port, &lift2_hall_data.gpio_callback))) {
        return err;
    }

    uint64_t lift_in_clock_rate;
    if ((err = pwm_get_cycles_per_sec(lift_in_dev, lift1_in1_pwm.channel, &lift_in_clock_rate))) {
        return err;
    }
    lift_loop_data.pulse_period_cycles = lift_in_clock_rate * lift_in_period_ns / NSEC_PER_SEC;

    struct counter_top_cfg cfg = {
        .ticks = counter_get_frequency(lift_loop_tick_dev) / LIFT_LOOP_TICK_FREQUENCY,
        .callback = lift_loop_tick_handler,
        .user_data = NULL,
        .flags = 0,
    };
    if ((err = counter_set_top_value(lift_loop_tick_dev, &cfg))) {
        return err;
    }
    counter_start(lift_loop_tick_dev);
    return 0;
}

enum lift_position lift_get_position(void) {
    enum lift_position result;
    K_SPINLOCK(&lift_lock) {
        result = lift_loop_data.position;
    }
    return result;
}

enum lift_state lift_get_state(void) {
    enum lift_state result;
    K_SPINLOCK(&lift_lock) {
        switch (lift_loop_data.loop_state) {
            case LIFT_LOOP_HALT:
                result = LIFT_STATE_HALT;
                break;
            case LIFT_LOOP_RUN:
                result = lift_loop_data.raise ? LIFT_STATE_RAISE : LIFT_STATE_LOWER;
                break;
            case LIFT_LOOP_DONE:
                result = LIFT_STATE_DONE;
                break;
            default:
                result = LIFT_STATE_ERROR;
                break;
        }
    }
    return result;
}

int lift_poll_standby(void) {
    if (lift_action_state == LIFT_ACTION_STANDBY_DONE) {
        return 0;
    }

    K_SPINLOCK(&lift_lock) {
        lift_loop_halt_l();
    }
    lift_action_state = LIFT_ACTION_STANDBY_DONE;
    return 0;
}

static int lift_poll_await_done_l(void) {
    if (lift_loop_data.loop_state > LIFT_LOOP_HALT) {
        lift_loop_feed_l();
        return 1; // in progress
    }
    if (lift_loop_data.loop_state != LIFT_LOOP_DONE) {
        lift_action_state = LIFT_ACTION_ABORT;
        return lift_loop_data.loop_error;
    }
    return 0; // done
}

static int lift_poll_move(enum lift_move move, bool raise, bool jog) {
    static enum lift_move actual_move;
    static bool actual_raise;
    static bool actual_jog;

    if (actual_move != move || actual_raise != raise || actual_jog != jog) {
        lift_action_state = LIFT_ACTION_ABORT;
        actual_move = move;
        actual_raise = raise;
        actual_jog = jog;
    }
    if (lift_action_state == LIFT_ACTION_MOVE_DONE) {
        return 0;
    }

    int err;
    K_SPINLOCK(&lift_lock) {
        if (lift_action_state != LIFT_ACTION_MOVE_TRAVEL) {
            if (jog) {
                lift_hall_reset_origin_l();
            }
            lift_loop_init_l(LIFT_LOOP_RUN, move, raise);
            lift_action_state = LIFT_ACTION_MOVE_TRAVEL;
        }
        if (lift_action_state == LIFT_ACTION_MOVE_TRAVEL) {
            if ((err = lift_poll_await_done_l())) {
                K_SPINLOCK_BREAK;
            }
            lift_loop_halt_l();
            lift_loop_data.loop_state = LIFT_ACTION_MOVE_DONE;
        }
        err = 0; // done
    }
    return err;
}

int lift_poll_raise(void) {
    return lift_poll_move(LIFT_MOVE_TANDEM, true, false);
}

int lift_poll_lower(void) {
    return lift_poll_move(LIFT_MOVE_TANDEM, false, false);
}

int lift_poll_jog(enum lift_move move, bool raise) {
    return lift_poll_move(move, raise, true);
}
