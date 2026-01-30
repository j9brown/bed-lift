#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <zephyr/device.h>

/** @brief Status: Fault pin is active.
 * This extended fault is only reported when reading the fault register or handling a fault callback.
 */
#define DRV8434S_EX_STATUS_FAULT (0x80)
/** @brief Status: SPI communication error.
 * This extended fault is only reported when reading the fault register or handling a fault callback.
 */
#define DRV8434S_EX_STATUS_SPI_ERROR (0x40)
/** @brief Status: Supply undervoltage lockout fault condition. */
#define DRV8434S_STATUS_UVLO (0x20)
/** @brief Status: Charge pump undervoltage fault condition. */
#define DRV8434S_STATUS_CPUV (0x10)
/** @brief Status: Overcurrent fault condition. */
#define DRV8434S_STATUS_OCP (0x08)
/** @brief Status: Motor stall condition. */
#define DRV8434S_STATUS_STL (0x04)
/** @brief Status: Overtemperature warning or overtemperature shutdown condition. */
#define DRV8434S_STATUS_TF (0x02)
/** @brief Status: Open load condition. */
#define DRV8434S_STATUS_OL (0x01)

/**
 * @brief Start the chain of motor driver devices.
 *
 * Wakes all devices if the devicetree specifies a sleep pin.
 * Configures each device, disables the outputs, and clears faults.
 * The initial state of the step indexer is 45 degrees (71% current) when exiting sleep.
 *
 * @param dev DRV8434S device node.
 * @param num_devices Number of devices in the chain, must equal the number of devices specified in the devicetree configuration.
 * The driver checks this value to ensure that the client has sized its buffers correctly.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is already started or if num_devices does not match the actual length of the chain.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_start(const struct device *dev, unsigned num_devices);

/**
 * @brief Stop the chain of motor driver devices.
 *
 * Disables the output of each device.
 * Puts all devices to sleep if the devicetree specifies a sleep pin.
 *
 * @param dev DRV8434S device node.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is already stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_stop(const struct device *dev);

/**
 * @brief Common parameters for motor driver requests.
 */
struct drv8434s_options {
    /** @brief Buffer for num_devices status reports. Populated with the status of each device in the chain before it processes the request, if not NULL.
     * Reports bits from DRV8434S_STATUS_*. Excludes extended status flags.
     */
    uint8_t *status_buf;
    /** @brief Asks every device in the chain to clear its fault register after it processes the request, if true. */
    bool clear_fault;
};

/**
 * @brief Get whether the fault pin is active.
 *
 * @param dev DRV8434S device node.
 * @retval 0 If the fault pin is not active.
 * @retval 1 If the fault pin is active.
 * @retval -ENOTSUP if the fault pin is not configured in the devicetree.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_has_fault(const struct device *dev);

#ifdef CONFIG_DRV8434S_FAULT_CALLBACK
/**
 * @brief Callback invoked when the fault pin becomes active.
 *
 * @param user_data User data for the callback.
 */
typedef void (*drv8434s_fault_callback_t)(void *user_data);

/**
 * @brief Sets a callback to invoke when the fault pin becomes active.
 *
 * @param dev DRV8434S device node.
 * @param callback The callback to set, or NULL to remove it.
 * @param user_data User data for the callback.
 * @retval 0 If successful.
 * @retval -ENOTSUP if the fault pin is not configured in the devicetree.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_set_fault_callback(const struct device *dev, drv8434s_fault_callback_t callback, void *user_data);
#endif /* CONFIG_DRV8434S_FAULT_CALLBACK*/

/**
 * @brief Get the fault status of the chain of motor driver devices.
 * 
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param ex_status_buf Buffer for num_devices status reports. Populated with the extended status of each device in the chain, if not NULL.
 * Reports bits from DRV8434S_EX_STATUS_* and DRV8434S_STATUS_*.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_get_fault_status(const struct device *dev, const struct drv8434s_options *options,
        uint8_t* ex_status_buf);

/**
 * @brief Set which devices in the chain have their outputs enabled.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param enabled_set Set bits with BIT64(index) for the devices whose outputs are to
 * be enabled, all other devices will have their outputs disabled.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_set_output_enable(const struct device *dev, const struct drv8434s_options *options,
        uint64_t enabled_set);

/**
 * @brief Set the stall threshold for all devices.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param enabled_set Set bits with BIT64(index) for the devices whose outputs are to
 * be enabled, all other devices will have their outputs disabled.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */

/**
 * @brief Determines the indexer step input.
 */
enum drv8434s_step_input {
    /** @brief The step input follows the driver's STEP pin. */
    DRV8434S_STEP_PIN = 0x00,
    /** @brief The step input is disabled. */
    DRV8434S_STEP_DISABLE = 0x10,
    /** @brief Each step request triggers a one-shot pulse of the step input. */
    DRV8434S_STEP_PULSE = 0x50,
};

/**
 * @brief Determines the indexer direction input.
 */
enum drv8434s_dir_input {
    /** @brief The direction input follows the driver's DIR pin. */
    DRV8434S_DIR_PIN = 0x00,
    /** @brief The direction input is low. */
    DRV8434S_DIR_LOW = 0x20,
    /** @brief The direction input is high. */
    DRV8434S_DIR_HIGH = 0xa0,
};

/**
 * @brief Determines the indexer microstep behavior as it advances.
 */
enum drv8434s_microstep_mode {
    /** @brief Full step with 100% current.
     * Produces more torque at high speed and is less efficient than DRV8434S_MICROSTEP_1. */
    DRV8434S_MICROSTEP_1_FULL_CURRENT = 0,
    /** @brief Full step with 71% current. */
    DRV8434S_MICROSTEP_1 = 1,
    /** @brief Non-circular 1/2 step with 100% current.
     * Produces more torque at high speed and is less efficient than DRV8434S_MICROSTEP_2. */
    DRV8434S_MICROSTEP_2_FULL_CURRENT = 2,
    /** @brief Circular 1/2 step. */
    DRV8434S_MICROSTEP_2 = 3,
    /** @brief Circular 1/4 step. */
    DRV8434S_MICROSTEP_4 = 4,
    /** @brief Circular 1/8  step. */
    DRV8434S_MICROSTEP_8 = 5,
    /** @brief Circular 1/16 step. */
    DRV8434S_MICROSTEP_16 = 6,
    /** @brief Circular 1/32 step. */
    DRV8434S_MICROSTEP_32 = 7,
    /** @brief Circular 1/64 step. */
    DRV8434S_MICROSTEP_64 = 8,
    /** @brief Circular 1/128 step. */
    DRV8434S_MICROSTEP_128 = 9,
    /** @brief Circular 1/256 step. */
    DRV8434S_MICROSTEP_256 = 10,
};

/**
 * @brief Encode a step request.
 *
 * @param step_input Step input behavior.
 * @param dir_input Direction input behavior.
 * @param microstep_mode Microstep mode.
 * @return The encoded request.
 */
static inline uint8_t drv8434s_make_step_request(enum drv8434s_step_input step_input,
        enum drv8434s_dir_input dir_input, enum drv8434s_microstep_mode microstep_mode) {
    return (uint8_t)step_input | (uint8_t)dir_input | (uint8_t)microstep_mode;
}

/**
 * @brief Callback for asynchronous step requests.
 *
 * @param dev DRV8434S device node.
 * @param result Result code of the request. 0 is success, -errno for failure.
 * @param user_data User data for the callback.
 */
typedef void (*drv8434s_step_callback_t)(const struct device *dev, int result, void *user_data);

#ifdef CONFIG_SPI_ASYNC
/**
 * @brief Asynchronously send a step request to each motor driver device in the chain.
 *
 * This function may block until prior bus transfers have completed so that the request
 * can be issued to the device.
 *
 * The caller must wait for the callback to return before performing another operation
 * with the device.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request. Because the request is asynchronous, the
 * status_buf member must not be on the stack.
 * @param step_requests One step request for each device in the chain. See drv8434s_make_step_request().
 * @param callback Callback to receive the result. If null, the call will be completed synchronously.
 * @param user_data User data for the callback.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_step_async(const struct device *dev, const struct drv8434s_options *options,
        const uint8_t* step_requests, drv8434s_step_callback_t callback, void* user_data);
#endif /* CONFIG_SPI_ASYNC */

/**
 * @brief Send a step request to each motor driver device in the chain.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param step_requests One step request for each device in the chain. See drv8434s_make_step_request().
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_step(const struct device *dev, const struct drv8434s_options *options,
        const uint8_t* step_requests);

/**
 * @brief Set the stall threshold.  Has the side-effect of disabling stall learning.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param stall_th Stall threshold, or 0 to disable stall detection.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_set_stall_threshold(const struct device *dev, const struct drv8434s_options *options, uint16_t stall_th);

/**
 * @brief Set or clear the STL_LRN bit to learn the stall threshold.
 *
 * Run the motor at steady state before calling this function.
 * Let the motor run for at least 16 electrical cycles then intentionally stall the motor.
 * Call drv8434s_get_stall_learn_status to obtain the result of learning.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param stall_learn_active_set Populated with BIT64(index) for each device to set STL_LRN = 1.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_set_stall_learn_active(const struct device *dev, const struct drv8434s_options *options,
        uint64_t stall_learn_active_set);

/**
 * @brief Get the status of the stall learning process and the learned stall thresholds.
 *
 * While stall learning is in progress, STL_LRN = 1.
 * When stall learning succeeds, each device sets STL_LRN = 0 and STL_LRN_OK = 1.
 * When stall learning fails, each device sets STL_LRN = 0 and STL_LRN_OK = 0.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param stall_learn_active_set Populated with BIT64(index) for each device that reports STL_LRN = 1.
 * @param stall_learn_ok_set Populated with BIT64(index) for each device that reports STL_LRN_OK = 1.
 * @param stall_th_buf Buffer for num_devices values. Populated with the value of each device's STALL_TH register.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_get_stall_learn_status(const struct device *dev, const struct drv8434s_options *options,
        uint64_t *stall_learn_active_set, uint64_t *stall_learn_ok_set, uint16_t *stall_th_buf);

/**
 * @brief Get contents of the TRQ_COUNT register for each motor.
 *
 * @param dev DRV8434S device node.
 * @param options Options for the request.
 * @param trq_count_buf Buffer for num_devices values. Populated with the value of each device's TRQ_COUNT register.
 * @retval 0 If successful.
 * @retval -EINVAL If the chain is stopped.
 * @retval -EBUSY If an asynchronous operation is in progress.
 * @retval -errno Negative errno code on failure.
 */
int drv8434s_get_trq_count(const struct device *dev, const struct drv8434s_options *options,
        uint16_t* trq_count_buf);
