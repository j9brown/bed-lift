/**
 * @file Component to implement external monitoring of the bed lift as an I2C target.
 *
 * PROTOCOL:
 * 
 *   I2C target address: 16 (arbitrarily chosen)
 *   Write 8 bit register number to target then read or write 32 bit little-endian encoded register value.
 *
 * REGISTER 0x00: STATUS (read-only)
 *
 *   Bit  0 -  1: Current bed state, @see enum bed_state
 *   Bit  2 -  3: Current bed pose, @see enum bed_pose
 *   Bit  4 -  5: Target bed pose, @see enum bed_pose
 *   Bit  8 - 10: Current lift position, @see enum lift_position
 *   Bit 11 - 13: Current lift state, @see enum lift_state
 *   Bit 14 - 15: Current span position, @see enum span_position
 *   Bit 16 - 17: Current span state, @see enum span_state
 *   Bit      21: Control inhibited; set to 1 when the control panel is inhibited from performing actions
 *   Bit      22: Control active; set to 1 when the control panel is performing an action
 *   Bit      23: Control error; set to 1 when the control panel's action has encountered an error
 *   Bit 24 - 31: Last reported control error (positive value)
 *
 * REGISTER 0x01: CONTROL (read-write)
 *
 *   Bit       0: Control inhibit; set to 1 to inhibit the control panel from performing actions
 *
 * REGISTER 0x02: LIFT DEBUG (read-only)
 *
 * REGISTER 0x03: SPAN DEBUG (read-only)
 *
 * REGISTER 0x04: SPAN TEST (read-write)
 */

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "monitor.h"

/*
 * Registers.
 */

enum monitor_register {
	MONITOR_REGISTER_STATUS = 0,
	MONITOR_REGISTER_SETTING = 1,
	MONITOR_REGISTER_LIFT_DEBUG = 2,
	MONITOR_REGISTER_SPAN_DEBUG = 3,
	MONITOR_REGISTER_SPAN_TEST = 4,
};

static atomic_t monitor_registers[5];

static inline bool monitor_register_is_valid(uint8_t reg) {
	return reg >= 0 && reg < ARRAY_SIZE(monitor_registers);
}

static inline bool monitor_register_is_externally_writable(uint8_t reg) {
	return reg == MONITOR_REGISTER_SETTING || reg == MONITOR_REGISTER_SPAN_TEST;
}

static inline void monitor_register_write(uint8_t reg, uint32_t value) {
	atomic_set(&monitor_registers[reg], value);
}

static inline uint32_t monitor_register_read(uint8_t reg) {
	return atomic_get(&monitor_registers[reg]);
}

/*
 * I2C target.
 */

static const struct device *monitor_i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c1));

#define MONITOR_I2C_TARGET_ADDRESS (16)

enum monitor_i2c_state {
	MONITOR_I2C_IDLE = 0,
	MONITOR_I2C_WRITE_REGISTER = 1,
	MONITOR_I2C_READ_OR_WRITE_VALUE = 2,
	MONITOR_I2C_WRITE_VALUE = 3,
	MONITOR_I2C_READ_VALUE = 4,
	MONITOR_I2C_READ_FINISHED = 5,
	MONITOR_I2C_ERROR = 6,
};

struct monitor_i2c_data {
	enum monitor_i2c_state state : 8;
	uint8_t reg;
	uint8_t remaining;
	uint32_t value;
};
static struct monitor_i2c_data monitor_i2c_data;

static int monitor_i2c_target_write_requested_handler(struct i2c_target_config *config) {
	monitor_i2c_data.state = MONITOR_I2C_WRITE_REGISTER;
	return 0;
}

static int monitor_i2c_target_write_received_handler(struct i2c_target_config *config, uint8_t val) {
	switch (monitor_i2c_data.state) {
		case MONITOR_I2C_WRITE_REGISTER:
			if (monitor_register_is_valid(val)) {
				monitor_i2c_data.reg = val;
				monitor_i2c_data.state = MONITOR_I2C_READ_OR_WRITE_VALUE;
				return 0;
			}
			monitor_i2c_data.state = MONITOR_I2C_ERROR;
			return -1;
		case MONITOR_I2C_READ_OR_WRITE_VALUE:
			monitor_i2c_data.state = MONITOR_I2C_WRITE_VALUE;
			monitor_i2c_data.value = 0;
			monitor_i2c_data.remaining = sizeof(uint32_t);
			__fallthrough;
		case MONITOR_I2C_WRITE_VALUE:
			monitor_i2c_data.value >>= 8;
			monitor_i2c_data.value |= val << 24;
			if (--monitor_i2c_data.remaining) {
				return 0;
			}
			if (monitor_register_is_externally_writable(monitor_i2c_data.reg)) {
				monitor_register_write(monitor_i2c_data.reg, monitor_i2c_data.value);
			}
			monitor_i2c_data.state = MONITOR_I2C_IDLE;
			return -1;
		default:
			monitor_i2c_data.state = MONITOR_I2C_ERROR;
			return -1;
	}
}

static int monitor_i2c_target_read_requested_or_processed_handler(struct i2c_target_config *config, uint8_t *val) {
	switch (monitor_i2c_data.state) {
		case MONITOR_I2C_READ_OR_WRITE_VALUE:
			monitor_i2c_data.state = MONITOR_I2C_READ_VALUE;
			monitor_i2c_data.value = monitor_register_read(monitor_i2c_data.reg);
			monitor_i2c_data.remaining = sizeof(uint32_t);
			__fallthrough;
		case MONITOR_I2C_READ_VALUE:
			*val = monitor_i2c_data.value & 0xff;
			monitor_i2c_data.value >>= 8;
			if (--monitor_i2c_data.remaining == 0) {
				monitor_i2c_data.state = MONITOR_I2C_READ_FINISHED;
			}
			return 0;
		case MONITOR_I2C_READ_FINISHED:
			// FIXME: We receive one additional call to read after the last byte has been transmitted.
			// It's unclear whether it is originating from the client (esphome with ESP-IDF) or is simply
			// an artifact of the I2C driver handling the TXIS interrupt condition.  Tolerate the overread
			// instead of reporting an error (which produces logspam).
			*val = 0;
			monitor_i2c_data.state = MONITOR_I2C_IDLE;
			return 0;
		default:
			monitor_i2c_data.state = MONITOR_I2C_ERROR;
			return -1;
	}
}

static int monitor_i2c_target_stop_handler(struct i2c_target_config *config) {
	return 0;
}

static struct i2c_target_callbacks monitor_i2c_target_callbacks = {
	.write_requested = monitor_i2c_target_write_requested_handler,
	.write_received = monitor_i2c_target_write_received_handler,
	.read_requested = monitor_i2c_target_read_requested_or_processed_handler,
	.read_processed = monitor_i2c_target_read_requested_or_processed_handler,
	.stop = monitor_i2c_target_stop_handler,
};

static struct i2c_target_config monitor_i2c_target_config = {
    .address = MONITOR_I2C_TARGET_ADDRESS,
    .callbacks = &monitor_i2c_target_callbacks,
};

int monitor_init(void) {
    return i2c_target_register(monitor_i2c_bus, &monitor_i2c_target_config);
}

void monitor_set_status(struct monitor_status value) {
	monitor_register_write(MONITOR_REGISTER_STATUS, *(uint32_t *)&value);
}

void monitor_set_lift_debug(struct monitor_lift_debug value) {
	monitor_register_write(MONITOR_REGISTER_LIFT_DEBUG, *(uint32_t *)&value);
}

void monitor_set_span_debug(struct monitor_span_debug value) {
	monitor_register_write(MONITOR_REGISTER_SPAN_DEBUG, *(uint32_t *)&value);
}

struct monitor_setting monitor_get_setting(void) {
	uint32_t value = monitor_register_read(MONITOR_REGISTER_SETTING);
	return *(struct monitor_setting *)&value;
}

struct monitor_span_test monitor_get_span_test(void) {
	uint32_t value = monitor_register_read(MONITOR_REGISTER_SPAN_TEST);
	return *(struct monitor_span_test *)&value;
}
