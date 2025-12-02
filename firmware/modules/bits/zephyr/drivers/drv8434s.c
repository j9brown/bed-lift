#define DT_DRV_COMPAT ti_drv8434s

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

#include "drv8434s.h"

struct drv8434s_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec fault_gpio;
	struct gpio_dt_spec sleep_gpio;
	unsigned num_devices;
	unsigned trq_dac : 4;
	unsigned decay : 3;
	unsigned toff : 2;
	unsigned rc_ripple : 3;
	unsigned stall_th : 12;
};

struct drv8434s_data {
	uint8_t *tx_buf;
	uint8_t *rx_buf;
	bool started;
#ifdef CONFIG_SPI_ASYNC
	bool async_busy;
	drv8434s_callback_t async_callback;
	void *async_user_data;
	uint8_t *async_status_buf;
	uint8_t *async_report_buf;
#endif
};

enum drv8434s_reg {
  DRV8434S_REG_FAULT_STATUS = 0x00,
  DRV8434S_REG_DIAG_STATUS1 = 0x01,
  DRV8434S_REG_DIAG_STATUS2 = 0x02,
  DRV8434S_REG_CTRL1 = 0x03,
  DRV8434S_REG_CTRL2 = 0x04,
  DRV8434S_REG_CTRL3 = 0x05,
  DRV8434S_REG_CTRL4 = 0x06,
  DRV8434S_REG_CTRL5 = 0x07,
  DRV8434S_REG_CTRL6 = 0x08,
  DRV8434S_REG_CTRL7 = 0x09,
  DRV8434S_REG_CTRL8 = 0x0a,
  DRV8434S_REG_CTRL9 = 0x0b,
};

// An arbitrary value to confirm that the header made a successful round-trip
// through the chain of devices.
#define DRV8434S_HEADER2_COOKIE (0x1a)

#define DRV8434S_OL_MODE_DEFAULT (0)
#define DRV8434S_EN_OL_DEFAULT (1)
#define DRV8434S_OCP_MODE_DEFAULT (0)
#define DRV8434S_OTSD_MODE_DEFAULT (0)
#define DRV8434S_OTW_REP_DEFAULT (1)
#define DRV8434S_STL_REP_DEFAULT (1)
#define DRV8434S_EN_SSC_REP_DEFAULT (1)
#define DRV8434S_TRQ_SCALE_DEFAULT (0)

#define DRV8434S_DIAG_STATUS2_STL_LRN_OK (0x10)

#define DRV8434S_BUF_SIZE(num_devices) ((num_devices) * 2 + 2)

static inline void drv8434s_tx_buf_header1(uint8_t *tx_buf, unsigned num_devices) {
	tx_buf[0] = 0x80 | num_devices;
}

static inline void drv8434s_tx_buf_header2(uint8_t *tx_buf, bool clear_fault) {
	// Note: Clear fault takes effect after the chip select goes inactive.
	tx_buf[1] = (clear_fault ? 0xa0 : 0x80) | DRV8434S_HEADER2_COOKIE;
}

static inline void drv8434s_tx_buf_reg_write_one(uint8_t *tx_buf, unsigned num_devices,
		unsigned index, unsigned reg_addr, uint8_t reg_value) {
	tx_buf[1 + num_devices - index] = reg_addr << 1;
	tx_buf[1 + num_devices * 2 - index] = reg_value;
}

static inline void drv8434s_tx_buf_reg_write_all(uint8_t *tx_buf, unsigned num_devices,
		unsigned reg_addr, uint8_t reg_value) {
	for (unsigned i = 0; i < num_devices; i++) {
		drv8434s_tx_buf_reg_write_one(tx_buf, num_devices, i, reg_addr, reg_value);
	}
}

static inline void drv8434s_tx_buf_reg_read_one(uint8_t *tx_buf, unsigned num_devices,
		unsigned index, unsigned reg_addr) {
	tx_buf[1 + num_devices - index] = (reg_addr | 0x20) << 1;
	tx_buf[1 + num_devices * 2 - index] = 0;
}

static inline void drv8434s_tx_buf_reg_read_all(uint8_t *tx_buf, unsigned num_devices,
		unsigned reg_addr) {
	for (unsigned i = 0; i < num_devices; i++) {
		drv8434s_tx_buf_reg_read_one(tx_buf, num_devices, i, reg_addr);
	}
}

static inline uint8_t drv8434s_rx_buf_status(const uint8_t *rx_buf, unsigned num_devices, unsigned index) {
	return rx_buf[num_devices - index - 1];
}

static inline uint8_t drv8434s_rx_buf_report(const uint8_t *rx_buf, unsigned num_devices, unsigned index) {
	return rx_buf[1 + num_devices * 2 - index];
}

static inline uint8_t drv8434s_make_ctrl2(const struct drv8434s_config *config, bool output_enable) {
	return (output_enable ? 0x80 : 0x00) | ((config->toff & 0x02) << 3) | ((config->decay & 0x07) << 0);
}

static inline uint8_t drv8434s_make_ctrl5(const struct drv8434s_config *config, bool stall_learn) {
	return (stall_learn ? 0x30 : config->stall_th ? 0x10 : 0) | (DRV8434S_STL_REP_DEFAULT << 3);
}

static int drv8434s_transceive_complete(const struct drv8434s_config *config, struct drv8434s_data *data,
		uint8_t *status_buf, uint8_t *report_buf) {
	const unsigned num_devices = config->num_devices;
	uint8_t *tx_buf = data->tx_buf;
	uint8_t *rx_buf = data->rx_buf;
	if (rx_buf[num_devices] != tx_buf[0] ||
			rx_buf[num_devices + 1] != tx_buf[1]) {
		// Header failed to make a successful round-trip
		return -EIO;
	}
	for (unsigned i = 0; i < num_devices; i++) {
		uint8_t status = drv8434s_rx_buf_status(rx_buf, num_devices, i);
		if ((status & 0xc0) != 0xc0) {
			// Status bytes should have their leading bits set
			return -EIO;
		}
		if (status_buf) {
			status_buf[i] = status & 0x3f;
		}
		if (report_buf) {
			report_buf[i] = drv8434s_rx_buf_report(rx_buf, num_devices, i);
		}
	}
	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static void drv8434s_spi_callback(const struct device *spi_dev, int result, void *user_data) {
	const struct device *dev = user_data;
	const struct drv8434s_config *config = dev->config;
	struct drv8434s_data *data = dev->data;
	if (!result) {
		result = drv8434s_transceive_complete(config, data, data->async_status_buf, data->async_report_buf);
	}
	data->async_callback(dev, result, data->async_user_data);
	data->async_busy = false;
}
#endif

static int drv8434s_transceive(const struct device *dev, uint8_t *status_buf, uint8_t *report_buf,
		drv8434s_callback_t callback, void *user_data) {
	const struct drv8434s_config *config = dev->config;
	const unsigned num_devices = config->num_devices;
	struct drv8434s_data *data = dev->data;
	const struct spi_buf spi_tx_buf = { .buf = data->tx_buf, .len = DRV8434S_BUF_SIZE(num_devices) };
	const struct spi_buf spi_rx_buf = { .buf = data->rx_buf, .len = DRV8434S_BUF_SIZE(num_devices) };
	const struct spi_buf_set spi_tx_buf_set = { .buffers = &spi_tx_buf, .count = 1 };
	const struct spi_buf_set spi_rx_buf_set = { .buffers = &spi_rx_buf, .count = 1 };

	int err;
#ifdef CONFIG_SPI_ASYNC
	if (callback) {
		data->async_busy = true;
		data->async_callback = callback;
		data->async_user_data = user_data;
		data->async_status_buf = status_buf;
		data->async_report_buf = report_buf;
		if ((err = spi_transceive_cb(config->spi.bus, &config->spi.config, &spi_tx_buf_set, &spi_rx_buf_set,
				drv8434s_spi_callback, (void*)dev))) {
			data->async_busy = false;
			return err;
		}
		return 0;
	}
#endif
	if ((err = spi_transceive_dt(&config->spi, &spi_tx_buf_set, &spi_rx_buf_set))) {
		return err;
	}
	return drv8434s_transceive_complete(config, data, status_buf, report_buf);
}

static int drv8434s_init(const struct device *dev) {
	const struct drv8434s_config *config = dev->config;
	if (!spi_is_ready_dt(&config->spi) ||
			(config->fault_gpio.port && !gpio_is_ready_dt(&config->fault_gpio)) ||
			(config->sleep_gpio.port && !gpio_is_ready_dt(&config->sleep_gpio))) {
		return -ENODEV;
	}

	int err;
	if (config->fault_gpio.port && (err = gpio_pin_configure_dt(&config->fault_gpio, GPIO_INPUT))) {
		return err;
	}
	if (config->sleep_gpio.port && (err = gpio_pin_configure_dt(&config->sleep_gpio, GPIO_OUTPUT_ACTIVE))) {
		return err;
	}

	// All transfers use the same initial header byte so we set it just once here.
	const struct drv8434s_data *data = dev->data;
	drv8434s_tx_buf_header1(data->tx_buf, config->num_devices);
	return 0;
}

int drv8434s_start(const struct device *dev, unsigned num_devices) {
	const struct drv8434s_config *config = dev->config;
	struct drv8434s_data *data = dev->data;
	if (num_devices != config->num_devices) return -EINVAL;
	if (data->started) return -EINVAL;
#ifdef CONFIG_SPI_ASYNC
	if (data->async_busy) return -EBUSY;
#endif

	int err;
	if (config->sleep_gpio.port) {
		if ((err = gpio_pin_set_dt(&config->sleep_gpio, false))) {
			return err;
		}
		k_msleep(2); // Datasheet says wake-up time is between 0.8 and 1.2 ms
	}

	uint8_t *tx_buf = data->tx_buf;
	drv8434s_tx_buf_header2(tx_buf, true); // clear faults
	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL1,
		((config->trq_dac & 0x0f) << 4) | (DRV8434S_OL_MODE_DEFAULT << 1));
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		goto error_maybe_sleep;
	}

	drv8434s_tx_buf_header2(tx_buf, false); // don't clear faults
	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL2,
		drv8434s_make_ctrl2(config, false));
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		goto error_maybe_sleep;
	}

	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL3,
		drv8434s_make_step_request(false, false, DRV8434S_MICROSTEP_16));
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		goto error_maybe_sleep;
	}

	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL4,
		(DRV8434S_EN_OL_DEFAULT << 3) | (DRV8434S_OCP_MODE_DEFAULT << 2) |
		(DRV8434S_OTSD_MODE_DEFAULT << 1) | (DRV8434S_OTW_REP_DEFAULT << 0));
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		goto error_maybe_sleep;
	}

	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL5,
		drv8434s_make_ctrl5(config, false));
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		goto error_maybe_sleep;
	}

	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL6,
		config->stall_th & 0xff);
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		goto error_maybe_sleep;
	}

	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL7,
		((config->rc_ripple & 0x03) << 6) | (DRV8434S_EN_SSC_REP_DEFAULT << 5) |
		(DRV8434S_TRQ_SCALE_DEFAULT << 4) | ((config->stall_th & 0xf00) >> 8));
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		goto error_maybe_sleep;
	}

	data->started = true;
	return 0;

error_maybe_sleep:
	if (config->sleep_gpio.port) {
		gpio_pin_set_dt(&config->sleep_gpio, true);
	}
	return err;
}

int drv8434s_stop(const struct device *dev) {
	struct drv8434s_data *data = dev->data;
	if (!data->started) return -EINVAL;
#ifdef CONFIG_SPI_ASYNC
	if (data->async_busy) return -EBUSY;
#endif

	const struct drv8434s_config *config = dev->config;
	const unsigned num_devices = config->num_devices;
	uint8_t *tx_buf = data->tx_buf;
	drv8434s_tx_buf_header2(tx_buf, false); // don't clear faults
	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL2,
		drv8434s_make_ctrl2(config, false));
	int err1 = drv8434s_transceive(dev, NULL, NULL, NULL, NULL);
	int err2 = config->sleep_gpio.port ? gpio_pin_set_dt(&config->sleep_gpio, true) : 0;
	data->started = false;
	return err1 ? err1 : err2;
}

int drv8434s_has_fault(const struct device *dev) {
	const struct drv8434s_config *config = dev->config;
	if (!config->fault_gpio.port) {
		return -ENOTSUP;
	}
	return gpio_pin_get_dt(&config->fault_gpio);
}

int drv8434s_get_fault_status(const struct device *dev, const struct drv8434s_options *options,
        uint8_t *ex_status_buf) {
	__ASSERT(options, "options must not be NULL");

	struct drv8434s_data *data = dev->data;
	if (!data->started) return -EINVAL;
#ifdef CONFIG_SPI_ASYNC
	if (data->async_busy) return -EBUSY;
#endif

	const struct drv8434s_config *config = dev->config;
	const unsigned num_devices = config->num_devices;
	uint8_t *tx_buf = data->tx_buf;
	drv8434s_tx_buf_header2(tx_buf, options->clear_fault);
	drv8434s_tx_buf_reg_read_all(tx_buf, num_devices, DRV8434S_REG_FAULT_STATUS);
	return drv8434s_transceive(dev, options->status_buf, ex_status_buf, NULL, NULL);
}

int drv8434s_set_output_enable(const struct device *dev, const struct drv8434s_options *options,
        uint64_t enabled_set) {
	__ASSERT(options, "options must not be NULL");

	struct drv8434s_data *data = dev->data;
	if (!data->started) return -EINVAL;
#ifdef CONFIG_SPI_ASYNC
	if (data->async_busy) return -EBUSY;
#endif

	const struct drv8434s_config *config = dev->config;
	const unsigned num_devices = config->num_devices;
	uint8_t *tx_buf = data->tx_buf;
	drv8434s_tx_buf_header2(tx_buf, options->clear_fault);
	for (unsigned i = 0; i < num_devices; i++) {
		drv8434s_tx_buf_reg_write_one(tx_buf, num_devices, i, DRV8434S_REG_CTRL2,
			drv8434s_make_ctrl2(config, IS_BIT_SET(enabled_set, i)));
	}
	return drv8434s_transceive(dev, options->status_buf, NULL, NULL, NULL);
}

int drv8434s_step_async(const struct device *dev, const struct drv8434s_options *options,
        const uint8_t *step_requests, drv8434s_callback_t callback, void *user_data) {
	__ASSERT(options, "options must not be NULL");
	__ASSERT(step_requests, "step_requests must not be NULL");

	struct drv8434s_data *data = dev->data;
	if (!data->started) return -EINVAL;
#ifdef CONFIG_SPI_ASYNC
	if (data->async_busy) return -EBUSY;
#endif

	const struct drv8434s_config *config = dev->config;
	const unsigned num_devices = config->num_devices;
	uint8_t *tx_buf = data->tx_buf;
	drv8434s_tx_buf_header2(tx_buf, options->clear_fault);
	for (unsigned i = 0; i < num_devices; i++) {
		drv8434s_tx_buf_reg_write_one(tx_buf, num_devices, i, DRV8434S_REG_CTRL3,
			step_requests[i]);
	}
	return drv8434s_transceive(dev, options->status_buf, NULL, callback, user_data);
}

int drv8434s_step(const struct device *dev, const struct drv8434s_options *options,
        const uint8_t* step_requests) {
    return drv8434s_step_async(dev, options, step_requests, NULL, NULL);
}

int drv8434s_set_stall_learn_mode(const struct device *dev, const struct drv8434s_options *options,
        bool stall_learn) {
	__ASSERT(options, "options must not be NULL");

	struct drv8434s_data *data = dev->data;
	if (!data->started) return -EINVAL;
#ifdef CONFIG_SPI_ASYNC
	if (data->async_busy) return -EBUSY;
#endif

	const struct drv8434s_config *config = dev->config;
	const unsigned num_devices = config->num_devices;
	uint8_t *tx_buf = data->tx_buf;
	drv8434s_tx_buf_header2(tx_buf, options->clear_fault);
	drv8434s_tx_buf_reg_write_all(tx_buf, num_devices, DRV8434S_REG_CTRL5,
		drv8434s_make_ctrl5(config, stall_learn));
	return drv8434s_transceive(dev, options->status_buf, NULL, NULL, NULL);
}

int drv8434s_get_stall_learn_result(const struct device *dev, const struct drv8434s_options *options,
        uint64_t *learned_set, uint16_t *stall_th_buf) {
	__ASSERT(options, "options must not be NULL");

	struct drv8434s_data *data = dev->data;
	if (!data->started) return -EINVAL;
#ifdef CONFIG_SPI_ASYNC
	if (data->async_busy) return -EBUSY;
#endif

	const struct drv8434s_config *config = dev->config;
	const unsigned num_devices = config->num_devices;
	uint8_t *tx_buf = data->tx_buf;
	drv8434s_tx_buf_header2(tx_buf, options->clear_fault);
	drv8434s_tx_buf_reg_read_all(tx_buf, num_devices, DRV8434S_REG_DIAG_STATUS2);
	int err;
	if ((err = drv8434s_transceive(dev, options->status_buf, NULL, NULL, NULL))) {
		return err;
	}

	*learned_set = 0;
	uint8_t *rx_buf = data->rx_buf;
	for (unsigned i = 0; i < num_devices; i++) {
		if (drv8434s_rx_buf_report(rx_buf, num_devices, i) & DRV8434S_DIAG_STATUS2_STL_LRN_OK) {
			*learned_set |= BIT64(i);
		}
	}

	drv8434s_tx_buf_header2(tx_buf, false);
	drv8434s_tx_buf_reg_read_all(tx_buf, num_devices, DRV8434S_REG_CTRL6);
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		return err;
	}
	for (unsigned i = 0; i < num_devices; i++) {
		stall_th_buf[i] = drv8434s_rx_buf_report(rx_buf, num_devices, i);
	}
	drv8434s_tx_buf_reg_read_all(tx_buf, num_devices, DRV8434S_REG_CTRL7);
	if ((err = drv8434s_transceive(dev, NULL, NULL, NULL, NULL))) {
		return err;
	}
	for (unsigned i = 0; i < num_devices; i++) {
		stall_th_buf[i] |= (drv8434s_rx_buf_report(rx_buf, num_devices, i) & 0x0f) << 8;
	}
	return 0;
}

// TODO: In Zephyr 4.3, the delay property (50) in SPI_DT_SPEC_INST_GET has moved to the devicetree.
#define DRV8434S_CHECK_NUM_DEVICES(x) ((x) >= 1 && (x) <= 63)
#define DRV8434S_DEFINE(inst)                                                                       \
	BUILD_ASSERT(DRV8434S_CHECK_NUM_DEVICES(DT_PROP(DT_DRV_INST(inst), num_devices)));              \
	static const struct drv8434s_config drv8434s_##inst##_config = {                                \
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPHA | SPI_WORD_SET(8), 50),   \
		.fault_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(inst), fault_gpios, {0}),                     \
		.sleep_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(inst), sleep_gpios, {0}),                     \
		.num_devices = DT_PROP(DT_DRV_INST(inst), num_devices),                     \
		.trq_dac = DT_PROP(DT_DRV_INST(inst), trq_dac),                     \
		.decay = DT_PROP(DT_DRV_INST(inst), decay),                     \
		.toff = DT_PROP(DT_DRV_INST(inst), toff),                     \
		.rc_ripple = DT_PROP(DT_DRV_INST(inst), rc_ripple),                     \
		.stall_th = DT_PROP(DT_DRV_INST(inst), stall_th),                     \
    };                                                                                              \
	static uint8_t drv8434s_##inst##_tx_buf[DRV8434S_BUF_SIZE(DT_PROP(DT_DRV_INST(inst), num_devices))]; \
	static uint8_t drv8434s_##inst##_rx_buf[DRV8434S_BUF_SIZE(DT_PROP(DT_DRV_INST(inst), num_devices))]; \
                                                                                                   \
	static struct drv8434s_data drv8434s_##inst##_data = {                                       \
		.tx_buf = drv8434s_##inst##_tx_buf,                                                \
		.rx_buf = drv8434s_##inst##_rx_buf,                                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, drv8434s_init, NULL, &drv8434s_##inst##_data,                    \
			      &drv8434s_##inst##_config, POST_KERNEL,                               \
			      CONFIG_DRV8434S_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV8434S_DEFINE)
