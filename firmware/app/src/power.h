/**
 * @file
 * @brief Controls the 5V regulator that powers the hall sensors and the expansion port.
 */

#pragma once

#include <stdint.h>

#include "errors.h"

int power_init(void);

#define POWER_DEMAND_LIFT (1u << 0)
#define POWER_DEMAND_SPAN (1u << 1)
#define POWER_DEMAND_TEST (1u << 2)

/**
 * @brief Requests that 5V be provided on behalf of the specified consumer.
 */
void power_5v_request(unsigned demand);

/**
 * @brief Releases the request for power on behalf of the specified consumer.
 */
void power_5v_release(unsigned demand);
