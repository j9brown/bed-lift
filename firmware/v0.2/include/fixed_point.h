/**
 * @file
 * @brief Fixed point math functions.
 */

#pragma once

#include <stdint.h>

/**
 * @brief A signed 16.15 bit fixed point value.
 */
typedef int32_t q15_t;

#define Q15_ONE ((q15_t)(1 << 15))
#define Q15_CONST(x) ((q15_t)((x) * 32768))

static inline int q15_to_int(q15_t x) {
    return x >> 15;
}

static inline q15_t int_to_q15(int x) {
    return x << 15;
}

static inline q15_t q15_clamp(q15_t value, q15_t min, q15_t max) {
    return value < min ? min : value > max ? max : value;
}

static inline q15_t q15_mul_q15(q15_t x, q15_t y) {
    return (x * y) >> 15;
}
