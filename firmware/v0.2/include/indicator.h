/**
 * @file
 * @brief Sets the status indicator color.
 */

#pragma once

#include <stdint.h>

#define INDICATOR_HUE_RED (0)
#define INDICATOR_HUE_AMBER (5462)
#define INDICATOR_HUE_YELLOW (10923)
#define INDICATOR_HUE_GREEN (21845)
#define INDICATOR_HUE_CYAN (32768)
#define INDICATOR_HUE_BLUE (43691)
#define INDICATOR_HUE_MAGENTA (54613)

/**
 * @brief Turn the indicator off.
 */
void indicator_off();

/**
 * @brief Turn the indicator on.
 *
 * @param hue Color hue in a range from 0 to 65536, cycles every 65536.
 */
void indicator_on(unsigned hue);

/**
 * @brief Defines a pattern of colors for an indicator.
 *
 * The indicator turns on with the given hue for on_time then off for off_time blinking up to cycle times.
 * The time intervals are specified in 0.1 second increments.
 * Terminate an array of pattern entries with an entry with cycles = 0.
 * If the last entry has zero on_time then the pattern repeats after off_time interval, otherwise it stops.
 * If the value of cycles is INDICATOR_PATTERN_PARAM_CYCLES then the pattern parameter determines the count.
 */
struct indicator_pattern_entry {
    unsigned hue : 16;
    unsigned on_time : 5;
    unsigned off_time : 5;
    unsigned cycles : 6;
};
typedef struct indicator_pattern_entry indicator_pattern_t;
#define INDICATOR_PATTERN_PARAM_CYCLES (0x3f)
#define INDICATOR_PATTERN_ENTRY(hue_, on_time_, off_time_, cycles_) { .hue = hue_, .on_time = on_time_, .off_time = off_time_, .cycles = cycles_ }
#define INDICATOR_PATTERN_ENTRY_REPEAT(delay_) INDICATOR_PATTERN_ENTRY(0, 0, delay_, 0)
#define INDICATOR_PATTERN_ENTRY_END INDICATOR_PATTERN_ENTRY(0, 1, 0, 0)
#define INDICATOR_PATTERN_ONCE(name, ...) static const indicator_pattern_t name[] = { __VA_ARGS__, INDICATOR_PATTERN_ENTRY_END }
#define INDICATOR_PATTERN_LOOP(name, delay_, ...) static const indicator_pattern_t name[] = { __VA_ARGS__, INDICATOR_PATTERN_ENTRY_REPEAT(delay_) }

/**
 * @brief Perform an indicator pattern.
 *
 * The pattern remains active until indicator_on or indicator_off are called.
 * Calling this function repeatedly with the same pattern does not restart it.
 *
 * @param pattern Pattern to perform, or NULL to stop the pattern at the end
 * its current on-time cycle or turn the indicator off immediately if no pattern
 * is running.
 * @param param Parameter value for INDICATOR_PATTERN_PARAM_CYCLES, 0 if unused.
 */
void indicator_pattern(const indicator_pattern_t *pattern, unsigned cycles);
