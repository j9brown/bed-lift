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
 * The indicator turns on with the given hue for on_time then off for off_time.
 * The time intervals are specified in 0.1 second increments.
 * Terminate an array of pattern entries with an entry with on_time = 0.
 * If the last entry has off_time = 0 then the pattern repeats otherwise it stops.
 */
struct indicator_pattern_entry {
    uint16_t hue;
    uint8_t on_time;
    uint8_t off_time;
};
typedef struct indicator_pattern_entry indicator_pattern_t;
#define INDICATOR_PATTERN_ENTRY(hue_, on_time_, off_time_) { .hue = hue_, .on_time = on_time_, .off_time = off_time_ }
#define INDICATOR_PATTERN_ENTRY_REPEAT INDICATOR_PATTERN_ENTRY(0, 0, 0)
#define INDICATOR_PATTERN_ENTRY_END INDICATOR_PATTERN_ENTRY(0, 0, 0xff)
#define INDICATOR_PATTERN_ONCE(name, ...) const indicator_pattern_t name[] = { __VA_ARGS__, INDICATOR_PATTERN_ENTRY_END }
#define INDICATOR_PATTERN_LOOP(name, ...) const indicator_pattern_t name[] = { __VA_ARGS__, INDICATOR_PATTERN_ENTRY_REPEAT }

/**
 * @brief Perform an indicator pattern.
 *
 * The pattern remains active until indicator_on or indicator_off are called.
 * Calling this function repeatedly with the same pattern does not restart it.
 *
 * @param pattern Pattern to perform, or NULL to stop the pattern at the end
 * its current on-time cycle or turn the indicator off immediately if no pattern
 * is running.
 */
void indicator_pattern(const indicator_pattern_t *pattern);
