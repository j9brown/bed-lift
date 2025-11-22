/**
 * @file
 * @brief Sets the status indicator color.
 */

#pragma once

#include <stdint.h>

#define INDICATOR_HUE_RED (0)
#define INDICATOR_HUE_YELLOW (10923)
#define INDICATOR_HUE_GREEN (21845)
#define INDICATOR_HUE_CYAN (32768)
#define INDICATOR_HUE_BLUE (43691)
#define INDICATOR_HUE_MAGENTA (54613)

/**
 * @brief Turn the status indicator off.
 */
void status_indicator_off();

/**
 * @brief Turn the status indicator on.
 * 
 * @param hue Color hue in a range from 0 to 65536, cycles every 65536.
 */
void status_indicator_on(unsigned hue);
