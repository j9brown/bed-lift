#pragma once

#include <stdint.h>

// TODO: Transform this into a device driver

#define INDICATOR_HUE_RED (0)
#define INDICATOR_HUE_YELLOW (10923)
#define INDICATOR_HUE_GREEN (21845)
#define INDICATOR_HUE_CYAN (32768)
#define INDICATOR_HUE_BLUE (43691)
#define INDICATOR_HUE_MAGENTA (54613)

/**
 * Turns the status indicator off.
 */
void status_indicator_off();

/**
 * Turns the status indicator on.
 * 
 * @param hue The color hue in a range from 0 to 65536, cycles every 65536.
 */
void status_indicator_on(unsigned hue);
