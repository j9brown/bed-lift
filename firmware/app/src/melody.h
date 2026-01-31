/**
 * @file
 * @brief Plays melodies through a piezo buzzer.
 */

#pragma once

#include <stdint.h>

/**
 * @brief A musical tone in a melody.
 * 
 * The tone's frequency is specified in Hz.  If it is zero, no tone is played (performs a rest).
 * The tone's duration is specified in ms.  If it is zero, the melody ends.
 */
struct melody_tone {
    uint16_t freq;
    uint16_t duration;
};
_Static_assert(sizeof(struct melody_tone) == 4, "");
typedef struct melody_tone melody_t;

#define MELODY_NOTE_C6 (1047)
#define MELODY_NOTE_D6 (1175)
#define MELODY_NOTE_E6 (1319)
#define MELODY_NOTE_E6 (1319)
#define MELODY_NOTE_F6 (1397)
#define MELODY_NOTE_G6 (1568)

#define MELODY_TONE(freq_, duration_) { .freq = (freq_), .duration = (duration_) }
#define MELODY_NOTE(name_, duration_) MELODY_TONE(_CONCAT(MELODY_NOTE_, name_), (duration_))
#define MELODY_REST(duration_) { .freq = 0, .duration = (duration_) }
#define MELODY_END { .freq = 0, .duration = 0 }
#define MELODY(name, ...) static const melody_t name[] = { __VA_ARGS__, MELODY_END }

int melody_init(void);

/**
 * @brief Stops the melody.
 */
void melody_stop(void);

/**
 * @brief Plays a melody.
 *
 * Calling this function repeatedly with the same melody does not restart it.
 *
 * @param melody Melody to perform, or NULL to stop the current melody
 */
void melody_play(const melody_t *melody);
