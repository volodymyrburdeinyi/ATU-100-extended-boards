/* tune_algo.c — the one translation unit that compiles the tuning algorithm.
 *
 * tune_algo.h holds the algorithm bodies so that tools/atusim.c can compile the
 * exact same source with a host compiler. This file supplies the firmware side
 * of the contract documented at the top of that header: the globals, the relay
 * and measurement HAL, and the platform's measure_freq().
 */

#include "cross_compiler.h"
#include "globals.h"
#include "relay.h"
#include "swr.h"
#include "tune_api.h"

/* no-op stub in main.c — headless station, no display */
void lcd_ind(void);

/* ── frequency measurement ──
 * Returns 0 until 74HC4060 counter hardware is wired to RD0. With no measured
 * frequency the state machine falls back to the kHz hint from "t HHHH", which
 * is how band memory works on a headless station. */
static unsigned int measure_freq(void)
{
   return 0;
}

#include "tune_algo.h"
