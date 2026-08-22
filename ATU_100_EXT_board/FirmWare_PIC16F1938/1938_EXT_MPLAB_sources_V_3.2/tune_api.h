/* tune_api.h — what the rest of the firmware may call in the tuning algorithm.
 *
 * The algorithm itself lives in tune_algo.h and is compiled into exactly one
 * translation unit (tune_algo.c). Everything not declared here is internal to
 * it. main.c and uart_cmd.c include this header, not tune_algo.h — otherwise
 * each of them gets a private copy of the whole algorithm and of g_tune_ctx.
 */
#ifndef TUNE_API_H
#define TUNE_API_H

/* Queue a tune. Returns immediately; no-op if one is already running.
 * The main loop drives it to completion via tune_tick(). */
void tune_start(void);

/* Run one sub-phase of a queued tune. Returns 1 on the call that finishes it
 * (relay state settled, g_c_tune_exit valid), 0 otherwise — including when
 * idle. Each call is short enough to keep the main loop responsive. */
unsigned char tune_tick(void);

/* Non-zero while a tune is in progress. */
unsigned char tune_busy(void);

/* Band table index for a frequency in kHz, or 0xFF if out of band. */
unsigned char freq_to_band_idx(unsigned int kHz);

#endif /* TUNE_API_H */
