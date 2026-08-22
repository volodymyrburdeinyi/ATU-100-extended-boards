/* tune_algo.h — ATU-100 EXT tuning algorithm
 *
 * Single source of truth for:
 *   coarse_cap / coarse_tune / sharp_cap / sharp_ind
 *   band_slot_save / freq_to_band_idx / BAND_LO / BAND_HI
 *   tune_start / tune_tick / tune_busy  (cooperative state machine)
 *
 * Included by exactly ONE translation unit per program — it defines g_tune_ctx:
 *   tune_algo.c       — firmware build (MPLAB X, XC8)
 *   tools/atusim.c    — simulator build (gcc)
 * Other firmware modules use tune_api.h.
 *
 * Requirements from the including translation unit:
 *   Types     : unsigned char, unsigned int, int, char
 *   Defines   : EEPROM_BAND_*, EEPROM_SLOT_*, EEPROM_BAND_N, EEPROM_BAND_SUB_N,
 *               EEPROM_BAND_FREQ_TOL_KHZ, EEPROM_BAND_SLOT_0, EEPROM_BAND_SLOT_STRIDE
 *   Globals   : g_c_ind, g_c_cap, g_c_SW, g_c_step_cap, g_c_step_ind,
 *               g_c_L_mult, g_c_C_mult, g_i_SWR, g_i_P_max, g_i_swr_a,
 *               g_b_rready, g_char_p_cnt, g_b_tx_seen, g_char_tune_effort,
 *               e_c_b_L_linear, e_c_b_C_linear, e_c_num_L_q, e_c_num_C_q,
 *               e_i_tenths_init_max_swr, e_c_b_Loss_ind
 *   [UART]    : g_i_uart_freq_hint, g_b_slot_saved, g_c_tune_exit, g_b_debug_mode
 *   HAL fns   : get_swr(), set_ind(), set_cap(), set_sw(), atu_reset(),
 *               lcd_ind(), eeprom_read(), eeprom_write(),
 *               Vdelay_ms(), Delay_ms(), CLRWDT()
 *   Platform  : measure_freq() — firmware returns 0; sim returns sim_freq
 *
 * UART must be defined in both firmware (cross_compiler.h) and simulator
 * (explicit #define) so that g_b_slot_saved / g_c_tune_exit compile in both.
 */
#ifndef TUNE_ALGO_H
#define TUNE_ALGO_H

/* ── DBG telemetry helpers (firmware only; compiled away in sim) ──
 * MPLAB_COMPILER is only defined when cross_compiler.h is included (firmware TU).
 * The simulator defines UART but not MPLAB_COMPILER, so these helpers compile away
 * and never reference uart_puts or IntToStr, which do not exist in the sim. */
#if defined(UART) && defined(MPLAB_COMPILER)
    extern unsigned char g_b_debug_mode;
    void uart_cmd_proc(void);   /* forward decl — defined in uart_cmd.c */

    /* Emit one integer field "key=val" with no newline. Caller wraps in
     * if (g_b_debug_mode) so the flag is checked only once per line. */
    static void tune_dbg_uint(const char *key, int val)
    {
        char l_s[7];
        unsigned char l_i = 0;
        uart_puts(key);
        IntToStr(val, l_s);
        while (l_i < 5u && l_s[l_i] == ' ') l_i++;
        uart_puts(l_s + l_i);
    }
#endif /* UART && MPLAB_COMPILER */

/* ── forward: platform-specific frequency measurement ── */
static unsigned int measure_freq(void);

#ifdef UART
/* Set by the "q" command. Declared here, ahead of the scan loops that poll it —
 * they are the first code in this file to reference it. */
extern unsigned char g_b_tune_abort;
#endif

/* ── band table — 10 ham bands, lo/hi in kHz ── */
/* 160m  80m  40m   30m    20m    17m    15m    12m    10m    6m */
static const unsigned int BAND_LO[10] = {1800, 3500, 7000, 10100, 14000, 18068, 21000, 24890, 28000, 50000};
static const unsigned int BAND_HI[10] = {2000, 4000, 7300, 10150, 14350, 18168, 21450, 24990, 29700, 54000};

unsigned char freq_to_band_idx(unsigned int kHz)
{
   unsigned char l_b;
   for (l_b = 0; l_b < (unsigned char)EEPROM_BAND_N; l_b++)
      if (kHz >= BAND_LO[l_b] && kHz <= BAND_HI[l_b]) return l_b;
   return 0xFF;
}

/* ── coarse capacitor scan ── */
static void coarse_cap(void)
{
   unsigned char l_coarse_cap_step = 3;
   unsigned char l_coarse_cap_count;
   int l_coarse_cap_min_swr;

   g_c_cap = 0;
   set_cap(g_c_cap);
   g_c_step_cap = l_coarse_cap_step;
   get_swr();
   if (g_i_SWR == 0)
      return;
   l_coarse_cap_min_swr = g_i_SWR;
   for (l_coarse_cap_count = l_coarse_cap_step; l_coarse_cap_count <= 31;)
   {
      set_cap(l_coarse_cap_count * g_c_C_mult);
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR < l_coarse_cap_min_swr)
      {
         l_coarse_cap_min_swr = g_i_SWR;
         g_c_cap = l_coarse_cap_count * g_c_C_mult;
         g_c_step_cap = l_coarse_cap_step;
         if (g_i_SWR < 120)
            break;
      }
      l_coarse_cap_count += l_coarse_cap_step;
      if (e_c_b_C_linear == 0 && l_coarse_cap_count == 9)
         l_coarse_cap_count = 8;
      else if (e_c_b_C_linear == 0 && l_coarse_cap_count == 17)
      {
         l_coarse_cap_count = 16;
         l_coarse_cap_step = 4;
      }
   }
   set_cap(g_c_cap);
   return;
}

/* ── coarse inductor+capacitor 2-D scan ── */
static void coarse_tune(void)
{
   unsigned char l_coarse_tune_step = 3;
   unsigned char l_coarse_tune_count;
   unsigned char l_coarse_tune_mem_cap, l_coarse_tune_mem_step_cap;
   int l_coarse_tune_min_swr;

   l_coarse_tune_mem_cap = 0;
   g_c_step_ind = l_coarse_tune_step;
   l_coarse_tune_mem_step_cap = 3;
   l_coarse_tune_min_swr = 9999;
   for (l_coarse_tune_count = 0; l_coarse_tune_count <= 31;)
   {
      CLRWDT();  /* up to 11 L × 11 C × ~30 ms each ≈ 3.6 s > 1 s WDT timeout */
#if defined(UART) && defined(MPLAB_COMPILER)
      /* Safe to call here: daemon sends only 'q' during a tune; any other
       * command's TX response (~10 ms) is within the relay-step budget. */
      uart_cmd_proc();
      if (g_b_tune_abort) return;
#endif
      set_ind(l_coarse_tune_count * g_c_L_mult);
      coarse_cap();
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR < l_coarse_tune_min_swr)
      {
         l_coarse_tune_min_swr = g_i_SWR;
         g_c_ind = l_coarse_tune_count * g_c_L_mult;
         l_coarse_tune_mem_cap = g_c_cap;
         g_c_step_ind = l_coarse_tune_step;
         l_coarse_tune_mem_step_cap = g_c_step_cap;
         if (g_i_SWR < 120)
            break;
      }
      l_coarse_tune_count += l_coarse_tune_step;
      if (e_c_b_L_linear == 0 && l_coarse_tune_count == 9)
         l_coarse_tune_count = 8;
      else if (e_c_b_L_linear == 0 && l_coarse_tune_count == 17)
      {
         l_coarse_tune_count = 16;
         l_coarse_tune_step = 4;
      }
   }
   g_c_cap = l_coarse_tune_mem_cap;
   set_ind(g_c_ind);
   set_cap(g_c_cap);
   g_c_step_cap = l_coarse_tune_mem_step_cap;
   Delay_ms(10);
   return;
}

/* ── fine capacitor scan (sharp pass) — full-range min-tracking ── */
static void sharp_cap(void)
{
   unsigned char l_sharp_cap_range, l_sharp_cap_count, l_sharp_cap_max_range, l_sharp_cap_min_range;
   int l_sharp_cap_min_SWR;
#if defined(UART) && defined(MPLAB_COMPILER)
   unsigned char l_uart_check = 0;
#endif
   l_sharp_cap_range = g_c_step_cap * g_c_C_mult;

   l_sharp_cap_max_range = g_c_cap + l_sharp_cap_range;
   if (l_sharp_cap_max_range > 32 * g_c_C_mult - 1)
      l_sharp_cap_max_range = 32 * g_c_C_mult - 1;
   if (g_c_cap > l_sharp_cap_range)
      l_sharp_cap_min_range = g_c_cap - l_sharp_cap_range;
   else
      l_sharp_cap_min_range = 0;
   g_c_cap = l_sharp_cap_min_range;
   set_cap(g_c_cap);
   get_swr();
   if (g_i_SWR == 0)
      return;
   l_sharp_cap_min_SWR = g_i_SWR;
   for (l_sharp_cap_count = l_sharp_cap_min_range + g_c_C_mult;
        l_sharp_cap_count <= l_sharp_cap_max_range;
        l_sharp_cap_count += g_c_C_mult)
   {
      CLRWDT();  /* full-range scan with retries: up to ~30 steps × 3 reads × ~10 ms each */
#if defined(UART) && defined(MPLAB_COMPILER)
      if (++l_uart_check >= 5u) { l_uart_check = 0; uart_cmd_proc(); if (g_b_tune_abort) return; }
#endif
      set_cap(l_sharp_cap_count);
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR >= l_sharp_cap_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR >= l_sharp_cap_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR < l_sharp_cap_min_SWR)
      {
         l_sharp_cap_min_SWR = g_i_SWR;
         g_c_cap = l_sharp_cap_count;
         if (g_i_SWR < 120)
            break;
      }
      /* no else break — scan full range */
   }
   set_cap(g_c_cap);
   return;
}

/* ── fine inductor scan (sharp pass) — full-range min-tracking ── */
static void sharp_ind(void)
{
   unsigned char l_sharp_ind_range, l_sharp_ind_count, l_sharp_ind_max_range, l_sharp_ind_min_range;
   int l_sharp_ind_min_SWR;
#if defined(UART) && defined(MPLAB_COMPILER)
   unsigned char l_uart_check = 0;
#endif
   l_sharp_ind_range = g_c_step_ind * g_c_L_mult;

   l_sharp_ind_max_range = g_c_ind + l_sharp_ind_range;
   if (l_sharp_ind_max_range > 32 * g_c_L_mult - 1)
      l_sharp_ind_max_range = 32 * g_c_L_mult - 1;
   if (g_c_ind > l_sharp_ind_range)
      l_sharp_ind_min_range = g_c_ind - l_sharp_ind_range;
   else
      l_sharp_ind_min_range = 0;
   g_c_ind = l_sharp_ind_min_range;
   set_ind(g_c_ind);
   get_swr();
   if (g_i_SWR == 0)
      return;
   l_sharp_ind_min_SWR = g_i_SWR;
   for (l_sharp_ind_count = l_sharp_ind_min_range + g_c_L_mult;
        l_sharp_ind_count <= l_sharp_ind_max_range;
        l_sharp_ind_count += g_c_L_mult)
   {
      CLRWDT();  /* full-range scan with retries: up to ~30 steps × 3 reads × ~10 ms each */
#if defined(UART) && defined(MPLAB_COMPILER)
      if (++l_uart_check >= 5u) { l_uart_check = 0; uart_cmd_proc(); if (g_b_tune_abort) return; }
#endif
      set_ind(l_sharp_ind_count);
      get_swr();
      if (g_i_SWR == 0)
         return;
      if (g_i_SWR >= l_sharp_ind_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR >= l_sharp_ind_min_SWR)
      {
         Delay_ms(10);
         get_swr();
      }
      if (g_i_SWR < l_sharp_ind_min_SWR)
      {
         l_sharp_ind_min_SWR = g_i_SWR;
         g_c_ind = l_sharp_ind_count;
         if (g_i_SWR < 120)
            break;
      }
      /* no else break — scan full range */
   }
   set_ind(g_c_ind);
   return;
}

/* ── save tuned relay position to EEPROM band memory ──
 * Saves if SWR < 300 and a valid frequency is known.
 * IND byte is written last — it is the commit sentinel (0xFF = empty slot).
 * Eviction policy: update existing slot within ±25 kHz, else fill empty,
 * else replace the sub-slot with the worst stored SWR.                     */
static void band_slot_save(char l_probe_matched, unsigned int l_freq_kHz)
{
   unsigned char l_band, l_sub, l_slot_idx, l_base, l_ind, l_sw_swr, l_swr10;
   unsigned int  l_sf, l_diff;
   unsigned char l_existing_sub, l_empty_sub, l_worst_sub, l_worst_swr;
   if (l_probe_matched) return;
   if (g_i_SWR == 0 || g_i_SWR >= 300) return;
   if (l_freq_kHz == 0) return;
   l_band = freq_to_band_idx(l_freq_kHz);
   if (l_band == 0xFF) return;
   l_existing_sub = 0xFF;
   l_empty_sub    = 0xFF;
   l_worst_sub    = 0;
   l_worst_swr    = 0;
   for (l_sub = 0; l_sub < (unsigned char)EEPROM_BAND_SUB_N; l_sub++) {
      l_slot_idx = (unsigned char)(l_band * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
      l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
      l_ind = eeprom_read(l_base + EEPROM_SLOT_IND);
      if (l_ind == 0xFF) { if (l_empty_sub == 0xFF) l_empty_sub = l_sub; continue; }
      l_sf = (unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_LO)
           | ((unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_HI) << 8);
      if (l_sf == 0) { if (l_empty_sub == 0xFF) l_empty_sub = l_sub; continue; }
      l_diff = (l_sf > l_freq_kHz) ? (unsigned int)(l_sf - l_freq_kHz)
                                    : (unsigned int)(l_freq_kHz - l_sf);
      if (l_diff <= (unsigned int)EEPROM_BAND_FREQ_TOL_KHZ) { l_existing_sub = l_sub; break; }
      l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
      l_swr10  = (unsigned char)(l_sw_swr & 0x7Fu);
      if (l_swr10 > l_worst_swr) { l_worst_swr = l_swr10; l_worst_sub = l_sub; }
   }
   if (l_existing_sub != 0xFF)   l_sub = l_existing_sub;
   else if (l_empty_sub != 0xFF) l_sub = l_empty_sub;
   else                          l_sub = l_worst_sub;
   l_slot_idx = (unsigned char)(l_band * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
   l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
   l_swr10 = (unsigned char)(g_i_SWR / 10);
   if (l_swr10 > 127) l_swr10 = 127;
   /* write IND last — it doubles as the commit byte (0xFF = empty sentinel).
      If power fails before IND is written, the slot stays empty and is safe to reuse. */
   eeprom_write(l_base + EEPROM_SLOT_FREQ_LO, (unsigned char)(l_freq_kHz & 0xFFu));
   eeprom_write(l_base + EEPROM_SLOT_FREQ_HI, (unsigned char)(l_freq_kHz >> 8));
   eeprom_write(l_base + EEPROM_SLOT_CAP,     (unsigned char)g_c_cap);
   eeprom_write(l_base + EEPROM_SLOT_SW_SWR,  (unsigned char)(((unsigned char)(g_c_SW & 1u) << 7) | l_swr10));
   eeprom_write(l_base + EEPROM_SLOT_IND,     (unsigned char)g_c_ind);
   g_b_slot_saved = 1;
}

/* ── Phase 6: cooperative tune state machine ── */
typedef enum {
    TS_IDLE = 0,
    TS_INIT,        /* snapshot pre-tune state, get initial SWR */
    TS_PROBE,       /* band memory probe — EEPROM reads only */
    TS_RESET,       /* atu_reset + Delay_ms(50) + get_swr */
    TS_COARSE,      /* coarse_tune() for current SW pass */
    TS_SHARP_IND,   /* sharp_ind() for current SW pass */
    TS_SHARP_CAP,   /* sharp_cap() for current SW pass */
    TS_SW_COMPARE,  /* compare pass0 vs pass1, pick winner */
    TS_EXTRA_IND,   /* extra sharp_ind if e_c_num_L_q > 5 */
    TS_EXTRA_CAP,   /* extra sharp_cap if e_c_num_C_q > 5 */
    TS_SAVE,        /* band_slot_save then → TS_IDLE */
    TS_ABORT        /* restore pre_ind/cap/sw then → TS_IDLE */
} tune_state_t;

typedef struct {
    tune_state_t  state;
    /* pre-tune relay snapshot — restored on abort or TX-inhibit */
    unsigned char pre_ind, pre_cap;
    char          pre_sw;
    /* frequency and probe result */
    unsigned int  freq_kHz;
    char          probe_matched;
    /* two-pass SW comparison (cap on antenna side vs TX side) */
    unsigned char pass;          /* 0 = original SW, 1 = flipped SW */
    int           swr_before_flip;   /* SWR before deciding to flip */
    unsigned char pass0_ind, pass0_cap;
    char          pass0_sw;
    int           pass0_swr;    /* best SWR from pass 0 */
    /* exit code to commit when reaching TS_SAVE */
    unsigned char pending_exit;
} tune_ctx_t;

/* Defined here, which is sound because this file is compiled into exactly one
 * translation unit per program: tune_algo.c in the firmware, atusim.c in the
 * simulator. Other modules reach the state machine through tune_api.h.     */
tune_ctx_t g_tune_ctx;

void tune_start(void)
{
    /* If already running, do not restart */
    if (g_tune_ctx.state != TS_IDLE) return;
    g_tune_ctx.state = TS_INIT;
#ifdef UART
    g_b_tune_abort = 0;
#endif
}

unsigned char tune_busy(void)
{
    return g_tune_ctx.state != TS_IDLE;
}

/* Restore the L/C step multipliers to their configured values.
 * TS_EXTRA_IND and TS_EXTRA_CAP force a multiplier to 1 to get a finer scan;
 * every terminal state has to undo that, or the *next* tune scans a fraction
 * of the available range with no indication that anything is wrong. */
static void tune_restore_mults(void)
{
    if (e_c_num_L_q == 5)      g_c_L_mult = 1;
    else if (e_c_num_L_q == 6) g_c_L_mult = 2;
    else if (e_c_num_L_q == 7) g_c_L_mult = 4;
    if (e_c_num_C_q == 5)      g_c_C_mult = 1;
    else if (e_c_num_C_q == 6) g_c_C_mult = 2;
    else if (e_c_num_C_q == 7) g_c_C_mult = 4;
}

/* tune_tick() — cooperative state machine dispatcher.
 * Returns 0 while running, 1 when done (state returned to TS_IDLE).
 * Each call executes exactly one sub-phase so the main loop can service
 * UART between phases without relay.c changes.                          */
unsigned char tune_tick(void)
{
    tune_ctx_t *ctx = &g_tune_ctx;

    switch (ctx->state) {

    case TS_IDLE:
        return 0;

    case TS_INIT:
        /* Snapshot pre-tune relay positions for abort restore */
        ctx->pre_ind = g_c_ind;
        ctx->pre_cap = g_c_cap;
        ctx->pre_sw  = g_c_SW;
        /* Clear per-tune counters */
        g_char_p_cnt      = 0;
        g_i_P_max         = 0;
        g_char_tune_effort = 0;
        g_b_slot_saved    = 0;
        g_c_tune_exit     = 0;
        /* Must be cleared per tune. g_b_tx_seen latches "power was present",
         * which turns a later power dip into a TX-inhibit abort; carried over
         * from a previous tune it makes get_swr() abort before the radio has
         * even keyed, so every tune after the first would silently do nothing.
         * g_b_rready likewise latches the tune-button release. */
        g_b_tx_seen       = 0;
        g_b_rready        = 0;
        ctx->probe_matched = 0;
        ctx->pass          = 0;
        /* Measure frequency; fall back to UART hint if radio is silent */
        ctx->freq_kHz = measure_freq();
#ifdef UART
        if (ctx->freq_kHz == 0 && g_i_uart_freq_hint != 0) {
            ctx->freq_kHz = g_i_uart_freq_hint;
            g_i_uart_freq_hint = 0;
        }
#endif
        get_swr();
        if (g_i_SWR < 110) {
            g_c_tune_exit = 1;
            ctx->pending_exit = 1;
            ctx->state = TS_SAVE;
            return 0;
        }
        ctx->state = TS_PROBE;
        return 0;

    case TS_PROBE:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        if (ctx->freq_kHz == 0) {
            /* No frequency — skip probe, reset SW, proceed to full tune */
            g_c_SW = 0;
            set_sw(g_c_SW);
            ctx->state = TS_RESET;
            return 0;
        }
        {
            unsigned char l_band_idx = freq_to_band_idx(ctx->freq_kHz);
            if (l_band_idx != 0xFF) {
                unsigned char l_sub, l_slot_idx, l_base, l_ind, l_sw_swr;
                unsigned int  l_sf, l_diff;
                for (l_sub = 0; l_sub < (unsigned char)EEPROM_BAND_SUB_N; l_sub++) {
                    l_slot_idx = (unsigned char)(l_band_idx * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
                    l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
                    l_ind = eeprom_read(l_base + EEPROM_SLOT_IND);
                    if (l_ind == 0xFF) continue;
                    l_sf = (unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_LO)
                         | ((unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_HI) << 8);
                    if (l_sf == 0) continue;
                    l_diff = (l_sf > ctx->freq_kHz) ? (unsigned int)(l_sf - ctx->freq_kHz)
                                                     : (unsigned int)(ctx->freq_kHz - l_sf);
                    if (l_diff > (unsigned int)EEPROM_BAND_FREQ_TOL_KHZ) continue;
                    /* Slot matched — apply relays and measure live SWR */
                    g_c_ind = l_ind;
                    g_c_cap = eeprom_read(l_base + EEPROM_SLOT_CAP);
                    l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
                    g_c_SW = (char)((l_sw_swr >> 7) & 1u);
                    set_ind(g_c_ind);
                    set_cap(g_c_cap);
                    set_sw(g_c_SW);
                    get_swr();
                    if (g_i_SWR == 0) {
                        /* TX inhibit during probe — restore pre-tune relays */
                        g_c_tune_exit = 2;
                        ctx->pending_exit = 2;
                        ctx->state = TS_ABORT;
                        return 0;
                    }
                    if (g_i_SWR < 150) {
                        /* Probe hit — upsert if current SWR is better than stored */
                        unsigned char l_stored_swr10 = l_sw_swr & 0x7Fu;
                        g_c_tune_exit = 3;
                        ctx->pending_exit = 3;
                        if (g_i_SWR > 0 && (unsigned char)((unsigned int)g_i_SWR / 10u) < l_stored_swr10)
                            ctx->probe_matched = 0;   /* let band_slot_save overwrite */
                        else
                            ctx->probe_matched = 1;   /* suppress save — stored SWR is already good */
                        ctx->state = TS_SAVE;
                        return 0;
                    }
                    /* Probe applied but SWR still ≥ 150 — fall through to full tune */
                    break;
                }
            }
            /* No usable match — reset SW before full scan */
            g_c_SW = 0;
            set_sw(g_c_SW);
        }
        ctx->state = TS_RESET;
        return 0;

    case TS_RESET:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        g_char_tune_effort = 0;
        atu_reset();
        if (e_c_b_Loss_ind == 0)
            lcd_ind();
        Delay_ms(50);
        get_swr();
        g_i_swr_a = g_i_SWR;
        if (g_i_SWR == 0) {
            ctx->pending_exit = 4;
            ctx->state = TS_ABORT;
            return 0;
        }
        if (g_i_SWR < 110) {
            ctx->pending_exit = 5;
            ctx->state = TS_SAVE;
            return 0;
        }
        if (e_i_tenths_init_max_swr > 110 && g_i_SWR > e_i_tenths_init_max_swr) {
            ctx->pending_exit = 6;
            ctx->state = TS_SAVE;
            return 0;
        }
        /* Store entry SWR for later "good enough improvement" check in TS_SHARP_CAP */
        ctx->swr_before_flip = g_i_SWR;
        ctx->state = TS_COARSE;
        return 0;

    case TS_COARSE:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        coarse_tune();
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        if (g_i_SWR == 0) {
            atu_reset();
            ctx->pending_exit = 7;
            ctx->state = TS_ABORT;
            return 0;
        }
        get_swr();
        if (g_i_SWR == 0) {
            atu_reset();
            ctx->pending_exit = 7;
            ctx->state = TS_ABORT;
            return 0;
        }
        if (g_i_SWR < 120) {
            if (ctx->pass == 0) {
                ctx->pending_exit = 8;
                ctx->state = TS_SAVE;
            } else {
                ctx->state = TS_SW_COMPARE;
            }
            return 0;
        }
        ctx->state = TS_SHARP_IND;
        return 0;

    case TS_SHARP_IND:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        sharp_ind();
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        if (g_i_SWR == 0) {
            atu_reset();
            ctx->pending_exit = 7;
            ctx->state = TS_ABORT;
            return 0;
        }
        get_swr();
        if (g_i_SWR == 0) {
            atu_reset();
            ctx->pending_exit = 7;
            ctx->state = TS_ABORT;
            return 0;
        }
#if defined(UART) && defined(MPLAB_COMPILER)
        if (g_b_debug_mode) {
            uart_puts("DBG FINE_IND");
            tune_dbg_uint(" ind=", (int)g_c_ind);
            tune_dbg_uint(" swr=", g_i_SWR);
            uart_puts("\r\n");
        }
#endif
        if (g_i_SWR < 120) {
            if (ctx->pass == 0) {
                ctx->pending_exit = 8;
                ctx->state = TS_SAVE;
            } else {
                ctx->state = TS_SW_COMPARE;
            }
            return 0;
        }
        ctx->state = TS_SHARP_CAP;
        return 0;

    case TS_SHARP_CAP:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        sharp_cap();
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        if (g_i_SWR == 0) {
            atu_reset();
            ctx->pending_exit = 7;
            ctx->state = TS_ABORT;
            return 0;
        }
        get_swr();
        if (g_i_SWR == 0) {
            atu_reset();
            ctx->pending_exit = 7;
            ctx->state = TS_ABORT;
            return 0;
        }
#if defined(UART) && defined(MPLAB_COMPILER)
        if (g_b_debug_mode) {
            uart_puts("DBG FINE_CAP");
            tune_dbg_uint(" cap=", (int)g_c_cap);
            tune_dbg_uint(" swr=", g_i_SWR);
            uart_puts("\r\n");
        }
#endif
        if (ctx->pass == 0) {
            if (g_i_SWR < 120) {
                ctx->pending_exit = 8;
                ctx->state = TS_SAVE;
                return 0;
            }
            /* Good enough improvement — no need for SW flip */
            if (g_i_SWR < 200 && g_i_SWR < ctx->swr_before_flip
                    && (ctx->swr_before_flip - g_i_SWR) > 100) {
                ctx->pending_exit = 8;
                ctx->state = TS_SAVE;
                return 0;
            }
            /* Save pass 0 best result before flipping SW */
            ctx->pass0_swr = g_i_SWR;
            ctx->pass0_ind = g_c_ind;
            ctx->pass0_cap = g_c_cap;
            ctx->pass0_sw  = g_c_SW;
            ctx->swr_before_flip = g_i_SWR;
            /* Flip SW and measure immediately */
            if (g_c_SW == 1) g_c_SW = 0; else g_c_SW = 1;
            atu_reset();
            set_sw(g_c_SW);
            Delay_ms(50);
            get_swr();
            if (g_i_SWR < 120) {
                /* Flipped SW already good — go straight to compare */
                ctx->state = TS_SW_COMPARE;
                return 0;
            }
            /* Run full coarse+sharp pass with flipped SW */
            ctx->pass = 1;
            ctx->state = TS_COARSE;
            return 0;
        }
        /* pass == 1: proceed to compare */
        ctx->state = TS_SW_COMPARE;
        return 0;

    case TS_SW_COMPARE:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
#if defined(UART) && defined(MPLAB_COMPILER)
        if (g_b_debug_mode) {
            uart_puts("DBG SW_FINE");
            tune_dbg_uint(" swr=", g_i_SWR);
            uart_puts("\r\n");
        }
#endif
        /* If pass1 is worse than pass0, restore pass0 relay state */
        if (g_i_SWR > ctx->pass0_swr) {
            g_c_SW  = ctx->pass0_sw;
            g_c_ind = ctx->pass0_ind;
            g_c_cap = ctx->pass0_cap;
            set_sw(g_c_SW);
            set_ind(g_c_ind);
            set_cap(g_c_cap);
            get_swr();
        }
        /* Decide next step based on current SWR and multiplier settings */
        if (g_i_SWR == 0) {
            ctx->pending_exit = 7;
            ctx->state = TS_ABORT;
            return 0;
        }
        if (g_i_SWR < 120) {
            ctx->pending_exit = 8;
            ctx->state = TS_SAVE;
            return 0;
        }
        if (e_c_num_C_q == 5 && e_c_num_L_q == 5) {
            ctx->pending_exit = 9;
            ctx->state = TS_SAVE;
            return 0;
        }
        if (e_c_num_L_q > 5) {
            ctx->state = TS_EXTRA_IND;
            return 0;
        }
        if (e_c_num_C_q > 5) {
            ctx->state = TS_EXTRA_CAP;
            return 0;
        }
        ctx->pending_exit = 13;
        ctx->state = TS_SAVE;
        return 0;

    case TS_EXTRA_IND:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        g_c_step_ind = g_c_L_mult;
        g_c_L_mult   = 1;
        sharp_ind();
        if (g_i_SWR == 0) {
            ctx->pending_exit = 10;
            ctx->state = TS_ABORT;
            return 0;
        }
        get_swr();
        if (g_i_SWR < 120) {
            ctx->pending_exit = 11;
            ctx->state = TS_SAVE;
            return 0;
        }
        if (e_c_num_C_q > 5) {
            ctx->state = TS_EXTRA_CAP;
            return 0;
        }
        ctx->pending_exit = 13;
        ctx->state = TS_SAVE;
        return 0;

    case TS_EXTRA_CAP:
#ifdef UART
        if (g_b_tune_abort) { ctx->state = TS_ABORT; return 0; }
#endif
        g_c_step_cap = g_c_C_mult;
        g_c_C_mult   = 1;
        sharp_cap();
        if (g_i_SWR == 0) {
            ctx->pending_exit = 12;
            ctx->state = TS_ABORT;
            return 0;
        }
        get_swr();
        ctx->pending_exit = 13;
        ctx->state = TS_SAVE;
        return 0;

    case TS_SAVE:
        tune_restore_mults();
        g_c_tune_exit = ctx->pending_exit;
        /* Exits that represent successful/partial matches where we save */
        if (ctx->pending_exit != 2u && ctx->pending_exit != 4u
                && ctx->pending_exit != 6u && ctx->pending_exit != 7u
                && ctx->pending_exit != 10u && ctx->pending_exit != 12u) {
            band_slot_save(ctx->probe_matched, ctx->freq_kHz);
        }
#if defined(UART) && defined(MPLAB_COMPILER)
        if (g_b_debug_mode) {
            uart_puts("DBG DONE");
            tune_dbg_uint(" exit=", (int)g_c_tune_exit);
            tune_dbg_uint(" ind=", (int)g_c_ind);
            tune_dbg_uint(" cap=", (int)g_c_cap);
            tune_dbg_uint(" sw=", (int)g_c_SW);
            tune_dbg_uint(" swr=", g_i_SWR);
            tune_dbg_uint(" saved=", (int)g_b_slot_saved);
            uart_puts("\r\n");
        }
#endif
        CLRWDT();
        ctx->state = TS_IDLE;
        return 1;

    case TS_ABORT:
        tune_restore_mults();
        g_c_ind = ctx->pre_ind;
        g_c_cap = ctx->pre_cap;
        g_c_SW  = ctx->pre_sw;
        set_ind(ctx->pre_ind);
        set_cap(ctx->pre_cap);
        set_sw(ctx->pre_sw);
        if (g_c_tune_exit == 0)
            g_c_tune_exit = ctx->pending_exit;
#if defined(UART) && defined(MPLAB_COMPILER)
        if (g_b_debug_mode) {
            uart_puts("DBG DONE");
            tune_dbg_uint(" exit=", (int)g_c_tune_exit);
            tune_dbg_uint(" ind=", (int)g_c_ind);
            tune_dbg_uint(" cap=", (int)g_c_cap);
            tune_dbg_uint(" sw=", (int)g_c_SW);
            tune_dbg_uint(" swr=", g_i_SWR);
            tune_dbg_uint(" saved=", (int)g_b_slot_saved);
            uart_puts("\r\n");
        }
#endif
        ctx->state = TS_IDLE;
        return 1;

    default:
        /* Unreachable on correct hardware — treat as abort to be safe */
        ctx->state = TS_IDLE;
        return 1;
    }
}

#endif /* TUNE_ALGO_H */
