/* tune_algo.h — ATU-100 EXT tuning algorithm
 *
 * Single source of truth for:
 *   coarse_cap / coarse_tune / sharp_cap / sharp_ind / sub_tune
 *   band_slot_save / freq_to_band_idx / BAND_LO / BAND_HI
 *   tune
 *
 * Included by:
 *   main.h            — firmware build (MPLAB X, XC8)
 *   tools/atusim.c    — simulator build (gcc)
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
 *   [UART]    : g_i_uart_freq_hint, g_b_slot_saved, g_c_tune_exit
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

/* ── forward: platform-specific frequency measurement ── */
static unsigned int measure_freq(void);

/* ── band table — 10 ham bands, lo/hi in kHz ── */
/* 160m  80m  40m   30m    20m    17m    15m    12m    10m    6m */
static const unsigned int BAND_LO[10] = {1800, 3500, 7000, 10100, 14000, 18068, 21000, 24890, 28000, 50000};
static const unsigned int BAND_HI[10] = {2000, 4000, 7300, 10150, 14350, 18168, 21450, 24990, 29700, 54000};

static unsigned char freq_to_band_idx(unsigned int kHz)
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

/* ── two-pass SW search (cap-on-antenna vs cap-on-tx) ── */
static void sub_tune(void)
{
   int l_int_swr_mem;
   unsigned char l_int_ind_mem, l_int_cap_mem;
   //
   l_int_swr_mem = g_i_SWR;
   /* pass 1 — SW at current position */
   coarse_tune();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_ind();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_cap();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   //
   if (g_i_SWR < 200 && g_i_SWR < l_int_swr_mem && (l_int_swr_mem - g_i_SWR) > 100)
      return;
   l_int_swr_mem = g_i_SWR;
   l_int_ind_mem = g_c_ind;
   l_int_cap_mem = g_c_cap;
   //
   if (g_c_SW == 1)
      g_c_SW = 0;
   else
      g_c_SW = 1;
   atu_reset();
   set_sw(g_c_SW);
   Delay_ms(50);
   get_swr();
   if (g_i_SWR < 120)
      return;
   /* pass 2 — SW at opposite position */
   coarse_tune();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_ind();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   sharp_cap();
   if (g_i_SWR == 0) { atu_reset(); return; }
   get_swr();
   if (g_i_SWR < 120) return;
   //
   if (g_i_SWR > l_int_swr_mem)
   {
      if (g_c_SW == 1)
         g_c_SW = 0;
      else
         g_c_SW = 1;
      set_sw(g_c_SW);
      g_c_ind = l_int_ind_mem;
      g_c_cap = l_int_cap_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      get_swr();
   }
   //
   CLRWDT();
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

/* ── main tuning orchestrator ──
 * Call sequence:
 *   1. Quick check: SWR already < 1.1:1 → save and return (exit 1)
 *   2. Probe band memory for stored match within ±25 kHz → recall and return (exit 2/3)
 *   3. Full search: atu_reset → sub_tune → sharp fine passes → save and return (exit 4-13)
 * g_c_tune_exit records which path was taken (diagnostic, shown in status line).
 * g_b_slot_saved is set by band_slot_save() if a write occurred.            */
static void tune(void)
{
   unsigned char l_tune_ind_mem, l_tune_cap_mem;
   char l_tune_sw_mem;
   unsigned int l_freq_kHz;
   char l_probe_matched = 0;
   CLRWDT();
   //
   g_char_p_cnt = 0;
   g_i_P_max = 0;
   g_char_tune_effort = 0;
   g_b_slot_saved = 0;
   g_c_tune_exit  = 0;
   //
   g_b_rready = 0;
   g_b_tx_seen = 0;
   l_tune_ind_mem = g_c_ind;
   l_tune_cap_mem = g_c_cap;
   l_tune_sw_mem  = g_c_SW;
   l_freq_kHz = measure_freq();
#ifdef UART
   if (l_freq_kHz == 0 && g_i_uart_freq_hint != 0) { l_freq_kHz = g_i_uart_freq_hint; g_i_uart_freq_hint = 0; }
#endif
   get_swr();
   if (g_i_SWR < 110)
   {
      g_c_tune_exit = 1;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   /* probe band memory: search the 3 sub-slots for the current band */
   if (l_freq_kHz > 0)
   {
      unsigned char l_band_idx = freq_to_band_idx(l_freq_kHz);
      if (l_band_idx != 0xFF)
      {
         unsigned char l_sub, l_slot_idx, l_base, l_ind, l_sw_swr;
         unsigned int  l_sf, l_diff;
         for (l_sub = 0; l_sub < (unsigned char)EEPROM_BAND_SUB_N; l_sub++)
         {
            l_slot_idx = (unsigned char)(l_band_idx * (unsigned char)EEPROM_BAND_SUB_N + l_sub);
            l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_slot_idx * (unsigned char)EEPROM_BAND_SLOT_STRIDE);
            l_ind = eeprom_read(l_base + EEPROM_SLOT_IND);
            if (l_ind == 0xFF) continue;
            l_sf = (unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_LO)
                 | ((unsigned int)eeprom_read(l_base + EEPROM_SLOT_FREQ_HI) << 8);
            if (l_sf == 0) continue;
            l_diff = (l_sf > l_freq_kHz) ? (unsigned int)(l_sf - l_freq_kHz)
                                          : (unsigned int)(l_freq_kHz - l_sf);
            if (l_diff > (unsigned int)EEPROM_BAND_FREQ_TOL_KHZ) continue;
            g_c_ind = l_ind;
            g_c_cap = eeprom_read(l_base + EEPROM_SLOT_CAP);
            l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
            g_c_SW = (char)((l_sw_swr >> 7) & 1u);
            set_ind(g_c_ind);
            set_cap(g_c_cap);
            set_sw(g_c_SW);
            get_swr();
            if (g_i_SWR == 0)
            {
               g_c_tune_exit = 2;
               g_c_ind = l_tune_ind_mem; g_c_cap = l_tune_cap_mem; g_c_SW = l_tune_sw_mem;
               set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
               return;
            }
            if (g_i_SWR < 150)
            {
               /* upsert: overwrite stored SWR only when current measurement improved */
               unsigned char l_stored_swr10 = l_sw_swr & 0x7Fu;
               g_c_tune_exit = 3;
               if (g_i_SWR > 0 && (unsigned char)((unsigned int)g_i_SWR / 10u) < l_stored_swr10)
                  band_slot_save(0, l_freq_kHz);
               else
                  l_probe_matched = 1;
               return;
            }
         }
         /* no sub-slot matched — reset SW before full tune */
         g_c_SW = 0;
         set_sw(g_c_SW);
      }
   }
   g_char_tune_effort = 0;
   atu_reset();
   if (e_c_b_Loss_ind == 0)
      lcd_ind();
   Delay_ms(50);
   get_swr();
   g_i_swr_a = g_i_SWR;
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 4;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (g_i_SWR < 110)
   {
      g_c_tune_exit = 5;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   if (e_i_tenths_init_max_swr > 110 && g_i_SWR > e_i_tenths_init_max_swr)
   { g_c_tune_exit = 6; return; }
   //
   sub_tune();
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 7;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (g_i_SWR < 120)
   {
      g_c_tune_exit = 8;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   if (e_c_num_C_q == 5 && e_c_num_L_q == 5)
   {
      g_c_tune_exit = 9;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }

   if (e_c_num_L_q > 5)
   {
      g_c_step_ind = g_c_L_mult;
      g_c_L_mult = 1;
      sharp_ind();
   }
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 10;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (g_i_SWR < 120)
   {
      g_c_tune_exit = 11;
      band_slot_save(l_probe_matched, l_freq_kHz);
      return;
   }
   if (e_c_num_C_q > 5)
   {
      g_c_step_cap = g_c_C_mult;
      g_c_C_mult = 1;
      sharp_cap();
   }
   if (g_i_SWR == 0)
   {
      g_c_tune_exit = 12;
      g_c_ind = l_tune_ind_mem;
      g_c_cap = l_tune_cap_mem;
      g_c_SW  = l_tune_sw_mem;
      set_ind(g_c_ind);
      set_cap(g_c_cap);
      set_sw(g_c_SW);
      return;
   }
   if (e_c_num_L_q == 5)
      g_c_L_mult = 1;
   else if (e_c_num_L_q == 6)
      g_c_L_mult = 2;
   else if (e_c_num_L_q == 7)
      g_c_L_mult = 4;
   if (e_c_num_C_q == 5)
      g_c_C_mult = 1;
   else if (e_c_num_C_q == 6)
      g_c_C_mult = 2;
   else if (e_c_num_C_q == 7)
      g_c_C_mult = 4;
   get_swr();
   g_c_tune_exit = 13;
   band_slot_save(l_probe_matched, l_freq_kHz);
   CLRWDT();
   return;
}

#endif /* TUNE_ALGO_H */
