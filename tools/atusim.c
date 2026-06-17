/* atusim.c — ATU-100 EXT algorithmic simulator
 *
 * Build: gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
 *
 * The coarse_cap/coarse_tune/sharp_cap/sharp_ind/sub_tune/tune/band_slot_save
 * functions below are verbatim copies from main.h. When the firmware algorithm
 * changes, mirror those changes here — they are marked with "VERBATIM: main.h".
 *
 * Scenarios:
 *   S1  bimodal 30m delta loop — global min at high-L must win over local min at low-L
 *   S2  flat/unmatchable       — constant SWR, algorithm must terminate without crash
 *   S3  simple unimodal 20m   — single minimum, algorithm must find it
 *   S4  TX-inhibit mid-scan   — pre-scan position must be restored, no hang
 *   S5  band probe hit (kHz)  — slot at 7074 kHz recalled, SWR < 150, no full tune
 *   S6  new slot write        — no existing slot, full tune saves at 7074 kHz
 *   S7  cross-band probe miss — 40m tune, 20m slot present, probe misses, full tune
 *   S8  upsert update         — existing slot within 25 kHz updated in place
 *   S9  evict worst-SWR       — all 3 sub-slots full, worst SWR evicted for new freq
 *   S10 cross-band isolation  — 40m tune never touches 20m band slots
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

/* ── charbits type (from cross_compiler.h) ── */
typedef union {
    unsigned char bytes;
    struct {
        unsigned B0:1; unsigned B1:1; unsigned B2:1; unsigned B3:1;
        unsigned B4:1; unsigned B5:1; unsigned B6:1; unsigned B7:1;
    } bits;
} charbits;

/* ── EEPROM defines (from cross_compiler.h) ── */
#define EEPROM_BAND_FORMAT_CELL  0x36
#define EEPROM_BAND_FORMAT_VER   2
#define EEPROM_BAND_N            10
#define EEPROM_BAND_SUB_N        3
#define EEPROM_BAND_SLOT_COUNT   30
#define EEPROM_BAND_EFFORT_THR   20
#define EEPROM_BAND_SLOT_0       0x38
#define EEPROM_BAND_SLOT_STRIDE  5
#define EEPROM_BAND_FREQ_TOL_KHZ 25
#define EEPROM_SLOT_FREQ_LO      0
#define EEPROM_SLOT_FREQ_HI      1
#define EEPROM_SLOT_IND          2
#define EEPROM_SLOT_CAP          3
#define EEPROM_SLOT_SW_SWR       4

/* ── simulated EEPROM ── */
static unsigned char sim_eeprom[256];

static unsigned char eeprom_read(unsigned char addr)  { return sim_eeprom[addr]; }
static void eeprom_write(unsigned char addr, unsigned char val) { sim_eeprom[addr] = val; }

static void eeprom_reset(void) { memset(sim_eeprom, 0xFF, sizeof(sim_eeprom)); }

/* ── firmware globals (from main.h) ── */
static char g_c_ind = 0, g_c_cap = 0;
static char g_c_SW = 0;
static char g_c_step_cap = 0, g_c_step_ind = 0;
static char g_c_L_mult = 4, g_c_C_mult = 4;
int g_i_SWR = 0, g_i_PWR = 5, g_i_P_max = 0, g_i_swr_a = 0;
static char g_b_rready = 0, g_char_p_cnt = 0;
static char g_b_tx_seen = 0;
static unsigned char g_char_tune_effort = 0;
static char e_c_b_L_linear = 0, e_c_b_C_linear = 0;
static char e_c_num_L_q = 7, e_c_num_C_q = 7;
static int  e_i_watts_min_for_start = 1, e_i_watts_max_for_start = 0;
static int  e_i_tenths_init_max_swr = 0;
static char e_c_b_Loss_ind = 0;

/* ── scenario state ── */
typedef int (*swr_fn)(unsigned char ind, unsigned char cap, unsigned char sw);
static swr_fn current_model = NULL;
static int sim_inhibit_on_call = -1;
static int sim_call_n = 0;
static unsigned int sim_freq = 0;

/* ── hardware stubs ── */
#define CLRWDT() do {} while(0)
static void Vdelay_ms(int ms) { (void)ms; }
static void Delay_ms(int ms)  { (void)ms; }
static void show_pwr(int p, int s) { (void)p; (void)s; }
static void lcd_ind(void)    {}
static void show_reset(void) {}

static void set_ind(char ind) { g_c_ind = ind; Vdelay_ms(0); }
static void set_cap(char cap) { g_c_cap = cap; Vdelay_ms(0); }
static void set_sw(char sw)   { g_c_SW  = sw;  Vdelay_ms(0); }

static void atu_reset(void) {
    g_c_ind = 0; g_c_cap = 0;
    set_ind(g_c_ind); set_cap(g_c_cap);
}

/* VERBATIM: main.h measure_freq() */
static unsigned int measure_freq(void) {
    return sim_freq;   /* sim: controlled by sim_freq; firmware: returns 0 until hardware wired */
}

/* ── simulated get_pwr / get_swr ── */
static void get_pwr(void) {
    sim_call_n++;
    if (sim_inhibit_on_call >= 0 && sim_call_n >= sim_inhibit_on_call) {
        g_i_PWR = 0;
        g_i_SWR = 999;
        return;
    }
    g_i_PWR = 5;
    g_i_SWR = current_model ? current_model(g_c_ind, g_c_cap, g_c_SW) : 999;
}

/* VERBATIM: main.h get_swr() */
static void get_swr(void) {
    get_pwr();
    if (g_char_p_cnt != 100)
    {
        g_char_p_cnt += 1;
        if (g_i_PWR > g_i_P_max)
            g_i_P_max = g_i_PWR;
    }
    if (g_char_tune_effort < 255) g_char_tune_effort++;
    if (g_i_PWR >= e_i_watts_min_for_start)
        g_b_tx_seen = 1;
    while (g_i_PWR < e_i_watts_min_for_start) {
        if (g_b_tx_seen == 1) {
            g_i_SWR = 0;
            return;
        }
        return;
    }
}

/* ── algorithm functions: VERBATIM from main.h ────────────────────────────
 * Keep in sync with:
 *   ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/main.h
 * ───────────────────────────────────────────────────────────────────────── */

static void coarse_cap(void)
{
    char l_coarse_cap_step = 3;
    char l_coarse_cap_count;
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
        if (e_c_b_C_linear == 0 & l_coarse_cap_count == 9)
            l_coarse_cap_count = 8;
        else if (e_c_b_C_linear == 0 & l_coarse_cap_count == 17)
        {
            l_coarse_cap_count = 16;
            l_coarse_cap_step = 4;
        }
    }
    set_cap(g_c_cap);
    return;
}

static void coarse_tune(void)
{
    char l_coarse_tune_step = 3;
    char l_coarse_tune_count;
    char l_coarse_tune_mem_cap, l_coarse_tune_mem_step_cap;
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
        if (e_c_b_L_linear == 0 & l_coarse_tune_count == 9)
            l_coarse_tune_count = 8;
        else if (e_c_b_L_linear == 0 & l_coarse_tune_count == 17)
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

static void sharp_cap(void)
{
    char l_sharp_cap_range, l_sharp_cap_count, l_sharp_cap_max_range, l_sharp_cap_min_range;
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
        else
            break;
    }
    set_cap(g_c_cap);
    return;
}

static void sharp_ind(void)
{
    char l_sharp_ind_range, l_sharp_ind_count, l_sharp_ind_max_range, l_sharp_ind_min_range;
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
        else
            break;
    }
    set_ind(g_c_ind);
    return;
}

static void sub_tune(void)
{
    int l_int_swr_mem, l_int_ind_mem, l_int_cap_mem;

    l_int_swr_mem = g_i_SWR;
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

    if (g_i_SWR < 200 & g_i_SWR < l_int_swr_mem & (l_int_swr_mem - g_i_SWR) > 100)
        return;
    l_int_swr_mem = g_i_SWR;
    l_int_ind_mem = g_c_ind;
    l_int_cap_mem = g_c_cap;

    if (g_c_SW == 1) g_c_SW = 0; else g_c_SW = 1;
    atu_reset();
    set_sw(g_c_SW);
    Delay_ms(50);
    get_swr();
    if (g_i_SWR < 120) return;

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

    if (g_i_SWR > l_int_swr_mem) {
        if (g_c_SW == 1) g_c_SW = 0; else g_c_SW = 1;
        set_sw(g_c_SW);
        g_c_ind = (char)(l_int_ind_mem);
        g_c_cap = (char)(l_int_cap_mem);
        set_ind(g_c_ind);
        set_cap(g_c_cap);
        get_swr();
    }
    CLRWDT();
    return;
}

/* VERBATIM: main.h band table + freq_to_band_idx() */
static const unsigned int BAND_LO[10] = {1800, 3500, 7000, 10100, 14000, 18068, 21000, 24890, 28000, 50000};
static const unsigned int BAND_HI[10] = {2000, 4000, 7300, 10150, 14350, 18168, 21450, 24990, 29700, 54000};

static unsigned char freq_to_band_idx(unsigned int kHz)
{
    unsigned char l_b;
    for (l_b = 0; l_b < (unsigned char)EEPROM_BAND_N; l_b++)
        if (kHz >= BAND_LO[l_b] && kHz <= BAND_HI[l_b]) return l_b;
    return 0xFF;
}

/* VERBATIM: main.h band_slot_save() */
static void band_slot_save(char l_probe_matched, unsigned int l_freq_kHz)
{
    unsigned char l_band, l_sub, l_slot_idx, l_base, l_ind, l_sw_swr, l_swr10;
    unsigned int  l_sf, l_diff;
    unsigned char l_existing_sub, l_empty_sub, l_worst_sub, l_worst_swr;
    if (l_probe_matched) return;
    if (g_char_tune_effort <= EEPROM_BAND_EFFORT_THR) return;
    if (g_i_SWR == 0 || g_i_SWR >= 150) return;
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
    eeprom_write(l_base + EEPROM_SLOT_FREQ_LO, (unsigned char)(l_freq_kHz & 0xFFu));
    eeprom_write(l_base + EEPROM_SLOT_FREQ_HI, (unsigned char)(l_freq_kHz >> 8));
    eeprom_write(l_base + EEPROM_SLOT_IND,     (unsigned char)g_c_ind);
    eeprom_write(l_base + EEPROM_SLOT_CAP,     (unsigned char)g_c_cap);
    eeprom_write(l_base + EEPROM_SLOT_SW_SWR,  (unsigned char)(((unsigned char)(g_c_SW & 1u) << 7) | l_swr10));
}

/* VERBATIM: main.h tune() */
static void tune(void)
{
    char l_tune_ind_mem, l_tune_cap_mem, l_tune_sw_mem;
    unsigned int l_freq_kHz;
    char l_probe_matched = 0;
    CLRWDT();
    g_char_p_cnt = 0; g_i_P_max = 0; g_char_tune_effort = 0;
    g_b_rready = 0; g_b_tx_seen = 0;
    l_tune_ind_mem = g_c_ind;
    l_tune_cap_mem = g_c_cap;
    l_tune_sw_mem  = g_c_SW;
    get_swr();
    if (g_i_SWR < 110) return;
    l_freq_kHz = measure_freq();
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
                g_c_ind = (char)l_ind;
                g_c_cap = (char)eeprom_read(l_base + EEPROM_SLOT_CAP);
                l_sw_swr = eeprom_read(l_base + EEPROM_SLOT_SW_SWR);
                g_c_SW = (char)((l_sw_swr >> 7) & 1u);
                set_ind(g_c_ind);
                set_cap(g_c_cap);
                set_sw(g_c_SW);
                get_swr();
                if (g_i_SWR == 0)
                {
                    g_c_ind = l_tune_ind_mem; g_c_cap = l_tune_cap_mem; g_c_SW = l_tune_sw_mem;
                    set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
                    return;
                }
                if (g_i_SWR < 150)
                {
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
    if (e_c_b_Loss_ind == 0) lcd_ind();
    Delay_ms(50);
    get_swr();
    g_i_swr_a = g_i_SWR;
    if (g_i_SWR == 0)
    {
        g_c_ind = l_tune_ind_mem; g_c_cap = l_tune_cap_mem; g_c_SW = l_tune_sw_mem;
        set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
        return;
    }
    if (g_i_SWR < 110) return;
    if (e_i_tenths_init_max_swr > 110 & g_i_SWR > e_i_tenths_init_max_swr)
        return;
    sub_tune();
    if (g_i_SWR == 0)
    {
        g_c_ind = l_tune_ind_mem; g_c_cap = l_tune_cap_mem; g_c_SW = l_tune_sw_mem;
        set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
        return;
    }
    if (g_i_SWR < 120)
    {
        band_slot_save(l_probe_matched, l_freq_kHz);
        return;
    }
    if (e_c_num_C_q == 5 & e_c_num_L_q == 5)
    {
        band_slot_save(l_probe_matched, l_freq_kHz);
        return;
    }
    if (e_c_num_L_q > 5) {
        g_c_step_ind = g_c_L_mult;
        g_c_L_mult = 1;
        sharp_ind();
    }
    if (g_i_SWR == 0)
    {
        g_c_ind = l_tune_ind_mem; g_c_cap = l_tune_cap_mem; g_c_SW = l_tune_sw_mem;
        set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
        return;
    }
    if (g_i_SWR < 120)
    {
        band_slot_save(l_probe_matched, l_freq_kHz);
        return;
    }
    if (e_c_num_C_q > 5) {
        g_c_step_cap = g_c_C_mult;
        g_c_C_mult = 1;
        sharp_cap();
    }
    if (g_i_SWR == 0)
    {
        g_c_ind = l_tune_ind_mem; g_c_cap = l_tune_cap_mem; g_c_SW = l_tune_sw_mem;
        set_ind(g_c_ind); set_cap(g_c_cap); set_sw(g_c_SW);
        return;
    }
    if (e_c_num_L_q == 5) g_c_L_mult = 1;
    else if (e_c_num_L_q == 6) g_c_L_mult = 2;
    else if (e_c_num_L_q == 7) g_c_L_mult = 4;
    if (e_c_num_C_q == 5) g_c_C_mult = 1;
    else if (e_c_num_C_q == 6) g_c_C_mult = 2;
    else if (e_c_num_C_q == 7) g_c_C_mult = 4;
    band_slot_save(l_probe_matched, l_freq_kHz);
    CLRWDT();
    return;
}

/* ── SWR models ─────────────────────────────────────────────────────────────
 *
 * SWR is encoded as integer × 100: 100 = 1.0:1, 150 = 1.5:1, 999 = max/error.
 * Each model returns the SWR the antenna would show at a given relay position.
 * The shape is chosen to reproduce a known problem pattern, not to be physically
 * exact. What matters is that the algorithm behaviour under test is exercised.
 * ─────────────────────────────────────────────────────────────────────────── */

/*
 * S1 model: 41m delta loop on 10.1 MHz (30m band).
 * Two dips: local min at low-L (ind≈12,cap≈24,SWR≈175), global min at high-L
 * (ind≈96,cap≈32,SWR≈115). Non-greedy scan must reach the global minimum.
 */
static int model_bimodal_30m(unsigned char ind, unsigned char cap, unsigned char sw)
{
    if (sw != 0) return 999;
    double di = (double)ind, dc = (double)cap;
    double d_global = sqrt(pow((di - 96.0) / 18.0, 2.0) + pow((dc - 32.0) / 14.0, 2.0));
    double d_local  = sqrt(pow((di - 12.0) / 10.0, 2.0) + pow((dc - 24.0) / 10.0, 2.0));
    double swr = fmin(115.0 + d_global * 55.0, 175.0 + d_local * 45.0);
    if (swr < 100) swr = 100;
    if (swr > 999) swr = 999;
    return (int)swr;
}

/*
 * S2 model: unmatchable antenna. SWR constant at 350 everywhere.
 * Algorithm must terminate cleanly without crash or hang.
 */
static int model_flat(unsigned char ind, unsigned char cap, unsigned char sw)
{
    (void)ind; (void)cap; (void)sw;
    return 350;
}

/*
 * S3/S5/S6/S7/S8/S9 model: well-behaved dipole on 14.1 MHz (20m band).
 * Single minimum at ind=32, cap=24, SW=0, SWR≈105.
 */
static int model_simple_20m(unsigned char ind, unsigned char cap, unsigned char sw)
{
    if (sw != 0) return 220;
    double di = (double)ind, dc = (double)cap;
    double d = sqrt(pow((di - 32.0) / 12.0, 2.0) + pow((dc - 24.0) / 10.0, 2.0));
    double swr = 105.0 + d * 40.0;
    if (swr < 100) swr = 100;
    if (swr > 999) swr = 999;
    return (int)swr;
}

/* ── test infrastructure ── */

static void reset_state(void)
{
    g_c_ind = 0; g_c_cap = 0; g_c_SW = 0;
    g_c_step_cap = 0; g_c_step_ind = 0;
    g_c_L_mult = 4; g_c_C_mult = 4;
    g_i_SWR = 999; g_i_PWR = 5; g_i_P_max = 0; g_i_swr_a = 0;
    g_char_p_cnt = 0; g_char_tune_effort = 0; g_b_rready = 0; g_b_tx_seen = 0;
    sim_call_n = 0; sim_inhibit_on_call = -1; sim_freq = 0;
    e_i_tenths_init_max_swr = 0;
    eeprom_reset();
}

typedef struct { char ind; char cap; char sw; int swr; } result_t;

int main(void)
{
    int total = 0, passed = 0;

#define CHECK(label, cond, r) do { \
    int _ok = (cond); \
    passed += _ok; total++; \
    printf("%s  %-50s  ind=%3d cap=%3d sw=%d swr=%d\n", \
        _ok ? "PASS" : "FAIL", (label), \
        (int)(unsigned char)(r).ind, (int)(unsigned char)(r).cap, \
        (int)(r).sw, (r).swr); \
} while(0)

    /* S1: algorithm must reach the global minimum at high inductance */
    {
        reset_state();
        current_model = model_bimodal_30m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S1 bimodal 30m — global min at high L",
              r.ind >= 76 && r.swr < 130, r);
    }

    /* S2: must terminate cleanly with a valid SWR reading */
    {
        reset_state();
        current_model = model_flat;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S2 flat unmatchable — terminates, valid SWR",
              r.swr >= 100 && r.swr <= 999, r);
    }

    /* S3: must converge to SWR < 120 on a simple well-behaved load */
    {
        reset_state();
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S3 simple 20m — converges to SWR < 120",
              r.swr < 130, r);
    }

    /* S4: TX inhibit mid-scan — pre-scan position must be restored */
    {
        reset_state();
        g_c_ind = 40; g_c_cap = 24;
        current_model = model_simple_20m;
        sim_inhibit_on_call = 20;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S4 TX inhibit — pre-scan position restored",
              r.ind == 40 && r.cap == 24, r);
    }

    /* S5: 40m slot at 7074 kHz recalled by probe during tune() — no full tune */
    {
        /* Band 2 (40m) sub-slot 0: freq=7074 kHz, ind=32, cap=24, sw=0, swr=100 */
        /* slot base: EEPROM_BAND_SLOT_0 + (2*3+0)*5 = 0x38 + 30 = 0x56 */
        unsigned char s5_base = EEPROM_BAND_SLOT_0 + (unsigned char)((2*3+0)*5);
        reset_state();
        eeprom_write(s5_base + EEPROM_SLOT_FREQ_LO, (unsigned char)(7074u & 0xFF));
        eeprom_write(s5_base + EEPROM_SLOT_FREQ_HI, (unsigned char)(7074u >> 8));
        eeprom_write(s5_base + EEPROM_SLOT_IND,  32);
        eeprom_write(s5_base + EEPROM_SLOT_CAP,  24);
        eeprom_write(s5_base + EEPROM_SLOT_SW_SWR, (unsigned char)((0u << 7) | 10u));
        sim_freq = 7074;
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S5 band probe hit (7074 kHz) — slot recalled, SWR < 150",
              r.ind == 32 && r.cap == 24 && r.swr < 150, r);
    }

    /* S6: tune at 7074 kHz with no existing slot — saved to band 2 sub-slot 0 */
    {
        unsigned char s6_base = EEPROM_BAND_SLOT_0 + (unsigned char)((2*3+0)*5);
        reset_state();
        sim_freq = 7074;
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        unsigned int  saved_freq = (unsigned int)eeprom_read(s6_base + EEPROM_SLOT_FREQ_LO)
                                 | ((unsigned int)eeprom_read(s6_base + EEPROM_SLOT_FREQ_HI) << 8);
        unsigned char saved_ind  = eeprom_read(s6_base + EEPROM_SLOT_IND);
        CHECK("S6 new slot write — freq=7074, ind matches tune, SWR < 130",
              saved_freq == 7074 && saved_ind == (unsigned char)r.ind && r.swr < 130, r);
    }

    /* S7: slot for 14100 kHz (20m band 4), tune at 7074 kHz (40m band 2) — wrong band,
       probe skips all 20m sub-slots, full tune runs */
    {
        unsigned char s7_20m_base = EEPROM_BAND_SLOT_0 + (unsigned char)((4*3+0)*5);
        reset_state();
        eeprom_write(s7_20m_base + EEPROM_SLOT_FREQ_LO, (unsigned char)(14100u & 0xFF));
        eeprom_write(s7_20m_base + EEPROM_SLOT_FREQ_HI, (unsigned char)(14100u >> 8));
        eeprom_write(s7_20m_base + EEPROM_SLOT_IND, 64);
        eeprom_write(s7_20m_base + EEPROM_SLOT_CAP, 64);
        eeprom_write(s7_20m_base + EEPROM_SLOT_SW_SWR, (unsigned char)((0u << 7) | 20u));
        sim_freq = 7074;   /* 40m: probe looks at band 2 sub-slots, all empty → full tune */
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S7 wrong band probe miss — full tune, SWR < 130", r.swr < 130, r);
    }

    /* S8: upsert — 40m slot 0 has bad settings (SWR≥150), probe misses, full tune
       updates the same slot in place; 40m slot 1 with different freq is untouched */
    {
        unsigned char s8_b0 = EEPROM_BAND_SLOT_0 + (unsigned char)((2*3+0)*5);
        unsigned char s8_b1 = EEPROM_BAND_SLOT_0 + (unsigned char)((2*3+1)*5);
        reset_state();
        /* Sub-slot 0: freq=7074, bad relays (ind=64,cap=64 → SWR≥150 on model_simple_20m) */
        eeprom_write(s8_b0 + EEPROM_SLOT_FREQ_LO, (unsigned char)(7074u & 0xFF));
        eeprom_write(s8_b0 + EEPROM_SLOT_FREQ_HI, (unsigned char)(7074u >> 8));
        eeprom_write(s8_b0 + EEPROM_SLOT_IND, 64);
        eeprom_write(s8_b0 + EEPROM_SLOT_CAP, 64);
        eeprom_write(s8_b0 + EEPROM_SLOT_SW_SWR, (unsigned char)((0u << 7) | 20u));
        /* Sub-slot 1: freq=7200 (different sub-band, outside upsert TOL of 7074) */
        eeprom_write(s8_b1 + EEPROM_SLOT_FREQ_LO, (unsigned char)(7200u & 0xFF));
        eeprom_write(s8_b1 + EEPROM_SLOT_FREQ_HI, (unsigned char)(7200u >> 8));
        eeprom_write(s8_b1 + EEPROM_SLOT_IND,  8);
        eeprom_write(s8_b1 + EEPROM_SLOT_CAP, 30);
        eeprom_write(s8_b1 + EEPROM_SLOT_SW_SWR, (unsigned char)((0u << 7) | 12u));
        sim_freq = 7074;
        current_model = model_simple_20m;
        tune();
        /* Slot 0 should have new (good) ind; slot 1 (7200) must be untouched */
        unsigned char s8_ind0 = eeprom_read(s8_b0 + EEPROM_SLOT_IND);
        unsigned int  s8_f1   = (unsigned int)eeprom_read(s8_b1 + EEPROM_SLOT_FREQ_LO)
                              | ((unsigned int)eeprom_read(s8_b1 + EEPROM_SLOT_FREQ_HI) << 8);
        unsigned char s8_ind1 = eeprom_read(s8_b1 + EEPROM_SLOT_IND);
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S8 upsert — slot 0 updated in place, slot 1 (7200 kHz) untouched",
              s8_ind0 != 64 && s8_f1 == 7200 && s8_ind1 == 8 && r.swr < 150, r);
    }

    /* S9: eviction — all 3 sub-slots for 40m full with different freqs, tune at 7150
       (outside TOL=25 of all three) → evicts the slot with worst (highest) stored SWR */
    {
        unsigned char s9_b0 = EEPROM_BAND_SLOT_0 + (unsigned char)((2*3+0)*5);
        unsigned char s9_b1 = EEPROM_BAND_SLOT_0 + (unsigned char)((2*3+1)*5);
        unsigned char s9_b2 = EEPROM_BAND_SLOT_0 + (unsigned char)((2*3+2)*5);
        reset_state();
        /* Slot 0: 7000 kHz, swr10=14 (worst) */
        eeprom_write(s9_b0 + EEPROM_SLOT_FREQ_LO, (unsigned char)(7000u & 0xFF));
        eeprom_write(s9_b0 + EEPROM_SLOT_FREQ_HI, (unsigned char)(7000u >> 8));
        eeprom_write(s9_b0 + EEPROM_SLOT_IND, 10); eeprom_write(s9_b0 + EEPROM_SLOT_CAP, 20);
        eeprom_write(s9_b0 + EEPROM_SLOT_SW_SWR, (unsigned char)((0u << 7) | 14u));
        /* Slot 1: 7074 kHz, swr10=12 */
        eeprom_write(s9_b1 + EEPROM_SLOT_FREQ_LO, (unsigned char)(7074u & 0xFF));
        eeprom_write(s9_b1 + EEPROM_SLOT_FREQ_HI, (unsigned char)(7074u >> 8));
        eeprom_write(s9_b1 + EEPROM_SLOT_IND, 12); eeprom_write(s9_b1 + EEPROM_SLOT_CAP, 22);
        eeprom_write(s9_b1 + EEPROM_SLOT_SW_SWR, (unsigned char)((0u << 7) | 12u));
        /* Slot 2: 7200 kHz, swr10=11 (best) */
        eeprom_write(s9_b2 + EEPROM_SLOT_FREQ_LO, (unsigned char)(7200u & 0xFF));
        eeprom_write(s9_b2 + EEPROM_SLOT_FREQ_HI, (unsigned char)(7200u >> 8));
        eeprom_write(s9_b2 + EEPROM_SLOT_IND, 14); eeprom_write(s9_b2 + EEPROM_SLOT_CAP, 24);
        eeprom_write(s9_b2 + EEPROM_SLOT_SW_SWR, (unsigned char)((0u << 7) | 11u));
        /* Tune at 7150 kHz: |7150-7000|=150>25, |7150-7074|=76>25, |7150-7200|=50>25 → evict */
        sim_freq = 7150;
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        /* Slot 0 (worst, swr10=14) should now have 7150, slots 1+2 untouched */
        unsigned int  s9_f0  = (unsigned int)eeprom_read(s9_b0 + EEPROM_SLOT_FREQ_LO)
                             | ((unsigned int)eeprom_read(s9_b0 + EEPROM_SLOT_FREQ_HI) << 8);
        unsigned int  s9_f1  = (unsigned int)eeprom_read(s9_b1 + EEPROM_SLOT_FREQ_LO)
                             | ((unsigned int)eeprom_read(s9_b1 + EEPROM_SLOT_FREQ_HI) << 8);
        unsigned int  s9_f2  = (unsigned int)eeprom_read(s9_b2 + EEPROM_SLOT_FREQ_LO)
                             | ((unsigned int)eeprom_read(s9_b2 + EEPROM_SLOT_FREQ_HI) << 8);
        CHECK("S9 eviction — worst swr slot 0 replaced with 7150, slots 1+2 intact",
              s9_f0 == 7150 && s9_f1 == 7074 && s9_f2 == 7200 && r.swr < 150, r);
    }

    /* S10: cross-band isolation — tune on 40m (7074 kHz) saves to band 2 only;
       20m band 4 sub-slot 0 must remain empty */
    {
        unsigned char s10_20m = EEPROM_BAND_SLOT_0 + (unsigned char)((4*3+0)*5);
        reset_state();
        sim_freq = 7074;
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        unsigned char s10_20m_ind = eeprom_read(s10_20m + EEPROM_SLOT_IND);
        CHECK("S10 cross-band isolation — 40m tune, 20m band slots untouched",
              s10_20m_ind == 0xFF && r.swr < 130, r);
    }

    printf("\n%d/%d passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
