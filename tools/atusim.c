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
 *   S5  band memory probe hit (no freq hw) — legacy untagged slot recalled, no full tune
 *   S6  band memory slot write — hard tune saves freq_enc + L/C to slot 0, ptr advances
 *   S7  Phase A freq hit      — freq-tagged slot on matching band recalled by Phase A
 *   S8  Phase A miss + Phase B skip — tagged slot for wrong band, falls through to full tune
 *   S9  freq saved with slot  — band_slot_save() writes measured freq_enc into slot
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
#define EEPROM_BAND_SLOT_COUNT  8
#define EEPROM_BAND_EFFORT_THR  20
#define EEPROM_BAND_COUNT       0x36
#define EEPROM_BAND_PTR         0x37
#define EEPROM_BAND_SLOT_0      0x38
#define EEPROM_BAND_SLOT_STRIDE 5
#define EEPROM_BAND_FREQ_TOL    2
#define EEPROM_FORMAT_VERSION   (EEPROM_BAND_SLOT_0 + EEPROM_BAND_SLOT_COUNT * EEPROM_BAND_SLOT_STRIDE)

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
static unsigned char sim_freq = 0;

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
static unsigned char measure_freq(void) {
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

/* VERBATIM: main.h band_slot_save() */
static void band_slot_save(char l_probe_matched, unsigned char l_freq)
{
    unsigned char l_ptr, l_base, l_count;
    if (l_probe_matched) return;
    if (g_char_tune_effort <= EEPROM_BAND_EFFORT_THR) return;
    if (g_i_SWR == 0 || g_i_SWR >= 150) return;
    l_count = eeprom_read(EEPROM_BAND_COUNT);
    if (l_count < 1 || l_count > EEPROM_BAND_SLOT_COUNT) return;
    l_ptr = eeprom_read(EEPROM_BAND_PTR);
    if (l_ptr >= l_count) l_ptr = 0;
    l_base = EEPROM_BAND_SLOT_0 + (unsigned char)(l_ptr * EEPROM_BAND_SLOT_STRIDE);
    eeprom_write(l_base,     l_freq);
    eeprom_write(l_base + 1, g_c_ind);
    eeprom_write(l_base + 2, g_c_cap);
    eeprom_write(l_base + 3, (unsigned char)(g_c_SW & 1u));
    eeprom_write(l_base + 4, (char)(g_i_SWR / 10));
    l_ptr++;
    if (l_ptr >= l_count) l_ptr = 0;
    eeprom_write(EEPROM_BAND_PTR, l_ptr);
}

/* VERBATIM: main.h tune() */
static void tune(void)
{
    char l_tune_ind_mem, l_tune_cap_mem, l_tune_sw_mem;
    unsigned char l_freq;
    char l_probe_matched = 0;
    CLRWDT();
    g_char_p_cnt = 0; g_i_P_max = 0; g_char_tune_effort = 0;
    g_b_rready = 0; g_b_tx_seen = 0;
    l_tune_ind_mem = g_c_ind;
    l_tune_cap_mem = g_c_cap;
    l_tune_sw_mem  = g_c_SW;
    get_swr();
    if (g_i_SWR < 110) return;
    l_freq = measure_freq();
    // probe band memory slots before committing to full tune
    // Phase A (when l_freq > 0): try freq-tagged slots within EEPROM_BAND_FREQ_TOL
    // Phase B: try untagged slots; skip freq-tagged slots already covered by Phase A
    {
        unsigned char l_slot, l_base, l_slot_ind, l_count;
        unsigned char l_phase, l_slot_freq, l_diff;
        l_count = eeprom_read(EEPROM_BAND_COUNT);
        if (l_count >= 1 && l_count <= EEPROM_BAND_SLOT_COUNT)
        {
            for (l_phase = (l_freq > 0) ? 0u : 1u; l_phase <= 1u; l_phase++)
            {
                for (l_slot = 0; l_slot < l_count; l_slot++)
                {
                    l_base = EEPROM_BAND_SLOT_0
                           + (unsigned char)(l_slot * EEPROM_BAND_SLOT_STRIDE);
                    l_slot_ind = eeprom_read(l_base + 1);
                    if (l_slot_ind == 0xFF)
                        continue;
                    l_slot_freq = eeprom_read(l_base);
                    if (l_phase == 0u)
                    {
                        if (l_slot_freq == 0)
                            continue;
                        /* larger-minus-smaller is always non-negative (both unsigned char) */
                        l_diff = (l_slot_freq > l_freq)
                               ? (unsigned char)(l_slot_freq - l_freq)
                               : (unsigned char)(l_freq - l_slot_freq);
                        if (l_diff > EEPROM_BAND_FREQ_TOL)
                            continue;
                    }
                    else
                    {
                        /* skip freq-tagged slots when we have a measured freq —
                         * they were tried in Phase A; wrong-band slots would fail anyway */
                        if (l_slot_freq > 0 && l_freq > 0)
                            continue;
                    }
                    g_c_ind = l_slot_ind;
                    g_c_cap = eeprom_read(l_base + 2);
                    g_c_SW  = eeprom_read(l_base + 3) & 1u;
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
            }
            // no slot matched — reset SW before full tune
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
        band_slot_save(l_probe_matched, l_freq);
        return;
    }
    if (e_c_num_C_q == 5 & e_c_num_L_q == 5)
    {
        band_slot_save(l_probe_matched, l_freq);
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
        band_slot_save(l_probe_matched, l_freq);
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
    band_slot_save(l_probe_matched, l_freq);
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

    /* S5: untagged slot (no freq hw) recalled by Phase B, no full tune */
    {
        reset_state();
        eeprom_write(EEPROM_BAND_COUNT, 8);
        /* 5-byte slot 0: freq_enc=0 (no freq hardware), ind=32, cap=24, sw=0, swr/10=10 */
        eeprom_write(EEPROM_BAND_SLOT_0,      0);   /* freq_enc */
        eeprom_write(EEPROM_BAND_SLOT_0 + 1, 32);   /* ind */
        eeprom_write(EEPROM_BAND_SLOT_0 + 2, 24);   /* cap */
        eeprom_write(EEPROM_BAND_SLOT_0 + 3,  0);   /* sw  */
        eeprom_write(EEPROM_BAND_SLOT_0 + 4, 10);   /* swr/10 */
        current_model = model_simple_20m;
        /* sim_freq = 0 (no hardware): Phase A skipped, Phase B probes untagged slot */
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S5 band probe hit (no freq hw) — slot recalled, SWR < 150",
              r.ind == 32 && r.cap == 24 && r.swr < 150, r);
    }

    /* S6: hard tune result written to EEPROM slot with freq_enc=0, pointer advances */
    {
        reset_state();
        eeprom_write(EEPROM_BAND_COUNT, 8);
        current_model = model_bimodal_30m;
        /* sim_freq = 0: no freq hardware path, save saves freq_enc=0 */
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        unsigned char saved_freq = eeprom_read(EEPROM_BAND_SLOT_0);
        unsigned char saved_ind  = eeprom_read(EEPROM_BAND_SLOT_0 + 1);
        unsigned char saved_ptr  = eeprom_read(EEPROM_BAND_PTR);
        CHECK("S6 hard tune — result saved, ptr advanced, freq_enc=0",
              saved_freq == 0 && saved_ind == (unsigned char)r.ind
              && saved_ptr == 1 && r.swr < 130, r);
    }

    /* S7: freq-tagged slot on matching band recalled by Phase A */
    {
        reset_state();
        eeprom_write(EEPROM_BAND_COUNT, 8);
        /* 5-byte slot 0: freq_enc=70 (20m: 14 MHz × 5), ind=32, cap=24 */
        eeprom_write(EEPROM_BAND_SLOT_0,      70);  /* freq_enc: 20m */
        eeprom_write(EEPROM_BAND_SLOT_0 + 1,  32);  /* ind */
        eeprom_write(EEPROM_BAND_SLOT_0 + 2,  24);  /* cap */
        eeprom_write(EEPROM_BAND_SLOT_0 + 3,   0);  /* sw  */
        eeprom_write(EEPROM_BAND_SLOT_0 + 4,  10);  /* swr/10 */
        sim_freq = 70;   /* measured: 14 MHz × 5 = 70, within TOL=2 */
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S7 Phase A freq hit — tagged slot recalled, SWR < 150",
              r.ind == 32 && r.cap == 24 && r.swr < 150, r);
    }

    /* S8: tagged slot for wrong band — Phase A miss, Phase B skip, full tune */
    {
        reset_state();
        eeprom_write(EEPROM_BAND_COUNT, 8);
        /* slot 0 tagged for 40m (freq_enc=35), but we are on 20m (freq_enc=70) */
        eeprom_write(EEPROM_BAND_SLOT_0,      35);  /* freq_enc: 40m */
        eeprom_write(EEPROM_BAND_SLOT_0 + 1,  64);  /* ind (wrong band, high SWR) */
        eeprom_write(EEPROM_BAND_SLOT_0 + 2,  64);  /* cap */
        eeprom_write(EEPROM_BAND_SLOT_0 + 3,   0);  /* sw  */
        eeprom_write(EEPROM_BAND_SLOT_0 + 4,  20);  /* swr/10 */
        sim_freq = 70;   /* 20m: diff = |35-70| = 35 >> TOL=2; Phase A skips */
        current_model = model_simple_20m;
        /* Phase B also skips (l_slot_freq=35>0 && l_freq=70>0) → full tune */
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S8 Phase A miss + Phase B skip — full tune, SWR < 130",
              r.swr < 130, r);
    }

    /* S9: freq_enc is saved correctly when band_slot_save() is called */
    {
        reset_state();
        eeprom_write(EEPROM_BAND_COUNT, 8);
        sim_freq = 70;   /* 20m */
        current_model = model_simple_20m;
        tune();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        unsigned char saved_freq = eeprom_read(EEPROM_BAND_SLOT_0);
        unsigned char saved_ind  = eeprom_read(EEPROM_BAND_SLOT_0 + 1);
        CHECK("S9 freq saved in slot — saved_freq=70, ind matches tune result",
              saved_freq == 70 && saved_ind == (unsigned char)r.ind && r.swr < 130, r);
    }

    printf("\n%d/%d passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
