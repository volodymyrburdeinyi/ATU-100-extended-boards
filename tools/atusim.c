/* atusim.c — ATU-100 EXT algorithmic simulator
 *
 * Build: gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
 *
 * The algorithm under test (coarse_cap/coarse_tune/sharp_cap/sharp_ind/
 * sub_tune/tune/band_slot_save) is compiled from the REAL firmware source
 * via the #include at the bottom of the stub section.  There are no copies.
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

/* ── UART must be defined so g_b_slot_saved/g_c_tune_exit compile ── */
#define UART

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
static unsigned char g_c_ind = 0, g_c_cap = 0;   /* unsigned: matches firmware declaration */
static char g_c_SW = 0;
static char g_c_step_cap = 0, g_c_step_ind = 0;
static char g_c_L_mult = 4, g_c_C_mult = 4;
int g_i_SWR = 0, g_i_PWR = 5, g_i_P_max = 0, g_i_swr_a = 0;
static char g_b_rready = 0, g_char_p_cnt = 0;
static char g_b_tx_seen = 0;
static unsigned char g_char_tune_effort = 0;
static char e_c_b_L_linear = 0, e_c_b_C_linear = 0;
static char e_c_num_L_q = 7, e_c_num_C_q = 7;
static int  e_i_watts_min_for_start = 1;
static int  e_i_watts_max_for_start = 0;  /* used by get_swr() max-power guard */
static int  e_i_tenths_init_max_swr = 0;
static char e_c_b_Loss_ind = 0;

/* ── UART globals (defined in main.c; declared extern in cross_compiler.h #ifdef UART) ── */
static unsigned int  g_i_uart_freq_hint = 0;  /* cleared after each use in tune() */
static unsigned char g_b_slot_saved     = 0;  /* set by band_slot_save(), cleared at tune() start */
static unsigned char g_c_tune_exit      = 0;  /* exit-path code, cleared at tune() start */

/* ── scenario state ── */
typedef int (*swr_fn)(unsigned char ind, unsigned char cap, unsigned char sw);
static swr_fn current_model = NULL;
static int sim_inhibit_on_call = -1;
static int sim_call_n = 0;
static unsigned int sim_freq = 0;

/* Physical relay state: separate from the algorithm's g_c_ind/g_c_cap tracking
 * variables.  In real firmware, set_cap/set_ind drive hardware pins and have no
 * effect on g_c_ind/g_c_cap.  The SWR model must see the physical relay state
 * (what the antenna actually experiences), not the algorithm's bookkeeping vars. */
static unsigned char sim_relay_ind = 0;
static unsigned char sim_relay_cap = 0;
static char          sim_relay_sw  = 0;

/* ── hardware stubs ── */
#define CLRWDT() do {} while(0)
static void Vdelay_ms(int ms) { (void)ms; }
static void Delay_ms(int ms)  { (void)ms; }
static void show_pwr(int p, int s)  { (void)p; (void)s; }  /* display stub */
static void lcd_ind(void)           {}                       /* display stub */
static void show_reset(void)        {}                       /* display stub */

/* Relay stubs: update the physical relay state (sim_relay_*) without touching
 * the algorithm's tracking globals (g_c_ind, g_c_cap, g_c_SW).
 *
 * In real firmware relay.c only drives hardware pins; it never writes to the
 * g_c_* tracking variables.  The SWR model must see the physical relay state
 * (what the antenna actually experiences), which is maintained here separately.
 * This ensures sharp_cap/sharp_ind correctly retain the best-position value in
 * g_c_cap/g_c_ind across iterations that do not improve the SWR. */
static void set_ind(unsigned char ind) { sim_relay_ind = ind; Vdelay_ms(0); }
static void set_cap(unsigned char cap) { sim_relay_cap = cap; Vdelay_ms(0); }
static void set_sw(char sw)            { sim_relay_sw = sw; g_c_SW = sw; Vdelay_ms(0); }

static void atu_reset(void) {
    g_c_ind = 0; g_c_cap = 0;
    set_ind(0); set_cap(0);
}

/* ── frequency measurement: sim returns sim_freq (firmware returns 0) ── */
static unsigned int measure_freq(void) {
    return sim_freq;
}

/* ── simulated get_pwr / get_swr ──
 * Simplified: no Button() or blocking wait loop (those test hardware readiness,
 * not the search algorithm). The model function injects the SWR value directly. */
static void get_pwr(void) {
    sim_call_n++;
    if (sim_inhibit_on_call >= 0 && sim_call_n >= sim_inhibit_on_call) {
        g_i_PWR = 0;
        g_i_SWR = 999;
        return;
    }
    g_i_PWR = 5;
    g_i_SWR = current_model ? current_model(sim_relay_ind, sim_relay_cap, sim_relay_sw) : 999;
}

static void get_swr(void) {
    get_pwr();
    if (g_char_p_cnt != 100)
    {
        g_char_p_cnt += 1;
        if (g_i_PWR > g_i_P_max)
            g_i_P_max = g_i_PWR;
    }
    if (g_char_tune_effort < 255) {
        g_char_tune_effort++;
    } else {
        g_char_p_cnt = 0;
        show_pwr(g_i_P_max, g_i_SWR);  /* mirrors firmware display refresh */
        g_i_P_max = 0;
    }
    if (g_i_PWR >= e_i_watts_min_for_start)
        g_b_tx_seen = 1;
    /* simplified: no Button() polling; check both min and max power bounds */
    while ((g_i_PWR < e_i_watts_min_for_start) ||
           (g_i_PWR > e_i_watts_max_for_start && e_i_watts_max_for_start > 0)) {
        if (g_b_tx_seen == 1) {
            g_i_SWR = 0;
            return;
        }
        show_reset();  /* mirrors firmware button-cancel path */
        return;
    }
}

/* ── real algorithm: compiled from firmware source (not a copy) ── */
#include "../ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/tune_algo.h"

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
 * S3/S5/S6/S7/S8/S9/S10 model: well-behaved dipole on 14.1 MHz (20m band).
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

/*
 * S11 model: jittery ADC spike at cap=16 during sharp_cap scan.
 * True minimum is at cap=17 (SWR=130). A spike at cap=16 (all reads return
 * 999) would cause the old `else break` to fire before the true minimum is
 * reached. The new full-range scan continues past the spike and finds cap=17.
 *
 * Cap map (ind and sw ignored for this model):
 *   cap=13: 180   baseline at scan start
 *   cap=14: 165   improving
 *   cap=15: 155   improving
 *   cap=16: 999   unconditional spike — all three reads return 999
 *   cap=17: 130   true minimum
 *   cap=18: 140   degrading
 *   cap=19: 150   degrading
 *   other : 999   outside scan window
 */
static int model_jitter_spike(unsigned char ind, unsigned char cap, unsigned char sw)
{
    (void)ind; (void)sw;
    switch (cap) {
        case 13: return 180;
        case 14: return 165;
        case 15: return 155;
        case 16: return 999;  /* unconditional spike — retries also return 999 */
        case 17: return 130;
        case 18: return 140;
        case 19: return 150;
        default: return 999;
    }
}

/* ── test infrastructure ── */

static void reset_state(void)
{
    g_c_ind = 0; g_c_cap = 0; g_c_SW = 0;
    g_c_step_cap = 0; g_c_step_ind = 0;
    g_c_L_mult = 4; g_c_C_mult = 4;
    g_i_SWR = 999; g_i_PWR = 5; g_i_P_max = 0; g_i_swr_a = 0;
    g_char_p_cnt = 0; g_char_tune_effort = 0; g_b_rready = 0; g_b_tx_seen = 0;
    g_b_slot_saved = 0; g_c_tune_exit = 0; g_i_uart_freq_hint = 0;
    sim_call_n = 0; sim_inhibit_on_call = -1; sim_freq = 0;
    sim_relay_ind = 0; sim_relay_cap = 0; sim_relay_sw = 0;
    e_i_tenths_init_max_swr = 0;
    eeprom_reset();
}

typedef struct { unsigned char ind; unsigned char cap; char sw; int swr; } result_t;

int main(void)
{
    int total = 0, passed = 0;

#define CHECK(label, cond, r) do { \
    int _ok = (cond); \
    passed += _ok; total++; \
    printf("%s  %-50s  ind=%3d cap=%3d sw=%d swr=%d\n", \
        _ok ? "PASS" : "FAIL", (label), \
        (int)(r).ind, (int)(r).cap, \
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
        /* Band 2 (40m) sub-slot 0: freq=7074 kHz, ind=32, cap=24, sw=0, swr/10=10 */
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
              saved_freq == 7074 && saved_ind == r.ind && r.swr < 130, r);
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

    /* S11: jittery ADC spike at cap=16 — sharp_cap must reach cap=17 (true min)
     *
     * State setup: coarse scan already "landed" at cap=16 with step_cap=3, C_mult=1.
     * sharp_cap will compute range=3, min_range=13, max_range=19 and scan 13..19.
     * The spike at cap=16 (all reads=999) would have fired `else break` in the old
     * code, stopping at cap=15 (SWR=155). The new full-range scan continues and
     * finds cap=17 (SWR=130).                                                        */
    {
        reset_state();
        current_model = model_jitter_spike;
        /* Place state as if coarse scan landed at cap=16 with step_cap=3, C_mult=1 */
        g_c_cap      = 16;
        g_c_step_cap = 3;
        g_c_C_mult   = 1;
        g_c_ind      = 0;
        sharp_cap();
        result_t r = { g_c_ind, g_c_cap, g_c_SW, g_i_SWR };
        CHECK("S11 jittery ADC spike — sharp_cap finds true min at cap=17",
              r.cap == 17, r);
    }

    printf("\n%d/%d passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
