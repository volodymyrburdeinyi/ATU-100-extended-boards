---
name: plan-interpolation
description: "Linear interpolation of relay values between stored band sub-slots — deferred, plan only"
metadata: 
  node_type: memory
  type: project
  originSessionId: c1a99cb0-4828-4375-abc1-7734d64bacca
---

Deferred feature: interpolate ind/cap between two stored sub-slots when recalling relay settings, instead of nearest-neighbor. Adversarially reviewed and found sound in principle, but requires careful implementation.

**Why:** Non-resonant antenna has smooth impedance curve; linear relay interpolation between two stored points gives a better starting position for the probe phase, reducing full-tune frequency.

**Where it goes:**
- New `band_interp_relays(unsigned int l_freq_kHz)` helper in `main.h` alongside `band_slot_save`
- Called from the probe block in `tune()` (after the 25 kHz exact-match scan fails)
- Called from `band_slot_apply_freq()` in `main.c` (UART `l HHHH` path)
- Mirrored verbatim in `tools/atusim.c` with 5 new scenarios (S11–S15)

**Guards required (from adversarial review):**
- Use `unsigned long` intermediate for multiply — 16-bit overflows on 10m/6m (127 × 4000 = 508 000)
- Compute delta as `(long)(int)ind_hi - (long)(int)ind_lo` — signed subtraction wrap if ind0 > ind1
- Guard `freq_hi == freq_lo` → return 0 (div-by-zero, undefined on PIC even if XC8 produces 0)
- Sort bracket pair by stored frequency, not sub-slot index — slots written in eviction order
- Clamp result to [0, 127] before assigning to `g_c_ind`/`g_c_cap`
- **Exclude band index 9 (6m)** — 2 MHz gaps at 50 MHz make linear relay-bit interpolation physically invalid; probe would return l_probe_matched=1 with wrong relays, suppressing recovery
- If bracket slots disagree on `sw`: use nearer slot's sw, proceed (wrong sw → full tune recovers)

**Algorithm sketch:**
1. Collect valid (ind != 0xFF) sub-slots for the band, count them
2. If count < 2 or band == 9: return 0
3. Find lo-slot (highest stored freq ≤ target) and hi-slot (lowest stored freq > target); if target is outside range use two nearest (extrapolation ok for non-resonant antenna, clamping handles limits)
4. Guard freq_lo == freq_hi → return 0
5. `l_di * l_dt / l_df` all as `long`; clamp; write globals; return 1

**Successful interpolation never saved to EEPROM** — `l_probe_matched = 1` prevents `band_slot_save`. Acceptable for fixed-frequency FT8 operation; intermediate freqs stay ephemeral. Could be addressed separately by relaxing the probe_matched guard.

**Simulator scenarios to add (S11–S15):**
- S11: two 40m slots, interpolate midpoint — result between endpoints
- S12: extrapolate beyond edge — clamped, no crash
- S13: 6m, two slots — nearest-neighbor used
- S14: differing sw between bracket slots — nearer sw used, ind/cap still interpolated
- S15: one valid slot — nearest-neighbor, no div-by-zero
