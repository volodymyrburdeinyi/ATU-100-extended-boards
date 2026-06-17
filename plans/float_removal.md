# Float removal from get_pwr()

## Motivation

The only use of `double` in the firmware is in `get_pwr()` (main.h). On XC8 free tier,
`double` is 24-bit software-emulated — no FPU on PIC16F1938. The float library costs
~2–4 KB of the 16 KB flash budget. Runtime cost (~0.5ms per call) is irrelevant given
the 20ms relay-settle delay that dominates every SWR measurement cycle.

**Do this if flash usage becomes a concern.** Not a runtime performance fix.

## The change

The `/1.414` (peak-to-RMS conversion) can be absorbed into the final divisor:

```
V_rms = V_peak / √2
P = V_rms² / R = V_peak² / (2R)
```

So divide by `2R` instead of `R`, and drop the `/1.414` entirely.

### Current (float)

```c
double l_doub_pwr;
l_doub_pwr = correction((int)(l_Forward * 3));  // or l_Forward * 3
l_doub_pwr = l_doub_pwr * e_c_K_Mult / 1000.0;
l_doub_pwr = l_doub_pwr / 1.414;
if (e_c_b_P_High == 1)
    l_doub_pwr = l_doub_pwr * l_doub_pwr / 50;   // 0–1500W
else
    l_doub_pwr = l_doub_pwr * l_doub_pwr / 5;    // 0–151W
l_doub_pwr = l_doub_pwr + 0.5;
g_i_PWR = (int)(l_doub_pwr);
```

### Proposed (integer)

```c
long l_pwr;
l_pwr = (long)(e_c_b_D_correction ? correction((int)(l_Forward * 3)) : (int)(l_Forward * 3))
        * e_c_K_Mult / 1000;
if (e_c_b_P_High == 1)
    g_i_PWR = (int)((l_pwr * l_pwr + 50) / 100);  // divisor 50→100 (absorbed √2²)
else
    g_i_PWR = (int)((l_pwr * l_pwr + 5)  / 10);   // divisor  5→10
```

The `+ 50` / `+ 5` terms replace `+ 0.5` rounding (integer equivalent).

## What we lose

Approximately 2–3% watt-reading precision due to integer truncation before squaring.
This is within the tandem-match coupler's own measurement uncertainty (~5–10% for a
hand-wound transformer). SWR accuracy is completely unaffected — `g_i_SWR` is already
pure integer arithmetic.

## Overflow check (K_Mult=32, ADC max ~4000)

- `correction(4000 * 3)` → ~12860 (int, fits)
- `12860 * 32` → 411,520 (long, fits)
- `/ 1000` → 411 (l_pwr)
- `411 * 411` → 168,921 (long, fits)
- `/ 100` → 1689 (int, fits — well under 32767)
