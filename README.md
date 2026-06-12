# ATU-100 EXT board firmware — WA1RCT XC8 port, bug fixes and band memory

Fork of the [N7DDC ATU-100](https://github.com/Dfinitski/N7DDC-ATU-100-mini-and-extended-boards) project.

**Compatible with the N7DDC ATU-100 extended board** (PIC16F1938, 7×7 L/C elements).

## What this fork adds

### Bug fixes

- **Fix 1** — `lcd_swr()` used global `g_i_SWR` instead of the function parameter; display showed wrong SWR after certain tune sequences.
- **Fix 2a/2b** — Greedy `else break` in `coarse_cap()` and `coarse_tune()` caused the algorithm to stop at a local SWR minimum. Removed; algorithm now scans the full range and keeps the global best.
- **Fix 3** — After SW-relay experiment in `sub_tune()`, SWR was read from a stale variable instead of re-measured. Now calls `get_swr()` after reverting SW.
- **Fix 5** — On rigs that inhibit TX when SWR exceeds a threshold (e.g. QMX+), `get_swr()` would deadlock waiting for power that never returns. A `g_b_tx_seen` flag distinguishes normal RX-wait from TX-inhibit; on inhibit, tune aborts cleanly and restores the pre-scan relay position.

### Band memory

8 EEPROM slots store previously found L/C/SW positions. At the start of each tune, all saved slots are probed before committing to a full scan. If a slot matches (SWR < 1.5:1), tuning is instant — no frequency counter or CAT interface required. Hard-won tune results are saved automatically (configurable effort threshold).

## Build

Open `ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/` in MPLAB X with XC8.

## Algorithm regression tests

```bash
gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
```

Run before flashing. The pre-commit hook runs this automatically.

## Original project

[N7DDC ATU-100 mini and extended boards](https://github.com/Dfinitski/N7DDC-ATU-100-mini-and-extended-boards) by David Fainitski N7DDC, XC8 port by WA1RCT.
