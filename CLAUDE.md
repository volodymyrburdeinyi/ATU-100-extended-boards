# ATU-100 EXT board firmware — project context

## Hardware

- **MCU**: PIC16F1938, XC8 compiler (MPLAB X)
- **Board**: N7DDC ATU-100 EXT board (7×7 L/C elements, up to 1500 W)
- **Station**: QMX+ 5 W, 41 m delta loop, 2 m 130 Ω symmetric feeder, balun, no display, remote placement, unattended WSJT-X FT8/FT4 operation

## XC8 type rules — read before touching any code

- `char` is **signed** by default in XC8. Use `unsigned char` explicitly for EEPROM addresses, relay bit-fields, and any value that must not go negative.
- `eeprom_read()` returns `unsigned char`. Storing the result in `char` silently sign-extends 0xFF to -1.
- Comparing `char` against `0xFF` is always false (signed -1 != unsigned int 255). Use `unsigned char` or cast with `(char)0xFF`.
- `int` is 16 bits on PIC.

## Variable naming (WA1RCT XC8 port convention)

| Prefix | Type |
|--------|------|
| `g_i_` | global int |
| `g_c_` | global char |
| `g_b_` | global boolean (char) |
| `e_i_` | EEPROM-backed int |
| `e_c_b_` | EEPROM-backed boolean |
| `l_` | local variable |

## Key globals

- `g_c_SW` — capacitor bank **position** relay (0 = antenna side, 1 = TX side). NOT SWR.
- `g_i_SWR` — SWR × 100 (100 = 1.0:1, 150 = 1.5:1, 999 = max). **0 = TX-inhibit abort sentinel**.
- `g_char_tune_effort` — unsigned char, count of `get_swr()` calls this tune session. Saturates at 255.
- `g_b_tx_seen` — set when power exceeded minimum during this tune; triggers graceful abort if power subsequently drops.

## EEPROM layout (256 bytes, 100k write cycles)

```
0x00–0x2F  settings and relay tables (cells_init reads these)
0x30–0x35  more settings (TANDEM_MATCH, DISPLAY_OFF_TIMER, etc.)
0x36       EEPROM_BAND_PTR  — circular write pointer for band slots (0–7)
0x37–0x56  band memory: 8 slots × 4 bytes (ind, cap, sw, swr/10)
0xFB–0xFF  LAST_SWR_L, LAST_SWR_H, LAST_SW, LAST_IND, LAST_CAP
```

Empty slot sentinel: ind byte == 0xFF. Initial EEPROM (from `initial_eeprom[]` in main.c) has 0xFF at 0x36–0xFF.

## Firmware files

- `ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/main.h` — algorithm functions (tune, sub_tune, coarse_tune, coarse_cap, sharp_ind, sharp_cap, band_slot_save, get_swr)
- `ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/main.c` — main loop, EEPROM init table, display/button handling
- `ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/cross_compiler.h` — EEPROM address defines, pin defines, XC8/MikroC portability layer

## Algorithm regression simulator

```bash
gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
```

6 scenarios: bimodal 30m, flat unmatchable, simple 20m, TX-inhibit abort, band probe hit, band slot write. Must all PASS before committing. The pre-commit hook runs this automatically.

## Pre-commit hook

```bash
cp hooks/pre-commit .git/hooks/pre-commit && chmod +x .git/hooks/pre-commit
```

Checks: Fix 1 (lcd_swr parameter), Fix 2a/2b (non-greedy coarse scan), Fix 3 (sub_tune SW-revert live SWR), band_slot_save unsigned char, probe loop unsigned char.


## Skills

- `/atu-review` — static fix verification + scenario matrix for all known fixes
