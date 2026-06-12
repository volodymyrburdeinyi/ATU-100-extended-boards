# ATU-100 EXT board firmware — WA1RCT XC8 port, bug fixes and band memory

Fork of the [N7DDC ATU-100](https://github.com/Dfinitski/N7DDC-ATU-100-mini-and-extended-boards) project.

**Compatible with the N7DDC ATU-100 extended board** (PIC16F1938, 7×7 L/C elements).

## EEPROM configuration

256 bytes, 100 000 write cycles. Written once at first power-on from the `initial_eeprom[]` table in `main.c`. To reconfigure, write the cell directly (MPLAB programmer, UART `c HH VV` command, or PICkit).

### Settings (0x00–0x0F)

| Address | Name | Default | Notes |
|---------|------|---------|-------|
| 0x00 | Display I²C address | 0x3C | SSD1306 |
| 0x01 | Display type | 0 | 0=SSD1306, 1=SH1106 |
| 0x02 | Automatic mode | 1 | 1=on |
| 0x03 | Timeout (×100 ms) | 100 | 10 s |
| 0x04 | SWR tune threshold (×10) | 15 | 1.5:1 |
| 0x05 | Min power to start (W) | 1 | |
| 0x06 | Max power (W) | 120 | |
| 0x07 | Display offset down | 0 | |
| 0x08 | Display offset left | 0 | |
| 0x09 | Max SWR to begin tune | 30 | 3.0:1 |
| 0x0A | Number of inductors | 7 | |
| 0x0B | Inductor step (linear) | 0 | 0=log |
| 0x0C | Number of capacitors | 7 | |
| 0x0D | Capacitor step (linear) | 0 | 0=log |
| 0x0E | Enable nonlinear diode | 0 | |
| 0x0F | Inverse inductance relay | 0 | |

### Relay tables (0x10–0x2F)

Inductor and capacitor relay bitmaps written by `cells_init()`. Values depend on the L/C network topology — do not edit unless rebuilding the relay table.

### Extended settings (0x30–0x35)

| Address | Name | Default | Notes |
|---------|------|---------|-------|
| 0x30 | Power measure level | 0 | |
| 0x31 | Tandem match | 0 | |
| 0x32 | Display off timer | 0 | |
| 0x33 | Additional indication | 0 | |
| 0x34 | Feeder loss (dB×10) | 0 | |
| 0x35 | Disable relays | 0 | |

### Band memory (0x36–0x57)

| Address | Name | Default | Notes |
|---------|------|---------|-------|
| 0x36 | Band slot count | 8 | **0 = feature off**, 1–8 = active slots |
| 0x37 | Write pointer | 0xFF | resets to 0 on first write |
| 0x38–0x57 | Slot data | 0xFF | 8 slots × 4 bytes each |

Each slot: `ind` (1 B), `cap` (1 B), `sw` (1 B, bit 0 only), `swr/10` (1 B). Empty slot sentinel: `ind == 0xFF`.

To disable band memory: write `0x00` to `0x36`.  
To limit to 4 slots: write `0x04` to `0x36`.

### Last known state (0xFB–0xFF)

Saved on each successful tune. Restored at power-on so the ATU starts from the last working position.

| Address | Name |
|---------|------|
| 0xFB | Last SWR (low byte) |
| 0xFC | Last SWR (high byte) |
| 0xFD | Last SW relay |
| 0xFE | Last inductance |
| 0xFF | Last capacitance |

## Band memory

8 EEPROM slots store previously found L/C/SW positions. At the start of each tune, all saved slots are probed before committing to a full scan. If a slot matches (SWR < 1.5:1), tuning is instant — no frequency counter or CAT interface required. Hard-won tune results are saved automatically once the algorithm has worked long enough (configurable effort threshold, default 20 `get_swr()` calls).

## Build

Open `ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/` in MPLAB X with XC8.

## Algorithm regression tests

```bash
gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
```

Run before flashing. The pre-commit hook runs this automatically.

## Bug fixes

- **Fix 1** — `lcd_swr()` used global `g_i_SWR` instead of the function parameter; display showed wrong SWR after certain tune sequences.
- **Fix 2a/2b** — Greedy `else break` in `coarse_cap()` and `coarse_tune()` caused the algorithm to stop at a local SWR minimum. Removed; algorithm now scans the full range and keeps the global best.
- **Fix 3** — After SW-relay experiment in `sub_tune()`, SWR was read from a stale variable instead of re-measured. Now calls `get_swr()` after reverting SW.
- **Fix 5** — On rigs that inhibit TX when SWR exceeds a threshold (e.g. QMX+), `get_swr()` would deadlock waiting for power that never returns. A `g_b_tx_seen` flag distinguishes normal RX-wait from TX-inhibit; on inhibit, tune aborts cleanly and restores the pre-scan relay position.

## Original project

[N7DDC ATU-100 mini and extended boards](https://github.com/Dfinitski/N7DDC-ATU-100-mini-and-extended-boards) by David Fainitski N7DDC, XC8 port by WA1RCT.
