# ATU-100 EXT board firmware — WA1RCT XC8 port, bug fixes and band memory

Fork of the [N7DDC ATU-100](https://github.com/Dfinitski/N7DDC-ATU-100-mini-and-extended-boards) project.

**Compatible with the N7DDC ATU-100 extended board** (PIC16F1938, 7×7 L/C elements).

## EEPROM configuration

256 bytes, 100 000 write cycles. Initial values are embedded in the hex file via `initial_eeprom[]` in `main.c` and programmed by PICkit at flash time. To reconfigure individual cells after deployment, use the MPLAB programmer or PICkit directly.

### Settings (0x00–0x0F)

| Address | Name | Default | Notes |
|---------|------|---------|-------|
| 0x00 | Display I²C address | 0x78 | 0x3C shifted left (8-bit form) |
| 0x01 | Display type | 5 | |
| 0x02 | Automatic mode | 1 | 1=on |
| 0x03 | Timeout (×100 ms) | 21 | 2.1 s |
| 0x04 | SWR tune threshold (BCD×10) | 0x13 | 1.3:1; delta = threshold−1.0 |
| 0x05 | Min power to start (W) | 1 | |
| 0x06 | Max power (W) | 0 | |
| 0x07 | Display offset down | 0 | |
| 0x08 | Display offset left | 2 | |
| 0x09 | Max SWR to begin tune | 0 | |
| 0x0A | Number of inductors | 7 | |
| 0x0B | Inductor step (linear) | 0 | 0=log |
| 0x0C | Number of capacitors | 7 | |
| 0x0D | Capacitor step (linear) | 0 | 0=log |
| 0x0E | Enable nonlinear diode | 1 | |
| 0x0F | Inverse inductance relay | 0 | |

### Relay tables (0x10–0x2F)

Inductor and capacitor relay bitmaps written by `cells_init()`. Values depend on the L/C network topology — do not edit unless rebuilding the relay table.

### Extended settings (0x30–0x35)

| Address | Name | Default | Notes |
|---------|------|---------|-------|
| 0x30 | Power measure level | 0 | |
| 0x31 | Tandem match | 16 | |
| 0x32 | Display off timer | 0 | |
| 0x33 | Additional indication (feeder loss display) | 0 | 1=on |
| 0x34 | Feeder loss (tenths dB, BCD) | 0 | |
| 0x35 | Disable relays | 0 | |

### Band memory (0x36–0x5F)

| Address | Name | Default | Notes |
|---------|------|---------|-------|
| 0x36 | Band slot count | 8 | **0 = feature off**, 1–8 = active slots |
| 0x37 | Write pointer | 0xFF | resets to 0 on first write |
| 0x38–0x5F | Slot data | 0xFF | 8 slots × 5 bytes each |

Each slot: `freq_enc` (1 B, 0 = no freq hardware; RF_MHz × 5 otherwise), `ind` (1 B), `cap` (1 B), `sw` (1 B, bit 0 only), `swr/10` (1 B). Empty slot sentinel: `ind == 0xFF` (byte +1).

`freq_enc` encoding: 80m = 17, 40m = 35, 30m = 50, 20m = 70, 17m = 91, 15m = 105, 12m = 124, 10m = 140.

To disable band memory: write `0x00` to `0x36`.  
To limit to 4 slots: write `0x04` to `0x36`.

### Format version

| Address | Name | Default | Notes |
|---------|------|---------|-------|
| `EEPROM_BAND_SLOT_0 + EEPROM_BAND_SLOT_COUNT × 5` | EEPROM format version | 0x01 | computed — follows last slot; set by `cells_init()` on first boot if mismatch; clears 0x36 to protect against old-format slots |

With default 8 slots this is 0x60. Increasing `EEPROM_BAND_SLOT_COUNT` to 12 moves it to 0x74 automatically. Max practical count: 12 (9 HF bands + redundancy). Hard ceiling: 38 slots before colliding with 0xFB last-known-state area.

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

## UART serial control

**Port:** RB1 = TX (PIC output), RB2 = RX (PIC input). 9600 baud, 8N1, bit-bang.  
Enable by ensuring `#define UART` is active in `cross_compiler.h` (it is by default).

Commands are terminated by `\r` or `\n`. Responses end with `\r\n`.

| Command | Example | Response |
|---------|---------|----------|
| `t` | `t` | status line (see `?`) |
| `t HH` | `t 46` | tune with freq hint `freq_enc=0x46` (14 MHz); status line |
| `l HH` | `l 46` | recall slot for freq, apply relays: `RECALL IND=.. CAP=.. SW=.. SWR=..` or `NOMATCH` |
| `m` | `m` | dump all band slots, one line each |
| `e HH` | `e 36` | read EEPROM cell: `EEPROM[0x36]=0x08` |
| `c HH VV` | `c 36 08` | write EEPROM cell |
| `a` | `a` | toggle auto mode: `OK AUTO=1` |
| `r` | `r` | reset relays: `OK RESET` |
| `?` | `?` | `IND=.. CAP=.. SW=.. SWR=.. AUTO=.. SLOTS=..` |

**freq_enc** = RF frequency in kHz ÷ 200 (= MHz × 5). Fits in one byte. Tolerance for slot matching: ±2 units (±400 kHz).

| Band | MHz | freq_enc (decimal) | hex |
|------|----|---------------------|-----|
| 80m | 3.5 | 17 | `11` |
| 40m | 7.0 | 35 | `23` |
| 30m | 10.1 | 50 | `32` |
| 20m | 14.0 | 70 | `46` |
| 17m | 18.1 | 90 | `5a` |
| 15m | 21.0 | 105 | `69` |
| 10m | 28.0 | 140 | `8c` |

**Typical script workflow (no frequency counter hardware):**

1. Script detects band change via WSJT-X UDP or CAT.
2. Computes `freq_enc = kHz // 200`, sends `l HH` to pre-position relays from band memory.
3. If `NOMATCH`: sends `t HH` to run a full tune and save the result tagged with the freq.
4. On subsequent band returns: `l HH` hits the saved slot, tune is instant.

## Remote operation daemon

`tools/atu-daemon.py` bridges WSJT-X to the ATU-100 and provides a live terminal interface. Python 3 stdlib only — no pip installs required.

```bash
python3 tools/atu-daemon.py /dev/ttyUSB0
python3 tools/atu-daemon.py --help
```

On every WSJT-X band change the daemon recalls the matching saved slot. If no slot matches, it runs a full tune and saves the result. Commands are held while WSJT-X is transmitting.

**Terminal keys** (no Enter needed): `1–9` recall band · `t` tune current band · `r` reset relays · `:` command line · `h` help · `q` quit

The band row at the bottom highlights the active band so the key mapping is self-teaching:

```
1·160  2·80  3·40  4·30  [5·20]  6·17  7·15  8·12  9·10
```

Connect a USB-UART converter to **RB1 (TX)** and **RB2 (RX)** on the ATU-100 board. 9600 baud 8N1.

## Build

Open `ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2/` in MPLAB X IDE with XC8 compiler. No external libraries required.

Install the pre-commit hook to run regression tests before every commit:

```bash
cp hooks/pre-commit .git/hooks/pre-commit && chmod +x .git/hooks/pre-commit
```

## Algorithm regression tests

```bash
gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
```

9 scenarios must all PASS before flashing. The pre-commit hook runs this automatically.

## Bug fixes

- **Fix 1** — `lcd_swr()` used global `g_i_SWR` instead of the function parameter; display showed wrong SWR after certain tune sequences.
- **Fix 2a/2b** — Greedy `else break` in `coarse_cap()` and `coarse_tune()` caused the algorithm to stop at a local SWR minimum. Removed; algorithm now scans the full range and keeps the global best.
- **Fix 3** — After SW-relay experiment in `sub_tune()`, SWR was read from a stale variable instead of re-measured. Now calls `get_swr()` after reverting SW.
- **Fix 5** — On rigs that inhibit TX when SWR exceeds a threshold (e.g. QMX+), `get_swr()` would deadlock waiting for power that never returns. A `g_b_tx_seen` flag distinguishes normal RX-wait from TX-inhibit; on inhibit, tune aborts cleanly and restores the pre-scan relay position.

## Acknowledgements

- [N7DDC ATU-100 mini and extended boards](https://github.com/Dfinitski/N7DDC-ATU-100-mini-and-extended-boards) — original firmware by David Fainitski N7DDC
- [WA1RCT](https://github.com/wa1rct) — XC8 / MPLAB X port that this fork is based on
- [edsonbrusque/ATU-100](https://github.com/edsonbrusque/ATU-100) — UART implementation reference (RB1/RB2 pin assignment)
