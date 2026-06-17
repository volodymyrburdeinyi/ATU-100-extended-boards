# Band memory — 3 sub-slots per band (implemented)

## What was built

10 bands × 3 sub-slots × 5 bytes = 150 bytes of EEPROM band memory.
kHz-based addressing replaces the old freq_enc (200 kHz/step) scheme.
6m (50 MHz) added as band 9, key '0' in daemon.

## Why kHz instead of freq_enc

freq_enc (RF_MHz × 5, 200 kHz resolution) cannot distinguish 7.025 from 7.074 —
both encode to 35. With 3 sub-slots per band needing independent frequency identity,
1 kHz resolution is required. Raw kHz fits in uint16 (max 54000 for 6m).

Freed the old freq_enc byte by packing sw (1 bit) + swr/10 (7 bits) into one byte.
Slot stays 5 bytes total.

## EEPROM layout

```
0x36  format version byte (must be 0x02; cells_init wipes 150 bytes on mismatch)
0x37  reserved
0x38–0xCF  band slots: 10 bands × 3 sub-slots × 5 bytes
0xD0–0xFA  unused
0xFB–0xFF  LAST_SWR_L, LAST_SWR_H, LAST_SW, LAST_IND, LAST_CAP
```

Slot layout (5 bytes at base `0x38 + slot_idx * 5`):

| Offset | Name           | Content |
|--------|----------------|---------|
| 0      | FREQ_LO        | kHz & 0xFF |
| 1      | FREQ_HI        | kHz >> 8 |
| 2      | IND            | relay bits 0–6; 0xFF = empty sentinel |
| 3      | CAP            | relay bits 0–6 |
| 4      | SW_SWR_PACKED  | bit7 = sw (0/1), bits 6–0 = swr/10 clamped to 127 |

## Band table

| idx | Band | lo kHz | hi kHz | key |
|-----|------|--------|--------|-----|
| 0 | 160m | 1800 | 2000 | 1 |
| 1 | 80m  | 3500 | 4000 | 2 |
| 2 | 40m  | 7000 | 7300 | 3 |
| 3 | 30m  | 10100 | 10150 | 4 |
| 4 | 20m  | 14000 | 14350 | 5 |
| 5 | 17m  | 18068 | 18168 | 6 |
| 6 | 15m  | 21000 | 21450 | 7 |
| 7 | 12m  | 24890 | 24990 | 8 |
| 8 | 10m  | 28000 | 29700 | 9 |
| 9 | 6m   | 50000 | 54000 | 0 |

## Save logic (band_slot_save)

Guards: probe must have missed, tune_effort > 20, SWR < 150, freq known, band valid.

1. Scan 3 sub-slots for band:
   - If stored freq within 25 kHz of target → upsert in place (break)
   - Track first empty sub-slot
   - Track worst stored swr10 (for eviction)
2. Write to: upsert target → first empty → worst-SWR slot (evict)

## Recall logic (band_slot_apply_freq)

Finds nearest stored freq within band, no distance cutoff — nearest-neighbor always.
Applies relays directly (no live SWR measurement — safe in RX).

## Probe in tune()

Scans 3 sub-slots for band, applies each within 25 kHz of target, calls get_swr().
SWR < 150 → probe_matched = 1, return early. No full tune needed.
If no match → full tune (coarse + sharp + sub_tune).

## UART protocol (V2, kHz-based)

- `t HHHH` — tune with freq hint in kHz hex (e.g. `t 1BA2` = 7074 kHz)
- `l HHHH` — recall nearest slot for freq (e.g. `l 1BA2`)
- `m`       — dump all 30 slots
- `e HH` / `c HH VV` — EEPROM read/write

g_i_uart_freq_hint is unsigned int (was unsigned char in old design).

## Adversarial findings (from interpolation review, base-design impact)

These were found during adversarial review of the interpolation proposal but describe
properties of the base implementation worth knowing:

- **SWR-based eviction can cluster slots**: evicting worst-SWR slot may remove the
  most frequency-distant slot, leaving two close-together slots. With nearest-neighbor
  recall this is fine — the retained slots both have better proven SWR. Only matters
  if interpolation is added later (see `plans/interpolation.md`).

- **measure_freq() always returns 0**: frequency knowledge depends entirely on the
  UART `t HHHH` hint. Button-triggered tunes without UART skip probe and go straight
  to full tune.

- **Three tunes at same kHz → 1 occupied sub-slot**: upsert tolerance 25 kHz means
  repeated FT8 operation at one frequency leaves the other 2 sub-slots empty.
  Nearest-neighbor still works fine; only matters for interpolation.

## Simulator coverage (10/10)

S1 bimodal 30m, S2 flat unmatchable, S3 simple 20m, S4 TX-inhibit abort,
S5 band probe hit (kHz recall), S6 new slot write (kHz save),
S7 cross-band probe miss, S8 upsert update, S9 evict worst-SWR, S10 cross-band isolation.

## Deferred

- Interpolation between sub-slots → `plans/interpolation.md`
- Non-blocking tune() refactor → `plans/nonblocking_refactor.md` (in memory)
