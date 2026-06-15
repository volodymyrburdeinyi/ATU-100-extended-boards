# ATU-100 tools — context for AI assistants

## Files

- `atu-daemon.py` — WSJT-X UDP → ATU-100 serial bridge with terminal UI. Zero external dependencies.
- `atusim.c` — standalone algorithm regression simulator. Compile and run before any firmware commit.

## freq_enc encoding

Both tools share the same frequency encoding used in firmware EEPROM and UART commands:

```
freq_enc = kHz / 200  (integer division)  =  MHz × 5
```

| Band | Freq   | freq_enc (dec) | freq_enc (hex) |
|------|--------|----------------|----------------|
| 160m | 1.8 MHz | 9              | 0x09           |
| 80m  | 3.5 MHz | 17             | 0x11           |
| 40m  | 7.0 MHz | 35             | 0x23           |
| 30m  | 10.1 MHz| 50             | 0x32           |
| 20m  | 14.0 MHz| 70             | 0x46           |
| 17m  | 18.1 MHz| 90             | 0x5a           |
| 15m  | 21.0 MHz| 105            | 0x69           |
| 12m  | 24.9 MHz| 124            | 0x7c           |
| 10m  | 28.0 MHz| 140            | 0x8c           |

WSJT-X sends frequency in Hz; daemon converts with `enc = hz // 200_000`.
Firmware EEPROM tolerance: ±2 freq_enc units (`EEPROM_BAND_FREQ_TOL`).

## Serial protocol contract

Physical: RB1=TX (PIC→host), RB2=RX (host→PIC), 9600 8N1 bit-bang, USB-UART converter.

### Control protocol — host → PIC

Commands are ASCII terminated with `\r`. The printable-ASCII filter in firmware discards
any byte outside `0x20–0x7E`, so non-printable bytes reset the command buffer on the PIC side.

| Command    | Meaning                                  | Response keywords        |
|------------|------------------------------------------|--------------------------|
| `l HH\r`  | recall slot for freq_enc HH              | `RECALL` or `NOMATCH`    |
| `t HH\r`  | tune at freq hint HH                     | response contains `IND=` |
| `r\r`      | reset all relays to zero                 | any                      |
| `?\r`      | status query (IND CAP SW SWR AUTO SLOTS) | any                      |
| `e HH\r`  | read EEPROM cell at address HH (hex)     | any                      |
| `c HH VV\r`| write EEPROM cell HH with value VV      | any                      |
| `a\r`      | toggle auto-tune mode                    | any                      |
| `m\r`      | dump all saved band slots                | any                      |

`l` response detail:
- `RECALL IND=N CAP=N SW=N SWR=N` — slot found and relays applied; SWR is stored value × 10
- `NOMATCH` — no slot within ±2 freq_enc of requested freq

`t` response detail: contains `IND=` on success (daemon checks with `'IND=' in resp`).

### Display protocol — PIC → host (always flowing, unsolicited)

The firmware's `led_wr_str()` path sends display updates over the same TX pin on every screen
refresh. These are **not** responses to commands — they flow continuously whenever the PIC is
running with `UART` defined.

Format: `NNNN:content\r\n`

- `NNNN` — 4-digit zero-padded display position code (col × 1000 + row from OLED driver)
- Example: `2016:SWR=1.15\r\n`, `0000:TUNE\r\n`

The daemon identifies display lines with regex `^\d{4}:` and routes them to `_parse_display()`.
Everything else goes to `_response_queue` as a control response.

**If you change led_wr_str() output format or the `NNNN:` prefix, update `_DISP_RE` in the daemon.**

## Daemon architecture

### Threading model

| Thread        | Role                                                    |
|---------------|---------------------------------------------------------|
| `serial_reader` | Owns ALL serial reads. Routes display vs control lines. Never called from other threads. |
| `event_loop`  | Handles WSJT-X band-change events and deferred tune-after-TX. |
| `display_loop`| Redraws terminal UI on `need_redraw` event or 1 s timeout. |
| `input_loop`  | Reads keyboard; dispatches key/command to run_band/run_cmd. |
| `udp_loop`    | Receives WSJT-X Status packets on UDP 2237; updates `st['enc']` and `st['tx']`. |
| `run_band`    | Short-lived; acquires `serial_sem`, recalls or tunes. Non-blocking acquire (skips if busy). |
| `run_cmd`     | Short-lived; acquires `serial_sem`, sends raw command. |

All threads are `daemon=True`. Terminal setup and restore lives in `main()` via `atexit.register()`
— this is intentional so that daemon threads killed on exit do not leave the terminal broken.

### Serial concurrency

- `serial_sem` — semaphore that guards `os.write()`. Acquired non-blocking: if busy, caller logs
  "busy — try again" and returns immediately. This prevents two threads issuing commands
  simultaneously but never blocks the UI thread.
- `_response_queue` — all control responses land here via `serial_reader`. `serial_cmd()` drains
  stale entries then blocks on the queue for up to `timeout` seconds.
- `serial_reader` is the only code that calls `os.read()` on the serial fd. Do not call
  `os.read()` from `serial_cmd()` or anywhere else.

### Band change flow (WSJT-X auto mode)

1. `udp_loop` decodes Status packet → sets `st['enc']`, `st['tx']`; fires `st['event']`
2. `event_loop` wakes, checks `enc != last_enc` → spawns `run_band(port, enc)`
3. `run_band` → `serial_cmd(fd, 'l HH')` → 3 s timeout
   - `RECALL` → apply stored SWR, done
   - `NOMATCH` + TX active → set `pending_tune_enc`, defer
   - `NOMATCH` + RX → proceed to `serial_cmd(fd, 't HH')` → 45 s timeout
4. On TX-end (`prev_tx and not tx`): `event_loop` checks `pending_tune_enc`, spawns tune

### Shared state (`st` dict)

All fields guarded by `lock` except Events (thread-safe themselves):

| Key               | Type              | Meaning                                      |
|-------------------|-------------------|----------------------------------------------|
| `fd`              | int or None       | serial file descriptor; None = disconnected  |
| `enc`             | int               | current freq_enc from WSJT-X or key press    |
| `tx`              | bool              | True while WSJT-X is transmitting            |
| `swr`             | int               | SWR × 100 (150 = 1.50:1); 0 = unknown        |
| `status`          | str               | one-line status shown in UI                  |
| `pending_tune_enc`| int               | enc waiting for TX-end before tune; 0 = none |
| `retrying`        | bool              | True while serial reconnect thread is running|
| `line_mode`       | bool              | True when `:` command-line input is active   |
| `quit`            | threading.Event   | set to exit all threads                      |
| `event`           | threading.Event   | set by udp_loop to wake event_loop           |
| `need_redraw`     | threading.Event   | set to wake display_loop for immediate redraw|

## Simulator (atusim.c)

Runs 6 algorithm scenarios in pure C, no hardware required:

```bash
gcc -o /tmp/atusim tools/atusim.c -lm && /tmp/atusim
```

All 6 must print `PASS`. The pre-commit hook runs this automatically.

Scenarios: bimodal 30m (delta loop), flat unmatchable, simple 20m, TX-inhibit abort,
band probe hit, band slot write. Add a scenario here whenever you add a new algorithm path.
