#!/usr/bin/env python3
"""
atu-daemon.py — WSJT-X UDP 2237 → ATU-100 UART bridge
usage: python3 atu-daemon.py /dev/ttyUSB0
       python3 atu-daemon.py --help
"""

_HELP = """
ATU-100 daemon — bridges WSJT-X band changes to the ATU-100 automatically
and lets you control it directly from the same terminal.
Zero dependencies beyond Python 3 stdlib.

usage:
  python3 atu-daemon.py /dev/ttyUSB0

keys (no Enter needed):
  1 … 9   recall saved slot for that band (160m … 10m)
  t       tune current band (shown in display)
  r       reset relays to zero
  ?       query ATU status
  :       enter command line (Enter to send, Esc to cancel)
  h       this help
  q       quit

command line (after pressing :):
  t [HH]   tune; optional freq hint in hex  e.g. t 46  (14 MHz)
  l HH     recall saved slot for freq
  m        dump all saved band slots
  e HH     read EEPROM cell               e.g. e 36
  c HH VV  write EEPROM cell              e.g. c 36 08
  a        toggle auto mode
  r        reset relays
  ?        ATU status line (IND CAP SW SWR AUTO SLOTS)

freq_enc = kHz / 200 = MHz × 5
  160m=09  80m=11  40m=23  30m=32  20m=46  17m=5a  15m=69  10m=8c

serial port: RB1=TX, RB2=RX, 9600 8N1 (USB-UART converter)
"""

import sys
import os
import socket
import struct
import termios
import tty
import select
import time
import threading

WSJT_PORT       = 2237
WSJT_MAGIC      = 0xADBCCBDA
_SERIAL_BUF_MAX = 512

BANDS = [
    (12,  25,  '160m'),
    (25,  33,   '80m'),
    (33,  45,   '40m'),
    (45,  58,   '30m'),
    (58,  80,   '20m'),
    (80,  97,   '17m'),
    (97,  115,  '15m'),
    (115, 132,  '12m'),
    (132, 155,  '10m'),
]

# band key → freq_enc (centre of band, kHz / 200)
BAND_KEYS = {
    '1': (0x09, '160m'),
    '2': (0x11,  '80m'),
    '3': (0x23,  '40m'),
    '4': (0x32,  '30m'),
    '5': (0x46,  '20m'),
    '6': (0x5a,  '17m'),
    '7': (0x69,  '15m'),
    '8': (0x7c,  '12m'),
    '9': (0x8c,  '10m'),
}


def band(enc):
    return next((n for lo, hi, n in BANDS if lo <= enc < hi), '?')


# ── WSJT-X packet parsing ────────────────────────────────────────────────────

def parse_utf8(d, off):
    if off + 4 > len(d):
        return '', off
    ln = struct.unpack_from('>I', d, off)[0]
    if ln == 0xFFFFFFFF:
        return '', off + 4
    end = off + 4 + ln
    if end > len(d):
        return '', off + 4
    return d[off + 4:end].decode(errors='replace'), end


def parse_status(d):
    """Parse WSJT-X type-1 Status. Returns (freq_hz, transmitting) or None."""
    if len(d) < 12:
        return None
    magic, schema, mtype = struct.unpack_from('>III', d, 0)
    if magic != WSJT_MAGIC or schema < 2 or mtype != 1:
        return None
    _, off = parse_utf8(d, 12)             # ID
    if off + 8 > len(d):
        return None
    hz = struct.unpack_from('>Q', d, off)[0]
    off += 8
    for _ in range(4):                     # mode, dx_call, report, tx_mode
        _, off = parse_utf8(d, off)
    if off + 2 > len(d):
        return None
    off += 1                               # tx_enabled (skip)
    return hz, bool(d[off])


# ── serial port ──────────────────────────────────────────────────────────────

class SerialError(Exception):
    pass


def open_serial(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[3] = 0
    a[4] = termios.B9600
    a[5] = termios.B9600
    a[6][termios.VMIN]  = 0
    a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def serial_cmd(fd, cmd, timeout=5.0):
    """Send cmd\\r, return first response line or None on timeout.
    Raises SerialError on OS failure."""
    try:
        os.write(fd, (cmd + '\r').encode())
    except OSError as e:
        raise SerialError(str(e))
    buf = b''
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                chunk = os.read(fd, 64)
                if not chunk:
                    raise SerialError('EOF')
                buf += chunk
                if len(buf) > _SERIAL_BUF_MAX:
                    raise SerialError('response overflow')
                if b'\n' in buf:
                    return buf.split(b'\n')[0].strip().decode(errors='replace')
        except OSError as e:
            raise SerialError(str(e))
    return None


def valid_response(cmd, resp):
    if resp is None:
        return False
    if cmd.startswith('l'):
        return resp.startswith(('RECALL', 'NOMATCH', 'ERR'))
    if cmd.startswith('t'):
        return 'IND=' in resp
    return True


def _extract_swr(resp):
    if resp and 'SWR=' in resp:
        try:
            return int(resp.split('SWR=')[-1].split()[0])
        except (ValueError, IndexError):
            pass
    return 0


# ── shared state ─────────────────────────────────────────────────────────────

st = {
    'fd':               None,
    'enc':              0,
    'tx':               False,
    'swr':              0,
    'status':           'ready — press h for help',
    'status_time':      0.0,
    'retrying':         False,
    'line_mode':        False,
    'line_buf':         '',
    'quit':             threading.Event(),
    'event':            threading.Event(),
    'need_redraw':      threading.Event(),
    'pending_tune_enc': 0,   # set when recall hits NOMATCH during TX; tune fires on TX-end
}
lock       = threading.Lock()
serial_sem = threading.Semaphore(1)   # one serial operation at a time


def set_status(msg):
    with lock:
        st['status']      = msg
        st['status_time'] = time.monotonic()
    st['need_redraw'].set()


# ── serial reconnect ─────────────────────────────────────────────────────────

def open_serial_retry(port):
    while not st['quit'].is_set():
        try:
            fd = open_serial(port)
            with lock:
                st['fd']       = fd
                st['retrying'] = False
            set_status('reconnected')
            return
        except OSError:
            set_status('no device — retrying...')
            time.sleep(2)


def _serial_lost(port):
    """Call inside lock after detecting serial failure."""
    st['fd'] = None
    if not st['retrying']:
        st['retrying'] = True
        threading.Thread(target=open_serial_retry, args=(port,), daemon=True).start()


# ── band operations ──────────────────────────────────────────────────────────

def _do_band(port, enc, force_tune=False):
    """Recall or tune for enc. Caller must hold serial_sem."""
    bnd = band(enc)
    with lock:
        fd = st['fd']
    if fd is None:
        set_status('no device')
        return
    try:
        if not force_tune:
            set_status(f'recalling {bnd}...')
            resp = serial_cmd(fd, f'l {enc:02x}', timeout=3.0)
            if resp is not None and resp.startswith('RECALL'):
                swr = _extract_swr(resp)
                with lock:
                    st['swr'] = swr
                set_status(f'recalled {bnd}')
                return
            # NOMATCH — full tune needed; defer if TX active so we don't
            # change relays mid-transmission on the QMX+
            with lock:
                tx_now = st['tx']
            if tx_now:
                with lock:
                    st['pending_tune_enc'] = enc
                set_status(f'no {bnd} slot · will tune after TX')
                return

        set_status(f'tuning {bnd}...')
        t0   = time.monotonic()
        resp = serial_cmd(fd, f't {enc:02x}', timeout=45.0)
        if resp is None:
            set_status(f'tune timeout after {time.monotonic()-t0:.0f}s')
            return
        if not valid_response(f't {enc:02x}', resp):
            set_status(f'unexpected ATU response: {resp!r}')
            return
        swr = _extract_swr(resp)
        with lock:
            st['swr'] = swr
        saved = swr > 0 and swr < 150
        set_status(f'tuned {bnd} · slot saved' if saved else f'tuned {bnd}')

    except SerialError as e:
        set_status(f'serial error: {e}')
        with lock:
            _serial_lost(port)


def run_band(port, enc, force_tune=False):
    """Acquire serial_sem and run _do_band. Non-blocking: skip if busy."""
    if not serial_sem.acquire(blocking=False):
        set_status('busy — try again')
        return
    try:
        _do_band(port, enc, force_tune=force_tune)
    finally:
        serial_sem.release()


def run_cmd(port, cmd):
    """Acquire serial_sem and run a raw ATU command. Non-blocking: skip if busy."""
    if not serial_sem.acquire(blocking=False):
        set_status('busy — try again')
        return
    try:
        with lock:
            fd = st['fd']
        if fd is None:
            set_status('no device')
            return
        timeout = 45.0 if cmd.startswith('t') else 3.0
        try:
            resp = serial_cmd(fd, cmd, timeout=timeout)
            if resp is None:
                set_status('no response (timeout)')
                return
            swr = _extract_swr(resp)
            with lock:
                if swr:
                    st['swr'] = swr
            set_status(f'{resp}')
        except SerialError as e:
            set_status(f'serial error: {e}')
            with lock:
                _serial_lost(port)
    finally:
        serial_sem.release()


# ── UDP receiver ─────────────────────────────────────────────────────────────

def udp_loop(sock, port):
    sock.settimeout(1.0)
    while not st['quit'].is_set():
        try:
            d, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            time.sleep(0.5)
            continue
        r = parse_status(d)
        if not r:
            continue
        hz, tx = r
        enc = hz // 200_000
        with lock:
            changed  = enc > 0 and enc != st['enc']
            prev_tx  = st['tx']
            if enc > 0:
                st['enc'] = enc
            st['tx'] = tx
        if changed:
            # Recall fires immediately regardless of TX — relay pre-positioning
            # happens before QMX+ SWR protection kicks in.
            # Full tune (if needed after NOMATCH) is deferred until TX ends.
            st['event'].set()
        if prev_tx and not tx:
            # TX just ended — fire pending tune or any deferred band change
            st['event'].set()


# ── event loop (WSJT-X auto) ─────────────────────────────────────────────────

def event_loop(port):
    while not st['quit'].is_set():
        triggered = st['event'].wait(timeout=1.0)
        if triggered:
            st['event'].clear()
            with lock:
                enc     = st['enc']
                tx      = st['tx']
                pending = st['pending_tune_enc']
            # TX just ended: fire deferred tune if one is waiting
            if not tx and pending:
                with lock:
                    st['pending_tune_enc'] = 0
                threading.Thread(
                    target=run_band, args=(port, pending, True), daemon=True
                ).start()
            elif enc:
                # Normal band change: try recall (fires immediately, even during TX)
                threading.Thread(
                    target=run_band, args=(port, enc), daemon=True
                ).start()


# ── input ─────────────────────────────────────────────────────────────────────

def _dispatch_key(port, ch):
    if ch in BAND_KEYS:
        enc, _ = BAND_KEYS[ch]
        with lock:
            st['enc'] = enc
        threading.Thread(target=run_band, args=(port, enc), daemon=True).start()
    elif ch == 't':
        with lock:
            enc = st['enc']
        if enc:
            threading.Thread(target=run_band, args=(port, enc, True), daemon=True).start()
        else:
            set_status('no band selected — press 1-9 first')
    elif ch == 'r':
        threading.Thread(target=run_cmd, args=(port, 'r'), daemon=True).start()
    elif ch == '?':
        threading.Thread(target=run_cmd, args=(port, '?'), daemon=True).start()
    elif ch in ('h', 'H'):
        sys.stdout.write('\033[2J\033[H' + _HELP + '\npress any key to return\n')
        sys.stdout.flush()
        # wait for a key without blocking the quit path
        while not st['quit'].is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.3)
            if r:
                sys.stdin.read(1)
                break
        st['need_redraw'].set()
    elif ch == ':':
        with lock:
            st['line_mode'] = True
            st['line_buf']  = ''
        st['need_redraw'].set()
    elif ch in ('q', 'Q', '\x03', '\x04'):
        st['quit'].set()


def _dispatch_line(port, cmd):
    if cmd in ('help', 'h'):
        sys.stdout.write('\033[2J\033[H' + _HELP + '\npress any key to return\n')
        sys.stdout.flush()
        while not st['quit'].is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.3)
            if r:
                sys.stdin.read(1)
                break
        st['need_redraw'].set()
    else:
        threading.Thread(target=run_cmd, args=(port, cmd), daemon=True).start()


def input_loop(port):
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while not st['quit'].is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not r:
                continue
            ch = sys.stdin.read(1)
            if not ch:
                break

            with lock:
                lm  = st['line_mode']
                buf = st['line_buf']

            if lm:
                if ch in ('\r', '\n'):
                    cmd = buf.strip()
                    with lock:
                        st['line_mode'] = False
                        st['line_buf']  = ''
                    st['need_redraw'].set()
                    if cmd:
                        _dispatch_line(port, cmd)
                elif ch == '\x1b':
                    with lock:
                        st['line_mode'] = False
                        st['line_buf']  = ''
                    set_status('cancelled')
                elif ch in ('\x7f', '\x08'):
                    with lock:
                        st['line_buf'] = st['line_buf'][:-1]
                    st['need_redraw'].set()
                else:
                    with lock:
                        st['line_buf'] += ch
                    st['need_redraw'].set()
            else:
                _dispatch_key(port, ch)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ── display ───────────────────────────────────────────────────────────────────

_GRN = '\033[32m'
_YLW = '\033[33m'
_RED = '\033[31m'
_DIM = '\033[2m'
_RST = '\033[0m'


def _swr_str(swr):
    if swr <= 0:
        return ''
    colour = _GRN if swr < 130 else (_YLW if swr < 200 else _RED)
    return f'{colour}SWR {swr / 100:.2f}{_RST}'


def _age(t):
    if t <= 0:
        return ''
    s = int(time.monotonic() - t)
    return f'{_DIM} · {s}s ago{_RST}' if s > 2 else ''


def draw(port):
    with lock:
        enc       = st['enc']
        tx        = st['tx']
        swr       = st['swr']
        status    = st['status']
        stime     = st['status_time']
        fd        = st['fd']
        line_mode = st['line_mode']
        line_buf  = st['line_buf']

    now  = time.strftime('%H:%M:%S')
    sep  = '─' * max(1, 38 - len(port))
    tx_s = f'{_RED}TX{_RST}' if tx else f'{_GRN}RX{_RST}'
    freq = f'{enc * 0.2:.3f} MHz' if enc else '—'
    bnd  = band(enc)            if enc else '—'

    line2 = f'  {bnd:5}  {freq}   {tx_s}'
    if swr:
        line2 += f'   {_swr_str(swr)}'
    if fd is None:
        line2 += f'   {_RED}no device{_RST}'

    # band row — active band highlighted, others dimmed
    active_key = next(
        (k for k, (ke, _) in BAND_KEYS.items() if abs(ke - enc) <= 2),
        None
    ) if enc else None

    band_row = '  '
    for k in '123456789':
        ke, bname = BAND_KEYS[k]
        short = bname.replace('m', '')
        if k == active_key:
            band_row += f'{_GRN}[{k}·{short}]{_RST}  '
        else:
            band_row += f'{_DIM}{k}·{short}{_RST}  '

    if line_mode:
        controls = f'cmd> {line_buf}_'
    else:
        controls = f'{_DIM}t tune   r reset   : command   h help   q quit{_RST}'

    sys.stdout.write(
        f'\033[2J\033[H'
        f' ATU-100  {port} {sep} {now}\n\n'
        f'{line2}\n\n'
        f'  {status}{_age(stime)}\n\n'
        f'{band_row}\n'
        f'  {controls}'
    )
    sys.stdout.flush()


def display_loop(port):
    while not st['quit'].is_set():
        draw(port)
        st['need_redraw'].wait(timeout=1.0)
        st['need_redraw'].clear()


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print(_HELP)
        sys.exit(0)
    port = sys.argv[1]

    try:
        fd = open_serial(port)
        with lock:
            st['fd'] = fd
    except OSError:
        with lock:
            st['retrying'] = True
        threading.Thread(target=open_serial_retry, args=(port,), daemon=True).start()

    # UDP — optional; script is fully usable without WSJT-X
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass
        sock.bind(('', WSJT_PORT))
        set_status(f'listening for WSJT-X on :{WSJT_PORT}')
    except OSError as e:
        if sock:
            sock.close()
        sock = None
        set_status(f'ready (WSJT-X UDP unavailable: {e})')

    threads = [
        (event_loop,   (port,)),
        (display_loop, (port,)),
        (input_loop,   (port,)),
    ]
    if sock is not None:
        threads.insert(0, (udp_loop, (sock, port)))

    for target, args in threads:
        threading.Thread(target=target, args=args, daemon=True).start()

    print('\033[?25l', end='', flush=True)   # hide cursor
    try:
        st['quit'].wait()
    except KeyboardInterrupt:
        pass
    finally:
        print('\033[?25h\033[2J\033[H')      # restore cursor, clear
        if sock:
            sock.close()


if __name__ == '__main__':
    main()
