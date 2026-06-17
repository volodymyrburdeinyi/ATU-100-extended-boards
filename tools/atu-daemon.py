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
  0 … 9   recall saved slot for that band (6m … 160m, key 0 = 6m)
  t       tune current band (shown in display)
  r       reset relays to zero
  ?       query ATU status
  :       enter command line (Enter to send, Esc to cancel)
  h       this help
  q       quit

command line (after pressing :):
  t [HHHH]   tune; optional freq hint in kHz hex  e.g. t 1BA2  (7074 kHz)
  l HHHH     recall nearest slot for freq in kHz  e.g. l 1BA2
  m          dump all saved band sub-slots (30 total)
  e HH       read EEPROM cell                     e.g. e 36
  c HH VV    write EEPROM cell                    e.g. c 36 02
  a          toggle auto mode
  r          reset relays
  ?          ATU status line (IND CAP SW SWR AUTO)

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
import queue
import re
import atexit

WSJT_PORT       = 2237
WSJT_MAGIC      = 0xADBCCBDA
_SERIAL_BUF_MAX = 512

BANDS = [
    (1800,  2000,  '160m'),
    (3500,  4000,   '80m'),
    (7000,  7300,   '40m'),
    (10100, 10150,  '30m'),
    (14000, 14350,  '20m'),
    (18068, 18168,  '17m'),
    (21000, 21450,  '15m'),
    (24890, 24990,  '12m'),
    (28000, 29700,  '10m'),
    (50000, 54000,   '6m'),
]

# band key → centre freq in kHz
BAND_KEYS = {
    '1': (1900,  '160m'),
    '2': (3750,   '80m'),
    '3': (7074,   '40m'),
    '4': (10136,  '30m'),
    '5': (14074,  '20m'),
    '6': (18100,  '17m'),
    '7': (21074,  '15m'),
    '8': (24915,  '12m'),
    '9': (28074,  '10m'),
    '0': (50313,   '6m'),
}


def band(khz):
    return next((n for lo, hi, n in BANDS if lo <= khz <= hi), '?')


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


# ── async reader routing ─────────────────────────────────────────────────────

_response_queue = queue.Queue()          # control-protocol lines from PIC
_DISP_RE  = re.compile(r'^\d{4}:')      # display-protocol prefix  e.g. "2016:"
_SWR_RE   = re.compile(r'SWR=(\d+\.\d+)')  # SWR inside display string
_CTRL_SWR_RE = re.compile(r'\bSWR=(\d+)\b')  # SWR in control-protocol (×100 integer)


def _fmt_ctrl_resp(resp):
    """Reformat SWR=NNN (×100) → SWR=N.NN in raw control-protocol responses."""
    def _repl(m):
        v = int(m.group(1))
        return f'SWR={v/100:.2f}' if v > 0 else 'SWR=0.00'
    return _CTRL_SWR_RE.sub(_repl, resp)


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
    """Send cmd\\r; return first control-protocol response from reader queue.
    Raises SerialError on OS write failure."""
    # Drain stale control responses that arrived between commands
    while True:
        try:
            _response_queue.get_nowait()
        except queue.Empty:
            break
    try:
        os.write(fd, (cmd + '\r').encode())
    except OSError as e:
        raise SerialError(str(e))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        try:
            return _response_queue.get(timeout=min(remaining, 0.2))
        except queue.Empty:
            continue
    return None


def valid_response(cmd, resp):
    if resp is None:
        return False
    if cmd.startswith('l'):
        return resp.startswith(('RECALL', 'NOMATCH', 'ERR'))
    if cmd.startswith('t'):
        return resp.startswith('IND=')
    return True


def _parse_display(line):
    """Extract SWR from a display-protocol line and update shared state."""
    m = _SWR_RE.search(line)
    if m:
        try:
            swr = int(round(float(m.group(1)) * 100))
            with lock:
                st['swr'] = swr
            st['need_redraw'].set()
        except ValueError:
            pass


def serial_reader(port):
    """Background thread: reads all PIC output, routes lines to display parser
    or response queue."""
    buf    = b''
    cur_fd = None
    while not st['quit'].is_set():
        with lock:
            fd = st['fd']
        if fd != cur_fd:
            buf    = b''
            cur_fd = fd
        if cur_fd is None:
            time.sleep(0.3)
            continue
        try:
            r, _, _ = select.select([cur_fd], [], [], 0.5)
            if not r:
                continue
            chunk = os.read(cur_fd, 256)
            if not chunk:
                with lock:
                    if st['fd'] == cur_fd:
                        _serial_lost(port)
                buf    = b''
                cur_fd = None
                continue
            buf += chunk
            while b'\n' in buf:
                line_b, buf = buf.split(b'\n', 1)
                line = line_b.strip().decode(errors='replace')
                if not line:
                    continue
                if _DISP_RE.match(line):
                    _parse_display(line)
                else:
                    _response_queue.put(line)
        except OSError:
            with lock:
                if st['fd'] == cur_fd:
                    _serial_lost(port)
            buf    = b''
            cur_fd = None


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
    'khz':              0,
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
    'pending_tune_khz': 0,   # set when recall hits NOMATCH during TX; tune fires on TX-end
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

def _do_band(port, khz, force_tune=False):
    """Recall or tune for khz. Caller must hold serial_sem."""
    bnd = band(khz)
    with lock:
        fd = st['fd']
    if fd is None:
        set_status('no device')
        return
    try:
        if not force_tune:
            set_status(f'recalling {bnd}...')
            resp = serial_cmd(fd, f'l {khz:04x}', timeout=3.0)
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
                    st['pending_tune_khz'] = khz
                set_status(f'no {bnd} slot · will tune after TX')
                return

        set_status(f'tuning {bnd}...')
        t0   = time.monotonic()
        resp = serial_cmd(fd, f't {khz:04x}', timeout=45.0)
        if resp is None:
            set_status(f'tune timeout after {time.monotonic()-t0:.0f}s')
            return
        if not valid_response(f't {khz:04x}', resp):
            set_status(f'unexpected ATU response: {resp!r}')
            return
        swr = _extract_swr(resp)
        with lock:
            st['swr'] = swr
        saved = 'SAVED=1' in (resp or '')
        set_status(f'tuned {bnd} · slot saved' if saved else f'tuned {bnd}')

    except SerialError as e:
        set_status(f'serial error: {e}')
        with lock:
            _serial_lost(port)


def run_band(port, khz, force_tune=False):
    """Acquire serial_sem and run _do_band. Non-blocking: skip if busy."""
    if not serial_sem.acquire(blocking=False):
        set_status('busy — try again')
        return
    try:
        _do_band(port, khz, force_tune=force_tune)
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
            set_status(_fmt_ctrl_resp(resp))
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
        khz = hz // 1000
        with lock:
            changed  = khz > 0 and khz != st['khz']
            prev_tx  = st['tx']
            if khz > 0:
                st['khz'] = khz
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
    last_khz = 0
    while not st['quit'].is_set():
        triggered = st['event'].wait(timeout=1.0)
        if triggered:
            st['event'].clear()
            with lock:
                khz     = st['khz']
                tx      = st['tx']
                pending = st['pending_tune_khz']
            # TX just ended with a pending tune — fire it
            if not tx and pending:
                with lock:
                    st['pending_tune_khz'] = 0
                threading.Thread(
                    target=run_band, args=(port, pending, True), daemon=True
                ).start()
            elif khz != last_khz:
                # Genuine band change — recall immediately (even during TX for pre-positioning)
                last_khz = khz
                threading.Thread(
                    target=run_band, args=(port, khz), daemon=True
                ).start()


# ── input ─────────────────────────────────────────────────────────────────────

def _dispatch_key(port, ch):
    if ch in BAND_KEYS:
        khz, _ = BAND_KEYS[ch]
        with lock:
            st['khz'] = khz
        threading.Thread(target=run_band, args=(port, khz), daemon=True).start()
    elif ch == 't':
        with lock:
            khz = st['khz']
        if khz:
            threading.Thread(target=run_band, args=(port, khz, True), daemon=True).start()
        else:
            set_status('no band selected — press 0-9 first')
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
        khz       = st['khz']
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
    freq = f'{khz / 1000:.3f} MHz' if khz else '—'
    bnd  = band(khz)            if khz else '—'

    line2 = f'  {bnd:5}  {freq}   {tx_s}'
    if swr:
        line2 += f'   {_swr_str(swr)}'
    if fd is None:
        line2 += f'   {_RED}no device{_RST}'

    # band row — active band highlighted, others dimmed
    active_key = next(
        (k for k, (_, bname) in BAND_KEYS.items() if bname == band(khz)),
        None
    ) if khz else None

    band_row = '  '
    for k in '123456789' + '0':
        _, bname = BAND_KEYS[k]
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

    # Terminal setup — owned here so atexit always restores it regardless of
    # which thread causes the exit (daemon threads don't run their finally blocks)
    _stdin_fd  = sys.stdin.fileno()
    _old_term  = termios.tcgetattr(_stdin_fd)
    def _restore_term():
        try:
            termios.tcsetattr(_stdin_fd, termios.TCSADRAIN, _old_term)
            sys.stdout.write('\033[?25h\033[0m')
            sys.stdout.flush()
        except Exception:
            pass
        os.system('stty sane 2>/dev/null')
    atexit.register(_restore_term)
    tty.setcbreak(_stdin_fd)

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
        (serial_reader, (port,)),
        (event_loop,    (port,)),
        (display_loop,  (port,)),
        (input_loop,    (port,)),
    ]
    if sock is not None:
        threads.insert(0, (udp_loop, (sock, port)))

    for target, args in threads:
        threading.Thread(target=target, args=args, daemon=True).start()

    sys.stdout.write('\033[?25l')   # hide cursor
    sys.stdout.flush()
    try:
        st['quit'].wait()
    except KeyboardInterrupt:
        st['quit'].set()
    finally:
        if sock:
            sock.close()


if __name__ == '__main__':
    main()
