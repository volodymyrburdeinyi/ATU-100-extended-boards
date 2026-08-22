#!/bin/sh
# Host compile + link check for the PIC16F1938 firmware.
#
# XC8 is not available on every machine, and a firmware that does not link is
# indistinguishable from a firmware that was never built. This compiles every
# translation unit in the MPLAB project against stubbed SFRs (tools/hostbuild/xc.h)
# and links them, so undefined symbols across translation units — the failure mode
# that a header full of `static` globals produces — fail here instead of at
# programming time.
#
# It proves nothing about timing, peripheral behaviour or code size.
# Algorithm behaviour is covered by tools/atusim.c.
#
# Usage: tools/hostbuild/build.sh   (exit 0 = compiles and links)

set -e

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
FW="$ROOT/ATU_100_EXT_board/FirmWare_PIC16F1938/1938_EXT_MPLAB_sources_V_3.2"
OUT="${TMPDIR:-/tmp}/atu-hostbuild"

CC=${CC:-cc}
# -Dmain=fw_main: the firmware's void main() is renamed so host_stubs.c can
# supply a real int main() and the link is a genuine whole-program link.
CFLAGS="-std=gnu11 -Dmain=fw_main -I$HERE -I$FW
        -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-parameter
        -Wno-unused-function -Wno-char-subscripts"

rm -rf "$OUT"
mkdir -p "$OUT"

# Translation units listed in nbproject/configurations.xml
UNITS="main.c globals.c tune_algo.c pic_init.c cross_compiler.c uart.c relay.c swr.c uart_cmd.c"

echo "== compiling =="
for u in $UNITS; do
    [ -f "$FW/$u" ] || { echo "SKIP  $u (not present)"; continue; }
    if $CC $CFLAGS -c "$FW/$u" -o "$OUT/${u%.c}.o" 2> "$OUT/${u%.c}.log"; then
        n=$(grep -c 'warning:' "$OUT/${u%.c}.log" || true)
        echo "ok    $u  ($n warnings)"
    else
        echo "FAIL  $u"
        cat "$OUT/${u%.c}.log"
        exit 1
    fi
done

# without -Dmain=fw_main, so this file supplies the real entry point
$CC -std=gnu11 -I"$HERE" -c "$HERE/host_stubs.c" -o "$OUT/host_stubs.o"

echo "== linking =="
if $CC "$OUT"/*.o -o "$OUT/firmware" 2> "$OUT/link.log"; then
    echo "ok    linked"
else
    echo "FAIL  link"
    cat "$OUT/link.log"
    exit 1
fi

echo "== warnings =="
grep -h 'warning:' "$OUT"/*.log | sed 's/.*warning: //' | sort | uniq -c | sort -rn | head -20 || true

echo
echo "host build OK"
