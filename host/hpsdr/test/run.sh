#!/usr/bin/env bash
# The openHPSDR protocol 1 wire logic against synthetic frames (from UberSDR's bridge; its protocol version 4
# decoder tests went with the decoder: this bridge takes kbridge's stream instead).
#
# The protocol 1 command decode is the half that fails silently: a wrong sample-rate code or a mis-shifted register
# address does not crash, it hands the client a correctly framed picture of the wrong thing. Run under the
# sanitizers: every length comes off the wire.
set -uo pipefail

cd "$(dirname "$0")"
BUILD="${TMPDIR:-/tmp}/kiwi-hpsdr-test"
mkdir -p "$BUILD"
pass=0; fail=0

if ! gcc -std=gnu11 -Wall -Wextra -O1 -g -fsanitize=address,undefined -I.. \
        -o "$BUILD/p1_framing" p1_framing.c ../hpsdr_p1.c 2>"$BUILD/p1build.log"; then
    echo "FAIL p1-framing build"; sed 's/^/    /' "$BUILD/p1build.log" | head -20; fail=$((fail+1))
elif "$BUILD/p1_framing" >"$BUILD/p1.log" 2>&1; then
    p1pass=$(grep -c '^PASS ' "$BUILD/p1.log")
    echo "PASS p1-framing ($p1pass cases)"; pass=$((pass+1))
else
    echo "FAIL p1-framing"; grep '^FAIL ' "$BUILD/p1.log" | sed 's/^/    /' | head -10; fail=$((fail+1))
fi

echo
echo "passed $pass, failed $fail"
[ "$fail" -eq 0 ]
