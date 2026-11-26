#!/bin/bash
# build_opus.sh: libopus 1.4 for kweb, cross-built static for the Beagle (Cortex-A8) into build/arm/opus-fixed.
# FIXED-POINT on purpose: measured on the A8 (20 ms frames, 24 kHz mono, ~19 kbit/s, SILK/hybrid) the fixed-point build with its
# ARMv7 NEON/EDSP assembly costs 4.3 % of the core at complexity 0 (6.6 % at 3); the float build 17-23 % (the A8's
# scalar VFP is slow). kweb then uses CELT only (restricted low delay): 1.4-2.5 % per stream on the fixed-point build.
set -e
cd "$(dirname "$0")/.."
ROOT=$PWD
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"
curl -sSfL -o opus-1.4.tar.gz https://downloads.xiph.org/releases/opus/opus-1.4.tar.gz
echo "c9b32b4253be5ae63d1ff16eea06b94b5f0f2951b7a02aceef58e3a3ce49c51f  opus-1.4.tar.gz" | sha256sum -c -
tar xzf opus-1.4.tar.gz
cd opus-1.4
./configure --host=arm-linux-gnueabihf --disable-shared --enable-static --disable-extra-programs --disable-doc \
    --enable-fixed-point --prefix="$ROOT/build/arm/opus-fixed" \
    CFLAGS="-O2 -march=armv7-a -mtune=cortex-a8 -mfpu=neon -mfloat-abi=hard" > configure.log
make -j"$(nproc)" -s > /dev/null
make install -s > /dev/null
echo "libopus: $ROOT/build/arm/opus-fixed/lib/libopus.a"
