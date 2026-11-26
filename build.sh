#!/usr/bin/env bash
# build.sh: build everything kiwi-offload needs, on a Linux PC (x86-64 or arm64).
#
#   ./build.sh            the Beagle release (build/kiwi-offload-<version>.tar.gz) and the HPSDR bridge for this PC
#   ./build.sh beagle     only the Beagle release
#   ./build.sh hpsdr      only the HPSDR bridge (build/host/kiwi-hpsdr-bridge_<arch>, kiwi-hpsdr-client_<arch>)
#   ./build.sh test       the host-side tests (GPS maths, kweb DSP, HPSDR framing)
#   ./build.sh clean      remove build/ (the downloaded PRU toolchain in vendor/ is kept)
#
# Not built here:
#   - the FPGA image: prebuilt in firmware/ (rebuilding needs Vivado 2024.2: tools/vivado_build.sh, see README.md)
#   - the kodma kernel module: it must match the Beagle's running kernel, so tools/install.sh builds it on the Beagle
#
# Needs: make, gcc, g++, curl, tar, xz, sha512sum and the ARM cross compilers (Debian/Ubuntu:
#   sudo apt install build-essential curl xz-utils gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf).
# The GNU PRU toolchain is downloaded into vendor/gnupru/ on first use (checksum verified), unless PRU_CC is set.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
ROOT=$PWD

GNUPRU_VER=2026.05
GNUPRU_URL=https://github.com/dinuxbg/gnupru/releases/download/$GNUPRU_VER
declare -A GNUPRU_SHA512=(
    [amd64]=391ea6f22ae6abed6d388048d19405957dcab3402ecbd25a253b15487786c20cd11fcf21d63350b198dec98530adf191b16808daeeb37fb17bd4dea84808d27b
    [aarch64]=c4a9eb75a9c7bbd16e0c391c015285dad4e35e5052413edc5dc092c76b488b44dee7d18d2a35a1a26909b406db32d9bfaad5033ff9977914b69b6eb5b2d8170d
)
CROSS=${CROSS:-arm-linux-gnueabihf-}
JOBS=${JOBS:-$(nproc)}

say()  { printf '\n==> %s\n' "$*"; }
die()  { printf 'build.sh: %s\n' "$*" >&2; exit 1; }
need() { command -v "$1" >/dev/null 2>&1 || die "$1 not found${2:+ ($2)}"; }

check_common() {
    for t in make gcc g++ tar; do need "$t" "sudo apt install build-essential"; done
}

check_beagle() {
    check_common
    need curl "sudo apt install curl"
    need "${CROSS}gcc" "sudo apt install gcc-arm-linux-gnueabihf"
    need "${CROSS}g++" "sudo apt install g++-arm-linux-gnueabihf"
}

# GNU PRU toolchain (no distribution packages it): the prebuilt release for this machine, into vendor/gnupru/
pru_toolchain() {
    if [[ -n ${PRU_CC:-} ]]; then
        [[ -x $PRU_CC ]] || die "PRU_CC=$PRU_CC is not executable"
        return
    fi
    PRU_CC=$ROOT/vendor/gnupru/pru-elf/bin/pru-gcc
    [[ -x $PRU_CC ]] && return
    local arch
    case $(uname -m) in
        x86_64) arch=amd64 ;;
        aarch64|arm64) arch=aarch64 ;;
        *) die "no prebuilt GNU PRU toolchain for $(uname -m): build it from github.com/dinuxbg/gnupru and set PRU_CC" ;;
    esac
    need xz "sudo apt install xz-utils"; need sha512sum
    local tarball=pru-elf-$GNUPRU_VER.$arch.tar.xz
    say "GNU PRU toolchain $GNUPRU_VER ($arch) -> vendor/gnupru/"
    mkdir -p vendor/gnupru
    curl -fSL --progress-bar -o "vendor/gnupru/$tarball.part" "$GNUPRU_URL/$tarball"
    echo "${GNUPRU_SHA512[$arch]}  vendor/gnupru/$tarball.part" | sha512sum -c --quiet - \
        || { rm -f "vendor/gnupru/$tarball.part"; die "checksum mismatch for $tarball"; }
    mv "vendor/gnupru/$tarball.part" "vendor/gnupru/$tarball"
    tar -xJf "vendor/gnupru/$tarball" -C vendor/gnupru
    [[ -x $PRU_CC ]] || die "unpacked $tarball but found no $PRU_CC"
}

build_beagle() {
    check_beagle
    pru_toolchain
    say "Beagle release (kbridge, kweb, PRU firmware, web decoders, FPGA image, eCPU code, kodma source)"
    make -j"$JOBS" CROSS="$CROSS" PRU_CC="$PRU_CC" bridge pru wasm
    make -j"$JOBS" -C kweb CROSS="$CROSS"
    make CROSS="$CROSS" PRU_CC="$PRU_CC" dist
}

build_hpsdr() {
    check_common
    say "HPSDR bridge for this machine"
    make -j"$JOBS" -C host/hpsdr
    mkdir -p build/host
    cp host/hpsdr/build/kiwi-hpsdr-bridge_* host/hpsdr/build/kiwi-hpsdr-client_* build/host/
    ls -1 build/host/
}

run_tests() {
    check_common
    say "tests: GPS maths"
    make kgps_test
    say "tests: kweb DSP"
    make -C kweb test
    say "tests: HPSDR protocol 1 framing"
    host/hpsdr/test/run.sh
}

what=${1:-all}
case $what in
    all)    build_beagle; build_hpsdr ;;
    beagle) build_beagle ;;
    hpsdr)  build_hpsdr ;;
    test)   run_tests ;;
    clean)  rm -rf build; make -C host/hpsdr clean >/dev/null 2>&1 || true; exit 0 ;;
    -h|--help|help) sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; exit 0 ;;
    *) die "unknown target '$what' (all | beagle | hpsdr | test | clean | help)" ;;
esac

if [[ $what == all || $what == beagle ]]; then
    tarball=$(ls -t build/kiwi-offload-*.tar.gz | head -1)
    say "done"
    echo "Beagle release: $tarball"
    echo "  copy it to the Kiwi, then: tar xzf $(basename "$tarball") && sudo ./$(basename "$tarball" .tar.gz)/tools/install.sh"
    [[ $what == all ]] && echo "HPSDR bridge:   build/host/ (see host/hpsdr/README.md)"
fi
