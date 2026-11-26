#!/usr/bin/env bash
# install.sh: set up a KiwiSDR's BeagleBone (KiwiSDR 1 or 2) (Debian 11, armhf) to run kiwi-offload (kbridge + kweb) from scratch.
# Run on the Beagle as root from an unpacked release (make dist) or a checkout with the ARM artifacts in build/:
#   sudo ./tools/install.sh [--dir /home/debian/ko] [--user debian] [--no-start] [--keep-logs] [--dry-run]
# Idempotent: files are only replaced when their content differs, an existing kiwi-offload.conf is never touched, and
# a re-run with nothing to change leaves a running kiwi-offload alone. What it does:
#   1. stops and disables the stock KiwiSDR server (kiwid.service; not uninstalled: systemctl enable --now kiwid undoes it)
#   2. packages: chrony (replaces systemd-timesyncd), and to build kodma: linux-headers-$(uname -r), make, gcc
#   3. <dir>: kbridge, kweb, kweb-www/, kiwi-offload.sh, the FPGA image + eCPU code, kmod/ (kodma source), and
#      kiwi-offload.conf from config/kiwi-offload.conf.example only if there is none
#   4. PRU firmware in /lib/firmware (kiwi-offload-pru0.elf, kiwi-offload-pru1.elf, kiwi-offload-lanes0.elf)
#   5. kodma (DDR rings): built here against the running kernel, /lib/modules/<kernel>/extra/kodma.ko, loaded at boot
#      through /etc/modules-load.d/kodma.conf. After a kernel update: re-run this script (it rebuilds the module).
#   6. chrony: config/chrony-kiwi-offload.conf as /etc/chrony/conf.d/kiwi-offload.conf (GPS time from kbridge; NTP for
#      the RFC 1918 ranges: other networks go in a site file such as /etc/chrony/conf.d/local.conf, left alone here)
#   6b. logs in RAM, none on the eMMC (unless --keep-logs): the systemd journal volatile (/run, capped 16 MB:
#      /etc/systemd/journald.conf.d/kiwi-offload.conf), rsyslog disabled (it copied every line to /var/log). kbridge's and
#      kweb's own logs are in /run/kiwi-offload, capped by them (4 MB / 1 MB). Logs then go at each reboot.
#   7. systemd: kiwi-offload.target with kiwi-offload-kbridge.service (root) and kiwi-offload-kweb.service (the kweb
#      user), each restarted on failure; enabled at boot and (re)started unless --no-start. The hardware watchdog:
#      /etc/systemd/system.conf.d/kiwi-offload-watchdog.conf (RuntimeWatchdogSec=30).
set -euo pipefail
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"     # release or checkout root (same layout)

KO_DIR=/home/debian/ko
KO_USER=debian
DRY=0
START=1
KEEP_LOGS=0
usage() { sed -n '2,4p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dir) KO_DIR=${2:?--dir needs a directory}; shift ;;
        --user) KO_USER=${2:?--user needs a user name}; shift ;;
        --no-start) START=0 ;;
        --keep-logs) KEEP_LOGS=1 ;;
        -n|--dry-run) DRY=1 ;;
        -h|--help) usage 0 ;;
        *) echo "unknown option: $1" >&2; usage 2 ;;
    esac
    shift
done
KO_DIR=${KO_DIR%/}
CONF="$KO_DIR/kiwi-offload.conf"
KVER=$(uname -r)
UNIT=kiwi-offload.target
UNITS=(kiwi-offload.target kiwi-offload-kbridge.service kiwi-offload-kweb.service)

# ---- output and bookkeeping: every change goes through run/put and is listed in the summary
CHANGES=(); WARNINGS=(); NOTES=()
say()  { printf '\n== %s\n' "$*"; }
info() { printf '   %s\n' "$*"; }
act()  { CHANGES+=("$*"); printf '   %s%s\n' "$( ((DRY)) && echo 'would: ' )" "$*"; }
warn() { WARNINGS+=("$*"); printf '   WARNING: %s\n' "$*"; }
die()  { printf 'install.sh: %s\n' "$*" >&2; exit 1; }
# run a command that changes the system (only printed with --dry-run)
run()  { if ((DRY)); then printf '   [dry-run] %s\n' "$*"; else "$@"; fi; }

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

# ---- stop kiwi-offload before replacing anything it uses (only once, and only if something actually changes)
STOPPED=0
ensure_stopped() {
    ((STOPPED)) && return 0
    STOPPED=1
    if systemctl is-active --quiet "$UNIT" 2>/dev/null || systemctl is-active --quiet kiwi-offload.service 2>/dev/null; then
        act "stop $UNIT (started again at the end)"
        run systemctl stop "$UNIT" kiwi-offload-kbridge.service kiwi-offload-kweb.service
        run systemctl stop kiwi-offload.service 2>/dev/null || true
    elif pgrep -x kbridge >/dev/null || pgrep -x kweb >/dev/null; then
        act "stop kbridge/kweb started by hand (kiwi-offload.sh stop)"
        run sh "$SRC/tools/kiwi-offload.sh" stop >/dev/null
    fi
}

# put SRC DST MODE OWNER [keep-running]: install SRC as DST if the content differs (atomically: copy + rename, so a
# running binary is never written into). Sets PUT_CHANGED=1/0. Mode and owner are corrected even when unchanged.
PUT_CHANGED=0
put() {
    local src=$1 dst=$2 mode=$3 owner=$4 keep=${5:-}
    if [[ -f $dst ]] && cmp -s "$src" "$dst"; then
        PUT_CHANGED=0
        ((DRY)) || { chmod "$mode" "$dst"; chown "$owner" "$dst"; }
        return 0
    fi
    PUT_CHANGED=1
    [[ $keep == keep-running ]] || ensure_stopped
    act "$( [[ -e $dst ]] && echo update || echo install ) $dst"
    ((DRY)) && return 0
    mkdir -p "$(dirname "$dst")"
    cp "$src" "$dst.ko-new"
    chmod "$mode" "$dst.ko-new"; chown "$owner" "$dst.ko-new"
    mv -f "$dst.ko-new" "$dst"
}

# ELF machine of a file (40 = ARM, 144 = TI PRU): catches an x86 build copied over by mistake
elf_machine() { od -An -tu2 -j18 -N2 "$1" 2>/dev/null | tr -d ' '; }
pkg_installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; }

# ==== 0. checks
say "checks"
VERSION=$(cat "$SRC/VERSION" 2>/dev/null || git -C "$SRC" describe --always --dirty 2>/dev/null || echo unknown)
info "source $SRC (version $VERSION), install dir $KO_DIR, kweb user $KO_USER, kernel $KVER"
((DRY)) && info "dry run: nothing is changed"
# not root / not a Beagle is fatal, except for a dry run (to read what it would do anywhere)
chk() { if ((DRY)); then warn "$1 (ignored for --dry-run)"; else die "$1"; fi; }
[[ $EUID -eq 0 ]] || chk "must run as root (sudo $0)"
[[ $(uname -m) == armv7* ]] || chk "not an ARMv7 machine ($(uname -m)): run this on the Kiwi's BeagleBone"
MODEL=$(tr -d '\0' 2>/dev/null < /proc/device-tree/model || true)
[[ $MODEL == *BeagleBone* ]] || chk "not a BeagleBone (device-tree model '${MODEL:-?}')"
[[ $(cat /etc/debian_version 2>/dev/null) == 11* ]] || warn "not Debian 11 ($(cat /etc/debian_version 2>/dev/null || echo ?)): untested"
id "$KO_USER" >/dev/null 2>&1 || chk "user $KO_USER does not exist (--user)"
KO_GROUP=$(id -gn "$KO_USER" 2>/dev/null || echo "$KO_USER")

# everything we install, at its path in the release / checkout (make dist gathers exactly these)
BIN_KBRIDGE=build/arm/kbridge
BIN_KWEB=build/arm/kweb/kweb
PRU_FW=("build/pru/kpru.elf:kiwi-offload-pru0.elf" "build/pru/kpru1.elf:kiwi-offload-pru1.elf"
        "build/pru/klanes.elf:kiwi-offload-lanes0.elf")    # bridge/kbridge.c rproc_fw() names
BITSTREAM=firmware/stream_v25_13_lanes.bit
AOUT=upstream/bitstreams/KiwiSDR.rx8.wf3.aout
need=("$BIN_KBRIDGE" "$BIN_KWEB" "${PRU_FW[@]%%:*}" "$BITSTREAM" "$AOUT" tools/kiwi-offload.sh kweb/www/index.html
      kmod/kodma.c kmod/Makefile config/kiwi-offload.conf.example config/chrony-kiwi-offload.conf
      "${UNITS[@]/#/config/}" config/watchdog-kiwi-offload.conf config/kiwi-offload-pps.service config/kiwi-offload-pps.dts
      config/journald-kiwi-offload.conf)
missing=()
for f in "${need[@]}"; do [[ -f $SRC/$f ]] || missing+=("$f"); done
((${#missing[@]} == 0)) || die "missing in $SRC: ${missing[*]} (build with 'make dist' and copy the release over)"
for b in "$BIN_KBRIDGE" "$BIN_KWEB"; do
    [[ $(elf_machine "$SRC/$b") == 40 ]] || die "$b is not an ARM binary (cross-build it: make bridge; make -C kweb)"
done
for p in "${PRU_FW[@]%%:*}"; do
    [[ $(elf_machine "$SRC/$p") == 144 ]] || die "$p is not a PRU ELF (make pru)"
done

# what kbridge needs from the stock Kiwi image (nothing to install: warn only)
command -v config-pin >/dev/null || warn "config-pin not found: kbridge uses it for the SPI and PRU pins (bb-cape-overlays)"
grep -qs '^enable_uboot_cape_universal=1' /boot/uEnv.txt \
    || warn "/boot/uEnv.txt has no enable_uboot_cape_universal=1: config-pin needs cape-universal"
for r in 1 2; do
    [[ -e /sys/class/remoteproc/remoteproc$r ]] || warn "no /sys/class/remoteproc/remoteproc$r (PRU remoteproc): PRU mode will fail"
done

# ==== 1. the stock KiwiSDR server must not run (disabled, not removed)
say "stock kiwid"
if systemctl cat kiwid.service >/dev/null 2>&1; then
    if systemctl is-enabled --quiet kiwid 2>/dev/null || systemctl is-active --quiet kiwid 2>/dev/null; then
        act "systemctl disable --now kiwid (undo: systemctl enable --now kiwid)"
        run systemctl disable --now kiwid
        run systemctl reset-failed kiwid 2>/dev/null || true     # kiwid exits non-zero on SIGTERM: harmless
    else
        info "kiwid already disabled and stopped"
    fi
else
    info "no kiwid.service on this system"
fi

# ==== 2. packages
say "packages"
KDIR=/lib/modules/$KVER/build
pkgs=()
CHRONY_NEW=0
pkg_installed chrony || { pkgs+=(chrony); CHRONY_NEW=1; }
[[ -d $KDIR ]] || pkgs+=("linux-headers-$KVER")
command -v make >/dev/null || pkgs+=(make)
command -v gcc >/dev/null || pkgs+=(gcc)
if ((${#pkgs[@]})); then
    act "apt-get install ${pkgs[*]}$( ((CHRONY_NEW)) && echo ' (chrony replaces systemd-timesyncd)' )"
    # (on the Kiwi the bullseye-security mirror 404s for armhf: an incomplete update is not fatal)
    run apt-get update -q || warn "apt-get update reported errors (continuing with the package lists there are)"
    run env DEBIAN_FRONTEND=noninteractive apt-get install -y -q -o Dpkg::Options::=--force-confold "${pkgs[@]}" \
        || die "could not install ${pkgs[*]} (kodma needs linux-headers-$KVER for the running kernel)"
else
    info "chrony, linux-headers-$KVER, make, gcc: present"
fi

# ==== 3. the install directory
say "install directory $KO_DIR"
if [[ ! -d $KO_DIR ]]; then
    act "create $KO_DIR (owner $KO_USER)"
    run install -d -o "$KO_USER" -g "$KO_GROUP" -m 0755 "$KO_DIR"
fi
# owned by the kweb user: kweb's admin page saves kiwi-offload.conf there (write + rename), kbridge (root) writes
# gps_last_fix there
OWN="$KO_USER:$KO_GROUP"
put "$SRC/$BIN_KBRIDGE"          "$KO_DIR/kbridge"              0755 "$OWN"
put "$SRC/$BIN_KWEB"             "$KO_DIR/kweb"                 0755 "$OWN"
put "$SRC/tools/kiwi-offload.sh" "$KO_DIR/kiwi-offload.sh"      0755 "$OWN"
put "$SRC/$BITSTREAM"            "$KO_DIR/$(basename "$BITSTREAM")" 0644 "$OWN"
put "$SRC/$AOUT"                 "$KO_DIR/$(basename "$AOUT")"      0644 "$OWN"
put "$SRC/config/kiwi-offload-pps.dts" "$KO_DIR/kiwi-offload-pps.dts" 0644 "$OWN"   # (the GPS 1 PPS input's overlay)
# the web page ([kweb] www = kweb-www, relative to the install directory). Files no longer in the release are left.
while IFS= read -r -d '' f; do
    rel=${f#"$SRC/kweb/www/"}
    [[ -d $KO_DIR/kweb-www/$(dirname "$rel") ]] || run install -d -o "$KO_USER" -g "$KO_GROUP" "$KO_DIR/kweb-www/$(dirname "$rel")"
    put "$f" "$KO_DIR/kweb-www/$rel" 0644 "$OWN"
done < <(find "$SRC/kweb/www" -type f -print0)
# kodma's source, so it can be rebuilt here (step 5; also by hand: make -C $KO_DIR/kmod)
[[ -d $KO_DIR/kmod ]] || run install -d -o "$KO_USER" -g "$KO_GROUP" "$KO_DIR/kmod"
put "$SRC/kmod/kodma.c"  "$KO_DIR/kmod/kodma.c"  0644 "$OWN" keep-running
put "$SRC/kmod/Makefile" "$KO_DIR/kmod/Makefile" 0644 "$OWN" keep-running
echo "$VERSION" > "$TMP/VERSION"
put "$TMP/VERSION" "$KO_DIR/VERSION" 0644 "$OWN" keep-running
# the configuration: never overwritten (kweb's admin page and the owner edit it)
if [[ -e $CONF ]]; then
    info "kept the existing $CONF"
else
    put "$SRC/config/kiwi-offload.conf.example" "$CONF" 0640 "$OWN"
    NOTES+=("new $CONF from the example: review it ([host] slices, [pool], [kweb] passwords), then systemctl restart $UNIT")
fi

# ==== 4. PRU firmware (kbridge hands these names to remoteproc)
say "PRU firmware"
for p in "${PRU_FW[@]}"; do
    put "$SRC/${p%%:*}" "/lib/firmware/${p##*:}" 0644 root:root
done

# ==== 5. kodma: built against the running kernel, loaded at boot
say "kodma module (kernel $KVER)"
KO_INST=/lib/modules/$KVER/extra/kodma.ko
KO_MOD_CHANGED=0
if ((DRY)); then
    [[ -d $KDIR ]] || info "(kernel headers not installed yet: installed in step 2)"
    info "[dry-run] make -C $KO_DIR/kmod KDIR=$KDIR, then install as $KO_INST if the build differs"
else
    [[ -d $KDIR ]] || die "no kernel headers in $KDIR (apt-get install linux-headers-$KVER)"
    # a module left from another kernel (the directory came from an earlier install): clean, so kbuild rebuilds it
    if [[ -f $KO_DIR/kmod/kodma.ko ]] && [[ $(modinfo -F vermagic "$KO_DIR/kmod/kodma.ko" 2>/dev/null) != "$KVER "* ]]; then
        make -s -C "$KO_DIR/kmod" KDIR="$KDIR" clean
    fi
    make -s -C "$KO_DIR/kmod" KDIR="$KDIR" || die "building kodma failed (headers for $KVER: $KDIR)"
    chown -R "$OWN" "$KO_DIR/kmod"
    put "$KO_DIR/kmod/kodma.ko" "$KO_INST" 0644 root:root
    KO_MOD_CHANGED=$PUT_CHANGED
fi
if ((KO_MOD_CHANGED)) || ! modinfo -k "$KVER" kodma >/dev/null 2>&1; then
    act "depmod -a $KVER"
    run depmod -a "$KVER"
fi
printf 'kodma\n' > "$TMP/kodma.conf"
put "$TMP/kodma.conf" /etc/modules-load.d/kodma.conf 0644 root:root keep-running
# load it now; a new build replaces a loaded old one (kbridge must not hold /dev/kodma: stopped first)
if [[ -d /sys/module/kodma ]]; then
    if ((KO_MOD_CHANGED)); then
        ensure_stopped
        act "reload kodma (new build)"
        if ! { run rmmod kodma && run modprobe kodma; }; then
            warn "could not reload kodma (in use?): reboot to load the new module"
        fi
    else
        info "kodma loaded ($(cat /sys/module/kodma/parameters/cached 2>/dev/null | sed 's/^1$/cached rings/;s/^0$/UNCACHED rings/'))"
    fi
else
    act "modprobe kodma"
    run modprobe kodma || warn "modprobe kodma failed (dmesg): kbridge with ddr = on will not start"
fi

# ==== 6. chrony: GPS time from kbridge (NTP shared memory unit 0) and NTP for the local network
say "chrony"
put "$SRC/config/chrony-kiwi-offload.conf" /etc/chrony/conf.d/kiwi-offload.conf 0644 root:root keep-running
CHRONY_CONF_CHANGED=$PUT_CHANGED
if [[ -f /etc/chrony/chrony.conf ]] && ! grep -Eq '^[[:space:]]*confdir[[:space:]]+/etc/chrony/conf.d' /etc/chrony/chrony.conf; then
    warn "/etc/chrony/chrony.conf does not read /etc/chrony/conf.d (add: confdir /etc/chrony/conf.d)"
fi
if ! systemctl is-enabled --quiet chrony 2>/dev/null; then
    act "systemctl enable chrony"
    run systemctl enable chrony
fi
if ((CHRONY_CONF_CHANGED || CHRONY_NEW)) || ! systemctl is-active --quiet chrony 2>/dev/null; then
    act "systemctl restart chrony"
    run systemctl restart chrony
else
    info "chrony configured and running"
fi

# ==== 6b. logs in RAM: the journal volatile and capped, rsyslog off (the eMMC is the root filesystem: no log wear)
say "logs (RAM only)"
if ((KEEP_LOGS)); then
    info "--keep-logs: the journal and rsyslog left as they are"
else
    put "$SRC/config/journald-kiwi-offload.conf" /etc/systemd/journald.conf.d/kiwi-offload.conf 0644 root:root
    if ((PUT_CHANGED)); then
        act "systemctl restart systemd-journald (volatile: the journal in /run, capped 16 MB)"
        run systemctl restart systemd-journald
    fi
    for u in rsyslog.service syslog.socket; do
        # (enabled or running; not is-enabled --quiet, which is true for a "static" unit such as syslog.socket too: that
        # one can't be disabled, so every run reported it as a change)
        st=$(systemctl is-enabled "$u" 2>/dev/null || true)
        if [[ $st == enabled || $st == enabled-runtime ]] || systemctl is-active --quiet "$u" 2>/dev/null; then
            act "systemctl disable --now $u (it copied the journal to /var/log on the eMMC)"
            run systemctl disable --now "$u"
        fi
    done
    [[ -d /var/log/journal ]] && info "the old persistent journal is still in /var/log/journal (journalctl --vacuum-size=1 or rm -r to free it)"
fi
for f in /tmp/kwb.log /tmp/kweb.log /tmp/kbridge.epochs /tmp/kbridge.bit /tmp/kbridge.spimode /tmp/kbridge.lock; do
    if [[ -e $f ]]; then act "rm $f (kiwi-offload's runtime files are in /run/kiwi-offload now)"; run rm -f "$f"; fi
done

# ==== 7. systemd: kiwi-offload.target (kbridge + kweb services), at boot and now; the hardware watchdog
say "systemd units"
# (an earlier install had one kiwi-offload.service running both: replaced by the target and two services)
if [[ -e /etc/systemd/system/kiwi-offload.service ]]; then
    ensure_stopped
    act "remove the old kiwi-offload.service (now kiwi-offload.target + two services)"
    run systemctl disable kiwi-offload.service 2>/dev/null || true
    run rm -f /etc/systemd/system/kiwi-offload.service
fi
UNITS_CHANGED=0
for u in "${UNITS[@]}"; do
    sed -e "s|@KO_DIR@|$KO_DIR|g" -e "s|@KO_USER@|$KO_USER|g" "$SRC/config/$u" > "$TMP/$u"
    put "$TMP/$u" "/etc/systemd/system/$u" 0644 root:root
    ((PUT_CHANGED)) && UNITS_CHANGED=1
done
if ((UNITS_CHANGED)) || [[ ${CHANGES[*]:-} == *"old kiwi-offload.service"* ]]; then
    act "systemctl daemon-reload"
    run systemctl daemon-reload
fi
for u in "${UNITS[@]}"; do
    if ! systemctl is-enabled --quiet "$u" 2>/dev/null; then
        act "systemctl enable $u"
        run systemctl enable "$u"
    fi
done
put "$SRC/config/watchdog-kiwi-offload.conf" /etc/systemd/system.conf.d/kiwi-offload-watchdog.conf 0644 root:root keep-running
if ((PUT_CHANGED)); then
    act "systemctl daemon-reexec (the watchdog setting)"
    run systemctl daemon-reexec
fi
# the GPS 1 PPS input (stream gateware v22 drives P9_26): /dev/pps0 before chrony starts, and chrony's PPS refclock
# (in its own file, written only when /dev/pps0 exists: chronyd exits on a missing PPS device)
sed -e "s|@KO_DIR@|$KO_DIR|g" "$SRC/config/kiwi-offload-pps.service" > "$TMP/kiwi-offload-pps.service"
put "$TMP/kiwi-offload-pps.service" /etc/systemd/system/kiwi-offload-pps.service 0644 root:root
if ((PUT_CHANGED)); then act "systemctl daemon-reload"; run systemctl daemon-reload; fi
if ! systemctl is-enabled --quiet kiwi-offload-pps.service 2>/dev/null; then
    act "systemctl enable kiwi-offload-pps.service"; run systemctl enable kiwi-offload-pps.service
fi
if ! systemctl is-active --quiet kiwi-offload-pps.service 2>/dev/null; then
    act "systemctl start kiwi-offload-pps.service, then restart chrony (its PPS refclock)"
    run systemctl start kiwi-offload-pps.service || warn "kiwi-offload-pps.service failed: chrony uses GPS alone"
    run systemctl restart chrony 2>/dev/null || true
fi
# the configuration names the FPGA image and eCPU code: they must be in the install directory
if [[ -r $CONF ]]; then
    for k in bitstream aout; do
        v=$(sed -n '/^\[kbridge\]/,/^\[/{s/^[[:space:]]*'"$k"'[[:space:]]*=[[:space:]]*\([^;#[:space:]]*\).*/\1/p}' "$CONF" | head -1)
        [[ -z $v || -f $KO_DIR/$v || -f $v ]] || warn "$CONF: $k = $v, but there is no $KO_DIR/$v"
    done
fi
START_RESULT="not started (--no-start)"
if ((START)); then
    if systemctl is-active --quiet "$UNIT" 2>/dev/null && ! ((STOPPED)); then
        START_RESULT="running (nothing it uses changed: not restarted)"
        info "$START_RESULT"
    else
        ensure_stopped       # kbridge/kweb started by hand would hold the hardware lock
        # (a service that failed repeatedly is held by systemd's start limit until its state is reset)
        run systemctl reset-failed "${UNITS[@]}" 2>/dev/null || true
        act "systemctl start $UNIT"
        if ((DRY)); then
            START_RESULT="would be started"
        elif systemctl start "$UNIT"; then
            START_RESULT="started"
            "$KO_DIR/kiwi-offload.sh" status "$CONF" | sed 's/^/   /' || true
        else
            START_RESULT="FAILED to start (journalctl -u kiwi-offload-kbridge -u kiwi-offload-kweb; /run/kiwi-offload/kwb.log, /run/kiwi-offload/kweb.log)"
            warn "$UNIT did not start: journalctl -u kiwi-offload-kbridge -u kiwi-offload-kweb, tail /run/kiwi-offload/kwb.log /run/kiwi-offload/kweb.log"
            tail -n 5 /run/kiwi-offload/kwb.log /run/kiwi-offload/kweb.log 2>/dev/null | sed 's/^/   /' || true
        fi
    fi
elif ((STOPPED)); then
    START_RESULT="stopped for the update and NOT started again (--no-start): systemctl start $UNIT"
fi

# ==== summary
say "summary$( ((DRY)) && echo ' (dry run: nothing was changed)' )"
info "version $VERSION in $KO_DIR (kbridge as root, kweb as $KO_USER), kernel $KVER"
if ((${#CHANGES[@]})); then
    info "$( ((DRY)) && echo 'would change' || echo 'changed' ):"
    for c in "${CHANGES[@]}"; do info "  - $c"; done
else
    info "nothing to change: already installed"
fi
info "$UNIT: $START_RESULT"
for n in "${NOTES[@]}"; do info "NOTE: $n"; done
for w in "${WARNINGS[@]}"; do info "WARNING: $w"; done
port=$(sed -n '/^\[kweb\]/,/^\[/{s/^[[:space:]]*port[[:space:]]*=[[:space:]]*\([0-9]*\).*/\1/p}' "$CONF" 2>/dev/null | head -1 || true)
ip=$(hostname -I 2>/dev/null | awk '{print $1}' || true)
info "web: http://${ip:-<beagle>}:${port:-8073}/   status: systemctl status $UNIT; $KO_DIR/kiwi-offload.sh status"
info "logs: /run/kiwi-offload/kwb.log (kbridge), /run/kiwi-offload/kweb.log (kweb)   time: chronyc sources"
