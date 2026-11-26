#!/bin/sh
# kiwi-offload.sh: start/stop kbridge and kweb on the Beagle from the ONE configuration file.
#   sudo tools/kiwi-offload.sh start|stop|restart|status [config]      (config default: /home/debian/ko/kiwi-offload.conf)
# kbridge runs as root (hardware); kweb as the 'debian' user (it needs no privileges). Logs: /run/kiwi-offload/kwb.log, /run/kiwi-offload/kweb.log
# (kbridge writes its log from a background thread: a slow flash can't stall the stream).
CONF=${2:-/home/debian/ko/kiwi-offload.conf}
# installed by tools/install.sh: systemd runs both (and restarts them on failure), so start/stop go through it
if [ "$1" != status ] && systemctl cat kiwi-offload.target >/dev/null 2>&1; then
    case "$1" in
        start|stop|restart) systemctl "$1" kiwi-offload.target kiwi-offload-kbridge.service kiwi-offload-kweb.service || exit 1
                            sleep 2
                            for p in kbridge kweb; do printf '%-8s %s\n' "$p" "$(pgrep -x $p >/dev/null && echo running || echo stopped)"; done
                            exit 0 ;;
    esac
fi
DIR=$(dirname "$CONF")
USER_KWEB=${KWEB_USER:-debian}

stop() {
    pkill -x kweb 2>/dev/null
    pkill -x kbridge 2>/dev/null
    for i in 1 2 3 4 5 6 7 8 9 10; do pgrep -x kbridge >/dev/null || break; sleep 0.5; done
}
start() {
    [ -r "$CONF" ] || { echo "no config $CONF"; exit 1; }
    cd "$DIR" || exit 1
    install -d -m 0755 /run/kiwi-offload                                  # logs and runtime state: RAM, not the eMMC
    install -m 0644 -o "$USER_KWEB" /dev/null /run/kiwi-offload/kweb.log
    ./kbridge load --config "$CONF" | tail -1 || exit 1
    (./kbridge wb2 --config "$CONF" > /run/kiwi-offload/kwb.log 2>&1 &)
    sleep 2
    pgrep -x kbridge >/dev/null || { echo "kbridge did not start:"; tail -5 /run/kiwi-offload/kwb.log; exit 1; }
    runuser -u "$USER_KWEB" -- sh -c "cd '$DIR' && nohup ./kweb --config '$CONF' > /run/kiwi-offload/kweb.log 2>&1 &"
    sleep 1
    pgrep -x kweb >/dev/null || { echo "kweb did not start:"; tail -5 /run/kiwi-offload/kweb.log; exit 1; }
    status
}
status() {
    for p in kbridge kweb; do printf '%-8s %s\n' "$p" "$(pgrep -x $p >/dev/null && echo running || echo stopped)"; done
    grep -m1 '^config ' /run/kiwi-offload/kwb.log 2>/dev/null
    return 0    # (kbridge's log can still be empty right after a start: not a failure; systemd checks start's status)
}
case "$1" in
    start) start ;;
    stop) stop; status ;;
    restart) stop; start ;;
    status) status ;;
    *) echo "usage: $0 start|stop|restart|status [config]"; exit 2 ;;
esac
