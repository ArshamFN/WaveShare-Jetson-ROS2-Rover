#!/bin/bash
# rover-net-watchdog: keep the rover reachable.
# Home Wi-Fi when it is in range; the rover hotspot when it is not.
# Leaves the hotspot for home Wi-Fi only after the hotspot has had no
# connected clients for IDLE_BEFORE_HOME seconds, so it never cuts off a
# pendant mid-run. When a reading fails or hangs, it takes no action.
set -u

IFACE="wlP1p1s0"
HOME_CON="[Home Wi-Fi name here]"
AP_CON="rover-ap"
CHECK_EVERY=15          # seconds between checks
NO_LINK_BEFORE_AP=60    # no Wi-Fi connection this long: start the hotspot
IDLE_BEFORE_HOME=300    # hotspot without clients this long: try home Wi-Fi
HOME_TRY_TIMEOUT=40     # seconds allowed for one home Wi-Fi attempt
AP_START_TIMEOUT=30     # seconds allowed for one hotspot start
READ_TIMEOUT=10         # seconds allowed for any status read

log() { echo "$*"; }

one_line() { tr '\n' ' ' | sed 's/ *$//'; }

why() {
    # Describe a failed command from its exit code and output.
    if [ "$1" -eq 124 ]; then
        echo "timed out"
    else
        printf 'exit %s: %s' "$1" "$(printf '%s' "$2" | one_line)"
    fi
}

read_link() {
    # One read of the Wi-Fi device. Sets LINK_STATE and LINK_CON.
    # Returns non-zero if NetworkManager could not be read.
    local out
    out="$(timeout "$READ_TIMEOUT" nmcli -t -f GENERAL.STATE,GENERAL.CONNECTION dev show "$IFACE" 2>/dev/null)" || return 1
    LINK_STATE="$(printf '%s\n' "$out" | sed -n 's/^GENERAL.STATE:\([0-9]*\).*/\1/p')"
    LINK_CON="$(printf '%s\n' "$out" | sed -n 's/^GENERAL.CONNECTION://p')"
    [ -n "$LINK_STATE" ]
}

station_count() {
    # Fail safe: if the client list cannot be read, report the hotspot as in
    # use, so a sensor fault can never cut off a connected pendant.
    local out rc
    out="$(timeout "$READ_TIMEOUT" iw dev "$IFACE" station dump 2>&1)"; rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "WARNING: station dump failed ($(why "$rc" "$out")), treating the hotspot as in use" >&2
        echo 1
        return
    fi
    printf '%s\n' "$out" | grep -c '^Station'
}

start_ap() {
    local out rc
    out="$(timeout $((AP_START_TIMEOUT + 10)) nmcli --wait "$AP_START_TIMEOUT" con up "$AP_CON" 2>&1)"; rc=$?
    if [ "$rc" -eq 0 ]; then
        log "hotspot started"
    else
        log "ERROR: hotspot failed to start ($(why "$rc" "$out"))"
    fi
}

try_home() {
    local out rc
    out="$(timeout $((HOME_TRY_TIMEOUT + 10)) nmcli --wait "$HOME_TRY_TIMEOUT" con up "$HOME_CON" 2>&1)"; rc=$?
    if [ "$rc" -eq 0 ]; then
        log "joined home Wi-Fi"
    else
        log "home Wi-Fi not available ($(why "$rc" "$out")), restoring hotspot"
        start_ap
    fi
}

no_link_since=""
idle_since=""
read_failed=0
log "started: iface=$IFACE home='$HOME_CON' hotspot='$AP_CON'"

while true; do
    now=$(date +%s)

    if ! read_link; then
        # Unknown state: never act on it, and leave the countdowns as they are.
        [ "$read_failed" -eq 0 ] && log "WARNING: cannot read the Wi-Fi state, taking no action until it can be read"
        read_failed=1
    else
        [ "$read_failed" -eq 1 ] && log "Wi-Fi state readable again"
        read_failed=0

        if [ "$LINK_STATE" = "100" ] && [ "$LINK_CON" = "$AP_CON" ]; then
            no_link_since=""
            if [ "$(station_count)" != "0" ]; then
                idle_since=""
            else
                [ -z "$idle_since" ] && idle_since=$now
                if [ $((now - idle_since)) -ge "$IDLE_BEFORE_HOME" ]; then
                    log "hotspot idle for ${IDLE_BEFORE_HOME}s, trying home Wi-Fi"
                    try_home
                    idle_since=""
                fi
            fi
        elif [ "$LINK_STATE" = "100" ]; then
            no_link_since=""
            idle_since=""
        else
            idle_since=""
            [ -z "$no_link_since" ] && no_link_since=$now
            if [ $((now - no_link_since)) -ge "$NO_LINK_BEFORE_AP" ]; then
                log "no Wi-Fi connection for ${NO_LINK_BEFORE_AP}s, starting hotspot"
                start_ap
                no_link_since=""
            fi
        fi
    fi
    sleep "$CHECK_EVERY"
done
