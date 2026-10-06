#!/usr/bin/env bash
set -u

CONF="/home/robot/controllers.txt"
STATE_DIR="/run/controller-autoconnect"
RESTART_COOLDOWN_SEC=20
DISCOVERY_COOLDOWN_SEC=30
LOOP_SLEEP_SEC=2

mkdir -p "$STATE_DIR"

log() {
    echo "[$(date -Is)] $*"
}

normalize_mac() {
    echo -n "$1" | tr -d '[:space:]' | tr '[:lower:]' '[:upper:]'
}

prepare_bluetooth() {
    /usr/bin/bluetoothctl power on >/dev/null 2>&1 || true
    /usr/bin/bluetoothctl agent on >/dev/null 2>&1 || true
    /usr/bin/bluetoothctl default-agent >/dev/null 2>&1 || true
    /usr/bin/bluetoothctl pairable on >/dev/null 2>&1 || true
}

device_available() {
    /usr/bin/bluetoothctl info "$1" >/dev/null 2>&1
}

is_paired() {
    /usr/bin/bluetoothctl info "$1" 2>/dev/null | grep -q "Paired: yes"
}

is_connected() {
    /usr/bin/bluetoothctl info "$1" 2>/dev/null | grep -q "Connected: yes"
}

discover_controller() {
    local mac="$1"
    local key="${mac//:/}"
    local stamp="$STATE_DIR/discovery-$key"
    local now last

    now="$(date +%s)"
    last=0
    if [ -r "$stamp" ]; then
        last="$(cat "$stamp" 2>/dev/null || echo 0)"
    fi

    if [ $((now - last)) -lt "$DISCOVERY_COOLDOWN_SEC" ]; then
        return 0
    fi

    echo "$now" > "$stamp"
    log "controller $mac is unknown; scanning for it (put the controller in Share+PS pairing mode)"
    timeout 15 /usr/bin/bluetoothctl --timeout 12 scan on >/dev/null 2>&1 || true
}

pair_controller() {
    local mac="$1"
    local out

    is_paired "$mac" && return 0
    out="$(timeout 20 /usr/bin/bluetoothctl pair "$mac" 2>&1 || true)"
    if is_paired "$mac"; then
        log "paired controller $mac"
    else
        echo "$out" | grep -Eiq 'failed|not available|org.bluez.Error' && log "pair attempt for $mac: $out"
    fi
}

trust_controller() {
    timeout 8 /usr/bin/bluetoothctl trust "$1" >/dev/null 2>&1 || true
}

refresh_ros_joystick_readers() {
    local mac="$1"
    local key="${mac//:/}"
    local stamp="$STATE_DIR/refreshed-$key"
    local now last

    now="$(date +%s)"
    last=0
    if [ -r "$stamp" ]; then
        last="$(cat "$stamp" 2>/dev/null || echo 0)"
    fi

    if [ $((now - last)) -lt "$RESTART_COOLDOWN_SEC" ]; then
        return 0
    fi

    echo "$now" > "$stamp"
    log "controller $mac connected; refreshing Clearpath joystick ROS readers"
    /bin/systemctl restart clearpath-platform.service clearpath-joy-x-combiner.service || true
}

mark_state() {
    local mac="$1"
    local value="$2"
    echo "$value" > "$STATE_DIR/connected-${mac//:/}"
}

previous_state() {
    local mac="$1"
    local file="$STATE_DIR/connected-${mac//:/}"
    [ -r "$file" ] && cat "$file" || echo "unknown"
}

prepare_bluetooth
log "controller-autoconnect started; config=$CONF"

while true; do
    prepare_bluetooth

    if [ -r "$CONF" ]; then
        while IFS= read -r line || [ -n "$line" ]; do
            mac="${line%%#*}"
            mac="$(normalize_mac "$mac")"
            [ -z "$mac" ] && continue

            if ! device_available "$mac"; then
                mark_state "$mac" "no"
                discover_controller "$mac"
                device_available "$mac" || continue
            fi

            pair_controller "$mac"
            trust_controller "$mac"

            if is_connected "$mac"; then
                if [ "$(previous_state "$mac")" != "yes" ]; then
                    mark_state "$mac" "yes"
                    refresh_ros_joystick_readers "$mac"
                fi
                continue
            fi

            mark_state "$mac" "no"
            out="$(timeout 12 /usr/bin/bluetoothctl connect "$mac" 2>&1 || true)"
            if echo "$out" | grep -q "Connection successful" || is_connected "$mac"; then
                log "auto-connected $mac"
                mark_state "$mac" "yes"
                refresh_ros_joystick_readers "$mac"
            else
                echo "$out" | grep -Eiq 'failed|not available|not connected|org.bluez.Error' && log "connect attempt for $mac: $out"
            fi
        done < "$CONF"
    else
        log "missing controller config: $CONF"
    fi

    sleep "$LOOP_SLEEP_SEC"
done
