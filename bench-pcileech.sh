#!/usr/bin/env bash
# Momentary PCILeech read-throughput benchmark.
#
# Usage:
#   ./bench-pcileech.sh [label] [size ...]
#
#   label   free-form label written to the results (default: current)
#   size    one or more of: 8M 64M 256M 1G 2G  (default: 1G)
#
# Env:
#   PCILEECH_DIR   directory containing the pcileech binary and LeechCore
#                  plugins (default: ~/Code/FPGA/PciLeech/binary/v4.17.5)
#   OUTDIR         result directory (default: <repo>/bench-results)
#   TIMEOUT        per-run timeout in seconds (default: 180)
#
# The dump range is chosen from low RAM and is not written to disk
# (-out none), so the measurement is host-visible memory-read throughput
# including TLP/USB protocol overhead.

set -u

REPO_DIR=$(cd "$(dirname "$0")" && pwd)
PCILEECH_DIR=${PCILEECH_DIR:-$HOME/Code/FPGA/PciLeech/binary/v4.17.5}
OUTDIR=${OUTDIR:-$REPO_DIR/bench-results}
TIMEOUT=${TIMEOUT:-180}

LABEL=${1:-current}
shift || true
SIZES=("$@")
if [ ${#SIZES[@]} -eq 0 ]; then
    SIZES=(1G)
fi

declare -A RANGES=(
    [8M]="0x1000000 0x1800000"
    [64M]="0x1000000 0x5000000"
    [256M]="0x1000000 0x11000000"
    [1G]="0x1000000 0x41000000"
    [2G]="0x1000000 0x81000000"
)

if [ ! -x "$PCILEECH_DIR/pcileech" ]; then
    echo "ERROR: pcileech not found at $PCILEECH_DIR/pcileech" >&2
    echo "Set PCILEECH_DIR to the directory containing the pcileech binary." >&2
    exit 1
fi

mkdir -p "$OUTDIR"
STAMP=$(date +%Y%m%d-%H%M%S)
CSV="$OUTDIR/bench-${LABEL}-${STAMP}.csv"
ALL_CSV="$OUTDIR/all.csv"

TOOL_VERSION=$(
    cd "$PCILEECH_DIR" || exit 1
    ./pcileech info 2>/dev/null | sed -n 's/^ *Version: *//p' | head -1
)
[ -n "$TOOL_VERSION" ] || TOOL_VERSION=unknown

FT601_DEV=""
for d in /sys/bus/usb/devices/*; do
    if [ "$(cat "$d/idVendor" 2>/dev/null)" = "0403" ] &&
       [ "$(cat "$d/idProduct" 2>/dev/null)" = "601f" ]; then
        FT601_DEV="$d"
        break
    fi
done
LINK_SPEED=$(cat "$FT601_DEV/speed" 2>/dev/null || echo unknown)

if [ ! -f "$CSV" ]; then
    echo "timestamp,label,tool_version,link_speed,size,min,max,bytes,elapsed_s,raw_MiB_s,raw_MB_s,valid_bytes,valid_MiB_s,valid_MB_s,reported_speed,pages_read,pages_failed,success" > "$CSV"
fi
if [ ! -f "$ALL_CSV" ]; then
    echo "timestamp,label,tool_version,link_speed,size,min,max,bytes,elapsed_s,raw_MiB_s,raw_MB_s,valid_bytes,valid_MiB_s,valid_MB_s,reported_speed,pages_read,pages_failed,success" > "$ALL_CSV"
fi

echo "PCILeech benchmark"
echo "  label:        $LABEL"
echo "  pcileech:     $TOOL_VERSION ($PCILEECH_DIR)"
echo "  FT601 link:   ${LINK_SPEED} Mbps"
echo "  results:      $CSV"
echo

printf '%-6s %10s %12s %12s %12s %-12s %-10s %s\n' \
    "size" "elapsed_s" "raw_MiB/s" "raw_MB/s" "valid_MiB/s" "reported" "pages" "failed"

for SIZE in "${SIZES[@]}"; do
    if [ -z "${RANGES[$SIZE]+x}" ]; then
        echo "WARNING: unknown size '$SIZE' (valid: ${!RANGES[*]})" >&2
        continue
    fi
    read -r MIN MAX <<< "${RANGES[$SIZE]}"
    BYTES=$((MAX - MIN))
    TMP=$(mktemp)
    START=$(date +%s.%N)
    (
        cd "$PCILEECH_DIR" || exit 1
        timeout "$TIMEOUT" ./pcileech dump -min "$MIN" -max "$MAX" -out none
    ) > "$TMP" 2>&1
    RC=$?
    END=$(date +%s.%N)
    ELAPSED=$(awk -v a="$START" -v b="$END" 'BEGIN { printf "%.3f", b - a }')
    MIB_S=$(awk -v b="$BYTES" -v t="$ELAPSED" 'BEGIN { printf "%.2f", (b / 1048576) / t }')
    MB_S=$(awk -v b="$BYTES" -v t="$ELAPSED" 'BEGIN { printf "%.2f", (b / 1000000) / t }')
    REPORTED=$(grep -aoE 'Speed:[[:space:]]+[0-9]+[[:space:]]*[kMG]B/s' "$TMP" | tail -1 | sed 's/^Speed:[[:space:]]*//')
    PAGE_READ=$(grep -aoE 'Pages read:[[:space:]]+[0-9]+ / [0-9]+' "$TMP" | tail -1 | sed 's/^Pages read:[[:space:]]*//')
    PAGE_FAIL=$(grep -aoE 'Pages failed:[[:space:]]+[0-9]+' "$TMP" | tail -1 | sed 's/^Pages failed:[[:space:]]*//')
    if grep -aq 'Memory Dump: Successful.' "$TMP"; then
        SUCCESS=yes
    else
        SUCCESS=no
    fi
    [ -n "$REPORTED" ] || REPORTED="n/a"
    [ -n "$PAGE_READ" ] || PAGE_READ="n/a"
    [ -n "$PAGE_FAIL" ] || PAGE_FAIL="n/a"

    PAGE_READ_COUNT=${PAGE_READ%% *}
    if [[ "$PAGE_READ_COUNT" =~ ^[0-9]+$ ]]; then
        VALID_BYTES=$((PAGE_READ_COUNT * 4096))
    else
        VALID_BYTES=0
    fi
    VALID_MIB_S=$(awk -v b="$VALID_BYTES" -v t="$ELAPSED" 'BEGIN { printf "%.2f", (t > 0) ? (b / 1048576) / t : 0 }')
    VALID_MB_S=$(awk -v b="$VALID_BYTES" -v t="$ELAPSED" 'BEGIN { printf "%.2f", (t > 0) ? (b / 1000000) / t : 0 }')

    printf '%-6s %10s %12s %12s %12s %-12s %-10s %s\n' \
        "$SIZE" "$ELAPSED" "$MIB_S" "$MB_S" "$VALID_MIB_S" "$REPORTED" "$PAGE_READ" "$PAGE_FAIL"

    LINE="$STAMP,$LABEL,$TOOL_VERSION,$LINK_SPEED,$SIZE,$MIN,$MAX,$BYTES,$ELAPSED,$MIB_S,$MB_S,$VALID_BYTES,$VALID_MIB_S,$VALID_MB_S,$REPORTED,$PAGE_READ,$PAGE_FAIL,$SUCCESS"
    echo "$LINE" >> "$CSV"
    echo "$LINE" >> "$ALL_CSV"

    if [ "$SUCCESS" != "yes" ]; then
        echo "--- last output ---"
        tail -20 "$TMP"
        echo "-------------------"
    fi
    rm -f "$TMP"
done

echo
echo "CSV: $CSV"
echo "All runs: $ALL_CSV"
