#!/bin/sh
# Write a saved parameter file back to the flight controller.
#
#   ./params/apply-params.sh params/rc-crsf-telem1.txt
#
# The daemon must already be running: this goes over its NSH bridge, which is
# the only way to reach the board while the daemon holds the serial port.
#
# Values are written and then saved to flash. Nothing is rebooted — PX4 picks up
# RC parameters without one, and rebooting a board mid-session is a surprise
# worth not springing on someone.

set -eu

FILE="${1:-}"
NSH="${NSH:-$(dirname "$0")/../../nsh/nsh}"

EXIT_USAGE=2
EXIT_NO_FILE=3
EXIT_NO_NSH=4
EXIT_NO_DAEMON=5

die() {
    code=$1
    shift
    printf 'error: %s\n' "$*" >&2
    exit "$code"
}

[ -n "$FILE" ] || die $EXIT_USAGE "usage: $0 <param-file>"
[ -f "$FILE" ] || die $EXIT_NO_FILE "no such parameter file: $FILE"
[ -x "$NSH" ] || die $EXIT_NO_NSH "nsh not found or not executable at $NSH (override with NSH=...)"

# Fail before writing anything rather than halfway through. A board left with
# half a calibration is worse than one left alone, because it looks configured.
"$NSH" --ws --cmd "param show SYS_AUTOSTART" >/dev/null 2>&1 \
    || die $EXIT_NO_DAEMON "cannot reach the flight controller — is the daemon running?"

applied=0
while IFS= read -r line || [ -n "$line" ]; do
    # Strip trailing comments, then surrounding whitespace.
    entry=$(printf '%s' "$line" | sed 's/#.*//' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')
    [ -n "$entry" ] || continue

    name=$(printf '%s' "$entry" | awk '{print $1}')
    value=$(printf '%s' "$entry" | awk '{print $2}')
    [ -n "$value" ] || die $EXIT_USAGE "no value for parameter $name in $FILE"

    printf '  %-18s %s\n' "$name" "$value"
    "$NSH" --ws --cmd "param set $name $value" >/dev/null
    applied=$((applied + 1))
done < "$FILE"

[ "$applied" -gt 0 ] || die $EXIT_NO_FILE "no parameters found in $FILE"

"$NSH" --ws --cmd "param save" >/dev/null
printf '\nApplied and saved %d parameters from %s\n' "$applied" "$FILE"
printf 'Verify with: %s --ws --cmd "param show -c"\n' "$NSH"
