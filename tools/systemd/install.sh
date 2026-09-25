#!/bin/bash
# Install the rover systemd user units and enable the always-on services.
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST_DIR="$HOME/.config/systemd/user"

mkdir -p "$DEST_DIR"
for unit in rover-bridge.service rover-control.service rover-bringup.service rover-nav2.service; do
    install -m 644 "$SRC_DIR/$unit" "$DEST_DIR/$unit"
    echo "Installed $DEST_DIR/$unit"
done

systemctl --user daemon-reload
systemctl --user enable rover-bridge.service rover-control.service

linger="$(loginctl show-user "$(whoami)" -p Linger)"
echo "$linger"
if [ "$linger" != "Linger=yes" ]; then
    echo "WARNING: lingering is not enabled, so user services stop at logout and do not start at boot." >&2
    echo "WARNING: enable it with: sudo loginctl enable-linger $(whoami)" >&2
fi
