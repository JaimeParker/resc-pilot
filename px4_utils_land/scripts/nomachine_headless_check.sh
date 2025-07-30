#!/bin/bash

# Function: Check for HDMI or DP display connections using xrandr

# Usage:
# 1. Create and install the script
# sudo cp nomachine_headless_check.sh /usr/local/bin/nomachine_headless_check.sh
# sudo chmod +x /usr/local/bin/nomachine_headless_check.sh

# 2. Create and install the systemd service
# sudo cp nomachine_headless.service /etc/systemd/system/nomachine_headless.service

# 3. Reload systemd and enable the service to run at boot
# sudo systemctl daemon-reexec
# sudo systemctl daemon-reload
# sudo systemctl enable nomachine_headless.service

# Author: Zhaohong Liu

CONNECTED_OUTPUT=$(xrandr --query | grep -E 'HDMI|DP' | grep ' connected')

if [ -z "$CONNECTED_OUTPUT" ]; then
    echo "[NoMachine Headless] No HDMI/DP display connected."
    echo "[NoMachine Headless] Stopping display manager and restarting NoMachine."

    # Stop the display manager using alias (portable across systems)
    # or use lightdm, gdm, sddm, etc. directly
    systemctl stop display-manager

    # Restart NoMachine service
    /etc/NX/nxserver --restart

else
    echo "[NoMachine Headless] HDMI or DP display detected. No action needed."
    echo "$CONNECTED_OUTPUT"
fi
