#!/bin/bash
#
# Switch the USB ethernet adapter between Franka Arm (Desk UI) and Franka Controller (FCI).
#
# Usage:
#   ./switch_franka.sh desk    # Connect to Franka Desk UI (192.168.0.1)
#   ./switch_franka.sh fci     # Connect to Franka Controller FCI (172.16.0.2)
#   ./switch_franka.sh status  # Show current IP
#
# The script auto-detects the USB ethernet adapter (any enx* interface that is UP).

set -e

# --- Configuration ---
DESK_IP="192.168.0.2/24"      # Our IP when connected to Desk (arm at 192.168.0.1)
FCI_IP="172.16.0.1/24"        # Our IP when connected to FCI  (controller at 172.16.0.2)
DESK_REMOTE="192.168.0.1"     # Desk web UI
FCI_REMOTE="172.16.0.2"       # FCI controller

# --- Find the active USB ethernet adapter ---
find_usb_iface() {
    # Look for any enx* interface (USB ethernet adapters get names like enx00e04c...)
    local iface
    for iface in $(ls /sys/class/net/ | grep '^enx'); do
        # Check if it exists as a network device
        if [ -d "/sys/class/net/$iface" ]; then
            echo "$iface"
            return 0
        fi
    done
    echo ""
    return 1
}

IFACE=$(find_usb_iface)
if [ -z "$IFACE" ]; then
    echo "ERROR: No USB ethernet adapter found (no enx* interface)."
    echo "       Plug in the USB-to-Ethernet adapter and try again."
    exit 1
fi

echo "USB adapter: $IFACE"

# --- Commands ---
case "${1:-status}" in
    desk|arm)
        echo "Switching to Franka Desk (arm) mode..."
        echo "  Our IP:  $DESK_IP"
        echo "  Remote:  $DESK_REMOTE"
        sudo ip addr flush dev "$IFACE"
        sudo ip addr add "$DESK_IP" dev "$IFACE"
        sudo ip link set "$IFACE" up
        sleep 1
        echo ""
        echo "Testing connection to Desk ($DESK_REMOTE)..."
        if ping -c 2 -W 2 "$DESK_REMOTE" > /dev/null 2>&1; then
            echo "OK -- Desk is reachable. Open http://$DESK_REMOTE in browser."
        else
            echo "WARNING: Cannot reach $DESK_REMOTE. Check cable."
        fi
        ;;

    fci|controller)
        echo "Switching to Franka Controller (FCI) mode..."
        echo "  Our IP:  $FCI_IP"
        echo "  Remote:  $FCI_REMOTE"
        sudo ip addr flush dev "$IFACE"
        sudo ip addr add "$FCI_IP" dev "$IFACE"
        sudo ip link set "$IFACE" up
        sleep 1
        echo ""
        echo "Testing connection to FCI ($FCI_REMOTE)..."
        if ping -c 2 -W 2 "$FCI_REMOTE" > /dev/null 2>&1; then
            echo "OK -- FCI is reachable. Ready for robot_server."
        else
            echo "WARNING: Cannot reach $FCI_REMOTE. Check cable."
        fi
        ;;

    status|s)
        echo "Current IP on $IFACE:"
        ip -4 addr show dev "$IFACE" 2>/dev/null | grep inet || echo "  (no IP assigned)"
        echo ""
        echo "Link state:"
        ip link show dev "$IFACE" | grep -o "state [A-Z]*"
        echo ""
        echo "Connectivity:"
        if ping -c 1 -W 1 "$FCI_REMOTE" > /dev/null 2>&1; then
            echo "  FCI ($FCI_REMOTE): REACHABLE"
        else
            echo "  FCI ($FCI_REMOTE): not reachable"
        fi
        if ping -c 1 -W 1 "$DESK_REMOTE" > /dev/null 2>&1; then
            echo "  Desk ($DESK_REMOTE): REACHABLE"
        else
            echo "  Desk ($DESK_REMOTE): not reachable"
        fi
        ;;

    *)
        echo "Usage: $0 {desk|fci|status}"
        echo ""
        echo "  desk / arm        - Set IP for Franka Desk UI  (192.168.0.2 -> 192.168.0.1)"
        echo "  fci / controller  - Set IP for Franka FCI      (172.16.0.1 -> 172.16.0.2)"
        echo "  status / s        - Show current IP and connectivity"
        exit 1
        ;;
esac
