#!/bin/bash

# Bring down can0 (keep it disabled)
ip link set can0 down 2>/dev/null || true

# Configure can1: Standard CAN at 500 kbit/s, CAN-FD off
ip link set can1 down 2>/dev/null || true
ip link set can1 type can bitrate 500000 fd off
ip link set can1 up

echo "CAN configuration complete:"
echo "  can0: DOWN"
echo "  can1: UP, 500 kbit/s, Standard CAN (FD off)"