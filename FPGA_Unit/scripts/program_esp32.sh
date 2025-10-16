#!/bin/bash

PORT=${1:-/dev/ttyUSB0}
FIRMWARE=${2:-firmware.bin}

echo "ESP32-CAM Programming Helper"
echo "============================"
echo "Port: $PORT"
echo ""

# Test connection
echo "Testing connection..."
esptool.py --port $PORT --baud 115200 chip_id

if [ $? -eq 0 ]; then
    echo "✓ ESP32 detected!"
    
    if [ -f "$FIRMWARE" ]; then
        echo "Programming firmware: $FIRMWARE"
        esptool.py --port $PORT --baud 115200 \
            --chip esp32 \
            write_flash -z 0x1000 $FIRMWARE
    else
        echo "Firmware file not found: $FIRMWARE"
        echo "Usage: $0 [port] [firmware.bin]"
    fi
else
    echo "✗ ESP32 not detected. Check:"
    echo "  1. FPGA programmed with passthrough"
    echo "  2. ESP32 powered (5V)"
    echo "  3. GPIO0 held LOW during reset for bootloader"
fi