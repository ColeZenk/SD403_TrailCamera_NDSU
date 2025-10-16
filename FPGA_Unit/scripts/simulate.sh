#!/bin/bash
set -e

echo "Building UART loopback test..."
yosys -p "
    read_verilog src/uart_loopback.v;
    synth_gowin -top uart_loopback -json build/loopback.json;
"

nextpnr-himbaechel \
    --json build/loopback.json \
    --write build/loopback_pnr.json \
    --device GW1NR-LV9QN88PC6/I5 \
    --vopt family=GW1N-9C \
    --vopt cst=constraints/tang_nano_9k.cst

gowin_pack -d GW1N-9C -o build/loopback.fs build/loopback_pnr.json

echo "Programming FPGA..."
openFPGALoader -b tangnano9k build/loopback.fs

echo "✓ UART loopback ready! Test with:"
echo "  screen /dev/ttyUSB0 115200"
echo "  (Type text, it should echo back)"