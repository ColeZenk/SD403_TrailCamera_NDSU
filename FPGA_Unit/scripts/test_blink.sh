#!/bin/bash
set -e

echo "Building blink test..."
yosys -p "
    read_verilog src/blink_test.v;
    synth_gowin -top blink_test -json build/blink.json;
"

nextpnr-himbaechel \
    --json build/blink.json \
    --write build/blink_pnr.json \
    --device GW1NR-LV9QN88PC6/I5 \
    --vopt family=GW1N-9C \
    --vopt cst=constraints/tang_nano_9k.cst

gowin_pack -d GW1N-9C -o build/blink.fs build/blink_pnr.json

echo "Programming FPGA with blink test..."
openFPGALoader -b tangnano9k build/blink.fs

echo "✓ LEDs should be blinking now!"