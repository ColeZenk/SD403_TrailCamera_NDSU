#!/bin/bash
PROJECT="fpga_out"

# Use OSS CAD Suite tools explicitly
YOSYS="$HOME/oss-cad-suite/bin/yosys"
NEXTPNR="$HOME/oss-cad-suite/bin/nextpnr-himbaechel"
PACK="$HOME/oss-cad-suite/bin/gowin_pack"

echo "==> Synthesizing Verilog with Yosys..."
$YOSYS -p "read_verilog camera_programmer.v; read_verilog top.v; synth_gowin -top top -json ${PROJECT}.json" 2>&1 | tee synth.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "ERROR: Synthesis failed"
    exit 1
fi

echo "==> Place and Route with nextpnr-himbaechel..."
$NEXTPNR --json ${PROJECT}.json \
    --write ${PROJECT}_pnr.json \
    --device GW1NR-LV9QN88PC6/I5 \
    --vopt family=GW1N-9C \
    --vopt cst=tang_nano_9k.cst 2>&1 | tee pnr.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "ERROR: Place and route failed"
    exit 1
fi

echo "==> Generating bitstream..."
$PACK -d GW1N-9C -o ${PROJECT}.fs ${PROJECT}_pnr.json

if [ $? -ne 0 ]; then
    echo "ERROR: Bitstream generation failed"
    exit 1
fi

echo ""
echo "✓ Build complete: ${PROJECT}.fs"
echo ""
echo "To program:"
echo "  SRAM (temporary):   openFPGALoader -b tangnano9k ${PROJECT}.fs"
echo "  Flash (persistent): openFPGALoader -b tangnano9k -f ${PROJECT}.fs"
