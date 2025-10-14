#!/bin/bash
# Build script using open-source toolchain (apicula)

PROJECT="fpga_out"
DEVICE="GW1NR-9C"
FAMILY="GW1N-9C"

echo "==> Synthesizing with Yosys..."
yosys -p "
    read_vhdl -vhdl2008 top.vhd;
    synth_gowin -top top -json ${PROJECT}.json
" 2>&1 | tee synth.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "ERROR: Synthesis failed"
    exit 1
fi

echo "==> Place and Route with nextpnr-gowin..."
nextpnr-gowin \
    --json ${PROJECT}.json \
    --write ${PROJECT}_pnr.json \
    --device ${DEVICE} \
    --family ${FAMILY} \
    --cst tang_nano_9k.cst \
    2>&1 | tee pnr.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "ERROR: Place and route failed"
    exit 1
fi

echo "==> Generating bitstream..."
gowin_pack -d ${FAMILY} -o ${PROJECT}.fs ${PROJECT}_pnr.json

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