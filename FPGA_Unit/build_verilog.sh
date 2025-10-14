#!/bin/bash
PROJECT="fpga_out"

echo "==> Synthesizing Verilog with Yosys..."
yosys -p "read_verilog top.v; synth_gowin -top top -json ${PROJECT}.json" 2>&1 | tee synth.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "ERROR: Synthesis failed"
    exit 1
fi

echo "==> Place and Route with nextpnr-gowin..."
nextpnr-gowin \
    --json ${PROJECT}.json \
    --write ${PROJECT}_pnr.json \
    --freq 27 \
    --device GW1NR-LV9QN88PC6/I5 \
    --family GW1N-9C \
    --cst tang_nano_9k.cst 2>&1 | tee pnr.log

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo "ERROR: Place and route failed"
    exit 1
fi

echo "==> Generating bitstream..."
# Try the Python module directly with compatibility mode
python3 -m apycula.gowin_pack -d GW1N-9C -o ${PROJECT}.fs ${PROJECT}_pnr.json 2>&1 || \
    # Fallback: use nextpnr's built-in packer
    nextpnr-gowin --json ${PROJECT}.json --device GW1NR-LV9QN88PC6/I5 --family GW1N-9C --write ${PROJECT}_pnr.json --pack-only --bitstream ${PROJECT}.fs

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
