#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Configuration
PROJECT_NAME="esp32_programmer"
TOP_MODULE="top"
DEVICE="GW1NR-LV9QN88PC6/I5"
FAMILY="GW1N-9C"

echo -e "${GREEN}==> ESP32-CAM FPGA Programmer Build Script${NC}"
echo "========================================"

# Check for tools
echo -e "${YELLOW}Checking tools...${NC}"
command -v yosys >/dev/null 2>&1 || { echo -e "${RED}yosys not found! Install oss-cad-suite${NC}"; exit 1; }
command -v nextpnr-himbaechel >/dev/null 2>&1 || { echo -e "${RED}nextpnr not found!${NC}"; exit 1; }
command -v gowin_pack >/dev/null 2>&1 || { echo -e "${RED}gowin_pack not found!${NC}"; exit 1; }

# Clean
echo -e "${YELLOW}Cleaning old builds...${NC}"
rm -rf build/
mkdir -p build/

# Synthesis
echo -e "${GREEN}==> Synthesizing Verilog...${NC}"
yosys -p "
    read_verilog src/top.v src/uart_passthrough.v;
    hierarchy -check -top ${TOP_MODULE};
    synth_gowin -top ${TOP_MODULE} -json build/${PROJECT_NAME}.json;
" | tee build/synth.log

# Place and Route
echo -e "${GREEN}==> Place and Route...${NC}"
nextpnr-himbaechel \
    --json build/${PROJECT_NAME}.json \
    --write build/${PROJECT_NAME}_pnr.json \
    --device ${DEVICE} \
    --vopt family=${FAMILY} \
    --vopt cst=constraints/tang_nano_9k.cst \
    2>&1 | tee build/pnr.log

# Pack bitstream
echo -e "${GREEN}==> Generating bitstream...${NC}"
gowin_pack -d GW1N-9C -o build/${PROJECT_NAME}.fs build/${PROJECT_NAME}_pnr.json

# Summary
echo ""
echo -e "${GREEN}✓ Build successful!${NC}"
echo -e "  Bitstream: ${GREEN}build/${PROJECT_NAME}.fs${NC}"
echo ""
echo "To program the FPGA:"
echo -e "  ${YELLOW}openFPGALoader -b tangnano9k build/${PROJECT_NAME}.fs${NC}"