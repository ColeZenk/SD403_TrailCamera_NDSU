cat > Makefile << 'EOF'
# Makefile for Tang Nano 9K with Open-Source Toolchain (apicula)

PROJECT = fpga_out
DEVICE = GW1NR-9C
FAMILY = GW1N-9C
TOP = top

# Source files
VHDL_SRC = top.vhd
CST_FILE = tang_nano_9k.cst

# Tools
YOSYS = yosys
NEXTPNR = nextpnr-gowin
PACK = gowin_pack
PROGRAMMER = openFPGALoader

.PHONY: all synth pnr bitstream program program-flash clean

all: bitstream

synth: $(PROJECT).json

$(PROJECT).json: $(VHDL_SRC)
	@echo "==> Running synthesis..."
	$(YOSYS) -p "read_vhdl -vhdl2008 $(VHDL_SRC); synth_gowin -top $(TOP) -json $(PROJECT).json"

pnr: $(PROJECT)_pnr.json

$(PROJECT)_pnr.json: $(PROJECT).json $(CST_FILE)
	@echo "==> Running place and route..."
	$(NEXTPNR) --json $(PROJECT).json \
		--write $(PROJECT)_pnr.json \
		--device $(DEVICE) \
		--family $(FAMILY) \
		--cst $(CST_FILE)

bitstream: $(PROJECT).fs

$(PROJECT).fs: $(PROJECT)_pnr.json
	@echo "==> Generating bitstream..."
	$(PACK) -d $(FAMILY) -o $(PROJECT).fs $(PROJECT)_pnr.json
	@echo ""
	@echo "✓ Build complete: $(PROJECT).fs"

program: bitstream
	@echo "==> Programming Tang Nano 9K (SRAM)..."
	$(PROGRAMMER) -b tangnano9k $(PROJECT).fs

program-flash: bitstream
	@echo "==> Programming Tang Nano 9K (Flash)..."
	$(PROGRAMMER) -b tangnano9k -f $(PROJECT).fs

clean:
	@echo "==> Cleaning build files..."
	rm -f $(PROJECT).json $(PROJECT)_pnr.json $(PROJECT).fs
	rm -f synth.log pnr.log

help:
	@echo "Tang Nano 9K Build System (Open-Source Toolchain)"
	@echo ""
	@echo "Targets:"
	@echo "  all            - Build bitstream (default)"
	@echo "  synth          - Run synthesis only"
	@echo "  pnr            - Run place and route only"
	@echo "  program        - Program SRAM (temporary)"
	@echo "  program-flash  - Program flash (persistent)"
	@echo "  clean          - Remove build files"
EOF