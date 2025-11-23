# Trail Camera FPGA Build System
# Tang Nano 9K (GW1NR-9C)

PROJECT = trail_camera
BOARD = tangnano9k
FAMILY = GW1N-9C
DEVICE = GW1NR-LV9QN88PC6/I5

# Directory structure
SRC_DIR = src/rtl
CONSTR_DIR = constraints
BUILD_DIR = build

# Toolchain
YOSYS = yosys
NEXTPNR = nextpnr-himbaechel
GOWIN_PACK = gowin_pack
PROGRAMMER = openFPGALoader

# Source files
TOP_MODULE = top
VERILOG_SOURCES = $(SRC_DIR)/top.v \
                  $(SRC_DIR)/esp_interface.v \
                  $(SRC_DIR)/lcd_controller.v

CONSTRAINTS = $(CONSTR_DIR)/pins.cst

# Build products (in build directory)
SYNTH_JSON = $(BUILD_DIR)/$(PROJECT).json
PNR_JSON = $(BUILD_DIR)/$(PROJECT)_pnr.json
BITSTREAM = $(BUILD_DIR)/$(PROJECT).fs

#=============================================================================
# Targets
#=============================================================================

.PHONY: all clean program flash dirs help

all: dirs $(BITSTREAM)

# Create build directory
dirs:
	@mkdir -p $(BUILD_DIR)

# Synthesis
$(SYNTH_JSON): $(VERILOG_SOURCES) | dirs
	@echo "=== Synthesis ==="
	$(YOSYS) -p "read_verilog -sv $(VERILOG_SOURCES); synth_gowin -top $(TOP_MODULE) -json $@"

# Place and Route
$(PNR_JSON): $(SYNTH_JSON) $(CONSTRAINTS)
	@echo "=== Place and Route ==="
	$(NEXTPNR) --json $< --write $@ --device $(DEVICE) --vopt family=$(FAMILY) --vopt cst=$(CONSTRAINTS)

# Pack bitstream
$(BITSTREAM): $(PNR_JSON)
	@echo "=== Packing Bitstream ==="
	$(GOWIN_PACK) -d $(FAMILY) -o $@ $<
	@echo "=== Build Complete ==="
	@ls -lh $@

# Program FPGA
program: $(BITSTREAM)
	@echo "=== Programming FPGA ==="
	$(PROGRAMMER) -b $(BOARD) $<

# Quick reprogram (skip rebuild)
flash:
	@echo "=== Flashing Existing Bitstream ==="
	$(PROGRAMMER) -b $(BOARD) $(BITSTREAM)

# Clean build artifacts
clean:
	@echo "=== Cleaning Build Directory ==="
	rm -rf $(BUILD_DIR)

# Help
help:
	@echo "Trail Camera FPGA Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all       - Build bitstream (default)"
	@echo "  program   - Build and program FPGA"
	@echo "  flash     - Reprogram without rebuild"
	@echo "  clean     - Remove build artifacts"
	@echo ""
	@echo "Quick workflow:"
	@echo "  make           - Build"
	@echo "  make program   - Build and flash"
	@echo "  make flash     - Flash without rebuild"

.PHONY: all clean program flash dirs help
