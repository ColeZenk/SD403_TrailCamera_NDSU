# ESP32-CAM FPGA Programmer Makefile

.PHONY: all build test clean program simulate help

# Default target
all: build

# Build main project
build:
	@./scripts/build.sh

# Test builds
test-blink:
	@./scripts/test_blink.sh

test-loopback:
	@./scripts/test_loopback.sh

# Simulate
sim simulate:
	@./scripts/simulate.sh

# Program FPGA
program: build
	openFPGALoader -b tangnano9k build/esp32_programmer.fs

# Program ESP32
esp32:
	@./scripts/program_esp32.sh

# Clean build files
clean:
	rm -rf build/
	rm -f *.vcd *.json *.fs

# Show schematic
schematic: build
	yosys -p "read_verilog src/top.v src/uart_passthrough.v; \
	          hierarchy -top top; \
	          synth_gowin; \
	          show"

# Help
help:
	@echo "ESP32-CAM FPGA Programmer - Make targets:"
	@echo ""
	@echo "  make build        - Build the main UART passthrough"
	@echo "  make program      - Build and program FPGA"
	@echo "  make test-blink   - Build and run LED blink test"
	@echo "  make test-loopback - Build and run UART loopback test"
	@echo "  make simulate     - Run Verilog simulation"
	@echo "  make schematic    - View design schematic"
	@echo "  make esp32        - Program ESP32 through FPGA"
	@echo "  make clean        - Remove build files"
	@echo ""