#!/bin/bash
# Find all GPIO pin assignments across ESP32 projects

PROJECT_ROOT="../ESP_Ecosystem/$1"

echo "=== GPIO Pin Usage Analysis ==="
echo "Searching in: $PROJECT_ROOT"
echo ""

# Find GPIO_NUM_ definitions
echo "=== GPIO_NUM_X Definitions ==="
grep -rn "GPIO_NUM_[0-9]\+" "$PROJECT_ROOT" \
    --include="*.c" --include="*.h" \
    | grep -v "Binary\|build/" \
    | sort -t: -k3 -V

echo ""
echo "=== UART Pin Assignments ==="
grep -rn "UART_\(TX\|RX\)_PIN\|PIN_NUM_\(MOSI\|MISO\|CLK\|CS\)" "$PROJECT_ROOT" \
    --include="*.c" --include="*.h" \
    | grep -v "Binary\|build/"

echo ""
echo "=== SPI Pin Assignments ==="
grep -rn "PIN_NUM_\(MOSI\|MISO\|SCLK\|CLK\|CS\)" "$PROJECT_ROOT" \
    --include="*.c" --include="*.h" \
    | grep -v "Binary\|build/"
