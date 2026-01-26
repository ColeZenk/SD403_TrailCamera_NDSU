#!/bin/bash
# Generate comprehensive pin allocation report

OUTPUT="${1:-pin_allocation.md}"

cat > "$OUTPUT" << 'REPORT'
# GPIO Pin Allocation Report

Generated: $(date)

## ESP32-CAM Pins
REPORT

echo "" >> "$OUTPUT"
echo "### Pin Definitions" >> "$OUTPUT"
grep -rh "#define.*GPIO_NUM" esp32cam/ \
    --include="*.h" --include="*.c" \
    | grep -v "Binary\|build/" \
    | sed 's/^/    /' >> "$OUTPUT"

echo "" >> "$OUTPUT"
echo "## ESP32-S3 WROOM Pins" >> "$OUTPUT"
grep -rh "#define.*GPIO_NUM" esp32s3_wroom/ \
    --include="*.h" --include="*.c" \
    | grep -v "Binary\|build/" \
    | sed 's/^/    /' >> "$OUTPUT"

echo "" >> "$OUTPUT"
echo "## ESP32 DevKitV1 Pins" >> "$OUTPUT"
grep -rh "#define.*GPIO_NUM" esp32_devkitv1/ \
    --include="*.h" --include="*.c" \
    | grep -v "Binary\|build/" \
    | sed 's/^/    /' >> "$OUTPUT"

echo "" >> "$OUTPUT"
echo "## Pin Summary Table" >> "$OUTPUT"
echo "" >> "$OUTPUT"
echo "| Pin | ESP32-CAM | ESP32-S3 | DevKitV1 | Notes |" >> "$OUTPUT"
echo "|-----|-----------|----------|----------|-------|" >> "$OUTPUT"

# Extract unique pin numbers
for pin in $(seq 0 48); do
    cam=$(grep -rh "GPIO_NUM_$pin" esp32cam/ --include="*.h" --include="*.c" 2>/dev/null | grep -v build | head -1 | cut -d' ' -f2 || echo "-")
    s3=$(grep -rh "GPIO_NUM_$pin" esp32s3_wroom/ --include="*.h" --include="*.c" 2>/dev/null | grep -v build | head -1 | cut -d' ' -f2 || echo "-")
    dk=$(grep -rh "GPIO_NUM_$pin" esp32_devkitv1/ --include="*.h" --include="*.c" 2>/dev/null | grep -v build | head -1 | cut -d' ' -f2 || echo "-")
    
    if [ "$cam" != "-" ] || [ "$s3" != "-" ] || [ "$dk" != "-" ]; then
        echo "| GPIO $pin | $cam | $s3 | $dk | |" >> "$OUTPUT"
    fi
done

echo "" >> "$OUTPUT"
echo "Generated pin allocation report: $OUTPUT"
