#!/bin/bash
# Detect GPIO pin conflicts across projects

echo "=== Pin Conflict Analysis ==="
echo ""

# Extract all GPIO assignments into temporary file
temp_pins=$(mktemp)

# Search for pattern: #define NAME GPIO_NUM_XX
grep -rh "GPIO_NUM_[0-9]\+" . \
    --include="*.c" --include="*.h" \
    | grep -v "Binary\|build/" \
    | grep -oP "GPIO_NUM_\K[0-9]+" \
    | sort -n | uniq -c | sort -rn > "$temp_pins"

echo "Pin usage frequency (higher = potential conflict):"
cat "$temp_pins"

echo ""
echo "=== Pins Used Multiple Times ==="
awk '$1 > 1 {print "GPIO " $2 " used " $1 " times"}' "$temp_pins"

rm "$temp_pins"
