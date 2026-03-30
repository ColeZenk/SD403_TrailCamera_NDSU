#!/bin/bash
# Quick lookup: which files use a specific pin?

if [ -z "$1" ]; then
    echo "Usage: $0 <pin_number>"
    echo "Example: $0 13"
    exit 1
fi

PIN=$1
echo "=== Files using GPIO $PIN ==="
echo ""

grep -rn "GPIO_NUM_$PIN\b" . \
    --include="*.c" --include="*.h" \
    --color=always \
    | grep -v "Binary\|build/"
