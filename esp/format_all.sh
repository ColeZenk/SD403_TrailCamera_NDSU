#!/bin/bash

echo "Formatting all ESP32 projects..."

# Format esp32cam
if [ -d "esp32cam/src" ]; then
    echo "Formatting esp32cam..."
    find esp32cam/src/ esp32cam/include/ -type f \( -name "*.c" -o -name "*.h" \) -exec clang-format -i {} \; 2>/dev/null
fi

# Format esp32s3_wroom
if [ -d "esp32s3_wroom/src" ]; then
    echo "Formatting esp32s3_wroom..."
    find esp32s3_wroom/src/ esp32s3_wroom/include/ -type f \( -name "*.c" -o -name "*.h" \) -exec clang-format -i {} \; 2>/dev/null
fi

# Format esp32_devkitv1
if [ -d "esp32_devkitv1/src" ]; then
    echo "Formatting esp32_devkitv1..."
    find esp32_devkitv1/src/ esp32_devkitv1/include/ -type f \( -name "*.c" -o -name "*.h" \) -exec clang-format -i {} \; 2>/dev/null
fi

echo "✓ All projects formatted!"
