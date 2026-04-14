#!/bin/bash

echo "Formatting all ESP32 projects..."

# Format esp32cam/main
if [ -d "esp32cam/main/src" ]; then
    echo "Formatting esp32cam/main..."
    find esp32cam/main/src/ esp32cam/main/include/ -type f \( -name "*.c" -o -name "*.h" \) -exec clang-format -i {} \; 2>/dev/null
fi

# Format esp32s3_wroom/main
if [ -d "esp32s3_wroom/main/src" ]; then
    echo "Formatting esp32s3_wroom/main..."
    find esp32s3_wroom/main/src/ esp32s3_wroom/main/include/ -type f \( -name "*.c" -o -name "*.h" \) -exec clang-format -i {} \; 2>/dev/null
fi

# Format esp32_devkitv1/main
if [ -d "esp32_devkitv1/main/src" ]; then
    echo "Formatting esp32_devkitv1/main..."
    find esp32_devkitv1/main/src/ esp32_devkitv1/main/include/ -type f \( -name "*.c" -o -name "*.h" \) -exec clang-format -i {} \; 2>/dev/null
fi

echo "✓ All projects formatted!"
