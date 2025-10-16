# ESP32-CAM FPGA Programmer

Use Tang Nano 9K as a UART bridge to program ESP32-CAM modules.

## Quick Start

1. **Install Dependencies (Ubuntu)**
```bash
# Install OSS CAD Suite
wget https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-01-20/oss-cad-suite-linux-x64-20240120.tgz
tar -xzf oss-cad-suite-linux-x64-20240120.tgz
echo 'export PATH="$HOME/oss-cad-suite/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Install other tools
sudo apt install make iverilog gtkwave screen python3-pip
pip3 install esptool
```

2. **Build and Program FPGA**
```bash
make build
make program
```

3. **Test Connection**
```bash
# LED blink test
make test-blink

# UART loopback test
make test-loopback
```

4. **Program ESP32-CAM**
```bash
# Manually enter bootloader: Hold GPIO0 LOW while resetting
# Then run:
esptool.py --port /dev/ttyUSB0 --baud 115200 chip_id
```

## Project Structure
- `src/` - Verilog source files
- `test/` - Testbenches
- `constraints/` - Pin mapping
- `scripts/` - Build and test scripts
- `build/` - Generated files (gitignored)

## Hardware Connections

| Tang Nano 9K | ESP32-CAM | Function |
|-------------|-----------|----------|
| Pin 86      | GPIO1     | ESP32 TX |
| Pin 85      | GPIO3     | ESP32 RX |
| GND         | GND       | Ground   |
| -           | 5V        | External |

## LED Indicators
- LED0: Heartbeat (FPGA running)
- LED1: USB RX activity
- LED2: ESP32 TX activity
- LED3: Fast blink

## VS Code Integration
- Press `Ctrl+Shift+B` to build
- Use Tasks (`Ctrl+Shift+P` → "Tasks: Run Task") for other operations

## Troubleshooting

**No communication:**
- Check USB device: `ls /dev/ttyUSB*`
- Check permissions: `sudo chmod 666 /dev/ttyUSB0`

**ESP32 not detected:**
- Ensure 5V power to ESP32-CAM
- Hold GPIO0 LOW during reset for bootloader mode
- Try slower baud rate: `--baud 9600`

## License
MIT