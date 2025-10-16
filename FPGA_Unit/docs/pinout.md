# Tang Nano 9K to ESP32-CAM Pinout

## Critical Connections

### UART (Programming Interface)
- **Tang Pin 86** (data_bus[0]) ← **ESP32 GPIO1** (U0TXD)
- **Tang Pin 85** (data_bus[1]) → **ESP32 GPIO3** (U0RXD)

### Power
- **Tang GND** — **ESP32 GND**
- **External 5V** → **ESP32 5V** (200mA minimum)

### Control (Optional)
- **Tang Pin 71** (prog_enable) → **ESP32 GPIO0** (boot mode)

## Pin Mapping Table

| Function      | Tang Nano Pin | ESP32 Pin | Direction |
|--------------|---------------|-----------|-----------|
| UART TX      | 86            | GPIO1     | ESP→Tang  |
| UART RX      | 85            | GPIO3     | Tang→ESP  |
| Boot Control | 71            | GPIO0     | Tang→ESP  |
| Camera Enable| 81            | -         | Tang→CAM  |

## USB UART (Built-in)
- Pin 18: USB RX (from PC)
- Pin 17: USB TX (to PC)