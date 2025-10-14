# Tang Nano 9K FPGA Wiring Guide
## ESP32-S3 WROOM + Tang Nano 9K + ESP32-CAM System

---

## Overview
The Tang Nano 9K FPGA acts as a UART multiplexer and parallel camera interface between the ESP32-S3 WROOM (master) and ESP32-CAM (slave).

---

## ESP32-S3 WROOM → Tang Nano 9K Connections

### UART Interface (for programming passthrough)
```
ESP32-S3 Pin    → Tang Nano 9K I/O Pin
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GPIO 17 (TX)    → WROOM_UART_RX input
GPIO 18 (RX)    → WROOM_UART_TX output
```

### Control Signals
```
ESP32-S3 Pin    → Tang Nano 9K I/O Pin    → Purpose
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GPIO 1          → CAM_IO0                  → ESP32-CAM boot control (via FPGA)
GPIO 2          → CAM_RESET                → ESP32-CAM reset control (via FPGA)
GPIO 21         → PROG_MODE_SELECT         → UART multiplexer control
                                              HIGH = Route UART to ESP32-CAM
                                              LOW  = UART disconnected
```

### SPI Interface (for future camera data transfer)
```
ESP32-S3 Pin    → Tang Nano 9K I/O Pin
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
GPIO 10 (MOSI)  → SPI_MOSI (future use)
GPIO 11 (MISO)  → SPI_MISO (future use)
GPIO 12 (SCLK)  → SPI_SCLK (future use)
GPIO 13 (CS)    → SPI_CS   (future use)
```

### Power
```
ESP32-S3 WROOM  → Tang Nano 9K
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
3.3V            → 3V3
GND             → GND
```

---

## Tang Nano 9K → ESP32-CAM Connections

### Parallel Camera Interface (8-bit)
```
Tang I/O Pin    → ESP32-CAM Pin    → GPIO
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAM_DATA0       → Pin 3            → GPIO12
CAM_DATA1       → Pin 4            → GPIO13
CAM_DATA2       → Pin 5            → GPIO15
CAM_DATA3       → Pin 6            → GPIO14
CAM_DATA4       → Pin 7            → GPIO2
CAM_DATA5       → Pin 8            → GPIO4
CAM_DATA6       → Pin 9            → GPIO16
CAM_DATA7       → Pin 10           → GPIO0
```

### Camera Control (reused after UART boot)
```
Tang I/O Pin    → ESP32-CAM Pin    → GPIO      → Purpose
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAM_PCLK        → Pin 13           → GPIO1     → Pixel clock
CAM_VSYNC       → Pin 12           → GPIO3     → Frame sync
```

### UART Interface (for programming)
```
Tang I/O Pin         → ESP32-CAM Pin    → Purpose
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAM_UART_TX          → RX               → FPGA sends to CAM
CAM_UART_RX          → TX               → FPGA receives from CAM
```

### Boot Control (passed through from ESP32-S3)
```
Tang I/O Pin    → ESP32-CAM Pin    → Purpose
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
CAM_IO0         → IO0              → Boot mode select
CAM_RESET       → EN/RST           → Reset control
```

### Power
```
Tang Nano 9K    → ESP32-CAM
━━━━━━━━━━━━━━━━━━━━━━━━━━━
GND             → GND (Pin 2)
```
**Note:** ESP32-CAM requires 5V external power supply!

---

## PC → ESP32-S3 WROOM Connection

```
PC USB          → ESP32-S3 WROOM
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
USB Type-C      → USB-C port
```
- Console UART0 on GPIO 43 (TX), GPIO 44 (RX)
- Used for menu commands and monitoring

---

## FPGA Logic Requirements

The Tang Nano 9K FPGA must implement:

### 1. UART Multiplexer
```verilog
// When PROG_MODE_SELECT = HIGH:
//   WROOM_UART_RX/TX ↔ CAM_UART_TX/RX
// When PROG_MODE_SELECT = LOW:
//   UART lines disconnected
```

### 2. GPIO Passthrough
```verilog
// Always pass through:
CAM_IO0 <= CAM_IO0_from_WROOM
CAM_RESET <= CAM_RESET_from_WROOM
```

### 3. Parallel Camera Interface (future)
- Capture 8-bit data on PCLK rising edge
- Buffer frames between VSYNC pulses
- Stream to ESP32-S3 via SPI

---

## Usage Workflow

### Programming ESP32-CAM:

1. **ESP32-S3 sends command 'p'** (press p in terminal)
   
2. **ESP32-S3 firmware does:**
   ```c
   esp32cam_enable_fpga_passthrough();  // Set GPIO21 HIGH
   esp32cam_enter_programming_mode();   // Control IO0 and RESET
   ```

3. **FPGA switches UART routing:**
   - ESP32-S3 UART1 (GPIO17/18) connects to ESP32-CAM UART
   
4. **User flashes ESP32-CAM:**
   ```bash
   cd esp32cam && idf.py -p /dev/ttyACM0 flash monitor
   ```
   - esptool talks directly to ESP32-CAM via FPGA routing
   - Console remains active for monitoring!

5. **ESP32-S3 exits programming mode:**
   ```c
   esp32cam_disable_fpga_passthrough();  // Set GPIO21 LOW
   esp32cam_reset_to_normal_mode();      // IO0 HIGH, reset
   ```

---

## Advantages of FPGA-Based Design

✅ **Hardware multiplexing** - No software driver conflicts  
✅ **Console stays active** - Can monitor during programming  
✅ **Full speed** - No buffering delays  
✅ **Clean separation** - Each component has dedicated function  
✅ **Expandable** - Easy to add SPI camera data transfer  
✅ **Reliable** - Hardware switching is deterministic  

---

## Pin Summary Table

| Function           | ESP32-S3 Pin | Tang Pin Name      | ESP32-CAM Pin |
|--------------------|--------------|--------------------|---------------|
| Console UART TX    | GPIO 43      | -                  | -             |
| Console UART RX    | GPIO 44      | -                  | -             |
| UART1 TX to FPGA   | GPIO 17      | WROOM_UART_RX      | -             |
| UART1 RX from FPGA | GPIO 18      | WROOM_UART_TX      | -             |
| CAM IO0 Control    | GPIO 1       | CAM_IO0            | IO0           |
| CAM Reset Control  | GPIO 2       | CAM_RESET          | EN/RST        |
| FPGA Mode Select   | GPIO 21      | PROG_MODE_SELECT   | -             |
| SPI MOSI           | GPIO 10      | SPI_MOSI           | -             |
| SPI MISO           | GPIO 11      | SPI_MISO           | -             |
| SPI SCLK           | GPIO 12      | SPI_SCLK           | -             |
| SPI CS             | GPIO 13      | SPI_CS             | -             |
| -                  | -            | CAM_DATA[7:0]      | GPIO0,16,4... |
| -                  | -            | CAM_PCLK           | GPIO1         |
| -                  | -            | CAM_VSYNC          | GPIO3         |
| -                  | -            | CAM_UART_TX        | RX            |
| -                  | -            | CAM_UART_RX        | TX            |

---

## Next Steps

1. **Wire up the connections** as shown above
2. **Flash ESP32-S3** with the updated firmware:
   ```bash
   cd esp32s3_wroom && idf.py -p /dev/ttyACM0 flash monitor
   ```
3. **Program FPGA** with UART multiplexer logic (Verilog code needed)
4. **Test programming mode** by pressing 'p' in monitor
5. **Flash ESP32-CAM** while monitoring ESP32-S3 console

The ESP32-S3 firmware is now ready. The FPGA just needs the multiplexer logic!
