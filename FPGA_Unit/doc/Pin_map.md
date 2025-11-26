## Tang Nano 9K Pin Map Reference
## Official pinout for GW1NR-9C FPGA board

### LEFT SIDE PINOUT
                                                                                            
| breadboard_L | number_L | IO        | onboard     | special pin  | LEFT ALOCATION(s)    |
|--------------|----------|-----------|-------------|--------------|----------------------|
| 24           | 38       | IOB31B    |             |              |                      |
| 23           | 37       | IOB31A    |             |              |                      |
| 22           | 36       | IOB29B    |             | GCLKC_4      |                      |
| 21           | 39       | IOB33A    |             |              |                      |
| 20           | 25       | IOB8A     |             |              |                      |
| 19           | 26       | IOB8B     |             |              |                      |
| 18           | 27       | IOB11A    |             |              |                      |
| 17           | 28       | IOB11B    |             |              |                      |
| 16           | 29       | IOB13A    |             |              |                      |
| 15           | 30       | IOB13B    |             |              |                      |
| 14           | 33       | IOB23A    | RGB_DE      |              |                      |
| 13           | 34       | IOB23B    | RGB_VS      |              |                      |
| 12           | 40       | IOB33B    | RGB_HS      |              |                      |
| 11           | 35       | IOB29A    | RGB_CK      | GCLKT_4      |                      |
| 10           | 41       | IOB41A    | RGB_B7      |              |                      |
| 9            | 42       | IOB41B    | RGB_B6      |              |                      |
| 8            | 51       | IOR17B    | RGB_B5      | GCLKC_3      |                      |
| 7            | 53       | IOR15B    | RGB_B4      |              |                      |
| 6            | 54       | IOR15A    | RGB_B3      |              |                      |
| 5            | 55       | IOR14B    | RGB_G7      |              |                      |
| 4            | 56       | IOR14A    | RGB_G6      |              |                      |
| 3            | 57       | IOR13A    | RGB_G5      |              |                      |
| 2            | 68       | IOT42B    | RGB_G4      | HDMI_CKN     |                      |
| 1            | 69       | IOT42A    | RGB_G3      | HDMI_CKP     |                      |

### RIGHT SIDE PINOUT
                                                                                            
| breadboard_R | number_R | IO        | onboard     | special pin  |  RIGHT ALOCATION(s)  |
|--------------|----------|-----------|-------------|--------------|----------------------|
| 24           | 63       | IOR5A     | RGB_INIT    |              |                      |
| 23           | 86       | IOT8A     | RGB_BL      |              |                      |
| 22           | 85       | IOT8B     |             |              |                      |
| 21           | 84       | IOT10A    |             |              |                      |
| 20           | 83       | IOT10B    |             |              |                      |
| 19           | 82       | IOT11A    |             |              |                      |
| 18           | 81       | IOT11B    |             |              |                      |
| 17           | 80       | IOT12A    |             |              |                      |
| 16           | 79       | IOT12B    |             |              |                      |
| 15           | 77       | IOT37A    | SPILCD_Mo   |              |                      |
| 14           | 76       | IOT37B    | SPILCD_Ck   |              |                      |
| 13           | 75       | IOT38A    | RGB_R3      | HDMI_D2P     |                      |
| 12           | 74       | IOT38B    | RGB_R4      | HDMI_D2N     |                      |
| 11           | 73       | IOT39A    | RGB_R5      | HDMI_D1P     |                      |
| 10           | 72       | IOT39B    | RGB_R6      | HDMI_D1N     |                      |
| 9            | 71       | IOT41A    | RGB_R7      | HDMI_D0P     |                      |
| 8            | 70       | IOT41B    | RGB_G2      | HDMI_D0N     |                      |
| 7            | 5V       |           |             |              |                      |
| 6            | 48       | IOT24B    | SPILCD_CS   |              |                      |
| 5            | 49       | IOT24A    | SPILCD_RS   |              |                      |
| 4            | 31       | IOT15A    | RGB_INIT    |              |                      |
| 3            | 32       | IOT15B    | RGB_INIT    |              |                      |
| 2            | GND      |           |             |              |                      |
| 1            | 3V3      |           |             |              |                      |

### KEY GROUPS

**LCD RGB Interface:**
- DE: Pin 33 (IOB23A)
- VSYNC: Pin 34 (IOB23B)
- HSYNC: Pin 40 (IOB33B)
- CLK: Pin 35 (IOB29A)
- RGB data pins: 41, 42, 51, 53-57, 68-77

**SPI LCD Interface:**
- CS: Pin 48 (IOT24B)
- RS: Pin 49 (IOT24A)
- MOSI: Pin 77 (IOT37A)
- CLK: Pin 76 (IOT37B)

**Available GPIO (Right side):**
- Pins 80-86 (IOT8A through IOT12B)

**Power:**
- 5V: Right side pin 7
- 3.3V: Right side pin 1
- GND: Right side pin 2

**Special Clocks:**
- GCLKC_4: Pin 36
- GCLKT_4: Pin 35
- GCLKC_3: Pin 51
