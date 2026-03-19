# FPGA I2C GPIO Expander Module

## Summary
Implement an I2C slave GPIO expander on the Tang Nano 9K to handle stepper motor control and LCD button inputs. This frees DevKit pins and consolidates peripheral I/O on the FPGA where it belongs.

## Motivation
- Motor control is moving off the DevKit onto the FPGA for PCB consolidation
- 5 LCD control buttons need to be read locally on the FPGA anyway
- 9 GPIOs total — textbook GPIO expander use case
- I2C is already available on the DevKit (`i2c_bus.c`)

## Register Map

| Address | Name       | Direction | Bits      | Description                        |
|---------|------------|-----------|-----------|------------------------------------|
| `0x00`  | OUTPUT_REG | W         | [0] STEP  | Stepper step pulse                 |
|         |            |           | [1] DIR   | Stepper direction                  |
|         |            |           | [2] EN    | Stepper enable (active low)        |
| `0x01`  | INPUT_REG  | R         | [4:0] BTN | LCD buttons (active low, debounced)|

**I2C Address:** `0x27`

## Verilog Module Requirements
- I2C slave state machine (`i2c_slave.v`)
  - Standard mode (100 kHz) minimum, fast mode (400 kHz) preferred
  - Address decode for `0x27`
  - Register read/write logic
  - No clock stretching required
- Button debounce logic (integrate into existing or new module)
- Wire motor outputs to new PCB stepper driver pins in `top.v`
- Wire button inputs from PCB header in `top.v`
- Remove LCD button logic from `lcd_controller.v` (currently uses on-board btn)

## Integration Points
- `top.v` — add I2C pins to port list, instantiate `i2c_slave`
- `lcd_controller.v` — replace `btn` input with 5-bit button bus from expander
- `motor.c` on DevKit — replace GPIO stepper drive with I2C writes to `0x00`
- `i2c_bus.c` on DevKit — add `motor_i2c_write()` helper

## PCB Notes
- Expose SDA/SCL on PCB header, connect to DevKit I2C bus
- Pull-ups on SDA/SCL (4.7kΩ to 3.3V) on PCB
- 5 button footprints with pull-ups to 3.3V
- Stepper driver (e.g. A4988 or DRV8825) driven by FPGA STEP/DIR/EN

## Acceptance Criteria
- [ ] I2C slave responds to address `0x27`
- [ ] DevKit can write `OUTPUT_REG` and stepper moves
- [ ] DevKit can read `INPUT_REG` and see button states
- [ ] LCD controller responds to all 5 buttons
- [ ] Validated on Tang Nano 9K before PCB spin
