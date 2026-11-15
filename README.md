# Open Source Scientific Sheet Press

The controller for a heated sheet press that turns plastic into flat sheets and
test specimens, from the paper
[Open-source cold and hot scientific sheet press for investigating polymer-based material properties](https://doi.org/10.1016/j.ohx.2024.e00566).
The paper covers the press itself. This repository holds its control electronics,
firmware and enclosure.

## Contents

- [`src/main/`](src/main/): the firmware, a two-channel PID temperature
  controller. It reads two thermocouples through MAX31855
  amplifiers, switches two solid-state relays, shows a 16×2 LCD menu driven by a
  rotary encoder, and logs to an SD card.
- [`electrical/press_electrical_v1/`](electrical/press_electrical_v1/): the KiCad
  schematic, with [PDF](electrical/press_electrical_v1/press_electrical_v1.pdf) and
  SVG exports.
- [`docs/Press_Elec_BOM.ods`](docs/Press_Elec_BOM.ods): the electrical bill of
  materials.
- [Enclosure](docs/enclosure_design.md): renders of the electrical enclosure, and a
  link to its Onshape CAD.

## Build the firmware

The firmware targets the Arduino Nano Every (ATmega4809), using the Arduino megaAVR
boards package. Open `src/main/main.ino` in the Arduino IDE. It needs these
libraries:

- From the Library Manager: Adafruit MAX31855, PID (`PID_v1`), SdFat (version 2),
  LiquidMenu, and LiquidCrystal_I2C. For the last, use the one LiquidMenu's config
  points to, [johnrickman/LiquidCrystal_I2C](https://github.com/johnrickman/LiquidCrystal_I2C).
- ClickEncoder, from [0xPIT/encoder](https://github.com/0xPIT/encoder). It is not
  in the Library Manager.

LiquidMenu drives a parallel LCD by default. In its `LiquidMenu_config.h`, switch
it to the I2C display. Comment out both defines in the parallel `LiquidCrystal`
block (`LIQUIDMENU_LIBRARY` and `DisplayClass`), and uncomment both in the I2C
block. Pins are set in `src/main/config/hardware_def.h`.

The firmware is no longer developed.

## Use it

- [GUI](docs/gui_usage.md): the LCD menu and encoder.
- [Serial commands](docs/command_usage.md): setpoint, duration, PID gains and state,
  over USB serial.

## Licence

Hardware is under CERN-OHL-S-2.0. Firmware and documentation are under
GPL-3.0-only. These are the licences declared for the project's
[OSHWA certification CA000055](https://certification.oshwa.org/ca000055.html).
Some vendored files keep their own terms. [`LICENSING.md`](LICENSING.md) says which
licence covers which file.

## Citation

Woods, M. C., Brooks, C. K., & Pearce, J. M. (2024). Open-source cold and hot
scientific sheet press for investigating polymer-based material properties.
*HardwareX*, 19, e00566. https://doi.org/10.1016/j.ohx.2024.e00566

The press was developed by the
[FAST Research Group](https://www.appropedia.org/FAST) at Western University.
