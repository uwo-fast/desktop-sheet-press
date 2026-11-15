# Licensing

This document states which licence covers which parts of this repository.
[What applies where](#what-applies-where) is the operative section. The binding terms
are the full texts in [`LICENSES/`](LICENSES/).

The licences are those the project declared for its
[OSHWA certification CA000055](https://certification.oshwa.org/ca000055.html)
(August 2024), by component:

- **Hardware:** CERN Open Hardware Licence Version 2, Strongly Reciprocal
  (`CERN-OHL-S-2.0`), [`LICENSES/CERN-OHL-S-2.0.txt`](LICENSES/CERN-OHL-S-2.0.txt)
- **Software and documentation:** GNU General Public License v3.0 only
  (`GPL-3.0-only`), [`LICENSES/GPL-3.0-only.txt`](LICENSES/GPL-3.0-only.txt)

The root [`LICENSE`](LICENSE) is a copy of the GPL text. GitHub reports only one
licence per repository and reads that file, so this document is where the full
picture lives.

## What applies where

| Path | Licence |
| --- | --- |
| `src/`, except the three third-party files below | `GPL-3.0-only` |
| `electrical/press_electrical_v1/`: the KiCad project, schematic and symbols, and the PDF and SVG exported from them | `CERN-OHL-S-2.0` |
| `docs/Press_Elec_BOM.ods`, the electrical bill of materials | `CERN-OHL-S-2.0` |
| `docs/*.PNG`, renders of the electrical enclosure | `CERN-OHL-S-2.0` |
| `README.md`, `docs/*.md`, `electrical/README.md` | `GPL-3.0-only` |
| `.gitattributes`, `.gitignore`, `LICENSING.md` | `GPL-3.0-only` |

The enclosure CAD itself is kept in Onshape (linked from
[`docs/enclosure_design.md`](docs/enclosure_design.md)), not in this repository.

## Third-party components

These keep their own terms, unaffected by this document.

| Path | Origin | Licence |
| --- | --- | --- |
| `electrical/Adafruit-MAX31855/` | [Adafruit MAX31855 breakout](https://github.com/adafruit/Adafruit-MAX31855-breakout-board) | CC BY-SA 3.0, [`license.txt`](electrical/Adafruit-MAX31855/license.txt) |
| `electrical/Adafruit-MicroSD+/` | [Adafruit MicroSD breakout](https://github.com/adafruit/MicroSD-breakout-board) | CC BY-SA 3.0, [`license.txt`](electrical/Adafruit-MicroSD+/license.txt) |
| `electrical/Nano_KiCAD-master/` | [Nano_KiCAD](https://github.com/botbotty/Nano_KiCAD) | CC BY-SA 4.0, [`LICENSE.md`](electrical/Nano_KiCAD-master/LICENSE.md) |
| `src/main/EveryTimerB.h`, `src/main/EveryTimerB.cpp`, `src/main/MegaAvr20MHz.h` | [Arduino-Nano-Every-Timer-Controller-B](https://github.com/Kees-van-der-Oord/Arduino-Nano-Every-Timer-Controller-B) by Kees van der Oord | none stated upstream |

The main schematic includes the three KiCad sheets above as hierarchical sheets, so
its exported PDF and SVG contain those pages, which remain under CC BY-SA.

The timer files carry no licence notice. Their upstream repository has no licence
file either. They are not covered by the GPL grant above.

## Contributions

Unless you state otherwise, contributions you submit are licensed under whichever
of the licences above covers the files they touch, with no additional terms. By
submitting a contribution you represent that you have the right to license it on
those terms.

## Warranty and liability

The warranty and liability terms are those of the licences: GPL-3.0 sections 15-17
and CERN-OHL-S-2.0 section 6. Both disclaim warranty and limit liability.

The press heats plates to pressing temperatures from mains power. Building or
running it is at your own risk.

## Precedence

Where this document and the full licence texts conflict, the full texts prevail.
The scope table above is not commentary. It states which files are made available
under which terms.
