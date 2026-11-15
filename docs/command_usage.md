# Command Usage

Set the serial monitor to 115200 baud, with lines terminated by a newline.

Setpoints are in °C and durations in minutes. The setpoint and PID gains apply to
both control loops.

| Command            | Description                                                  | Example            |
| ------------------ | ------------------------------------------------------------ | ------------------ |
| `st=<value>`       | Sets the setpoint, °C.                                       | `st=25.5`          |
| `dt=<value>`       | Sets the process duration, minutes.                          | `dt=60`            |
| `dt+<value>`       | Increases the duration by that many minutes.                 | `dt+15`            |
| `dt-<value>`       | Decreases the duration by that many minutes.                 | `dt-10`            |
| `rt=<value>`       | Sets the remaining time, minutes.                            | `rt=30`            |
| `kp=<value>`       | Sets the proportional gain.                                  | `kp=2.5`           |
| `ki=<value>`       | Sets the integral gain.                                      | `ki=0.1`           |
| `kd=<value>`       | Sets the derivative gain.                                    | `kd=0.01`          |
| `prep`             | Sets the state to preparing.                                 | `prep`             |
| `active`           | Sets the state to active.                                    | `active`           |
| `term`             | Sets the state to terminating.                               | `term`             |
| `standby`          | Sets the state to standby.                                   | `standby`          |
| `eeprom=reset`     | Partially resets the settings stored in EEPROM.              | `eeprom=reset`     |
| `eeprom=fullreset` | Fully resets the settings stored in EEPROM.                  | `eeprom=fullreset` |

## Manual output

| Command      | Description                                                       | Example   |
| ------------ | ----------------------------------------------------------------- | --------- |
| `sudo`       | Enters the manual state, in which the PID loops stop computing.   | `sudo`    |
| `o1=<value>` | In the manual state, sets relay 1's duty, 0–255.                  | `o1=64`   |
| `o2=<value>` | In the manual state, sets relay 2's duty, 0–255.                  | `o2=64`   |

**Warning:** in the manual state nothing regulates temperature. The duty you set
drives the heaters directly and is not capped. Under PID control the output is
limited to 191 of 255. `standby` leaves the manual state and sets both outputs to 0.
