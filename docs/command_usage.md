# Command Usage

Make sure that your serial monitor is set to baud rate '115200' and termination to 'New Line'.

| Command             | Description                                                                  | Example                |
|---------------------|------------------------------------------------------------------------------|------------------------|
| `st=<value>`        | Sets a new setpoint for all sensors.                                         | `st=25.5`              |
| `dt=<value>`        | Sets a new duration for the process.                                         | `dt=60`                |
| `dt+<value>`        | Increases the duration by the specified value.                               | `dt+15`                |
| `dt-<value>`        | Decreases the duration by the specified value.                               | `dt-10`                |
| `rt=<value>`        | Sets a new remaining duration for the process.                               | `rt=30`                |
| `kp=<value>`        | Sets a new proportional gain (Kp) for all sensors.                           | `kp=2.5`               |
| `ki=<value>`        | Sets a new integral gain (Ki) for all sensors.                               | `ki=0.1`               |
| `kd=<value>`        | Sets a new derivative gain (Kd) for all sensors.                             | `kd=0.01`              |
| `prep`              | Sets the machine state to preparing.                                         | `prep`                 |
| `active`            | Sets the machine state to active.                                            | `active`               |
| `term`              | Sets the machine state to terminating.                                       | `term`                 |
| `standby`           | Sets the machine state to standby.                                           | `standby`              |
| `eeprom=reset`      | Resets the EEPROM with a partial reset.                                      | `eeprom=reset`         |
| `eeprom=fullreset`  | Resets the EEPROM with a full reset.                                         | `eeprom=fullreset`     |