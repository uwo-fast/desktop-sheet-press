# software

The main current functional software can be found in 'main' folder, see 'command_usage.md' and/or 'gui_usage' under the main 'docs' folder for operation instructions.

## Overview

Dual channel PID temperature controller. Can be operated as two independent loops or cascaded.

## Important Notes

### Safeguard against NaN

Safeguard against NaN (Not a Number) outputs from PID controllers, which can occur if the temperature sensor provides invalid readings. This occurs every once in a while with the MAX31855 thermocouple amplifier.If the PID output is NaN, perform a bumpless reset by:

1. Switching the PID controller to MANUAL mode to halt automatic control.
2. Resetting the output to 0 to ensure no erroneous signals are sent to actuators.
3. Switching back to AUTOMATIC mode to reinitialize the PID controller's internal state.

This prevents the NaN value from persisting and propagating through subsequent control cycles.

### Generic Software Work

Currently the software from this project is no longer under development and future development efforts are being continued at [uwo-fast/OpenTempControl](https://github.com/uwo-fast/OpenTempControl)
