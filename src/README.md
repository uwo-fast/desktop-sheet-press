# software

The main current functional software can be found in 'main' folder, see 'command_usage.md' and/or 'gui_usage' under the main 'docs' folder for operation instructions.

## Important Notes

### Safeguard against NaN

Safeguard against NaN (Not a Number) outputs from PID controllers, which can occur if the temperature sensor provides invalid readings. This occurs every once in a while with the MAX31855 thermocouple amplifier.If the PID output is NaN, perform a bumpless reset by:

1. Switching the PID controller to MANUAL mode to halt automatic control.
2. Resetting the output to 0 to ensure no erroneous signals are sent to actuators.
3. Switching back to AUTOMATIC mode to reinitialize the PID controller's internal state.

This prevents the NaN value from persisting and propagating through subsequent control cycles.
