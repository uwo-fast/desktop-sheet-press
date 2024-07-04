#define _SERIAL_BAUD_ 115200 /**< Comms rate for serial, maximum before it got dicey was 115200*/

#ifdef _SERIALCMD_ || _DEVELOPMENT_
// Serial Communication Functions
void handleSerialCommands();
void printSerialData();
#endif

