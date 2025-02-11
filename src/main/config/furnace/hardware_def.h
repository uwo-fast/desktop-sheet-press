#ifndef HARDWARE_DEF_H
#define HARDWARE_DEF_H

// ------------------------------
// Hardware Definition
// ------------------------------

// Thermocouple Pins
#define PIN_TC_DO 4
#define PIN_TC_CLK 5
#define PIN_TC_CS1 6 // CS1 is for the first thermocouple
#define PIN_TC_CS2 7 // CS2 is for the second thermocouple

// Relay Pins
#define PIN_SSR1 8 // SSR1 is for the first relay
#define PIN_SSR2 9 // SSR2 is for the second relay

// SD Card Pins
#define SD_CS 10
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK 13

// Rotary Encoder Pins
#define PIN_ENC_SW 17
#define PIN_ENC_CLK 2
#define PIN_ENC_DT 3
#define ENC_PULLUP false
#define ENC_STEPS 4
#define ENC_INTERRUP_MS 400

#endif // HARDWARE_DEF_H
