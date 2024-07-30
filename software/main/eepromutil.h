#ifndef EEPROMUTIL_H
#define EEPROMUTIL_H

#include <Arduino.h>
#include <EEPROM.h>

// EEPROM unique ID
#define EEA_ID 0
#define EEA_PDATA (EEA_ID + 4)
#define EE_UNIQUEID 0x18fae9c8

// EEPROM reset types
#define EE_FULL_RESET true
#define EE_PARTIAL_RESET false

void resetEeprom(boolean full);
void loadEeprom();
void updateEeprom();

#endif
