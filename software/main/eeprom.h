// EEPROM macros
#define EEA_ID 0               /**< Address of unique ID */
#define EEA_PDATA (EEA_ID + 4) /**< Eeprom address of program data */
#define EE_UNIQUEID 0x18fae9c8 /**< Unique Eeprom verification ID, arbitrary */
#define EE_FULL_RESET true     /**< Reset parameter to reset all Eeprom parameters */

void updateEeprom();
