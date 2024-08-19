/***************************************************************************************************/
/**
 * @file MAX31855soft.h
 * @brief Arduino library for MAX31855 K-Thermocouple to Digital Converter with software SPI.
 *
 * - MAX31855 max voltage: 3.6v
 * - K-type thermocouple accuracy: ±2°C to ±6°C
 * - Measurement range: -200°C to +700°C ±2°C, -270°C to +1372°C ±6°C
 * - Cold junction compensation range: -40°C to +125°C ±3°C
 * - Recommended: Add 10nF capacitor across T+ and T- to filter noise.
 *
 * @note Avoid placing heat sources near the converter for optimal performance.
 *
 * @source https://github.com/enjoyneering/MAX31855
 * @modified Cameron K Brooks
 *
 * @board Uno, Mini, Pro, Mega, Due, Leonardo, Blue Pill, NodeMCU, ESP32
 * @level 5v or 3.3v (varies by board)
 *
 * @license GNU GPL
 */
/***************************************************************************************************/

#ifndef MAX31855soft_h
#define MAX31855soft_h

// Uncomment to disable interrupts during bit-bang
// #define MAX31855_DISABLE_INTERRUPTS

#define MAX31855_SOFT_SPI

#include "MAX31855.h"

/**
 * @class MAX31855soft
 * @brief Class for interfacing with MAX31855 using software SPI.
 */
class MAX31855soft : public MAX31855
{
public:
  MAX31855soft(uint8_t cs, uint8_t so, uint8_t sck);
  void begin(void);
  int32_t readRawData(void);

private:
  uint8_t _so;
  uint8_t _sck;
};

#endif
