/***************************************************************************************************/
/**
 * @file MAX31855.h
 * @brief Arduino library for MAX31855 K-Thermocouple to Digital Converter with hardware SPI.
 *
 * - MAX31855 max voltage: 3.6v
 * - Maximum SPI bus speed: 5MHz
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

#ifndef MAX31855_h
#define MAX31855_h

#if defined(ARDUINO) && ((ARDUINO) >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#elif defined(_VARIANT_ARDUINO_STM32_) || defined(STM32)
#include <avr/pgmspace.h>
#endif

#ifndef MAX31855_SOFT_SPI
#include <SPI.h>
#endif

#define MAX31855_CONVERSION_POWER_UP_TIME 200
#define MAX31855_CONVERSION_TIME 100
#define MAX31855_THERMOCOUPLE_RESOLUTION 0.25
#define MAX31855_COLD_JUNCTION_RESOLUTION 0.0625

#define MAX31855_ID 31855
#define MAX31855_FORCE_READ_DATA 7
#define MAX31855_ERROR 2000

#define MAX31855_THERMOCOUPLE_OK 0
#define MAX31855_THERMOCOUPLE_SHORT_TO_VCC 1
#define MAX31855_THERMOCOUPLE_SHORT_TO_GND 2
#define MAX31855_THERMOCOUPLE_NOT_CONNECTED 3
#define MAX31855_THERMOCOUPLE_UNKNOWN 4
#define MAX31855_THERMOCOUPLE_READ_FAIL 5

/**
 * @class MAX31855
 * @brief Class for interfacing with MAX31855 using hardware SPI.
 */
class MAX31855
{
public:
  MAX31855(uint8_t cs);
  void begin(void);
  uint8_t detectThermocouple(int32_t rawValue = MAX31855_FORCE_READ_DATA);
  uint16_t getChipID(int32_t rawValue = MAX31855_FORCE_READ_DATA);
  double getTemperature(int32_t rawValue = MAX31855_FORCE_READ_DATA);
  double getColdJunctionTemperature(int32_t rawValue = MAX31855_FORCE_READ_DATA);
  virtual int32_t readRawData(void);

protected:
  uint8_t _cs;
};

#endif
