/***************************************************************************************************/
/**
 * @file MAX31855soft.cpp
 * @brief Implementation of MAX31855soft class.
 *
 * @source https://github.com/enjoyneering/MAX31855
 * @modified Cameron K Brooks
 * @license GNU GPL
 */
/***************************************************************************************************/

#include "MAX31855soft.h"

/**
 * @brief Constructor for software SPI.
 * @param cs Chip select pin.
 * @param so Serial data output pin.
 * @param sck Serial clock input pin.
 */
MAX31855soft::MAX31855soft(uint8_t cs, uint8_t so, uint8_t sck) : MAX31855(cs)
{
  _so = so;
  _sck = sck;
}

/**
 * @brief Initializes and configures software SPI.
 */
void MAX31855soft::begin(void)
{
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  pinMode(_so, INPUT);
  pinMode(_sck, OUTPUT);
  digitalWrite(_sck, LOW);

  delay(MAX31855_CONVERSION_POWER_UP_TIME);
}

/**
 * @brief Reads raw data from MAX31855 via software SPI.
 * @return Raw data as int32_t.
 */
int32_t MAX31855soft::readRawData(void)
{
  int32_t rawData = 0;

  digitalWrite(_cs, LOW);
  delayMicroseconds(1);
  digitalWrite(_cs, HIGH);
  delay(MAX31855_CONVERSION_TIME);
  digitalWrite(_cs, LOW);

#ifdef MAX31855_DISABLE_INTERRUPTS
  noInterrupts();
#endif

  for (int8_t i = 32; i > 0; i--)
  {
    digitalWrite(_sck, HIGH);
    rawData = (rawData << 1) | digitalRead(_so);
    digitalWrite(_sck, LOW);
  }

#ifdef MAX31855_DISABLE_INTERRUPTS
  interrupts();
#endif

  digitalWrite(_cs, HIGH);
  return rawData;
}
