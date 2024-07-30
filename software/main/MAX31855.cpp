/***************************************************************************************************/
/**
 * @file MAX31855.cpp
 * @brief Implementation of MAX31855 class.
 *
 * @source https://github.com/enjoyneering/MAX31855
 * @modified Cameron K Brooks
 * @license GNU GPL
 */
/***************************************************************************************************/

#include "MAX31855.h"

/**
 * @brief Constructor for hardware SPI.
 * @param cs Chip select pin.
 */
MAX31855::MAX31855(uint8_t cs)
{
  _cs = cs;
}

/**
 * @brief Initializes and configures hardware SPI.
 */
void MAX31855::begin(void)
{
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  SPI.begin();
  delay(MAX31855_CONVERSION_POWER_UP_TIME);
}

/**
 * @brief Checks if thermocouple is open, shorted to GND, or shorted to VCC.
 * @param rawValue Raw data value.
 * @return Thermocouple status.
 */
uint8_t MAX31855::detectThermocouple(int32_t rawValue)
{
  if (rawValue == MAX31855_FORCE_READ_DATA)
    rawValue = readRawData();

  if (rawValue == 0)
    return MAX31855_THERMOCOUPLE_READ_FAIL;

  if (bitRead(rawValue, 16) == 1)
  {
    if (bitRead(rawValue, 2) == 1)
      return MAX31855_THERMOCOUPLE_SHORT_TO_VCC;
    else if (bitRead(rawValue, 1) == 1)
      return MAX31855_THERMOCOUPLE_SHORT_TO_GND;
    else if (bitRead(rawValue, 0) == 1)
      return MAX31855_THERMOCOUPLE_NOT_CONNECTED;
    else
      return MAX31855_THERMOCOUPLE_UNKNOWN;
  }
  return MAX31855_THERMOCOUPLE_OK;
}

/**
 * @brief Checks chip ID.
 * @param rawValue Raw data value.
 * @return Chip ID.
 */
uint16_t MAX31855::getChipID(int32_t rawValue)
{
  if (rawValue == MAX31855_FORCE_READ_DATA)
    rawValue = readRawData();

  if (rawValue == 0)
    return MAX31855_THERMOCOUPLE_READ_FAIL;
  if (bitRead(rawValue, 17) == 0 && bitRead(rawValue, 3) == 0)
    return MAX31855_ID;

  return MAX31855_ERROR;
}

/**
 * @brief Reads temperature in Celsius.
 * @param rawValue Raw data value.
 * @return Temperature in Celsius.
 */
float MAX31855::getTemperature(int32_t rawValue)
{
  if (rawValue == MAX31855_FORCE_READ_DATA)
    rawValue = readRawData();

  if (detectThermocouple(rawValue) != MAX31855_THERMOCOUPLE_OK)
    return MAX31855_ERROR;

  rawValue = rawValue >> 18;

  return (float)rawValue * MAX31855_THERMOCOUPLE_RESOLUTION;
}

/**
 * @brief Reads cold junction temperature in Celsius.
 * @param rawValue Raw data value.
 * @return Cold junction temperature in Celsius.
 */
float MAX31855::getColdJunctionTemperature(int32_t rawValue)
{
  if (rawValue == MAX31855_FORCE_READ_DATA)
    rawValue = readRawData();

  if (getChipID(rawValue) != MAX31855_ID)
    return MAX31855_ERROR;

  rawValue = rawValue & 0x0000FFFF;
  rawValue = rawValue >> 4;

  return (float)rawValue * MAX31855_COLD_JUNCTION_RESOLUTION;
}

/**
 * @brief Reads raw data from MAX31855 via hardware SPI.
 * @return Raw data as int32_t.
 */
int32_t MAX31855::readRawData(void)
{
  int32_t rawData = 0;

  digitalWrite(_cs, LOW);
  delayMicroseconds(1);
  digitalWrite(_cs, HIGH);
  delay(MAX31855_CONVERSION_TIME);

  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs, LOW);

  for (uint8_t i = 0; i < 2; i++)
  {
    rawData = (rawData << 16) | SPI.transfer16(0x0000);
  }

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return rawData;
}
