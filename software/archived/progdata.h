#ifndef PROGDATA_H
#define PROGDATA_H

#include "config.h"



#endif // PROGDATA_H

/*
Fixed-width integer types and their usage:

1. Unsigned integers (only non-negative values):
   - uint8_t:  8-bit unsigned integer, range 0 to 255.
   - uint16_t: 16-bit unsigned integer, range 0 to 65,535.
   - uint32_t: 32-bit unsigned integer, range 0 to 4,294,967,295.
   - uint64_t: 64-bit unsigned integer, range 0 to 18,446,744,073,709,551,615.

2. Signed integers (positive and negative values):
   - int8_t:  8-bit signed integer, range -128 to 127.
   - int16_t: 16-bit signed integer, range -32,768 to 32,767.
   - int32_t: 32-bit signed integer, range -2,147,483,648 to 2,147,483,647.
   - int64_t: 64-bit signed integer, range -9,223,372,036,854,775,808 to 9,223,372,036,854,775,807.

Benefits of using fixed-width integers:
- Memory Efficiency: Allows for optimization of memory usage by selecting the smallest adequate type.
- Performance: Can enhance processing speed, especially in arrays or on certain architectures.
- Portability & Predictability: Ensures consistent behavior across different platforms.
- Code Clarity: Indicates the intended use and range of values, improving readability and maintenance.

These types are defined in the <cstdint> header and provide a standardized way to declare integers with specific sizes, ensuring consistent application behavior.
*/