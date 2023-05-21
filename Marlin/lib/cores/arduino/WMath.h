/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _WIRING_MATH_
#define _WIRING_MATH_

#include <math.h>
#include <stdint.h>

/**
 * @brief Initialize the pseudo-random number generator.
 * @param seed the number used to initialize the seed; cannot be zero.
 */
void randomSeed(unsigned int seed);

/**
 * @brief Generate a pseudo-random number with upper bound.
 * @param max An upper bound on the returned value, exclusive.
 * @return A pseudo-random number in the range [0,max).
 * @see randomSeed()
 */
long random(long max);

/**
 * @brief Generate a pseudo-random number with lower and upper bounds.
 * @param min Lower bound on the returned value, inclusive.
 * @param max Upper bound on the returned value, exclusive.
 * @return A pseudo-random number in the range [min, max).
 * @see randomSeed()
 */
long random(long min, long max);

/**
 * @brief Remap a number from one range to another.
 *
 * That is, a value equal to fromStart gets mapped to toStart, a value
 * of fromEnd to toEnd, and other values are mapped proportionately.
 *
 * Does not constrain value to lie within [fromStart, fromEnd].
 *
 * If a "start" value is larger than its corresponding "end", the
 * ranges are reversed, so map(n, 1, 10, 10, 1) would reverse the
 * range [1,10].
 *
 * Negative numbers may appear as any argument.
 *
 * @param value the value to map.
 * @param fromStart the beginning of the value's current range.
 * @param fromEnd the end of the value's current range.
 * @param toStart the beginning of the value's mapped range.
 * @param toEnd the end of the value's mapped range.
 * @return the mapped value.
 */
 // Fix by Pito 9/2017
  static inline int32_t map(int32_t value, int32_t fromStart, int32_t fromEnd,
     int32_t toStart, int32_t toEnd) {
     return ((int64_t)(value - fromStart) * (toEnd - toStart)) / (fromEnd - fromStart) +
         toStart;
 }

extern uint16_t makeWord( uint16_t w ) ;
extern uint16_t makeWord( uint8_t h, uint8_t l ) ;

#define word(...) makeWord(__VA_ARGS__)


#endif /* _WIRING_MATH_ */
