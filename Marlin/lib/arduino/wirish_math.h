/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/include/wirish/wirish_math.h
 * @brief Includes <math.h>; provides Wiring-compatible math routines.
 */

#ifndef _WIRISH_WIRISH_MATH_H_
#define _WIRISH_WIRISH_MATH_H_

#include <math.h>
#include <stdint.h>

/**
 * @brief Initialize the pseudo-random number generator.
 * @param seed the number used to initialize the seed; cannot be zero.
 */
void randomSeed(unsigned int seed);
long random(long max);
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

//#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105


/*
 * Roger Clark 20141113
 *
 * Added BitOrder definition from SAM wiring_constants.h, as its needed for SPI
 * as Maple doesn't have a wiring_constants file (though it probably should have in the long term to make it more compatible with the Arduino 1.0 + API
 * also added definition for EULER and SERIAL and DISPLAY, also from the same SAM header
 */
 
#define EULER 2.718281828459045235360287471352
#define SERIAL  0x0
#define DISPLAY 0x1 

#define min(a,b)                ((a)<(b)?(a):(b))
#define max(a,b)                ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)                ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg)            ((deg)*DEG_TO_RAD)
#define degrees(rad)            ((rad)*RAD_TO_DEG)
#define sq(x)                   ((x)*(x))

#define abs(x) (((x) > 0) ? (x) : -(x))

#ifdef math
double cos(double x);
double sin(double x);
double tan(double x);
double sqrt(double x);
double pow(double x, double y);
#endif
extern uint16_t makeWord( uint16_t w ) ;
extern uint16_t makeWord( uint8_t h, uint8_t l ) ;

#define word(...) makeWord(__VA_ARGS__)

#endif
