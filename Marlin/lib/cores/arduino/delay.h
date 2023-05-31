/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#ifndef _DELAY_
#define _DELAY_

#include <stdint.h>
#include "variant.h"
#include "drivers/sysclock/systick.h"

#include "hc32_ddl.h"
#include "hc32f460_clk.h"
#include "hc32f460_efm.h"
#include "hc32f460_utility.h"
#include "hc32f460_sram.h"
#include "hc32f460_interrupts.h"
#include "hc32f460_pwc.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * \brief Pauses the program for the amount of time (in miliseconds) specified as parameter.
   * (There are 1000 milliseconds in a second.)
   *
   * \param dwMs the number of milliseconds to pause (uint32_t)
   */
  void delay(uint32_t dwMs);

  /**
   * \brief Pauses the program for the amount of time (in microseconds) specified as parameter.
   *
   * \param dwUs the number of microseconds to pause (uint32_t)
   */
  void delayMicroseconds(uint32_t dwUs);

#ifdef __cplusplus
}
#endif

#endif /* _DELAY_ */
