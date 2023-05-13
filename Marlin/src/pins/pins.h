/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "startup.h"
#include "../core/boards.h"
#include "../../Configuration.h"
/**
 * File: pins/pins.h
 *
 * Include pins definitions
 *
 * Pins numbering schemes:
 *
 *  - Digital I/O pin number if used by READ/WRITE macros. (e.g., X_STEP_DIR)
 *    The FastIO headers map digital pins to their ports and functions.
 *
 *  - Analog Input number if used by analogRead or DAC. (e.g., TEMP_n_PIN)
 *    These numbers are the same in any pin mapping.
 */

#define MAX_E_STEPPERS 8

#if NONE(FET_ORDER_EEF, FET_ORDER_EEB, FET_ORDER_EFF, FET_ORDER_EFB, FET_ORDER_SF)
  #if   MB(RAMPS_13_EFB, RAMPS_14_EFB, RAMPS_PLUS_EFB, RAMPS_14_RE_ARM_EFB, RAMPS_SMART_EFB, RAMPS_DUO_EFB, RAMPS4DUE_EFB)
    #define FET_ORDER_EFB 1
  #elif MB(RAMPS_13_EEB, RAMPS_14_EEB, RAMPS_PLUS_EEB, RAMPS_14_RE_ARM_EEB, RAMPS_SMART_EEB, RAMPS_DUO_EEB, RAMPS4DUE_EEB)
    #define FET_ORDER_EEB 1
  #elif MB(RAMPS_13_EFF, RAMPS_14_EFF, RAMPS_PLUS_EFF, RAMPS_14_RE_ARM_EFF, RAMPS_SMART_EFF, RAMPS_DUO_EFF, RAMPS4DUE_EFF)
    #define FET_ORDER_EFF 1
  #elif MB(RAMPS_13_EEF, RAMPS_14_EEF, RAMPS_PLUS_EEF, RAMPS_14_RE_ARM_EEF, RAMPS_SMART_EEF, RAMPS_DUO_EEF, RAMPS4DUE_EEF)
    #define FET_ORDER_EEF 1
  #elif MB(RAMPS_13_SF,  RAMPS_14_SF,  RAMPS_PLUS_SF,  RAMPS_14_RE_ARM_SF,  RAMPS_SMART_SF,  RAMPS_DUO_SF,  RAMPS4DUE_SF)
    #define FET_ORDER_SF 1
  #elif HAS_MULTI_HOTEND || (HAS_EXTRUDERS && HAS_CUTTER)
    #if TEMP_SENSOR_BED
      #define FET_ORDER_EEB 1
    #else
      #define FET_ORDER_EEF 1
    #endif
  #elif TEMP_SENSOR_BED
    #define FET_ORDER_EFB 1
  #else
    #define FET_ORDER_EFF 1
  #endif
#endif

#if !(BOTH(HAS_WIRED_LCD, IS_NEWPANEL) && ANY(PANEL_ONE, VIKI2, miniVIKI, WYH_L12864, MINIPANEL, REPRAPWORLD_KEYPAD))
  #define HAS_FREE_AUX2_PINS 1
#endif

// Test the target within the included pins file
#ifdef __MARLIN_DEPS__
  #define NOT_TARGET(V...) 0
#else
  #define NOT_TARGET(V...) NONE(V)
#endif

//
// Anycubic Kobra
//

#if MB(AC_TRI_F1_V1)
  #include "hc32f46x/pins_AC_TRI_F1_V1.h"

//
// Linux Native Debug board
//

#elif MB(SIMULATED)
  #include "linux/pins_RAMPS_LINUX.h"           // Native or Simulation                   lin:linux_native mac:simulator_macos_debug mac:simulator_macos_release win:simulator_windows lin:simulator_linux_debug lin:simulator_linux_release

#else

  //
  // Obsolete or unknown board
  //

  #define BOARD_MKS_13                  99900
  #define BOARD_TRIGORILLA              99901
  #define BOARD_RURAMPS4D               99902
  #define BOARD_FORMBOT_TREX2           99903
  #define BOARD_BIQU_SKR_V1_1           99904
  #define BOARD_STM32F1R                99905
  #define BOARD_STM32F103R              99906
  #define BOARD_ESP32                   99907
  #define BOARD_STEVAL                  99908
  #define BOARD_STEVAL_3DP001V1         99908
  #define BOARD_BIGTREE_SKR_V1_1        99909
  #define BOARD_BIGTREE_SKR_V1_3        99910
  #define BOARD_BIGTREE_SKR_V1_4        99911
  #define BOARD_BIGTREE_SKR_V1_4_TURBO  99912
  #define BOARD_BIGTREE_BTT002_V1_0     99913
  #define BOARD_BIGTREE_SKR_PRO_V1_1    99914
  #define BOARD_BIGTREE_SKR_MINI_V1_1   99915
  #define BOARD_BIGTREE_SKR_MINI_E3     99916
  #define BOARD_BIGTREE_SKR_E3_DIP      99917
  #define BOARD_RUMBA32                 99918
  #define BOARD_RUMBA32_AUS3D           99919
  #define BOARD_RAMPS_DAGOMA            99920
  #define BOARD_RAMPS_LONGER3D_LK4PRO   99921
  #define BOARD_BTT_SKR_V2_0            99922
  #define BOARD_TH3D_EZBOARD_LITE_V2    99923
  #define BOARD_BTT_SKR_SE_BX           99924
  #define BOARD_MKS_MONSTER8            99925
  #define BOARD_LINUX_RAMPS             99926

  #if MB(MKS_13)
    #error "BOARD_MKS_13 has been renamed BOARD_MKS_GEN_13. Please update your configuration."
  #elif MB(TRIGORILLA)
    #error "BOARD_TRIGORILLA has been renamed BOARD_TRIGORILLA_13. Please update your configuration."
  #elif MB(RURAMPS4D)
    #error "BOARD_RURAMPS4D has been renamed BOARD_RURAMPS4D_11. Please update your configuration."
  #elif MB(FORMBOT_TREX2)
    #error "FORMBOT_TREX2 has been renamed BOARD_FORMBOT_TREX2PLUS. Please update your configuration."
  #elif MB(BIQU_SKR_V1_1)
    #error "BOARD_BIQU_SKR_V1_1 has been renamed BOARD_BTT_SKR_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_1)
    #error "BOARD_BIGTREE_SKR_V1_1 has been renamed BOARD_BTT_SKR_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_2)
    #error "BOARD_BIGTREE_SKR_V1_2 has been renamed BOARD_BTT_SKR_V1_2. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_3)
    #error "BOARD_BIGTREE_SKR_V1_3 has been renamed BOARD_BTT_SKR_V1_3. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_4)
    #error "BOARD_BIGTREE_SKR_V1_4 has been renamed BOARD_BTT_SKR_V1_4. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_4_TURBO)
    #error "BOARD_BIGTREE_SKR_V1_4_TURBO has been renamed BOARD_BTT_SKR_V1_4_TURBO. Please update your configuration."
  #elif MB(BIGTREE_BTT002_V1_0)
    #error "BOARD_BIGTREE_BTT002_V1_0 has been renamed BOARD_BTT_BTT002_V1_0. Please update your configuration."
  #elif MB(BIGTREE_SKR_PRO_V1_1)
    #error "BOARD_BIGTREE_SKR_PRO_V1_1 has been renamed BOARD_BTT_SKR_PRO_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_MINI_V1_1)
    #error "BOARD_BIGTREE_SKR_MINI_V1_1 has been renamed BOARD_BTT_SKR_MINI_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_MINI_E3)
    #error "BOARD_BIGTREE_SKR_MINI_E3 has been renamed BOARD_BTT_SKR_MINI_E3_V1_0. Please update your configuration."
  #elif MB(BIGTREE_SKR_E3_DIP)
    #error "BOARD_BIGTREE_SKR_E3_DIP has been renamed BOARD_BTT_SKR_E3_DIP. Please update your configuration."
  #elif MB(STM32F1R)
    #error "BOARD_STM32F1R has been renamed BOARD_STM32F103RE. Please update your configuration."
  #elif MB(STM32F103R)
    #error "BOARD_STM32F103R has been renamed BOARD_STM32F103RE. Please update your configuration."
  #elif MOTHERBOARD == BOARD_ESP32
    #error "BOARD_ESP32 has been renamed BOARD_ESPRESSIF_ESP32. Please update your configuration."
  #elif MB(STEVAL)
    #error "BOARD_STEVAL_3DP001V1 (BOARD_STEVAL) is no longer supported in Marlin."
  #elif MB(RUMBA32)
    #error "BOARD_RUMBA32 is now BOARD_RUMBA32_MKS or BOARD_RUMBA32_V1_0. Please update your configuration."
  #elif MB(RUMBA32_AUS3D)
    #error "BOARD_RUMBA32_AUS3D is now BOARD_RUMBA32_V1_0. Please update your configuration."
  #elif MB(RAMPS_DAGOMA)
    #error "BOARD_RAMPS_DAGOMA is now BOARD_DAGOMA_F5. Please update your configuration."
  #elif MB(RAMPS_LONGER3D_LK4PRO)
    #error "BOARD_RAMPS_LONGER3D_LK4PRO is now BOARD_LONGER3D_LKx_PRO. Please update your configuration."
  #elif MB(BTT_SKR_V2_0)
    #error "BOARD_BTT_SKR_V2_0 is now BOARD_BTT_SKR_V2_0_REV_A or BOARD_BTT_SKR_V2_0_REV_B. See https://bit.ly/3t5d9JQ for more information. Please update your configuration."
  #elif MB(TH3D_EZBOARD_LITE_V2)
    #error "BOARD_TH3D_EZBOARD_LITE_V2 is now BOARD_TH3D_EZBOARD_V2. Please update your configuration."
  #elif MB(BTT_SKR_SE_BX)
    #error "BOARD_BTT_SKR_SE_BX is now BOARD_BTT_SKR_SE_BX_V2 or BOARD_BTT_SKR_SE_BX_V3. Please update your configuration."
  #elif MB(MKS_MONSTER8)
    #error "BOARD_MKS_MONSTER8 is now BOARD_MKS_MONSTER8_V1 or BOARD_MKS_MONSTER8_V2. Please update your configuration."
  #elif MB(LINUX_RAMPS)
    #error "BOARD_LINUX_RAMPS is now BOARD_SIMULATED. Please update your configuration."
  #elif defined(MOTHERBOARD)
    #error "Unknown MOTHERBOARD value set in Configuration.h."
  #else
    #error "MOTHERBOARD not defined! Use '#define MOTHERBOARD BOARD_...' in Configuration.h."
  #endif

  #undef BOARD_MKS_13
  #undef BOARD_TRIGORILLA
  #undef BOARD_RURAMPS4D
  #undef BOARD_FORMBOT_TREX2
  #undef BOARD_BIQU_SKR_V1_1
  #undef BOARD_STM32F1R
  #undef BOARD_STM32F103R
  #undef BOARD_ESP32
  #undef BOARD_STEVAL
  #undef BOARD_STEVAL_3DP001V1
  #undef BOARD_BIGTREE_SKR_V1_1
  #undef BOARD_BIGTREE_SKR_V1_3
  #undef BOARD_BIGTREE_SKR_V1_4
  #undef BOARD_BIGTREE_SKR_V1_4_TURBO
  #undef BOARD_BIGTREE_BTT002_V1_0
  #undef BOARD_BIGTREE_SKR_PRO_V1_1
  #undef BOARD_BIGTREE_SKR_MINI_V1_1
  #undef BOARD_BIGTREE_SKR_MINI_E3
  #undef BOARD_BIGTREE_SKR_E3_DIP
  #undef BOARD_RUMBA32
  #undef BOARD_RUMBA32_AUS3D
  #undef BOARD_RAMPS_DAGOMA
  #undef BOARD_RAMPS_LONGER3D_LK4PRO
  #undef BOARD_BTT_SKR_V2_0
  #undef BOARD_TH3D_EZBOARD_LITE_V2
  #undef BOARD_BTT_SKR_SE_BX
  #undef BOARD_MKS_MONSTER8
  #undef BOARD_LINUX_RAMPS

#endif

//
// Post-process pins according to configured settings
//
#include "pins_postprocess.h"
