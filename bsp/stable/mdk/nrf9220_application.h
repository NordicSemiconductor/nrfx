/*

Copyright (c) 2010 - 2026, Nordic Semiconductor ASA All rights reserved.

SPDX-License-Identifier: BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of Nordic Semiconductor ASA nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef NRF9220_APPLICATION_H
#define NRF9220_APPLICATION_H

#ifdef __cplusplus
    extern "C" {
#endif


#ifdef NRF_APPLICATION                               /*!< Processor information is domain local.                               */


/* =========================================================================================================================== */
/* ================                                Interrupt Number Definition                                ================ */
/* =========================================================================================================================== */

typedef enum {
/* ===================================================== Core Interrupts ===================================================== */
  Reset_IRQn                             = -15,      /*!< -15 Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn                    = -14,      /*!< -14 Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                         = -13,      /*!< -13 Hard Fault, all classes of Fault                                 */
  MemoryManagement_IRQn                  = -12,      /*!< -12 Memory Management, MPU mismatch, including Access Violation and No
                                                          Match*/
  BusFault_IRQn                          = -11,      /*!< -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                          related Fault*/
  UsageFault_IRQn                        = -10,      /*!< -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition    */
  SecureFault_IRQn                       = -9,       /*!<  -9 Secure Fault Handler                                             */
  SVCall_IRQn                            = -5,       /*!<  -5 System Service Call via SVC instruction                          */
  DebugMonitor_IRQn                      = -4,       /*!<  -4 Debug Monitor                                                    */
  PendSV_IRQn                            = -2,       /*!<  -2 Pendable request for system service                              */
  SysTick_IRQn                           = -1,       /*!<  -1 System Tick Timer                                                */
/* ============================================== Processor Specific Interrupts ============================================== */
  SPU000_IRQn                            = 0,        /*!< 0 SPU000                                                             */
  MPC_IRQn                               = 1,        /*!< 1 MPC                                                                */
  CPUC_IRQn                              = 2,        /*!< 2 CPUC                                                               */
  MVDMA_IRQn                             = 3,        /*!< 3 MVDMA                                                              */
  SPU010_IRQn                            = 16,       /*!< 16 SPU010                                                            */
  WDT010_IRQn                            = 20,       /*!< 20 WDT010                                                            */
  WDT011_IRQn                            = 21,       /*!< 21 WDT011                                                            */
  IPCT_0_IRQn                            = 64,       /*!< 64 IPCT_0                                                            */
  IPCT_1_IRQn                            = 65,       /*!< 65 IPCT_1                                                            */
  CTI_0_IRQn                             = 66,       /*!< 66 CTI_0                                                             */
  CTI_1_IRQn                             = 67,       /*!< 67 CTI_1                                                             */
  SWI0_IRQn                              = 88,       /*!< 88 SWI0                                                              */
  SWI1_IRQn                              = 89,       /*!< 89 SWI1                                                              */
  SWI2_IRQn                              = 90,       /*!< 90 SWI2                                                              */
  SWI3_IRQn                              = 91,       /*!< 91 SWI3                                                              */
  SWI4_IRQn                              = 92,       /*!< 92 SWI4                                                              */
  SWI5_IRQn                              = 93,       /*!< 93 SWI5                                                              */
  SWI6_IRQn                              = 94,       /*!< 94 SWI6                                                              */
  SWI7_IRQn                              = 95,       /*!< 95 SWI7                                                              */
  BELLBOARD_0_IRQn                       = 96,       /*!< 96 BELLBOARD_0                                                       */
  BELLBOARD_1_IRQn                       = 97,       /*!< 97 BELLBOARD_1                                                       */
  BELLBOARD_2_IRQn                       = 98,       /*!< 98 BELLBOARD_2                                                       */
  BELLBOARD_3_IRQn                       = 99,       /*!< 99 BELLBOARD_3                                                       */
  GPIOTE130_0_IRQn                       = 104,      /*!< 104 GPIOTE130_0                                                      */
  GPIOTE130_1_IRQn                       = 105,      /*!< 105 GPIOTE130_1                                                      */
  GRTC_0_IRQn                            = 108,      /*!< 108 GRTC_0                                                           */
  GRTC_1_IRQn                            = 109,      /*!< 109 GRTC_1                                                           */
  AXONS_IRQn                             = 138,      /*!< 138 AXONS                                                            */
  IPCT120_0_IRQn                         = 209,      /*!< 209 IPCT120_0                                                        */
  VPR121_IRQn                            = 212,      /*!< 212 VPR121                                                           */
  TIMER120_IRQn                          = 226,      /*!< 226 TIMER120                                                         */
  SPIM120_SPIS120_UARTE120_IRQn          = 230,      /*!< 230 SPIM120_SPIS120_UARTE120                                         */
  VPR130_IRQn                            = 264,      /*!< 264 VPR130                                                           */
  AHBBUFFER130_IRQn                      = 266,      /*!< 266 AHBBUFFER130                                                     */
  IPCT130_0_IRQn                         = 289,      /*!< 289 IPCT130_0                                                        */
  RTC130_IRQn                            = 296,      /*!< 296 RTC130                                                           */
  WDT131_IRQn                            = 299,      /*!< 299 WDT131                                                           */
  WDT132_IRQn                            = 300,      /*!< 300 WDT132                                                           */
  EGU130_IRQn                            = 301,      /*!< 301 EGU130                                                           */
  SAADC_IRQn                             = 386,      /*!< 386 SAADC                                                            */
  TEMP_IRQn                              = 388,      /*!< 388 TEMP                                                             */
  NFCT_IRQn                              = 389,      /*!< 389 NFCT                                                             */
  PDM_IRQn                               = 403,      /*!< 403 PDM                                                              */
  TIMER130_IRQn                          = 418,      /*!< 418 TIMER130                                                         */
  TIMER131_IRQn                          = 419,      /*!< 419 TIMER131                                                         */
  PWM130_IRQn                            = 420,      /*!< 420 PWM130                                                           */
  SERIAL0_IRQn                           = 421,      /*!< 421 SERIAL0                                                          */
  SERIAL1_IRQn                           = 422,      /*!< 422 SERIAL1                                                          */
  TIMER132_IRQn                          = 434,      /*!< 434 TIMER132                                                         */
  TIMER133_IRQn                          = 435,      /*!< 435 TIMER133                                                         */
  PWM131_IRQn                            = 436,      /*!< 436 PWM131                                                           */
  SERIAL2_IRQn                           = 437,      /*!< 437 SERIAL2                                                          */
  SERIAL3_IRQn                           = 438,      /*!< 438 SERIAL3                                                          */
  SERIAL4_IRQn                           = 453,      /*!< 453 SERIAL4                                                          */
  SERIAL5_IRQn                           = 454,      /*!< 454 SERIAL5                                                          */
  SERIAL6_IRQn                           = 469,      /*!< 469 SERIAL6                                                          */
  SERIAL7_IRQn                           = 470,      /*!< 470 SERIAL7                                                          */
} IRQn_Type;

/* ==================================================== Interrupt Aliases ==================================================== */
#define SPIM120_IRQn                  SPIM120_SPIS120_UARTE120_IRQn
#define SPIM120_IRQHandler            SPIM120_SPIS120_UARTE120_IRQHandler
#define SPIS120_IRQn                  SPIM120_SPIS120_UARTE120_IRQn
#define SPIS120_IRQHandler            SPIM120_SPIS120_UARTE120_IRQHandler
#define UARTE120_IRQn                 SPIM120_SPIS120_UARTE120_IRQn
#define UARTE120_IRQHandler           SPIM120_SPIS120_UARTE120_IRQHandler
#define SPIM130_IRQn                  SERIAL0_IRQn
#define SPIM130_IRQHandler            SERIAL0_IRQHandler
#define SPIS130_IRQn                  SERIAL0_IRQn
#define SPIS130_IRQHandler            SERIAL0_IRQHandler
#define TWIM130_IRQn                  SERIAL0_IRQn
#define TWIM130_IRQHandler            SERIAL0_IRQHandler
#define TWIS130_IRQn                  SERIAL0_IRQn
#define TWIS130_IRQHandler            SERIAL0_IRQHandler
#define UARTE130_IRQn                 SERIAL0_IRQn
#define UARTE130_IRQHandler           SERIAL0_IRQHandler
#define SPIM131_IRQn                  SERIAL1_IRQn
#define SPIM131_IRQHandler            SERIAL1_IRQHandler
#define SPIS131_IRQn                  SERIAL1_IRQn
#define SPIS131_IRQHandler            SERIAL1_IRQHandler
#define TWIM131_IRQn                  SERIAL1_IRQn
#define TWIM131_IRQHandler            SERIAL1_IRQHandler
#define TWIS131_IRQn                  SERIAL1_IRQn
#define TWIS131_IRQHandler            SERIAL1_IRQHandler
#define UARTE131_IRQn                 SERIAL1_IRQn
#define UARTE131_IRQHandler           SERIAL1_IRQHandler
#define SPIM132_IRQn                  SERIAL2_IRQn
#define SPIM132_IRQHandler            SERIAL2_IRQHandler
#define SPIS132_IRQn                  SERIAL2_IRQn
#define SPIS132_IRQHandler            SERIAL2_IRQHandler
#define TWIM132_IRQn                  SERIAL2_IRQn
#define TWIM132_IRQHandler            SERIAL2_IRQHandler
#define TWIS132_IRQn                  SERIAL2_IRQn
#define TWIS132_IRQHandler            SERIAL2_IRQHandler
#define UARTE132_IRQn                 SERIAL2_IRQn
#define UARTE132_IRQHandler           SERIAL2_IRQHandler
#define SPIM133_IRQn                  SERIAL3_IRQn
#define SPIM133_IRQHandler            SERIAL3_IRQHandler
#define SPIS133_IRQn                  SERIAL3_IRQn
#define SPIS133_IRQHandler            SERIAL3_IRQHandler
#define TWIM133_IRQn                  SERIAL3_IRQn
#define TWIM133_IRQHandler            SERIAL3_IRQHandler
#define TWIS133_IRQn                  SERIAL3_IRQn
#define TWIS133_IRQHandler            SERIAL3_IRQHandler
#define UARTE133_IRQn                 SERIAL3_IRQn
#define UARTE133_IRQHandler           SERIAL3_IRQHandler

/* =========================================================================================================================== */
/* ================                           Processor and Core Peripheral Section                           ================ */
/* =========================================================================================================================== */

/* =========================== Configuration of the ARM Cortex-M33 Processor and Core Peripherals ============================ */
#define __CM33_REV                  r1p0             /*!< CM33 Core Revision                                                   */
#define __DSP_PRESENT                  1             /*!< DSP present or not                                                   */
#define __NVIC_PRIO_BITS               3             /*!< Number of Bits used for Priority Levels                              */
#define __VTOR_PRESENT                 1             /*!< CPU supports alternate Vector Table address                          */
#define __MPU_PRESENT                  1             /*!< MPU present                                                          */
#define __FPU_PRESENT                  1             /*!< FPU present                                                          */
#define __FPU_DP                       0             /*!< Double Precision FPU                                                 */
#define __INTERRUPTS_MAX             480             /*!< Size of interrupt vector table                                       */
#define __Vendor_SysTickConfig         0             /*!< Vendor SysTick Config implementation is used                         */
#define __SAUREGION_PRESENT            1             /*!< SAU present                                                          */
#define __NUM_SAUREGIONS               4             /*!< Number of regions                                                    */

#include "core_cm33.h"                               /*!< ARM Cortex-M33 processor and core peripherals                        */
#include "system_nrf.h"                              /*!< nrf9220_application System Library                                   */

#endif                                               /*!< NRF_APPLICATION                                                      */


#ifdef NRF_APPLICATION

  #define NRF_DOMAIN                    NRF_DOMAIN_APPLICATION

#endif                                               /*!< NRF_APPLICATION                                                      */


/* ========================================= Start of section using anonymous unions ========================================= */

#include "compiler_abstraction.h"

#if defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
  #pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
  #pragma clang diagnostic ignored "-Wnested-anon-types"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Unsupported compiler type
#endif

/* =========================================================================================================================== */
/* ================                                  Peripheral Address Map                                  ================ */
/* =========================================================================================================================== */

#define NRF_APPLICATION_UICREXTENDED_NS_BASE 0x00000000UL
#define NRF_APPLICATION_ICACHEDATA_S_BASE 0x02F00000UL
#define NRF_APPLICATION_ICACHEINFO_S_BASE 0x02F10000UL
#define NRF_APPLICATION_UICR_NS_BASE      0x0FFF0000UL
#define NRF_APPLICATION_BICR_NS_BASE      0x0FFF0800UL
#define NRF_APPLICATION_DCACHEDATA_S_BASE 0x22F00000UL
#define NRF_APPLICATION_DCACHEINFO_S_BASE 0x22F10000UL
#define NRF_APPLICATION_ETM_NS_BASE       0xE0041000UL
#define NRF_APPLICATION_CTI_S_BASE        0xE0042000UL
#define NRF_APPLICATION_ICACHE_S_BASE     0xE0082000UL
#define NRF_APPLICATION_DCACHE_S_BASE     0xE0083000UL
#define NRF_APPLICATION_SPU000_S_BASE     0x52000000UL
#define NRF_APPLICATION_MPC_S_BASE        0x52001000UL
#define NRF_APPLICATION_CPUC_S_BASE       0xE0080000UL
#define NRF_APPLICATION_MVDMA_NS_BASE     0x42003000UL
#define NRF_APPLICATION_MVDMA_S_BASE      0x52003000UL
#define NRF_APPLICATION_HSFLL_NS_BASE     0x4200D000UL
#define NRF_APPLICATION_HSFLL_S_BASE      0x5200D000UL
#define NRF_APPLICATION_SPU010_S_BASE     0x52010000UL
#define NRF_APPLICATION_MEMCONF_NS_BASE   0x42012000UL
#define NRF_APPLICATION_MEMCONF_S_BASE    0x52012000UL
#define NRF_APPLICATION_WDT010_NS_BASE    0x42014000UL
#define NRF_APPLICATION_WDT010_S_BASE     0x52014000UL
#define NRF_APPLICATION_WDT011_NS_BASE    0x42015000UL
#define NRF_APPLICATION_WDT011_S_BASE     0x52015000UL
#define NRF_APPLICATION_RESETINFO_NS_BASE 0x4201E000UL
#define NRF_APPLICATION_RESETINFO_S_BASE  0x5201E000UL
#define NRF_APPLICATION_IPCT_NS_BASE      0x42013000UL
#define NRF_APPLICATION_IPCT_S_BASE       0x52013000UL
#define NRF_APPLICATION_SWI0_NS_BASE      0x42058000UL
#define NRF_APPLICATION_SWI1_NS_BASE      0x42059000UL
#define NRF_APPLICATION_SWI2_NS_BASE      0x4205A000UL
#define NRF_APPLICATION_SWI3_NS_BASE      0x4205B000UL
#define NRF_APPLICATION_SWI4_NS_BASE      0x4205C000UL
#define NRF_APPLICATION_SWI5_NS_BASE      0x4205D000UL
#define NRF_APPLICATION_SWI6_NS_BASE      0x4205E000UL
#define NRF_APPLICATION_SWI7_NS_BASE      0x4205F000UL
#define NRF_APPLICATION_BELLBOARD_NS_BASE 0x4F09A000UL
#define NRF_APPLICATION_BELLBOARD_S_BASE  0x5F09A000UL

/* =========================================================================================================================== */
/* ================                                  Peripheral Declaration                                  ================ */
/* =========================================================================================================================== */

#define NRF_APPLICATION_UICREXTENDED_NS   ((NRF_UICREXTENDED_Type*)             NRF_APPLICATION_UICREXTENDED_NS_BASE)
#define NRF_APPLICATION_ICACHEDATA_S      ((NRF_ICACHEDATA_Type*)               NRF_APPLICATION_ICACHEDATA_S_BASE)
#define NRF_APPLICATION_ICACHEINFO_S      ((NRF_ICACHEINFO_Type*)               NRF_APPLICATION_ICACHEINFO_S_BASE)
#define NRF_APPLICATION_UICR_NS           ((NRF_UICR_Type*)                     NRF_APPLICATION_UICR_NS_BASE)
#define NRF_APPLICATION_BICR_NS           ((NRF_BICR_Type*)                     NRF_APPLICATION_BICR_NS_BASE)
#define NRF_APPLICATION_DCACHEDATA_S      ((NRF_DCACHEDATA_Type*)               NRF_APPLICATION_DCACHEDATA_S_BASE)
#define NRF_APPLICATION_DCACHEINFO_S      ((NRF_DCACHEINFO_Type*)               NRF_APPLICATION_DCACHEINFO_S_BASE)
#define NRF_APPLICATION_ETM_NS            ((NRF_ETM_Type*)                      NRF_APPLICATION_ETM_NS_BASE)
#define NRF_APPLICATION_CTI_S             ((NRF_CTI_Type*)                      NRF_APPLICATION_CTI_S_BASE)
#define NRF_APPLICATION_ICACHE_S          ((NRF_CACHE_Type*)                    NRF_APPLICATION_ICACHE_S_BASE)
#define NRF_APPLICATION_DCACHE_S          ((NRF_CACHE_Type*)                    NRF_APPLICATION_DCACHE_S_BASE)
#define NRF_APPLICATION_SPU000_S          ((NRF_SPU_Type*)                      NRF_APPLICATION_SPU000_S_BASE)
#define NRF_APPLICATION_MPC_S             ((NRF_MPC_Type*)                      NRF_APPLICATION_MPC_S_BASE)
#define NRF_APPLICATION_CPUC_S            ((NRF_CPUC_Type*)                     NRF_APPLICATION_CPUC_S_BASE)
#define NRF_APPLICATION_MVDMA_NS          ((NRF_MVDMA_Type*)                    NRF_APPLICATION_MVDMA_NS_BASE)
#define NRF_APPLICATION_MVDMA_S           ((NRF_MVDMA_Type*)                    NRF_APPLICATION_MVDMA_S_BASE)
#define NRF_APPLICATION_HSFLL_NS          ((NRF_HSFLL_Type*)                    NRF_APPLICATION_HSFLL_NS_BASE)
#define NRF_APPLICATION_HSFLL_S           ((NRF_HSFLL_Type*)                    NRF_APPLICATION_HSFLL_S_BASE)
#define NRF_APPLICATION_SPU010_S          ((NRF_SPU_Type*)                      NRF_APPLICATION_SPU010_S_BASE)
#define NRF_APPLICATION_MEMCONF_NS        ((NRF_MEMCONF_Type*)                  NRF_APPLICATION_MEMCONF_NS_BASE)
#define NRF_APPLICATION_MEMCONF_S         ((NRF_MEMCONF_Type*)                  NRF_APPLICATION_MEMCONF_S_BASE)
#define NRF_APPLICATION_WDT010_NS         ((NRF_WDT_Type*)                      NRF_APPLICATION_WDT010_NS_BASE)
#define NRF_APPLICATION_WDT010_S          ((NRF_WDT_Type*)                      NRF_APPLICATION_WDT010_S_BASE)
#define NRF_APPLICATION_WDT011_NS         ((NRF_WDT_Type*)                      NRF_APPLICATION_WDT011_NS_BASE)
#define NRF_APPLICATION_WDT011_S          ((NRF_WDT_Type*)                      NRF_APPLICATION_WDT011_S_BASE)
#define NRF_APPLICATION_RESETINFO_NS      ((NRF_RESETINFO_Type*)                NRF_APPLICATION_RESETINFO_NS_BASE)
#define NRF_APPLICATION_RESETINFO_S       ((NRF_RESETINFO_Type*)                NRF_APPLICATION_RESETINFO_S_BASE)
#define NRF_APPLICATION_IPCT_NS           ((NRF_IPCT_Type*)                     NRF_APPLICATION_IPCT_NS_BASE)
#define NRF_APPLICATION_IPCT_S            ((NRF_IPCT_Type*)                     NRF_APPLICATION_IPCT_S_BASE)
#define NRF_APPLICATION_SWI0_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI0_NS_BASE)
#define NRF_APPLICATION_SWI1_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI1_NS_BASE)
#define NRF_APPLICATION_SWI2_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI2_NS_BASE)
#define NRF_APPLICATION_SWI3_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI3_NS_BASE)
#define NRF_APPLICATION_SWI4_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI4_NS_BASE)
#define NRF_APPLICATION_SWI5_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI5_NS_BASE)
#define NRF_APPLICATION_SWI6_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI6_NS_BASE)
#define NRF_APPLICATION_SWI7_NS           ((NRF_SWI_Type*)                      NRF_APPLICATION_SWI7_NS_BASE)
#define NRF_APPLICATION_BELLBOARD_NS      ((NRF_BELLBOARD_Type*)                NRF_APPLICATION_BELLBOARD_NS_BASE)
#define NRF_APPLICATION_BELLBOARD_S       ((NRF_BELLBOARD_Type*)                NRF_APPLICATION_BELLBOARD_S_BASE)

/* =========================================================================================================================== */
/* ================                                    TrustZone Remapping                                    ================ */
/* =========================================================================================================================== */

#ifdef NRF_TRUSTZONE_NONSECURE                       /*!< Remap NRF_X_NS instances to NRF_X symbol for ease of use.            */
  #define NRF_APPLICATION_UICREXTENDED            NRF_APPLICATION_UICREXTENDED_NS
  #define NRF_APPLICATION_UICR                    NRF_APPLICATION_UICR_NS
  #define NRF_APPLICATION_BICR                    NRF_APPLICATION_BICR_NS
  #define NRF_APPLICATION_ETM                     NRF_APPLICATION_ETM_NS
  #define NRF_APPLICATION_MVDMA                   NRF_APPLICATION_MVDMA_NS
  #define NRF_APPLICATION_HSFLL                   NRF_APPLICATION_HSFLL_NS
  #define NRF_APPLICATION_MEMCONF                 NRF_APPLICATION_MEMCONF_NS
  #define NRF_APPLICATION_WDT010                  NRF_APPLICATION_WDT010_NS
  #define NRF_APPLICATION_WDT011                  NRF_APPLICATION_WDT011_NS
  #define NRF_APPLICATION_RESETINFO               NRF_APPLICATION_RESETINFO_NS
  #define NRF_APPLICATION_IPCT                    NRF_APPLICATION_IPCT_NS
  #define NRF_APPLICATION_SWI0                    NRF_APPLICATION_SWI0_NS
  #define NRF_APPLICATION_SWI1                    NRF_APPLICATION_SWI1_NS
  #define NRF_APPLICATION_SWI2                    NRF_APPLICATION_SWI2_NS
  #define NRF_APPLICATION_SWI3                    NRF_APPLICATION_SWI3_NS
  #define NRF_APPLICATION_SWI4                    NRF_APPLICATION_SWI4_NS
  #define NRF_APPLICATION_SWI5                    NRF_APPLICATION_SWI5_NS
  #define NRF_APPLICATION_SWI6                    NRF_APPLICATION_SWI6_NS
  #define NRF_APPLICATION_SWI7                    NRF_APPLICATION_SWI7_NS
  #define NRF_APPLICATION_BELLBOARD               NRF_APPLICATION_BELLBOARD_NS
#else                                                /*!< Remap NRF_X_S instances to NRF_X symbol for ease of use.             */
  #define NRF_APPLICATION_UICREXTENDED            NRF_APPLICATION_UICREXTENDED_NS
  #define NRF_APPLICATION_ICACHEDATA              NRF_APPLICATION_ICACHEDATA_S
  #define NRF_APPLICATION_ICACHEINFO              NRF_APPLICATION_ICACHEINFO_S
  #define NRF_APPLICATION_UICR                    NRF_APPLICATION_UICR_NS
  #define NRF_APPLICATION_BICR                    NRF_APPLICATION_BICR_NS
  #define NRF_APPLICATION_DCACHEDATA              NRF_APPLICATION_DCACHEDATA_S
  #define NRF_APPLICATION_DCACHEINFO              NRF_APPLICATION_DCACHEINFO_S
  #define NRF_APPLICATION_ETM                     NRF_APPLICATION_ETM_NS
  #define NRF_APPLICATION_CTI                     NRF_APPLICATION_CTI_S
  #define NRF_APPLICATION_ICACHE                  NRF_APPLICATION_ICACHE_S
  #define NRF_APPLICATION_DCACHE                  NRF_APPLICATION_DCACHE_S
  #define NRF_APPLICATION_SPU000                  NRF_APPLICATION_SPU000_S
  #define NRF_APPLICATION_MPC                     NRF_APPLICATION_MPC_S
  #define NRF_APPLICATION_CPUC                    NRF_APPLICATION_CPUC_S
  #define NRF_APPLICATION_MVDMA                   NRF_APPLICATION_MVDMA_S
  #define NRF_APPLICATION_HSFLL                   NRF_APPLICATION_HSFLL_S
  #define NRF_APPLICATION_SPU010                  NRF_APPLICATION_SPU010_S
  #define NRF_APPLICATION_MEMCONF                 NRF_APPLICATION_MEMCONF_S
  #define NRF_APPLICATION_WDT010                  NRF_APPLICATION_WDT010_S
  #define NRF_APPLICATION_WDT011                  NRF_APPLICATION_WDT011_S
  #define NRF_APPLICATION_RESETINFO               NRF_APPLICATION_RESETINFO_S
  #define NRF_APPLICATION_IPCT                    NRF_APPLICATION_IPCT_S
  #define NRF_APPLICATION_SWI0                    NRF_APPLICATION_SWI0_NS
  #define NRF_APPLICATION_SWI1                    NRF_APPLICATION_SWI1_NS
  #define NRF_APPLICATION_SWI2                    NRF_APPLICATION_SWI2_NS
  #define NRF_APPLICATION_SWI3                    NRF_APPLICATION_SWI3_NS
  #define NRF_APPLICATION_SWI4                    NRF_APPLICATION_SWI4_NS
  #define NRF_APPLICATION_SWI5                    NRF_APPLICATION_SWI5_NS
  #define NRF_APPLICATION_SWI6                    NRF_APPLICATION_SWI6_NS
  #define NRF_APPLICATION_SWI7                    NRF_APPLICATION_SWI7_NS
  #define NRF_APPLICATION_BELLBOARD               NRF_APPLICATION_BELLBOARD_S
#endif                                               /*!< NRF_TRUSTZONE_NONSECURE                                              */

/* =========================================================================================================================== */
/* ================                                  Local Domain Remapping                                  ================ */
/* =========================================================================================================================== */

#ifdef NRF_APPLICATION                               /*!< Remap NRF_DOMAIN_X instances to NRF_X symbol for ease of use.        */
  #ifdef NRF_TRUSTZONE_NONSECURE                     /*!< Remap only nonsecure instances.                                      */
    #define NRF_UICREXTENDED                      NRF_APPLICATION_UICREXTENDED
    #define NRF_UICR                              NRF_APPLICATION_UICR
    #define NRF_BICR                              NRF_APPLICATION_BICR
    #define NRF_ETM                               NRF_APPLICATION_ETM
    #define NRF_MVDMA                             NRF_APPLICATION_MVDMA
    #define NRF_HSFLL                             NRF_APPLICATION_HSFLL
    #define NRF_MEMCONF                           NRF_APPLICATION_MEMCONF
    #define NRF_WDT010                            NRF_APPLICATION_WDT010
    #define NRF_WDT011                            NRF_APPLICATION_WDT011
    #define NRF_RESETINFO                         NRF_APPLICATION_RESETINFO
    #define NRF_IPCT                              NRF_APPLICATION_IPCT
    #define NRF_SWI0                              NRF_APPLICATION_SWI0
    #define NRF_SWI1                              NRF_APPLICATION_SWI1
    #define NRF_SWI2                              NRF_APPLICATION_SWI2
    #define NRF_SWI3                              NRF_APPLICATION_SWI3
    #define NRF_SWI4                              NRF_APPLICATION_SWI4
    #define NRF_SWI5                              NRF_APPLICATION_SWI5
    #define NRF_SWI6                              NRF_APPLICATION_SWI6
    #define NRF_SWI7                              NRF_APPLICATION_SWI7
    #define NRF_BELLBOARD                         NRF_APPLICATION_BELLBOARD
  #else                                              /*!< Remap all instances.                                                 */
    #define NRF_UICREXTENDED                      NRF_APPLICATION_UICREXTENDED
    #define NRF_ICACHEDATA                        NRF_APPLICATION_ICACHEDATA
    #define NRF_ICACHEINFO                        NRF_APPLICATION_ICACHEINFO
    #define NRF_UICR                              NRF_APPLICATION_UICR
    #define NRF_BICR                              NRF_APPLICATION_BICR
    #define NRF_DCACHEDATA                        NRF_APPLICATION_DCACHEDATA
    #define NRF_DCACHEINFO                        NRF_APPLICATION_DCACHEINFO
    #define NRF_ETM                               NRF_APPLICATION_ETM
    #define NRF_CTI                               NRF_APPLICATION_CTI
    #define NRF_ICACHE                            NRF_APPLICATION_ICACHE
    #define NRF_DCACHE                            NRF_APPLICATION_DCACHE
    #define NRF_SPU000                            NRF_APPLICATION_SPU000
    #define NRF_MPC                               NRF_APPLICATION_MPC
    #define NRF_CPUC                              NRF_APPLICATION_CPUC
    #define NRF_MVDMA                             NRF_APPLICATION_MVDMA
    #define NRF_HSFLL                             NRF_APPLICATION_HSFLL
    #define NRF_SPU010                            NRF_APPLICATION_SPU010
    #define NRF_MEMCONF                           NRF_APPLICATION_MEMCONF
    #define NRF_WDT010                            NRF_APPLICATION_WDT010
    #define NRF_WDT011                            NRF_APPLICATION_WDT011
    #define NRF_RESETINFO                         NRF_APPLICATION_RESETINFO
    #define NRF_IPCT                              NRF_APPLICATION_IPCT
    #define NRF_SWI0                              NRF_APPLICATION_SWI0
    #define NRF_SWI1                              NRF_APPLICATION_SWI1
    #define NRF_SWI2                              NRF_APPLICATION_SWI2
    #define NRF_SWI3                              NRF_APPLICATION_SWI3
    #define NRF_SWI4                              NRF_APPLICATION_SWI4
    #define NRF_SWI5                              NRF_APPLICATION_SWI5
    #define NRF_SWI6                              NRF_APPLICATION_SWI6
    #define NRF_SWI7                              NRF_APPLICATION_SWI7
    #define NRF_BELLBOARD                         NRF_APPLICATION_BELLBOARD
  #endif                                             /*!< NRF_TRUSTZONE_NONSECURE                                              */
#endif                                               /*!< NRF_APPLICATION                                                      */

/* ========================================== End of section using anonymous unions ========================================== */

#if defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#endif


#ifdef __cplusplus
}
#endif
#endif /* NRF9220_APPLICATION_H */

