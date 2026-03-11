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

#ifndef NRF9220_GLOBAL_H
#define NRF9220_GLOBAL_H

#ifdef __cplusplus
    extern "C" {
#endif


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

#define NRF_MICR_NS_BASE                  0x0FFF6000UL
#define NRF_FICR_NS_BASE                  0x0FFFE000UL
#define NRF_STMDATA_NS_BASE               0xA0000000UL
#define NRF_STMDATA_S_BASE                0xA0000000UL
#define NRF_TDDCONF_NS_BASE               0xBF001000UL
#define NRF_TDDCONF_S_BASE                0xBF001000UL
#define NRF_STM_NS_BASE                   0xBF042000UL
#define NRF_TPIU_NS_BASE                  0xBF043000UL
#define NRF_ETB_NS_BASE                   0xBF044000UL
#define NRF_CTI210_NS_BASE                0xBF046000UL
#define NRF_CTI211_NS_BASE                0xBF047000UL
#define NRF_ATBREPLICATOR210_NS_BASE      0xBF048000UL
#define NRF_ATBREPLICATOR211_NS_BASE      0xBF049000UL
#define NRF_ATBREPLICATOR212_NS_BASE      0xBF04A000UL
#define NRF_ATBREPLICATOR213_NS_BASE      0xBF04B000UL
#define NRF_ATBFUNNEL210_NS_BASE          0xBF04C000UL
#define NRF_ATBFUNNEL211_NS_BASE          0xBF04D000UL
#define NRF_ATBFUNNEL212_NS_BASE          0xBF04E000UL
#define NRF_ATBFUNNEL213_NS_BASE          0xBF04F000UL
#define NRF_GPR_NS_BASE                   0xBF050000UL
#define NRF_GPIOTE130_NS_BASE             0x4F934000UL
#define NRF_GPIOTE130_S_BASE              0x5F934000UL
#define NRF_GRTC_NS_BASE                  0x4F99C000UL
#define NRF_GRTC_S_BASE                   0x5F99C000UL
#define NRF_IPCT120_NS_BASE               0x4F8D1000UL
#define NRF_IPCT120_S_BASE                0x5F8D1000UL
#define NRF_MUTEX120_NS_BASE              0x4F8D2000UL
#define NRF_VPR121_NS_BASE                0x4F8D4000UL
#define NRF_VPR121_S_BASE                 0x5F8D4000UL
#define NRF_DPPIC120_NS_BASE              0x4F8E1000UL
#define NRF_DPPIC120_S_BASE               0x5F8E1000UL
#define NRF_TIMER120_NS_BASE              0x4F8E2000UL
#define NRF_TIMER120_S_BASE               0x5F8E2000UL
#define NRF_SPIM120_NS_BASE               0x4F8E6000UL
#define NRF_SPIS120_NS_BASE               0x4F8E6000UL
#define NRF_UARTE120_NS_BASE              0x4F8E6000UL
#define NRF_SPIM120_S_BASE                0x5F8E6000UL
#define NRF_SPIS120_S_BASE                0x5F8E6000UL
#define NRF_UARTE120_S_BASE               0x5F8E6000UL
#define NRF_VPR130_NS_BASE                0x4F908000UL
#define NRF_VPR130_S_BASE                 0x5F908000UL
#define NRF_AHBBUFFER130_NS_BASE          0x4F90A000UL
#define NRF_AHBBUFFER130_S_BASE           0x5F90A000UL
#define NRF_IPCT130_NS_BASE               0x4F921000UL
#define NRF_IPCT130_S_BASE                0x5F921000UL
#define NRF_DPPIC130_NS_BASE              0x4F922000UL
#define NRF_DPPIC130_S_BASE               0x5F922000UL
#define NRF_MUTEX130_NS_BASE              0x4F927000UL
#define NRF_RTC130_NS_BASE                0x4F928000UL
#define NRF_RTC130_S_BASE                 0x5F928000UL
#define NRF_WDT131_NS_BASE                0x4F92B000UL
#define NRF_WDT131_S_BASE                 0x5F92B000UL
#define NRF_WDT132_NS_BASE                0x4F92C000UL
#define NRF_WDT132_S_BASE                 0x5F92C000UL
#define NRF_EGU130_NS_BASE                0x4F92D000UL
#define NRF_EGU130_S_BASE                 0x5F92D000UL
#define NRF_P0_NS_BASE                    0x4F938000UL
#define NRF_P1_NS_BASE                    0x4F938200UL
#define NRF_P2_NS_BASE                    0x4F938400UL
#define NRF_P5_NS_BASE                    0x4F938A00UL
#define NRF_P0_S_BASE                     0x5F938000UL
#define NRF_P1_S_BASE                     0x5F938200UL
#define NRF_P2_S_BASE                     0x5F938400UL
#define NRF_P5_S_BASE                     0x5F938A00UL
#define NRF_P10_NS_BASE                   0x4F939400UL
#define NRF_P12_NS_BASE                   0x4F939800UL
#define NRF_P10_S_BASE                    0x5F939400UL
#define NRF_P12_S_BASE                    0x5F939800UL
#define NRF_DPPIC131_NS_BASE              0x4F981000UL
#define NRF_DPPIC131_S_BASE               0x5F981000UL
#define NRF_SAADC_NS_BASE                 0x4F982000UL
#define NRF_SAADC_S_BASE                  0x5F982000UL
#define NRF_TEMP_NS_BASE                  0x4F984000UL
#define NRF_TEMP_S_BASE                   0x5F984000UL
#define NRF_NFCT_NS_BASE                  0x4F985000UL
#define NRF_NFCT_S_BASE                   0x5F985000UL
#define NRF_DPPIC132_NS_BASE              0x4F991000UL
#define NRF_DPPIC132_S_BASE               0x5F991000UL
#define NRF_PDM_NS_BASE                   0x4F993000UL
#define NRF_PDM_S_BASE                    0x5F993000UL
#define NRF_DPPIC133_NS_BASE              0x4F9A1000UL
#define NRF_DPPIC133_S_BASE               0x5F9A1000UL
#define NRF_TIMER130_NS_BASE              0x4F9A2000UL
#define NRF_TIMER130_S_BASE               0x5F9A2000UL
#define NRF_TIMER131_NS_BASE              0x4F9A3000UL
#define NRF_TIMER131_S_BASE               0x5F9A3000UL
#define NRF_PWM130_NS_BASE                0x4F9A4000UL
#define NRF_PWM130_S_BASE                 0x5F9A4000UL
#define NRF_SPIM130_NS_BASE               0x4F9A5000UL
#define NRF_SPIS130_NS_BASE               0x4F9A5000UL
#define NRF_TWIM130_NS_BASE               0x4F9A5000UL
#define NRF_TWIS130_NS_BASE               0x4F9A5000UL
#define NRF_UARTE130_NS_BASE              0x4F9A5000UL
#define NRF_SPIM130_S_BASE                0x5F9A5000UL
#define NRF_SPIS130_S_BASE                0x5F9A5000UL
#define NRF_TWIM130_S_BASE                0x5F9A5000UL
#define NRF_TWIS130_S_BASE                0x5F9A5000UL
#define NRF_UARTE130_S_BASE               0x5F9A5000UL
#define NRF_SPIM131_NS_BASE               0x4F9A6000UL
#define NRF_SPIS131_NS_BASE               0x4F9A6000UL
#define NRF_TWIM131_NS_BASE               0x4F9A6000UL
#define NRF_TWIS131_NS_BASE               0x4F9A6000UL
#define NRF_UARTE131_NS_BASE              0x4F9A6000UL
#define NRF_SPIM131_S_BASE                0x5F9A6000UL
#define NRF_SPIS131_S_BASE                0x5F9A6000UL
#define NRF_TWIM131_S_BASE                0x5F9A6000UL
#define NRF_TWIS131_S_BASE                0x5F9A6000UL
#define NRF_UARTE131_S_BASE               0x5F9A6000UL
#define NRF_DPPIC134_NS_BASE              0x4F9B1000UL
#define NRF_DPPIC134_S_BASE               0x5F9B1000UL
#define NRF_TIMER132_NS_BASE              0x4F9B2000UL
#define NRF_TIMER132_S_BASE               0x5F9B2000UL
#define NRF_TIMER133_NS_BASE              0x4F9B3000UL
#define NRF_TIMER133_S_BASE               0x5F9B3000UL
#define NRF_PWM131_NS_BASE                0x4F9B4000UL
#define NRF_PWM131_S_BASE                 0x5F9B4000UL
#define NRF_SPIM132_NS_BASE               0x4F9B5000UL
#define NRF_SPIS132_NS_BASE               0x4F9B5000UL
#define NRF_TWIM132_NS_BASE               0x4F9B5000UL
#define NRF_TWIS132_NS_BASE               0x4F9B5000UL
#define NRF_UARTE132_NS_BASE              0x4F9B5000UL
#define NRF_SPIM132_S_BASE                0x5F9B5000UL
#define NRF_SPIS132_S_BASE                0x5F9B5000UL
#define NRF_TWIM132_S_BASE                0x5F9B5000UL
#define NRF_TWIS132_S_BASE                0x5F9B5000UL
#define NRF_UARTE132_S_BASE               0x5F9B5000UL
#define NRF_SPIM133_NS_BASE               0x4F9B6000UL
#define NRF_SPIS133_NS_BASE               0x4F9B6000UL
#define NRF_TWIM133_NS_BASE               0x4F9B6000UL
#define NRF_TWIS133_NS_BASE               0x4F9B6000UL
#define NRF_UARTE133_NS_BASE              0x4F9B6000UL
#define NRF_SPIM133_S_BASE                0x5F9B6000UL
#define NRF_SPIS133_S_BASE                0x5F9B6000UL
#define NRF_TWIM133_S_BASE                0x5F9B6000UL
#define NRF_TWIS133_S_BASE                0x5F9B6000UL
#define NRF_UARTE133_S_BASE               0x5F9B6000UL

/* =========================================================================================================================== */
/* ================                                  Peripheral Declaration                                  ================ */
/* =========================================================================================================================== */

#define NRF_MICR_NS                       ((NRF_MICR_Type*)                     NRF_MICR_NS_BASE)
#define NRF_FICR_NS                       ((NRF_FICR_Type*)                     NRF_FICR_NS_BASE)
#define NRF_STMDATA_NS                    ((NRF_STMDATA_Type*)                  NRF_STMDATA_NS_BASE)
#define NRF_STMDATA_S                     ((NRF_STMDATA_Type*)                  NRF_STMDATA_S_BASE)
#define NRF_TDDCONF_NS                    ((NRF_TDDCONF_Type*)                  NRF_TDDCONF_NS_BASE)
#define NRF_TDDCONF_S                     ((NRF_TDDCONF_Type*)                  NRF_TDDCONF_S_BASE)
#define NRF_STM_NS                        ((NRF_STM_Type*)                      NRF_STM_NS_BASE)
#define NRF_TPIU_NS                       ((NRF_TPIU_Type*)                     NRF_TPIU_NS_BASE)
#define NRF_ETB_NS                        ((NRF_ETB_Type*)                      NRF_ETB_NS_BASE)
#define NRF_CTI210_NS                     ((NRF_CTI_Type*)                      NRF_CTI210_NS_BASE)
#define NRF_CTI211_NS                     ((NRF_CTI_Type*)                      NRF_CTI211_NS_BASE)
#define NRF_ATBREPLICATOR210_NS           ((NRF_ATBREPLICATOR_Type*)            NRF_ATBREPLICATOR210_NS_BASE)
#define NRF_ATBREPLICATOR211_NS           ((NRF_ATBREPLICATOR_Type*)            NRF_ATBREPLICATOR211_NS_BASE)
#define NRF_ATBREPLICATOR212_NS           ((NRF_ATBREPLICATOR_Type*)            NRF_ATBREPLICATOR212_NS_BASE)
#define NRF_ATBREPLICATOR213_NS           ((NRF_ATBREPLICATOR_Type*)            NRF_ATBREPLICATOR213_NS_BASE)
#define NRF_ATBFUNNEL210_NS               ((NRF_ATBFUNNEL_Type*)                NRF_ATBFUNNEL210_NS_BASE)
#define NRF_ATBFUNNEL211_NS               ((NRF_ATBFUNNEL_Type*)                NRF_ATBFUNNEL211_NS_BASE)
#define NRF_ATBFUNNEL212_NS               ((NRF_ATBFUNNEL_Type*)                NRF_ATBFUNNEL212_NS_BASE)
#define NRF_ATBFUNNEL213_NS               ((NRF_ATBFUNNEL_Type*)                NRF_ATBFUNNEL213_NS_BASE)
#define NRF_GPR_NS                        ((NRF_GPR_Type*)                      NRF_GPR_NS_BASE)
#define NRF_GPIOTE130_NS                  ((NRF_GPIOTE_Type*)                   NRF_GPIOTE130_NS_BASE)
#define NRF_GPIOTE130_S                   ((NRF_GPIOTE_Type*)                   NRF_GPIOTE130_S_BASE)
#define NRF_GRTC_NS                       ((NRF_GRTC_Type*)                     NRF_GRTC_NS_BASE)
#define NRF_GRTC_S                        ((NRF_GRTC_Type*)                     NRF_GRTC_S_BASE)
#define NRF_IPCT120_NS                    ((NRF_IPCT_Type*)                     NRF_IPCT120_NS_BASE)
#define NRF_IPCT120_S                     ((NRF_IPCT_Type*)                     NRF_IPCT120_S_BASE)
#define NRF_MUTEX120_NS                   ((NRF_MUTEX_Type*)                    NRF_MUTEX120_NS_BASE)
#define NRF_VPR121_NS                     ((NRF_VPR_Type*)                      NRF_VPR121_NS_BASE)
#define NRF_VPR121_S                      ((NRF_VPR_Type*)                      NRF_VPR121_S_BASE)
#define NRF_DPPIC120_NS                   ((NRF_DPPIC_Type*)                    NRF_DPPIC120_NS_BASE)
#define NRF_DPPIC120_S                    ((NRF_DPPIC_Type*)                    NRF_DPPIC120_S_BASE)
#define NRF_TIMER120_NS                   ((NRF_TIMER_Type*)                    NRF_TIMER120_NS_BASE)
#define NRF_TIMER120_S                    ((NRF_TIMER_Type*)                    NRF_TIMER120_S_BASE)
#define NRF_SPIM120_NS                    ((NRF_SPIM_Type*)                     NRF_SPIM120_NS_BASE)
#define NRF_SPIS120_NS                    ((NRF_SPIS_Type*)                     NRF_SPIS120_NS_BASE)
#define NRF_UARTE120_NS                   ((NRF_UARTE_Type*)                    NRF_UARTE120_NS_BASE)
#define NRF_SPIM120_S                     ((NRF_SPIM_Type*)                     NRF_SPIM120_S_BASE)
#define NRF_SPIS120_S                     ((NRF_SPIS_Type*)                     NRF_SPIS120_S_BASE)
#define NRF_UARTE120_S                    ((NRF_UARTE_Type*)                    NRF_UARTE120_S_BASE)
#define NRF_VPR130_NS                     ((NRF_VPR_Type*)                      NRF_VPR130_NS_BASE)
#define NRF_VPR130_S                      ((NRF_VPR_Type*)                      NRF_VPR130_S_BASE)
#define NRF_AHBBUFFER130_NS               ((NRF_AHBBUFFER_Type*)                NRF_AHBBUFFER130_NS_BASE)
#define NRF_AHBBUFFER130_S                ((NRF_AHBBUFFER_Type*)                NRF_AHBBUFFER130_S_BASE)
#define NRF_IPCT130_NS                    ((NRF_IPCT_Type*)                     NRF_IPCT130_NS_BASE)
#define NRF_IPCT130_S                     ((NRF_IPCT_Type*)                     NRF_IPCT130_S_BASE)
#define NRF_DPPIC130_NS                   ((NRF_DPPIC_Type*)                    NRF_DPPIC130_NS_BASE)
#define NRF_DPPIC130_S                    ((NRF_DPPIC_Type*)                    NRF_DPPIC130_S_BASE)
#define NRF_MUTEX130_NS                   ((NRF_MUTEX_Type*)                    NRF_MUTEX130_NS_BASE)
#define NRF_RTC130_NS                     ((NRF_RTC_Type*)                      NRF_RTC130_NS_BASE)
#define NRF_RTC130_S                      ((NRF_RTC_Type*)                      NRF_RTC130_S_BASE)
#define NRF_WDT131_NS                     ((NRF_WDT_Type*)                      NRF_WDT131_NS_BASE)
#define NRF_WDT131_S                      ((NRF_WDT_Type*)                      NRF_WDT131_S_BASE)
#define NRF_WDT132_NS                     ((NRF_WDT_Type*)                      NRF_WDT132_NS_BASE)
#define NRF_WDT132_S                      ((NRF_WDT_Type*)                      NRF_WDT132_S_BASE)
#define NRF_EGU130_NS                     ((NRF_EGU_Type*)                      NRF_EGU130_NS_BASE)
#define NRF_EGU130_S                      ((NRF_EGU_Type*)                      NRF_EGU130_S_BASE)
#define NRF_P0_NS                         ((NRF_GPIO_Type*)                     NRF_P0_NS_BASE)
#define NRF_P1_NS                         ((NRF_GPIO_Type*)                     NRF_P1_NS_BASE)
#define NRF_P2_NS                         ((NRF_GPIO_Type*)                     NRF_P2_NS_BASE)
#define NRF_P5_NS                         ((NRF_GPIO_Type*)                     NRF_P5_NS_BASE)
#define NRF_P0_S                          ((NRF_GPIO_Type*)                     NRF_P0_S_BASE)
#define NRF_P1_S                          ((NRF_GPIO_Type*)                     NRF_P1_S_BASE)
#define NRF_P2_S                          ((NRF_GPIO_Type*)                     NRF_P2_S_BASE)
#define NRF_P5_S                          ((NRF_GPIO_Type*)                     NRF_P5_S_BASE)
#define NRF_P10_NS                        ((NRF_GPIO_Type*)                     NRF_P10_NS_BASE)
#define NRF_P12_NS                        ((NRF_GPIO_Type*)                     NRF_P12_NS_BASE)
#define NRF_P10_S                         ((NRF_GPIO_Type*)                     NRF_P10_S_BASE)
#define NRF_P12_S                         ((NRF_GPIO_Type*)                     NRF_P12_S_BASE)
#define NRF_DPPIC131_NS                   ((NRF_DPPIC_Type*)                    NRF_DPPIC131_NS_BASE)
#define NRF_DPPIC131_S                    ((NRF_DPPIC_Type*)                    NRF_DPPIC131_S_BASE)
#define NRF_SAADC_NS                      ((NRF_SAADC_Type*)                    NRF_SAADC_NS_BASE)
#define NRF_SAADC_S                       ((NRF_SAADC_Type*)                    NRF_SAADC_S_BASE)
#define NRF_TEMP_NS                       ((NRF_TEMP_Type*)                     NRF_TEMP_NS_BASE)
#define NRF_TEMP_S                        ((NRF_TEMP_Type*)                     NRF_TEMP_S_BASE)
#define NRF_NFCT_NS                       ((NRF_NFCT_Type*)                     NRF_NFCT_NS_BASE)
#define NRF_NFCT_S                        ((NRF_NFCT_Type*)                     NRF_NFCT_S_BASE)
#define NRF_DPPIC132_NS                   ((NRF_DPPIC_Type*)                    NRF_DPPIC132_NS_BASE)
#define NRF_DPPIC132_S                    ((NRF_DPPIC_Type*)                    NRF_DPPIC132_S_BASE)
#define NRF_PDM_NS                        ((NRF_PDM_Type*)                      NRF_PDM_NS_BASE)
#define NRF_PDM_S                         ((NRF_PDM_Type*)                      NRF_PDM_S_BASE)
#define NRF_DPPIC133_NS                   ((NRF_DPPIC_Type*)                    NRF_DPPIC133_NS_BASE)
#define NRF_DPPIC133_S                    ((NRF_DPPIC_Type*)                    NRF_DPPIC133_S_BASE)
#define NRF_TIMER130_NS                   ((NRF_TIMER_Type*)                    NRF_TIMER130_NS_BASE)
#define NRF_TIMER130_S                    ((NRF_TIMER_Type*)                    NRF_TIMER130_S_BASE)
#define NRF_TIMER131_NS                   ((NRF_TIMER_Type*)                    NRF_TIMER131_NS_BASE)
#define NRF_TIMER131_S                    ((NRF_TIMER_Type*)                    NRF_TIMER131_S_BASE)
#define NRF_PWM130_NS                     ((NRF_PWM_Type*)                      NRF_PWM130_NS_BASE)
#define NRF_PWM130_S                      ((NRF_PWM_Type*)                      NRF_PWM130_S_BASE)
#define NRF_SPIM130_NS                    ((NRF_SPIM_Type*)                     NRF_SPIM130_NS_BASE)
#define NRF_SPIS130_NS                    ((NRF_SPIS_Type*)                     NRF_SPIS130_NS_BASE)
#define NRF_TWIM130_NS                    ((NRF_TWIM_Type*)                     NRF_TWIM130_NS_BASE)
#define NRF_TWIS130_NS                    ((NRF_TWIS_Type*)                     NRF_TWIS130_NS_BASE)
#define NRF_UARTE130_NS                   ((NRF_UARTE_Type*)                    NRF_UARTE130_NS_BASE)
#define NRF_SPIM130_S                     ((NRF_SPIM_Type*)                     NRF_SPIM130_S_BASE)
#define NRF_SPIS130_S                     ((NRF_SPIS_Type*)                     NRF_SPIS130_S_BASE)
#define NRF_TWIM130_S                     ((NRF_TWIM_Type*)                     NRF_TWIM130_S_BASE)
#define NRF_TWIS130_S                     ((NRF_TWIS_Type*)                     NRF_TWIS130_S_BASE)
#define NRF_UARTE130_S                    ((NRF_UARTE_Type*)                    NRF_UARTE130_S_BASE)
#define NRF_SPIM131_NS                    ((NRF_SPIM_Type*)                     NRF_SPIM131_NS_BASE)
#define NRF_SPIS131_NS                    ((NRF_SPIS_Type*)                     NRF_SPIS131_NS_BASE)
#define NRF_TWIM131_NS                    ((NRF_TWIM_Type*)                     NRF_TWIM131_NS_BASE)
#define NRF_TWIS131_NS                    ((NRF_TWIS_Type*)                     NRF_TWIS131_NS_BASE)
#define NRF_UARTE131_NS                   ((NRF_UARTE_Type*)                    NRF_UARTE131_NS_BASE)
#define NRF_SPIM131_S                     ((NRF_SPIM_Type*)                     NRF_SPIM131_S_BASE)
#define NRF_SPIS131_S                     ((NRF_SPIS_Type*)                     NRF_SPIS131_S_BASE)
#define NRF_TWIM131_S                     ((NRF_TWIM_Type*)                     NRF_TWIM131_S_BASE)
#define NRF_TWIS131_S                     ((NRF_TWIS_Type*)                     NRF_TWIS131_S_BASE)
#define NRF_UARTE131_S                    ((NRF_UARTE_Type*)                    NRF_UARTE131_S_BASE)
#define NRF_DPPIC134_NS                   ((NRF_DPPIC_Type*)                    NRF_DPPIC134_NS_BASE)
#define NRF_DPPIC134_S                    ((NRF_DPPIC_Type*)                    NRF_DPPIC134_S_BASE)
#define NRF_TIMER132_NS                   ((NRF_TIMER_Type*)                    NRF_TIMER132_NS_BASE)
#define NRF_TIMER132_S                    ((NRF_TIMER_Type*)                    NRF_TIMER132_S_BASE)
#define NRF_TIMER133_NS                   ((NRF_TIMER_Type*)                    NRF_TIMER133_NS_BASE)
#define NRF_TIMER133_S                    ((NRF_TIMER_Type*)                    NRF_TIMER133_S_BASE)
#define NRF_PWM131_NS                     ((NRF_PWM_Type*)                      NRF_PWM131_NS_BASE)
#define NRF_PWM131_S                      ((NRF_PWM_Type*)                      NRF_PWM131_S_BASE)
#define NRF_SPIM132_NS                    ((NRF_SPIM_Type*)                     NRF_SPIM132_NS_BASE)
#define NRF_SPIS132_NS                    ((NRF_SPIS_Type*)                     NRF_SPIS132_NS_BASE)
#define NRF_TWIM132_NS                    ((NRF_TWIM_Type*)                     NRF_TWIM132_NS_BASE)
#define NRF_TWIS132_NS                    ((NRF_TWIS_Type*)                     NRF_TWIS132_NS_BASE)
#define NRF_UARTE132_NS                   ((NRF_UARTE_Type*)                    NRF_UARTE132_NS_BASE)
#define NRF_SPIM132_S                     ((NRF_SPIM_Type*)                     NRF_SPIM132_S_BASE)
#define NRF_SPIS132_S                     ((NRF_SPIS_Type*)                     NRF_SPIS132_S_BASE)
#define NRF_TWIM132_S                     ((NRF_TWIM_Type*)                     NRF_TWIM132_S_BASE)
#define NRF_TWIS132_S                     ((NRF_TWIS_Type*)                     NRF_TWIS132_S_BASE)
#define NRF_UARTE132_S                    ((NRF_UARTE_Type*)                    NRF_UARTE132_S_BASE)
#define NRF_SPIM133_NS                    ((NRF_SPIM_Type*)                     NRF_SPIM133_NS_BASE)
#define NRF_SPIS133_NS                    ((NRF_SPIS_Type*)                     NRF_SPIS133_NS_BASE)
#define NRF_TWIM133_NS                    ((NRF_TWIM_Type*)                     NRF_TWIM133_NS_BASE)
#define NRF_TWIS133_NS                    ((NRF_TWIS_Type*)                     NRF_TWIS133_NS_BASE)
#define NRF_UARTE133_NS                   ((NRF_UARTE_Type*)                    NRF_UARTE133_NS_BASE)
#define NRF_SPIM133_S                     ((NRF_SPIM_Type*)                     NRF_SPIM133_S_BASE)
#define NRF_SPIS133_S                     ((NRF_SPIS_Type*)                     NRF_SPIS133_S_BASE)
#define NRF_TWIM133_S                     ((NRF_TWIM_Type*)                     NRF_TWIM133_S_BASE)
#define NRF_TWIS133_S                     ((NRF_TWIS_Type*)                     NRF_TWIS133_S_BASE)
#define NRF_UARTE133_S                    ((NRF_UARTE_Type*)                    NRF_UARTE133_S_BASE)

/* =========================================================================================================================== */
/* ================                                    TrustZone Remapping                                    ================ */
/* =========================================================================================================================== */

#ifdef NRF_TRUSTZONE_NONSECURE                       /*!< Remap NRF_X_NS instances to NRF_X symbol for ease of use.            */
  #define NRF_MICR                                NRF_MICR_NS
  #define NRF_FICR                                NRF_FICR_NS
  #define NRF_STMDATA                             NRF_STMDATA_NS
  #define NRF_TDDCONF                             NRF_TDDCONF_NS
  #define NRF_STM                                 NRF_STM_NS
  #define NRF_TPIU                                NRF_TPIU_NS
  #define NRF_ETB                                 NRF_ETB_NS
  #define NRF_CTI210                              NRF_CTI210_NS
  #define NRF_CTI211                              NRF_CTI211_NS
  #define NRF_ATBREPLICATOR210                    NRF_ATBREPLICATOR210_NS
  #define NRF_ATBREPLICATOR211                    NRF_ATBREPLICATOR211_NS
  #define NRF_ATBREPLICATOR212                    NRF_ATBREPLICATOR212_NS
  #define NRF_ATBREPLICATOR213                    NRF_ATBREPLICATOR213_NS
  #define NRF_ATBFUNNEL210                        NRF_ATBFUNNEL210_NS
  #define NRF_ATBFUNNEL211                        NRF_ATBFUNNEL211_NS
  #define NRF_ATBFUNNEL212                        NRF_ATBFUNNEL212_NS
  #define NRF_ATBFUNNEL213                        NRF_ATBFUNNEL213_NS
  #define NRF_GPR                                 NRF_GPR_NS
  #define NRF_GPIOTE130                           NRF_GPIOTE130_NS
  #define NRF_GRTC                                NRF_GRTC_NS
  #define NRF_IPCT120                             NRF_IPCT120_NS
  #define NRF_MUTEX120                            NRF_MUTEX120_NS
  #define NRF_VPR121                              NRF_VPR121_NS
  #define NRF_DPPIC120                            NRF_DPPIC120_NS
  #define NRF_TIMER120                            NRF_TIMER120_NS
  #define NRF_SPIM120                             NRF_SPIM120_NS
  #define NRF_SPIS120                             NRF_SPIS120_NS
  #define NRF_UARTE120                            NRF_UARTE120_NS
  #define NRF_VPR130                              NRF_VPR130_NS
  #define NRF_AHBBUFFER130                        NRF_AHBBUFFER130_NS
  #define NRF_IPCT130                             NRF_IPCT130_NS
  #define NRF_DPPIC130                            NRF_DPPIC130_NS
  #define NRF_MUTEX130                            NRF_MUTEX130_NS
  #define NRF_RTC130                              NRF_RTC130_NS
  #define NRF_WDT131                              NRF_WDT131_NS
  #define NRF_WDT132                              NRF_WDT132_NS
  #define NRF_EGU130                              NRF_EGU130_NS
  #define NRF_P0                                  NRF_P0_NS
  #define NRF_P1                                  NRF_P1_NS
  #define NRF_P2                                  NRF_P2_NS
  #define NRF_P5                                  NRF_P5_NS
  #define NRF_P10                                 NRF_P10_NS
  #define NRF_P12                                 NRF_P12_NS
  #define NRF_DPPIC131                            NRF_DPPIC131_NS
  #define NRF_SAADC                               NRF_SAADC_NS
  #define NRF_TEMP                                NRF_TEMP_NS
  #define NRF_NFCT                                NRF_NFCT_NS
  #define NRF_DPPIC132                            NRF_DPPIC132_NS
  #define NRF_PDM                                 NRF_PDM_NS
  #define NRF_DPPIC133                            NRF_DPPIC133_NS
  #define NRF_TIMER130                            NRF_TIMER130_NS
  #define NRF_TIMER131                            NRF_TIMER131_NS
  #define NRF_PWM130                              NRF_PWM130_NS
  #define NRF_SPIM130                             NRF_SPIM130_NS
  #define NRF_SPIS130                             NRF_SPIS130_NS
  #define NRF_TWIM130                             NRF_TWIM130_NS
  #define NRF_TWIS130                             NRF_TWIS130_NS
  #define NRF_UARTE130                            NRF_UARTE130_NS
  #define NRF_SPIM131                             NRF_SPIM131_NS
  #define NRF_SPIS131                             NRF_SPIS131_NS
  #define NRF_TWIM131                             NRF_TWIM131_NS
  #define NRF_TWIS131                             NRF_TWIS131_NS
  #define NRF_UARTE131                            NRF_UARTE131_NS
  #define NRF_DPPIC134                            NRF_DPPIC134_NS
  #define NRF_TIMER132                            NRF_TIMER132_NS
  #define NRF_TIMER133                            NRF_TIMER133_NS
  #define NRF_PWM131                              NRF_PWM131_NS
  #define NRF_SPIM132                             NRF_SPIM132_NS
  #define NRF_SPIS132                             NRF_SPIS132_NS
  #define NRF_TWIM132                             NRF_TWIM132_NS
  #define NRF_TWIS132                             NRF_TWIS132_NS
  #define NRF_UARTE132                            NRF_UARTE132_NS
  #define NRF_SPIM133                             NRF_SPIM133_NS
  #define NRF_SPIS133                             NRF_SPIS133_NS
  #define NRF_TWIM133                             NRF_TWIM133_NS
  #define NRF_TWIS133                             NRF_TWIS133_NS
  #define NRF_UARTE133                            NRF_UARTE133_NS
#else                                                /*!< Remap NRF_X_S instances to NRF_X symbol for ease of use.             */
  #define NRF_MICR                                NRF_MICR_NS
  #define NRF_FICR                                NRF_FICR_NS
  #define NRF_STMDATA                             NRF_STMDATA_S
  #define NRF_TDDCONF                             NRF_TDDCONF_S
  #define NRF_STM                                 NRF_STM_NS
  #define NRF_TPIU                                NRF_TPIU_NS
  #define NRF_ETB                                 NRF_ETB_NS
  #define NRF_CTI210                              NRF_CTI210_NS
  #define NRF_CTI211                              NRF_CTI211_NS
  #define NRF_ATBREPLICATOR210                    NRF_ATBREPLICATOR210_NS
  #define NRF_ATBREPLICATOR211                    NRF_ATBREPLICATOR211_NS
  #define NRF_ATBREPLICATOR212                    NRF_ATBREPLICATOR212_NS
  #define NRF_ATBREPLICATOR213                    NRF_ATBREPLICATOR213_NS
  #define NRF_ATBFUNNEL210                        NRF_ATBFUNNEL210_NS
  #define NRF_ATBFUNNEL211                        NRF_ATBFUNNEL211_NS
  #define NRF_ATBFUNNEL212                        NRF_ATBFUNNEL212_NS
  #define NRF_ATBFUNNEL213                        NRF_ATBFUNNEL213_NS
  #define NRF_GPR                                 NRF_GPR_NS
  #define NRF_GPIOTE130                           NRF_GPIOTE130_S
  #define NRF_GRTC                                NRF_GRTC_S
  #define NRF_IPCT120                             NRF_IPCT120_S
  #define NRF_MUTEX120                            NRF_MUTEX120_NS
  #define NRF_VPR121                              NRF_VPR121_S
  #define NRF_DPPIC120                            NRF_DPPIC120_S
  #define NRF_TIMER120                            NRF_TIMER120_S
  #define NRF_SPIM120                             NRF_SPIM120_S
  #define NRF_SPIS120                             NRF_SPIS120_S
  #define NRF_UARTE120                            NRF_UARTE120_S
  #define NRF_VPR130                              NRF_VPR130_S
  #define NRF_AHBBUFFER130                        NRF_AHBBUFFER130_S
  #define NRF_IPCT130                             NRF_IPCT130_S
  #define NRF_DPPIC130                            NRF_DPPIC130_S
  #define NRF_MUTEX130                            NRF_MUTEX130_NS
  #define NRF_RTC130                              NRF_RTC130_S
  #define NRF_WDT131                              NRF_WDT131_S
  #define NRF_WDT132                              NRF_WDT132_S
  #define NRF_EGU130                              NRF_EGU130_S
  #define NRF_P0                                  NRF_P0_S
  #define NRF_P1                                  NRF_P1_S
  #define NRF_P2                                  NRF_P2_S
  #define NRF_P5                                  NRF_P5_S
  #define NRF_P10                                 NRF_P10_S
  #define NRF_P12                                 NRF_P12_S
  #define NRF_DPPIC131                            NRF_DPPIC131_S
  #define NRF_SAADC                               NRF_SAADC_S
  #define NRF_TEMP                                NRF_TEMP_S
  #define NRF_NFCT                                NRF_NFCT_S
  #define NRF_DPPIC132                            NRF_DPPIC132_S
  #define NRF_PDM                                 NRF_PDM_S
  #define NRF_DPPIC133                            NRF_DPPIC133_S
  #define NRF_TIMER130                            NRF_TIMER130_S
  #define NRF_TIMER131                            NRF_TIMER131_S
  #define NRF_PWM130                              NRF_PWM130_S
  #define NRF_SPIM130                             NRF_SPIM130_S
  #define NRF_SPIS130                             NRF_SPIS130_S
  #define NRF_TWIM130                             NRF_TWIM130_S
  #define NRF_TWIS130                             NRF_TWIS130_S
  #define NRF_UARTE130                            NRF_UARTE130_S
  #define NRF_SPIM131                             NRF_SPIM131_S
  #define NRF_SPIS131                             NRF_SPIS131_S
  #define NRF_TWIM131                             NRF_TWIM131_S
  #define NRF_TWIS131                             NRF_TWIS131_S
  #define NRF_UARTE131                            NRF_UARTE131_S
  #define NRF_DPPIC134                            NRF_DPPIC134_S
  #define NRF_TIMER132                            NRF_TIMER132_S
  #define NRF_TIMER133                            NRF_TIMER133_S
  #define NRF_PWM131                              NRF_PWM131_S
  #define NRF_SPIM132                             NRF_SPIM132_S
  #define NRF_SPIS132                             NRF_SPIS132_S
  #define NRF_TWIM132                             NRF_TWIM132_S
  #define NRF_TWIS132                             NRF_TWIS132_S
  #define NRF_UARTE132                            NRF_UARTE132_S
  #define NRF_SPIM133                             NRF_SPIM133_S
  #define NRF_SPIS133                             NRF_SPIS133_S
  #define NRF_TWIM133                             NRF_TWIM133_S
  #define NRF_TWIS133                             NRF_TWIS133_S
  #define NRF_UARTE133                            NRF_UARTE133_S
#endif                                               /*!< NRF_TRUSTZONE_NONSECURE                                              */

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
#endif /* NRF9220_GLOBAL_H */

