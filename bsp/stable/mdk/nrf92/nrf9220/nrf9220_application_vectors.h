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

#ifndef NRF_DEVICE_VECTORS_H_
#define NRF_DEVICE_VECTORS_H_

/*---------------------------------------------------------------------------
  Exception / Interrupt Handler
 *---------------------------------------------------------------------------*/
/* Exceptions */
void Reset_Handler                                               (void);
__WEAK void NMI_Handler(void)
{
    while(1);
}

__WEAK void HardFault_Handler(void)
{
    while(1);
}

__WEAK void MemoryManagement_Handler(void)
{
    while(1);
}

__WEAK void BusFault_Handler(void)
{
    while(1);
}

__WEAK void UsageFault_Handler(void)
{
    while(1);
}

__WEAK void SecureFault_Handler(void)
{
    while(1);
}

__WEAK void SVC_Handler(void)
{
    while(1);
}

__WEAK void DebugMon_Handler(void)
{
    while(1);
}

__WEAK void PendSV_Handler(void)
{
    while(1);
}

__WEAK void SysTick_Handler(void)
{
    while(1);
}

/* Device specific interrupt handlers */
 __HANDLER("Default_Handler") void SPU000_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void MPC_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void CPUC_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void MVDMA_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SPU010_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void WDT010_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void WDT011_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void IPCT_0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void IPCT_1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void CTI_0_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void CTI_1_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SWI0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI1_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI2_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI3_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI4_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI5_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI6_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI7_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void BELLBOARD_0_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void BELLBOARD_1_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void BELLBOARD_2_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void BELLBOARD_3_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void GPIOTE130_0_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void GPIOTE130_1_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void GRTC_0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void AXONS_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void IPCT120_0_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPR121_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER120_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void SPIM120_SPIS120_UARTE120_IRQHandler                         (void);
 __HANDLER("Default_Handler") void VPR130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void AHBBUFFER130_IRQHandler                                     (void);
 __HANDLER("Default_Handler") void IPCT130_0_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void RTC130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void WDT131_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void WDT132_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void EGU130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void AUDIOPLL_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void SAADC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void TEMP_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void NFCT_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PDM_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void TIMER130_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void TIMER131_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void PWM130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SERIAL0_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SERIAL1_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void TIMER132_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void TIMER133_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void PWM131_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SERIAL2_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SERIAL3_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SERIAL4_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SERIAL5_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SERIAL6_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SERIAL7_IRQHandler                                          (void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

__VECTOR_TABLE_ATTRIBUTE const VECTOR_TABLE_Type __VECTOR_TABLE[] = {
    (VECTOR_TABLE_Type)(__STACK_BASE),
/* Exceptions */
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemoryManagement_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    SecureFault_Handler,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
/* Device specific interrupt handlers */
    SPU000_IRQHandler,
    MPC_IRQHandler,
    CPUC_IRQHandler,
    MVDMA_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SPU010_IRQHandler,
    0,
    0,
    0,
    WDT010_IRQHandler,
    WDT011_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    IPCT_0_IRQHandler,
    IPCT_1_IRQHandler,
    CTI_0_IRQHandler,
    CTI_1_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SWI0_IRQHandler,
    SWI1_IRQHandler,
    SWI2_IRQHandler,
    SWI3_IRQHandler,
    SWI4_IRQHandler,
    SWI5_IRQHandler,
    SWI6_IRQHandler,
    SWI7_IRQHandler,
    BELLBOARD_0_IRQHandler,
    BELLBOARD_1_IRQHandler,
    BELLBOARD_2_IRQHandler,
    BELLBOARD_3_IRQHandler,
    0,
    0,
    0,
    0,
    GPIOTE130_0_IRQHandler,
    GPIOTE130_1_IRQHandler,
    0,
    0,
    GRTC_0_IRQHandler,
    GRTC_1_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    AXONS_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    IPCT120_0_IRQHandler,
    0,
    0,
    VPR121_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    TIMER120_IRQHandler,
    0,
    0,
    0,
    SPIM120_SPIS120_UARTE120_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    VPR130_IRQHandler,
    0,
    AHBBUFFER130_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    IPCT130_0_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    RTC130_IRQHandler,
    0,
    0,
    WDT131_IRQHandler,
    WDT132_IRQHandler,
    EGU130_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    AUDIOPLL_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SAADC_IRQHandler,
    0,
    TEMP_IRQHandler,
    NFCT_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    PDM_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    TIMER130_IRQHandler,
    TIMER131_IRQHandler,
    PWM130_IRQHandler,
    SERIAL0_IRQHandler,
    SERIAL1_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    TIMER132_IRQHandler,
    TIMER133_IRQHandler,
    PWM131_IRQHandler,
    SERIAL2_IRQHandler,
    SERIAL3_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SERIAL4_IRQHandler,
    SERIAL5_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SERIAL6_IRQHandler,
    SERIAL7_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

__STATIC_FORCEINLINE void NRFPreInit()
{
}

#endif
