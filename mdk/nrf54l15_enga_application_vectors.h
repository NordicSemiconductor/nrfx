/*
Copyright (c) 2010 - 2024, Nordic Semiconductor ASA All rights reserved.

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
 __HANDLER("Default_Handler") void SWI00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SWI01_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SWI02_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SWI03_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SPU00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void MPC00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void AAR00_CCM00_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void ECB00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void CRACEN_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SERIAL00_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void RRAMC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void VPR00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void CTRLAP_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void CM33SS_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER00_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SPU10_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void TIMER10_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void RTC10_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void EGU10_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void RADIO_0_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void RADIO_1_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SPU20_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SERIAL20_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void SERIAL21_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void SERIAL22_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void EGU20_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void TIMER20_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void TIMER21_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void TIMER22_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void TIMER23_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void TIMER24_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void PWM20_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void PWM21_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void PWM22_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SAADC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void NFCT_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void TEMP_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void GPIOTE20_0_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void GPIOTE20_1_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void TAMPC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void I2S20_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void QDEC20_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void QDEC21_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_2_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_3_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SPU30_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SERIAL30_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void RTC30_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void COMP_LPCOMP_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void WDT30_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void WDT31_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void GPIOTE30_0_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void GPIOTE30_1_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void CLOCK_POWER_IRQHandler                                      (void);

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
    SWI00_IRQHandler,
    SWI01_IRQHandler,
    SWI02_IRQHandler,
    SWI03_IRQHandler,
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
    SPU00_IRQHandler,
    MPC00_IRQHandler,
    0,
    0,
    0,
    0,
    AAR00_CCM00_IRQHandler,
    ECB00_IRQHandler,
    CRACEN_IRQHandler,
    0,
    SERIAL00_IRQHandler,
    RRAMC_IRQHandler,
    VPR00_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    CTRLAP_IRQHandler,
    CM33SS_IRQHandler,
    0,
    TIMER00_IRQHandler,
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
    SPU10_IRQHandler,
    0,
    0,
    0,
    0,
    TIMER10_IRQHandler,
    RTC10_IRQHandler,
    EGU10_IRQHandler,
    0,
    0,
    RADIO_0_IRQHandler,
    RADIO_1_IRQHandler,
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
    SPU20_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    SERIAL20_IRQHandler,
    SERIAL21_IRQHandler,
    SERIAL22_IRQHandler,
    EGU20_IRQHandler,
    TIMER20_IRQHandler,
    TIMER21_IRQHandler,
    TIMER22_IRQHandler,
    TIMER23_IRQHandler,
    TIMER24_IRQHandler,
    0,
    0,
    0,
    PWM20_IRQHandler,
    PWM21_IRQHandler,
    PWM22_IRQHandler,
    SAADC_IRQHandler,
    NFCT_IRQHandler,
    TEMP_IRQHandler,
    0,
    0,
    GPIOTE20_0_IRQHandler,
    GPIOTE20_1_IRQHandler,
    TAMPC_IRQHandler,
    I2S20_IRQHandler,
    0,
    0,
    QDEC20_IRQHandler,
    QDEC21_IRQHandler,
    GRTC_0_IRQHandler,
    GRTC_1_IRQHandler,
    GRTC_2_IRQHandler,
    GRTC_3_IRQHandler,
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
    SPU30_IRQHandler,
    0,
    0,
    0,
    SERIAL30_IRQHandler,
    RTC30_IRQHandler,
    COMP_LPCOMP_IRQHandler,
    0,
    WDT30_IRQHandler,
    WDT31_IRQHandler,
    0,
    0,
    GPIOTE30_0_IRQHandler,
    GPIOTE30_1_IRQHandler,
    CLOCK_POWER_IRQHandler,
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
