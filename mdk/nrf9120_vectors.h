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
 __HANDLER("Default_Handler") void SPU_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void CLOCK_POWER_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQHandler                   (void);
 __HANDLER("Default_Handler") void SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQHandler                   (void);
 __HANDLER("Default_Handler") void SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQHandler                   (void);
 __HANDLER("Default_Handler") void SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQHandler                   (void);
 __HANDLER("Default_Handler") void GPIOTE0_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SAADC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void TIMER0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER2_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void RTC0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void RTC1_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void WDT_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void EGU0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void EGU1_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void EGU2_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void EGU3_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void EGU4_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void EGU5_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PWM0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PWM1_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PWM2_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PWM3_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PDM_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void I2S_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void IPC_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void FPU_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void GPIOTE1_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void KMU_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void CRYPTOCELL_IRQHandler                                       (void);

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
    SPU_IRQHandler,
    0,
    CLOCK_POWER_IRQHandler,
    0,
    0,
    SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQHandler,
    SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQHandler,
    SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQHandler,
    SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQHandler,
    0,
    GPIOTE0_IRQHandler,
    SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    0,
    0,
    RTC0_IRQHandler,
    RTC1_IRQHandler,
    0,
    0,
    WDT_IRQHandler,
    0,
    0,
    EGU0_IRQHandler,
    EGU1_IRQHandler,
    EGU2_IRQHandler,
    EGU3_IRQHandler,
    EGU4_IRQHandler,
    EGU5_IRQHandler,
    PWM0_IRQHandler,
    PWM1_IRQHandler,
    PWM2_IRQHandler,
    PWM3_IRQHandler,
    0,
    PDM_IRQHandler,
    0,
    I2S_IRQHandler,
    0,
    IPC_IRQHandler,
    0,
    FPU_IRQHandler,
    0,
    0,
    0,
    0,
    GPIOTE1_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    KMU_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    CRYPTOCELL_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
