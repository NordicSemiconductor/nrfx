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
 __HANDLER("Default_Handler") void POWER_CLOCK_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void RADIO_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void UARTE0_UART0_IRQHandler                                     (void);
 __HANDLER("Default_Handler") void TWIM0_TWIS0_TWI0_IRQHandler                                 (void);
 __HANDLER("Default_Handler") void SPIM0_SPIS0_SPI0_IRQHandler                                 (void);
 __HANDLER("Default_Handler") void GPIOTE_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SAADC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void TIMER0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER2_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void RTC0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void TEMP_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void RNG_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void ECB_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void CCM_AAR_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void WDT_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void RTC1_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void QDEC_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void COMP_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI0_EGU0_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void SWI1_EGU1_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void SWI2_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI3_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI4_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SWI5_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PWM0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PDM_IRQHandler                                              (void);

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
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
/* Device specific interrupt handlers */
    POWER_CLOCK_IRQHandler,
    RADIO_IRQHandler,
    UARTE0_UART0_IRQHandler,
    TWIM0_TWIS0_TWI0_IRQHandler,
    SPIM0_SPIS0_SPI0_IRQHandler,
    0,
    GPIOTE_IRQHandler,
    SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    RTC0_IRQHandler,
    TEMP_IRQHandler,
    RNG_IRQHandler,
    ECB_IRQHandler,
    CCM_AAR_IRQHandler,
    WDT_IRQHandler,
    RTC1_IRQHandler,
    QDEC_IRQHandler,
    COMP_IRQHandler,
    SWI0_EGU0_IRQHandler,
    SWI1_EGU1_IRQHandler,
    SWI2_IRQHandler,
    SWI3_IRQHandler,
    SWI4_IRQHandler,
    SWI5_IRQHandler,
    0,
    0,
    PWM0_IRQHandler,
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
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    /* Workaround for Errata 185 RAM: RAM corruption at extreme corners 
     * found at the Errata document for your device located
     * at https://infocenter.nordicsemi.com/index.jsp */
    
    if (*(volatile uint32_t *)0x10000130ul == 0xAul && *(volatile uint32_t *)0x10000134ul == 0x0ul){ 
        *(volatile uint32_t *)0x40000EE4ul = (*(volatile uint32_t *)0x40000EE4ul & ~0x00000070ul)  | 0x00000040ul;
    }
}

#endif
