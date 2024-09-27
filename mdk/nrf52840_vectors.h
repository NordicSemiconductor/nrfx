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
 __HANDLER("Default_Handler") void CLOCK_POWER_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void RADIO_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void UART0_UARTE0_IRQHandler                                     (void);
 __HANDLER("Default_Handler") void SPI0_SPIM0_SPIS0_TWI0_TWIM0_TWIS0_IRQHandler                (void);
 __HANDLER("Default_Handler") void SPI1_SPIM1_SPIS1_TWI1_TWIM1_TWIS1_IRQHandler                (void);
 __HANDLER("Default_Handler") void NFCT_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void GPIOTE_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SAADC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void TIMER0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER2_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void RTC0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void TEMP_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void RNG_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void ECB_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void AAR_CCM_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void WDT_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void RTC1_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void QDEC_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void COMP_LPCOMP_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void EGU0_SWI0_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void EGU1_SWI1_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void EGU2_SWI2_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void EGU3_SWI3_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void EGU4_SWI4_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void EGU5_SWI5_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void TIMER3_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER4_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void PWM0_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PDM_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void MWU_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void PWM1_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void PWM2_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SPI2_SPIM2_SPIS2_IRQHandler                                 (void);
 __HANDLER("Default_Handler") void RTC2_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void I2S_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void FPU_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void USBD_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void UARTE1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void QSPI_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void CRYPTOCELL_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void PWM3_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void SPIM3_IRQHandler                                            (void);

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
    CLOCK_POWER_IRQHandler,
    RADIO_IRQHandler,
    UART0_UARTE0_IRQHandler,
    SPI0_SPIM0_SPIS0_TWI0_TWIM0_TWIS0_IRQHandler,
    SPI1_SPIM1_SPIS1_TWI1_TWIM1_TWIS1_IRQHandler,
    NFCT_IRQHandler,
    GPIOTE_IRQHandler,
    SAADC_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    RTC0_IRQHandler,
    TEMP_IRQHandler,
    RNG_IRQHandler,
    ECB_IRQHandler,
    AAR_CCM_IRQHandler,
    WDT_IRQHandler,
    RTC1_IRQHandler,
    QDEC_IRQHandler,
    COMP_LPCOMP_IRQHandler,
    EGU0_SWI0_IRQHandler,
    EGU1_SWI1_IRQHandler,
    EGU2_SWI2_IRQHandler,
    EGU3_SWI3_IRQHandler,
    EGU4_SWI4_IRQHandler,
    EGU5_SWI5_IRQHandler,
    TIMER3_IRQHandler,
    TIMER4_IRQHandler,
    PWM0_IRQHandler,
    PDM_IRQHandler,
    0,
    0,
    MWU_IRQHandler,
    PWM1_IRQHandler,
    PWM2_IRQHandler,
    SPI2_SPIM2_SPIS2_IRQHandler,
    RTC2_IRQHandler,
    I2S_IRQHandler,
    FPU_IRQHandler,
    USBD_IRQHandler,
    UARTE1_IRQHandler,
    QSPI_IRQHandler,
    CRYPTOCELL_IRQHandler,
    0,
    0,
    PWM3_IRQHandler,
    0,
    SPIM3_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
