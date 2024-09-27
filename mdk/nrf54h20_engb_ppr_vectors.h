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
__WEAK void UserSoftware_Handler(void)
{
    while(1);
}

__WEAK void SuperVisorSoftware_Handler(void)
{
    while(1);
}

__WEAK void MachineSoftware_Handler(void)
{
    while(1);
}

__WEAK void UserTimer_Handler(void)
{
    while(1);
}

__WEAK void SuperVisorTimer_Handler(void)
{
    while(1);
}

__WEAK void MachineTimer_Handler(void)
{
    while(1);
}

__WEAK void UserExternal_Handler(void)
{
    while(1);
}

__WEAK void SuperVisorExternal_Handler(void)
{
    while(1);
}

__WEAK void MachineExternal_Handler(void)
{
    while(1);
}

__WEAK void CLICSoftware_Handler(void)
{
    while(1);
}

/* Device specific interrupt handlers */
 __HANDLER("Default_Handler") void VPRCLIC_0_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_1_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_2_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_3_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_4_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_5_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_6_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_7_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_8_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_9_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void VPRCLIC_10_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_11_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_12_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_13_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_14_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_15_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRTIM_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GPIOTE130_0_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void GPIOTE130_1_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void GRTC_0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_2_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TBM_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void USBHS_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void EXMIF_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void IPCT120_0_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void I3C120_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void VPR121_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void CAN120_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void MVDMA120_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void I3C121_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER120_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void TIMER121_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void PWM120_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SPIS120_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SPIM120_UARTE120_IRQHandler                                 (void);
 __HANDLER("Default_Handler") void SPIM121_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void VPR130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void IPCT130_0_IRQHandler                                        (void);
 __HANDLER("Default_Handler") void RTC130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void RTC131_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void WDT131_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void WDT132_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void EGU130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SAADC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void COMP_LPCOMP_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void TEMP_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void NFCT_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void TDM130_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void PDM_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void QDEC130_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void QDEC131_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void TDM131_IRQHandler                                           (void);
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
 __HANDLER("Default_Handler") void TIMER134_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void TIMER135_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void PWM132_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SERIAL4_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void SERIAL5_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void TIMER136_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void TIMER137_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void PWM133_IRQHandler                                           (void);
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
    UserSoftware_Handler,
    SuperVisorSoftware_Handler,
    MachineSoftware_Handler,
    0,
    UserTimer_Handler,
    SuperVisorTimer_Handler,
    0,
    MachineTimer_Handler,
    UserExternal_Handler,
    SuperVisorExternal_Handler,
    0,
    MachineExternal_Handler,
    CLICSoftware_Handler,
    0,
    0,
    0,
/* Device specific interrupt handlers */
    VPRCLIC_0_IRQHandler,
    VPRCLIC_1_IRQHandler,
    VPRCLIC_2_IRQHandler,
    VPRCLIC_3_IRQHandler,
    VPRCLIC_4_IRQHandler,
    VPRCLIC_5_IRQHandler,
    VPRCLIC_6_IRQHandler,
    VPRCLIC_7_IRQHandler,
    VPRCLIC_8_IRQHandler,
    VPRCLIC_9_IRQHandler,
    VPRCLIC_10_IRQHandler,
    VPRCLIC_11_IRQHandler,
    VPRCLIC_12_IRQHandler,
    VPRCLIC_13_IRQHandler,
    VPRCLIC_14_IRQHandler,
    VPRCLIC_15_IRQHandler,
    VPRTIM_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    GRTC_2_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    TBM_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    USBHS_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    EXMIF_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    I3C120_IRQHandler,
    VPR121_IRQHandler,
    0,
    0,
    0,
    CAN120_IRQHandler,
    MVDMA120_IRQHandler,
    0,
    0,
    0,
    0,
    I3C121_IRQHandler,
    0,
    0,
    0,
    TIMER120_IRQHandler,
    TIMER121_IRQHandler,
    PWM120_IRQHandler,
    SPIS120_IRQHandler,
    SPIM120_UARTE120_IRQHandler,
    SPIM121_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    RTC131_IRQHandler,
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
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    COMP_LPCOMP_IRQHandler,
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
    TDM130_IRQHandler,
    PDM_IRQHandler,
    QDEC130_IRQHandler,
    QDEC131_IRQHandler,
    0,
    TDM131_IRQHandler,
    0,
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
    TIMER134_IRQHandler,
    TIMER135_IRQHandler,
    PWM132_IRQHandler,
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
    TIMER136_IRQHandler,
    TIMER137_IRQHandler,
    PWM133_IRQHandler,
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


__attribute__((aligned(16), noreturn)) void Trap_Handler(void)
{
    __UNUSED uint32_t mcause = csr_read(CSR_MCAUSE);
    while(1);
}

__attribute__((used, section(".isr_return"), naked)) void isr_return(void)
{
    asm volatile ("mret");	
}

#endif
