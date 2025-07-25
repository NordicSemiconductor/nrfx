/*
Copyright (c) 2010 - 2025, Nordic Semiconductor ASA All rights reserved.

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
 __HANDLER("Default_Handler") void VPRCLIC_16_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_17_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_18_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_19_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_20_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_21_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRCLIC_22_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void VPRTIM_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SPU00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void MPC00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void AAR00_CCM00_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void ECB00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void VPR00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SERIAL00_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void MRAMC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void CTRLAP_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void CM33SS_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TIMER00_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void EGU00_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void CRACEN_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void USBHS_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void QSPI00_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void QSPI01_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void SERIAL01_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void SPU10_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void TIMER10_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void EGU10_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void RADIO_0_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void RADIO_1_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void IPCT10_0_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void IPCT10_1_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void IPCT10_2_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void IPCT10_3_IRQHandler                                         (void);
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
 __HANDLER("Default_Handler") void PDM20_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void PDM21_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void PWM20_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void PWM21_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void PWM22_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SAADC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void NFCT_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void TEMP_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void GPIOTE20_0_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void GPIOTE20_1_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void QDEC20_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void QDEC21_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_0_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_1_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_2_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_3_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_4_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void GRTC_5_IRQHandler                                           (void);
 __HANDLER("Default_Handler") void TDM_IRQHandler                                              (void);
 __HANDLER("Default_Handler") void SERIAL23_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void SERIAL24_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void TAMPC_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SPU30_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void SERIAL30_IRQHandler                                         (void);
 __HANDLER("Default_Handler") void COMP_LPCOMP_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void WDT30_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void WDT31_IRQHandler                                            (void);
 __HANDLER("Default_Handler") void GPIOTE30_0_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void GPIOTE30_1_IRQHandler                                       (void);
 __HANDLER("Default_Handler") void CLOCK_POWER_IRQHandler                                      (void);
 __HANDLER("Default_Handler") void VREGUSB_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void LFXO_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void LFRC_IRQHandler                                             (void);
 __HANDLER("Default_Handler") void HFXO64M_IRQHandler                                          (void);
 __HANDLER("Default_Handler") void AUDIOPLL_AUDIOPLLM_IRQHandler                               (void);

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
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    VPRCLIC_16_IRQHandler,
    VPRCLIC_17_IRQHandler,
    VPRCLIC_18_IRQHandler,
    VPRCLIC_19_IRQHandler,
    VPRCLIC_20_IRQHandler,
    VPRCLIC_21_IRQHandler,
    VPRCLIC_22_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    SPU00_IRQHandler,
    MPC00_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    AAR00_CCM00_IRQHandler,
    ECB00_IRQHandler,
    VPR00_IRQHandler,
    SERIAL00_IRQHandler,
    MRAMC_IRQHandler,
    0,
    0,
    0,
    CTRLAP_IRQHandler,
    0,
    CM33SS_IRQHandler,
    TIMER00_IRQHandler,
    0,
    0,
    EGU00_IRQHandler,
    CRACEN_IRQHandler,
    USBHS_IRQHandler,
    QSPI00_IRQHandler,
    QSPI01_IRQHandler,
    SERIAL01_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    0,
    EGU10_IRQHandler,
    0,
    0,
    RADIO_0_IRQHandler,
    RADIO_1_IRQHandler,
    0,
    IPCT10_0_IRQHandler,
    IPCT10_1_IRQHandler,
    IPCT10_2_IRQHandler,
    IPCT10_3_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    PDM20_IRQHandler,
    PDM21_IRQHandler,
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
    0,
    0,
    0,
    0,
    QDEC20_IRQHandler,
    QDEC21_IRQHandler,
    GRTC_0_IRQHandler,
    GRTC_1_IRQHandler,
    GRTC_2_IRQHandler,
    GRTC_3_IRQHandler,
    GRTC_4_IRQHandler,
    GRTC_5_IRQHandler,
    TDM_IRQHandler,
    0,
    0,
    0,
    0,
    SERIAL23_IRQHandler,
    SERIAL24_IRQHandler,
    TAMPC_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    0,
    COMP_LPCOMP_IRQHandler,
    0,
    WDT30_IRQHandler,
    WDT31_IRQHandler,
    0,
    0,
    GPIOTE30_0_IRQHandler,
    GPIOTE30_1_IRQHandler,
    CLOCK_POWER_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    VREGUSB_IRQHandler,
    LFXO_IRQHandler,
    LFRC_IRQHandler,
    HFXO64M_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    AUDIOPLL_AUDIOPLLM_IRQHandler,
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
