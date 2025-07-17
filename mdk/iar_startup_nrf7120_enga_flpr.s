; Copyright (c) 2009-2025 ARM Limited. All rights reserved.
; 
;     SPDX-License-Identifier: Apache-2.0
; 
; Licensed under the Apache License, Version 2.0 (the License); you may
; not use this file except in compliance with the License.
; You may obtain a copy of the License at
; 
;     www.apache.org/licenses/LICENSE-2.0
; 
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an AS IS BASIS, WITHOUT
; WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.
; 
; NOTICE: This file has been modified by Nordic Semiconductor ASA.

; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.

        MODULE  ?cstartup

#if defined(__STARTUP_CONFIG)

        #include "startup_config.h"

        #ifndef __STARTUP_CONFIG_STACK_ALIGNEMENT
        #define __STARTUP_CONFIG_STACK_ALIGNEMENT 3
        #endif
        
        SECTION CSTACK:DATA:NOROOT(__STARTUP_CONFIG_STACK_ALIGNEMENT)
        DS8 __STARTUP_CONFIG_STACK_SIZE

        SECTION HEAP:DATA:NOROOT(3)
        DS8 __STARTUP_CONFIG_HEAP_SIZE

#else

        ;; Stack size default : Defined in *.icf (linker file). Can be modified inside EW.
        ;; Heap size default : Defined in *.icf (linker file). Can be modified inside EW.

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

#endif


        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

        ALIGN 6 ;; Align to 64 byte boundary.
__vector_table
        DCD     sfe(CSTACK)
        DCD     UserSoftware_Handler
        DCD     SuperVisorSoftware_Handler
        DCD     MachineSoftware_Handler
        DCD     0                         ; Reserved
        DCD     UserTimer_Handler
        DCD     SuperVisorTimer_Handler
        DCD     0                         ; Reserved
        DCD     MachineTimer_Handler
        DCD     UserExternal_Handler
        DCD     SuperVisorExternal_Handler
        DCD     0                         ; Reserved
        DCD     MachineExternal_Handler
        DCD     CLICSoftware_Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved

        ; External Interrupts
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     VPRCLIC_16_IRQHandler
        DCD     VPRCLIC_17_IRQHandler
        DCD     VPRCLIC_18_IRQHandler
        DCD     VPRCLIC_19_IRQHandler
        DCD     VPRCLIC_20_IRQHandler
        DCD     VPRCLIC_21_IRQHandler
        DCD     VPRCLIC_22_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     VPRTIM_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SPU00_IRQHandler
        DCD     MPC00_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     AAR00_CCM00_IRQHandler
        DCD     ECB00_IRQHandler
        DCD     VPR00_IRQHandler
        DCD     SERIAL00_IRQHandler
        DCD     MRAMC_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     CTRLAP_IRQHandler
        DCD     0                         ; Reserved
        DCD     CM33SS_IRQHandler
        DCD     TIMER00_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     EGU00_IRQHandler
        DCD     CRACEN_IRQHandler
        DCD     USBHS_IRQHandler
        DCD     QSPI00_IRQHandler
        DCD     QSPI01_IRQHandler
        DCD     SERIAL01_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SPU10_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER10_IRQHandler
        DCD     0                         ; Reserved
        DCD     EGU10_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     RADIO_0_IRQHandler
        DCD     RADIO_1_IRQHandler
        DCD     0                         ; Reserved
        DCD     IPCT10_0_IRQHandler
        DCD     IPCT10_1_IRQHandler
        DCD     IPCT10_2_IRQHandler
        DCD     IPCT10_3_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SPU20_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SERIAL20_IRQHandler
        DCD     SERIAL21_IRQHandler
        DCD     SERIAL22_IRQHandler
        DCD     EGU20_IRQHandler
        DCD     TIMER20_IRQHandler
        DCD     TIMER21_IRQHandler
        DCD     TIMER22_IRQHandler
        DCD     TIMER23_IRQHandler
        DCD     TIMER24_IRQHandler
        DCD     0                         ; Reserved
        DCD     PDM20_IRQHandler
        DCD     PDM21_IRQHandler
        DCD     PWM20_IRQHandler
        DCD     PWM21_IRQHandler
        DCD     PWM22_IRQHandler
        DCD     SAADC_IRQHandler
        DCD     NFCT_IRQHandler
        DCD     TEMP_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     GPIOTE20_0_IRQHandler
        DCD     GPIOTE20_1_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     QDEC20_IRQHandler
        DCD     QDEC21_IRQHandler
        DCD     GRTC_0_IRQHandler
        DCD     GRTC_1_IRQHandler
        DCD     GRTC_2_IRQHandler
        DCD     GRTC_3_IRQHandler
        DCD     GRTC_4_IRQHandler
        DCD     GRTC_5_IRQHandler
        DCD     TDM_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SERIAL23_IRQHandler
        DCD     SERIAL24_IRQHandler
        DCD     TAMPC_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SPU30_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SERIAL30_IRQHandler
        DCD     0                         ; Reserved
        DCD     COMP_LPCOMP_IRQHandler
        DCD     0                         ; Reserved
        DCD     WDT30_IRQHandler
        DCD     WDT31_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     GPIOTE30_0_IRQHandler
        DCD     GPIOTE30_1_IRQHandler
        DCD     CLOCK_POWER_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     VREGUSB_IRQHandler
        DCD     LFXO_IRQHandler
        DCD     LFRC_IRQHandler
        DCD     HFXO64M_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     AUDIOPLL_AUDIOPLLM_IRQHandler

__Vectors_End
__Vectors                           EQU   __vector_table
__Vectors_Size                      EQU   __Vectors_End - __Vectors


; Default handlers.
        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler

    jal ra, SystemInit
    j __iar_program_start

        ; Dummy exception handlers


        PUBWEAK UserSoftware_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UserSoftware_Handler
        j .

        PUBWEAK SuperVisorSoftware_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SuperVisorSoftware_Handler
        j .

        PUBWEAK MachineSoftware_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MachineSoftware_Handler
        j .

        PUBWEAK UserTimer_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UserTimer_Handler
        j .

        PUBWEAK SuperVisorTimer_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SuperVisorTimer_Handler
        j .

        PUBWEAK MachineTimer_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MachineTimer_Handler
        j .

        PUBWEAK UserExternal_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UserExternal_Handler
        j .

        PUBWEAK SuperVisorExternal_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SuperVisorExternal_Handler
        j .

        PUBWEAK MachineExternal_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MachineExternal_Handler
        j .

        PUBWEAK CLICSoftware_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
CLICSoftware_Handler
        j .


       ; Dummy interrupt handlers

        PUBWEAK  VPRCLIC_16_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_16_IRQHandler
        j .

        PUBWEAK  VPRCLIC_17_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_17_IRQHandler
        j .

        PUBWEAK  VPRCLIC_18_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_18_IRQHandler
        j .

        PUBWEAK  VPRCLIC_19_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_19_IRQHandler
        j .

        PUBWEAK  VPRCLIC_20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_20_IRQHandler
        j .

        PUBWEAK  VPRCLIC_21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_21_IRQHandler
        j .

        PUBWEAK  VPRCLIC_22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_22_IRQHandler
        j .

        PUBWEAK  VPRTIM_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRTIM_IRQHandler
        j .

        PUBWEAK  SPU00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU00_IRQHandler
        j .

        PUBWEAK  MPC00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPC00_IRQHandler
        j .

        PUBWEAK  AAR00_CCM00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AAR00_CCM00_IRQHandler
        j .

        PUBWEAK  ECB00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ECB00_IRQHandler
        j .

        PUBWEAK  VPR00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPR00_IRQHandler
        j .

        PUBWEAK  SERIAL00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL00_IRQHandler
        j .

        PUBWEAK  MRAMC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MRAMC_IRQHandler
        j .

        PUBWEAK  CTRLAP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CTRLAP_IRQHandler
        j .

        PUBWEAK  CM33SS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CM33SS_IRQHandler
        j .

        PUBWEAK  TIMER00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER00_IRQHandler
        j .

        PUBWEAK  EGU00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EGU00_IRQHandler
        j .

        PUBWEAK  CRACEN_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CRACEN_IRQHandler
        j .

        PUBWEAK  USBHS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USBHS_IRQHandler
        j .

        PUBWEAK  QSPI00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QSPI00_IRQHandler
        j .

        PUBWEAK  QSPI01_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QSPI01_IRQHandler
        j .

        PUBWEAK  SERIAL01_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL01_IRQHandler
        j .

        PUBWEAK  SPU10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU10_IRQHandler
        j .

        PUBWEAK  TIMER10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER10_IRQHandler
        j .

        PUBWEAK  EGU10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EGU10_IRQHandler
        j .

        PUBWEAK  RADIO_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RADIO_0_IRQHandler
        j .

        PUBWEAK  RADIO_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RADIO_1_IRQHandler
        j .

        PUBWEAK  IPCT10_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT10_0_IRQHandler
        j .

        PUBWEAK  IPCT10_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT10_1_IRQHandler
        j .

        PUBWEAK  IPCT10_2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT10_2_IRQHandler
        j .

        PUBWEAK  IPCT10_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT10_3_IRQHandler
        j .

        PUBWEAK  SPU20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU20_IRQHandler
        j .

        PUBWEAK  SERIAL20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL20_IRQHandler
        j .

        PUBWEAK  SERIAL21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL21_IRQHandler
        j .

        PUBWEAK  SERIAL22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL22_IRQHandler
        j .

        PUBWEAK  EGU20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EGU20_IRQHandler
        j .

        PUBWEAK  TIMER20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER20_IRQHandler
        j .

        PUBWEAK  TIMER21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER21_IRQHandler
        j .

        PUBWEAK  TIMER22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER22_IRQHandler
        j .

        PUBWEAK  TIMER23_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER23_IRQHandler
        j .

        PUBWEAK  TIMER24_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER24_IRQHandler
        j .

        PUBWEAK  PDM20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PDM20_IRQHandler
        j .

        PUBWEAK  PDM21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PDM21_IRQHandler
        j .

        PUBWEAK  PWM20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM20_IRQHandler
        j .

        PUBWEAK  PWM21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM21_IRQHandler
        j .

        PUBWEAK  PWM22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM22_IRQHandler
        j .

        PUBWEAK  SAADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SAADC_IRQHandler
        j .

        PUBWEAK  NFCT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
NFCT_IRQHandler
        j .

        PUBWEAK  TEMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TEMP_IRQHandler
        j .

        PUBWEAK  GPIOTE20_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE20_0_IRQHandler
        j .

        PUBWEAK  GPIOTE20_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE20_1_IRQHandler
        j .

        PUBWEAK  QDEC20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QDEC20_IRQHandler
        j .

        PUBWEAK  QDEC21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QDEC21_IRQHandler
        j .

        PUBWEAK  GRTC_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_0_IRQHandler
        j .

        PUBWEAK  GRTC_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_1_IRQHandler
        j .

        PUBWEAK  GRTC_2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_2_IRQHandler
        j .

        PUBWEAK  GRTC_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_3_IRQHandler
        j .

        PUBWEAK  GRTC_4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_4_IRQHandler
        j .

        PUBWEAK  GRTC_5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_5_IRQHandler
        j .

        PUBWEAK  TDM_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TDM_IRQHandler
        j .

        PUBWEAK  SERIAL23_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL23_IRQHandler
        j .

        PUBWEAK  SERIAL24_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL24_IRQHandler
        j .

        PUBWEAK  TAMPC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TAMPC_IRQHandler
        j .

        PUBWEAK  SPU30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU30_IRQHandler
        j .

        PUBWEAK  SERIAL30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL30_IRQHandler
        j .

        PUBWEAK  COMP_LPCOMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP_LPCOMP_IRQHandler
        j .

        PUBWEAK  WDT30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT30_IRQHandler
        j .

        PUBWEAK  WDT31_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT31_IRQHandler
        j .

        PUBWEAK  GPIOTE30_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE30_0_IRQHandler
        j .

        PUBWEAK  GPIOTE30_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE30_1_IRQHandler
        j .

        PUBWEAK  CLOCK_POWER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CLOCK_POWER_IRQHandler
        j .

        PUBWEAK  VREGUSB_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VREGUSB_IRQHandler
        j .

        PUBWEAK  LFXO_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
LFXO_IRQHandler
        j .

        PUBWEAK  LFRC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
LFRC_IRQHandler
        j .

        PUBWEAK  HFXO64M_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
HFXO64M_IRQHandler
        j .

        PUBWEAK  AUDIOPLL_AUDIOPLLM_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AUDIOPLL_AUDIOPLLM_IRQHandler
        j .

        END


