; Copyright (c) 2009-2024 ARM Limited. All rights reserved.
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
        DCD     VPRCLIC_0_IRQHandler
        DCD     VPRCLIC_1_IRQHandler
        DCD     VPRCLIC_2_IRQHandler
        DCD     VPRCLIC_3_IRQHandler
        DCD     VPRCLIC_4_IRQHandler
        DCD     VPRCLIC_5_IRQHandler
        DCD     VPRCLIC_6_IRQHandler
        DCD     VPRCLIC_7_IRQHandler
        DCD     VPRCLIC_8_IRQHandler
        DCD     VPRCLIC_9_IRQHandler
        DCD     VPRCLIC_10_IRQHandler
        DCD     VPRCLIC_11_IRQHandler
        DCD     VPRCLIC_12_IRQHandler
        DCD     VPRCLIC_13_IRQHandler
        DCD     VPRCLIC_14_IRQHandler
        DCD     VPRCLIC_15_IRQHandler
        DCD     VPRCLIC_16_IRQHandler
        DCD     VPRCLIC_17_IRQHandler
        DCD     VPRCLIC_18_IRQHandler
        DCD     VPRCLIC_19_IRQHandler
        DCD     VPRCLIC_20_IRQHandler
        DCD     VPRCLIC_21_IRQHandler
        DCD     VPRCLIC_22_IRQHandler
        DCD     VPRCLIC_23_IRQHandler
        DCD     VPRCLIC_24_IRQHandler
        DCD     VPRCLIC_25_IRQHandler
        DCD     VPRCLIC_26_IRQHandler
        DCD     VPRCLIC_27_IRQHandler
        DCD     VPRCLIC_28_IRQHandler
        DCD     VPRCLIC_29_IRQHandler
        DCD     VPRCLIC_30_IRQHandler
        DCD     VPRCLIC_31_IRQHandler
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
        DCD     0                         ; Reserved
        DCD     RRAMC_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     CTRLAP_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER00_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     EGU00_IRQHandler
        DCD     CRACEN_IRQHandler
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
        DCD     0                         ; Reserved
        DCD     EGU20_IRQHandler
        DCD     TIMER20_IRQHandler
        DCD     TIMER21_IRQHandler
        DCD     TIMER22_IRQHandler
        DCD     TIMER23_IRQHandler
        DCD     TIMER24_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SAADC_IRQHandler
        DCD     0                         ; Reserved
        DCD     TEMP_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     GPIOTE20_0_IRQHandler
        DCD     GPIOTE20_1_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     GRTC_0_IRQHandler
        DCD     GRTC_1_IRQHandler
        DCD     GRTC_2_IRQHandler
        DCD     GRTC_3_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
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

        PUBWEAK  VPRCLIC_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_0_IRQHandler
        j .

        PUBWEAK  VPRCLIC_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_1_IRQHandler
        j .

        PUBWEAK  VPRCLIC_2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_2_IRQHandler
        j .

        PUBWEAK  VPRCLIC_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_3_IRQHandler
        j .

        PUBWEAK  VPRCLIC_4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_4_IRQHandler
        j .

        PUBWEAK  VPRCLIC_5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_5_IRQHandler
        j .

        PUBWEAK  VPRCLIC_6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_6_IRQHandler
        j .

        PUBWEAK  VPRCLIC_7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_7_IRQHandler
        j .

        PUBWEAK  VPRCLIC_8_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_8_IRQHandler
        j .

        PUBWEAK  VPRCLIC_9_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_9_IRQHandler
        j .

        PUBWEAK  VPRCLIC_10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_10_IRQHandler
        j .

        PUBWEAK  VPRCLIC_11_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_11_IRQHandler
        j .

        PUBWEAK  VPRCLIC_12_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_12_IRQHandler
        j .

        PUBWEAK  VPRCLIC_13_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_13_IRQHandler
        j .

        PUBWEAK  VPRCLIC_14_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_14_IRQHandler
        j .

        PUBWEAK  VPRCLIC_15_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_15_IRQHandler
        j .

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

        PUBWEAK  VPRCLIC_23_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_23_IRQHandler
        j .

        PUBWEAK  VPRCLIC_24_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_24_IRQHandler
        j .

        PUBWEAK  VPRCLIC_25_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_25_IRQHandler
        j .

        PUBWEAK  VPRCLIC_26_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_26_IRQHandler
        j .

        PUBWEAK  VPRCLIC_27_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_27_IRQHandler
        j .

        PUBWEAK  VPRCLIC_28_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_28_IRQHandler
        j .

        PUBWEAK  VPRCLIC_29_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_29_IRQHandler
        j .

        PUBWEAK  VPRCLIC_30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_30_IRQHandler
        j .

        PUBWEAK  VPRCLIC_31_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPRCLIC_31_IRQHandler
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

        PUBWEAK  RRAMC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RRAMC_IRQHandler
        j .

        PUBWEAK  CTRLAP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CTRLAP_IRQHandler
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

        PUBWEAK  SAADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SAADC_IRQHandler
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

        END


