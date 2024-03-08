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

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler
        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemoryManagement_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
        DCD     SecureFault_Handler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0                         ; Reserved
        DCD     PendSV_Handler
        DCD     SysTick_Handler

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
        DCD     SWI00_IRQHandler
        DCD     SWI01_IRQHandler
        DCD     SWI02_IRQHandler
        DCD     SWI03_IRQHandler
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
        DCD     AAR00_CCM00_IRQHandler
        DCD     ECB00_IRQHandler
        DCD     CRACEN_IRQHandler
        DCD     0                         ; Reserved
        DCD     SERIAL00_IRQHandler
        DCD     RRAMC_IRQHandler
        DCD     VPR00_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     CTRLAP_IRQHandler
        DCD     CM33SS_IRQHandler
        DCD     0                         ; Reserved
        DCD     TIMER00_IRQHandler
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
        DCD     SPU10_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER10_IRQHandler
        DCD     RTC10_IRQHandler
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
        DCD     SERIAL22_IRQHandler
        DCD     EGU20_IRQHandler
        DCD     TIMER20_IRQHandler
        DCD     TIMER21_IRQHandler
        DCD     TIMER22_IRQHandler
        DCD     TIMER23_IRQHandler
        DCD     TIMER24_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
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
        DCD     TAMPC_IRQHandler
        DCD     I2S20_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     QDEC20_IRQHandler
        DCD     QDEC21_IRQHandler
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
        DCD     SPU30_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SERIAL30_IRQHandler
        DCD     RTC30_IRQHandler
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
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler

        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__iar_program_start
        BX      R0

        ; Dummy exception handlers


        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B .

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B .

        PUBWEAK MemoryManagement_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemoryManagement_Handler
        B .

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B .

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B .

        PUBWEAK SecureFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SecureFault_Handler
        B .

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B .

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B .

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B .

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B .


       ; Dummy interrupt handlers

        PUBWEAK  SWI00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI00_IRQHandler
        B .

        PUBWEAK  SWI01_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI01_IRQHandler
        B .

        PUBWEAK  SWI02_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI02_IRQHandler
        B .

        PUBWEAK  SWI03_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI03_IRQHandler
        B .

        PUBWEAK  SPU00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU00_IRQHandler
        B .

        PUBWEAK  MPC00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPC00_IRQHandler
        B .

        PUBWEAK  AAR00_CCM00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
AAR00_CCM00_IRQHandler
        B .

        PUBWEAK  ECB00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
ECB00_IRQHandler
        B .

        PUBWEAK  CRACEN_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CRACEN_IRQHandler
        B .

        PUBWEAK  SERIAL00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL00_IRQHandler
        B .

        PUBWEAK  RRAMC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RRAMC_IRQHandler
        B .

        PUBWEAK  VPR00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPR00_IRQHandler
        B .

        PUBWEAK  CTRLAP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CTRLAP_IRQHandler
        B .

        PUBWEAK  CM33SS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CM33SS_IRQHandler
        B .

        PUBWEAK  TIMER00_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER00_IRQHandler
        B .

        PUBWEAK  SPU10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU10_IRQHandler
        B .

        PUBWEAK  TIMER10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER10_IRQHandler
        B .

        PUBWEAK  RTC10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RTC10_IRQHandler
        B .

        PUBWEAK  EGU10_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EGU10_IRQHandler
        B .

        PUBWEAK  RADIO_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RADIO_0_IRQHandler
        B .

        PUBWEAK  RADIO_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RADIO_1_IRQHandler
        B .

        PUBWEAK  SPU20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU20_IRQHandler
        B .

        PUBWEAK  SERIAL20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL20_IRQHandler
        B .

        PUBWEAK  SERIAL21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL21_IRQHandler
        B .

        PUBWEAK  SERIAL22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL22_IRQHandler
        B .

        PUBWEAK  EGU20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EGU20_IRQHandler
        B .

        PUBWEAK  TIMER20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER20_IRQHandler
        B .

        PUBWEAK  TIMER21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER21_IRQHandler
        B .

        PUBWEAK  TIMER22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER22_IRQHandler
        B .

        PUBWEAK  TIMER23_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER23_IRQHandler
        B .

        PUBWEAK  TIMER24_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER24_IRQHandler
        B .

        PUBWEAK  PWM20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM20_IRQHandler
        B .

        PUBWEAK  PWM21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM21_IRQHandler
        B .

        PUBWEAK  PWM22_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM22_IRQHandler
        B .

        PUBWEAK  SAADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SAADC_IRQHandler
        B .

        PUBWEAK  NFCT_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
NFCT_IRQHandler
        B .

        PUBWEAK  TEMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TEMP_IRQHandler
        B .

        PUBWEAK  GPIOTE20_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE20_0_IRQHandler
        B .

        PUBWEAK  GPIOTE20_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE20_1_IRQHandler
        B .

        PUBWEAK  TAMPC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TAMPC_IRQHandler
        B .

        PUBWEAK  I2S20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2S20_IRQHandler
        B .

        PUBWEAK  QDEC20_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QDEC20_IRQHandler
        B .

        PUBWEAK  QDEC21_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QDEC21_IRQHandler
        B .

        PUBWEAK  GRTC_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_0_IRQHandler
        B .

        PUBWEAK  GRTC_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_1_IRQHandler
        B .

        PUBWEAK  GRTC_2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_2_IRQHandler
        B .

        PUBWEAK  GRTC_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GRTC_3_IRQHandler
        B .

        PUBWEAK  SPU30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU30_IRQHandler
        B .

        PUBWEAK  SERIAL30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL30_IRQHandler
        B .

        PUBWEAK  RTC30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RTC30_IRQHandler
        B .

        PUBWEAK  COMP_LPCOMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP_LPCOMP_IRQHandler
        B .

        PUBWEAK  WDT30_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT30_IRQHandler
        B .

        PUBWEAK  WDT31_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT31_IRQHandler
        B .

        PUBWEAK  GPIOTE30_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE30_0_IRQHandler
        B .

        PUBWEAK  GPIOTE30_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE30_1_IRQHandler
        B .

        PUBWEAK  CLOCK_POWER_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CLOCK_POWER_IRQHandler
        B .

        END


