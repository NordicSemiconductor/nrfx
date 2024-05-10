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
        DCD     SPU000_IRQHandler
        DCD     MPC_IRQHandler
        DCD     0                         ; Reserved
        DCD     MVDMA_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SPU010_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     WDT010_IRQHandler
        DCD     WDT011_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     IPCT_0_IRQHandler
        DCD     IPCT_1_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SWI0_IRQHandler
        DCD     SWI1_IRQHandler
        DCD     SWI2_IRQHandler
        DCD     SWI3_IRQHandler
        DCD     SWI4_IRQHandler
        DCD     SWI5_IRQHandler
        DCD     SWI6_IRQHandler
        DCD     SWI7_IRQHandler
        DCD     BELLBOARD_0_IRQHandler
        DCD     BELLBOARD_1_IRQHandler
        DCD     BELLBOARD_2_IRQHandler
        DCD     BELLBOARD_3_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     GPIOTE130_0_IRQHandler
        DCD     GPIOTE130_1_IRQHandler
        DCD     GPIOTE131_0_IRQHandler
        DCD     GPIOTE131_1_IRQHandler
        DCD     GRTC_0_IRQHandler
        DCD     GRTC_1_IRQHandler
        DCD     GRTC_2_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TBM_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     USBHS_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     EXMIF_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     IPCT120_0_IRQHandler
        DCD     0                         ; Reserved
        DCD     I3C120_IRQHandler
        DCD     VPR121_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     CAN120_IRQHandler
        DCD     MVDMA120_IRQHandler
        DCD     0                         ; Reserved
        DCD     CAN121_IRQHandler
        DCD     MVDMA121_IRQHandler
        DCD     0                         ; Reserved
        DCD     I3C121_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER120_IRQHandler
        DCD     TIMER121_IRQHandler
        DCD     PWM120_IRQHandler
        DCD     SPIS120_IRQHandler
        DCD     SPIM120_UARTE120_IRQHandler
        DCD     SPIM121_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     VPR130_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     IPCT130_0_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     RTC130_IRQHandler
        DCD     RTC131_IRQHandler
        DCD     0                         ; Reserved
        DCD     WDT131_IRQHandler
        DCD     WDT132_IRQHandler
        DCD     EGU130_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     SAADC_IRQHandler
        DCD     COMP_LPCOMP_IRQHandler
        DCD     TEMP_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     I2S130_IRQHandler
        DCD     PDM_IRQHandler
        DCD     QDEC130_IRQHandler
        DCD     QDEC131_IRQHandler
        DCD     0                         ; Reserved
        DCD     I2S131_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER130_IRQHandler
        DCD     TIMER131_IRQHandler
        DCD     PWM130_IRQHandler
        DCD     SERIAL0_IRQHandler
        DCD     SERIAL1_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER132_IRQHandler
        DCD     TIMER133_IRQHandler
        DCD     PWM131_IRQHandler
        DCD     SERIAL2_IRQHandler
        DCD     SERIAL3_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER134_IRQHandler
        DCD     TIMER135_IRQHandler
        DCD     PWM132_IRQHandler
        DCD     SERIAL4_IRQHandler
        DCD     SERIAL5_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     TIMER136_IRQHandler
        DCD     TIMER137_IRQHandler
        DCD     PWM133_IRQHandler
        DCD     SERIAL6_IRQHandler
        DCD     SERIAL7_IRQHandler
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved
        DCD     0                         ; Reserved

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

        PUBWEAK  SPU000_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU000_IRQHandler
        B .

        PUBWEAK  MPC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MPC_IRQHandler
        B .

        PUBWEAK  MVDMA_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MVDMA_IRQHandler
        B .

        PUBWEAK  SPU010_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPU010_IRQHandler
        B .

        PUBWEAK  WDT010_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT010_IRQHandler
        B .

        PUBWEAK  WDT011_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT011_IRQHandler
        B .

        PUBWEAK  IPCT_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT_0_IRQHandler
        B .

        PUBWEAK  IPCT_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT_1_IRQHandler
        B .

        PUBWEAK  SWI0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI0_IRQHandler
        B .

        PUBWEAK  SWI1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI1_IRQHandler
        B .

        PUBWEAK  SWI2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI2_IRQHandler
        B .

        PUBWEAK  SWI3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI3_IRQHandler
        B .

        PUBWEAK  SWI4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI4_IRQHandler
        B .

        PUBWEAK  SWI5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI5_IRQHandler
        B .

        PUBWEAK  SWI6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI6_IRQHandler
        B .

        PUBWEAK  SWI7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SWI7_IRQHandler
        B .

        PUBWEAK  BELLBOARD_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BELLBOARD_0_IRQHandler
        B .

        PUBWEAK  BELLBOARD_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BELLBOARD_1_IRQHandler
        B .

        PUBWEAK  BELLBOARD_2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BELLBOARD_2_IRQHandler
        B .

        PUBWEAK  BELLBOARD_3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
BELLBOARD_3_IRQHandler
        B .

        PUBWEAK  GPIOTE130_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE130_0_IRQHandler
        B .

        PUBWEAK  GPIOTE130_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE130_1_IRQHandler
        B .

        PUBWEAK  GPIOTE131_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE131_0_IRQHandler
        B .

        PUBWEAK  GPIOTE131_1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
GPIOTE131_1_IRQHandler
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

        PUBWEAK  TBM_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TBM_IRQHandler
        B .

        PUBWEAK  USBHS_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
USBHS_IRQHandler
        B .

        PUBWEAK  EXMIF_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EXMIF_IRQHandler
        B .

        PUBWEAK  IPCT120_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT120_0_IRQHandler
        B .

        PUBWEAK  I3C120_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I3C120_IRQHandler
        B .

        PUBWEAK  VPR121_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPR121_IRQHandler
        B .

        PUBWEAK  CAN120_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN120_IRQHandler
        B .

        PUBWEAK  MVDMA120_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MVDMA120_IRQHandler
        B .

        PUBWEAK  CAN121_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
CAN121_IRQHandler
        B .

        PUBWEAK  MVDMA121_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
MVDMA121_IRQHandler
        B .

        PUBWEAK  I3C121_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I3C121_IRQHandler
        B .

        PUBWEAK  TIMER120_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER120_IRQHandler
        B .

        PUBWEAK  TIMER121_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER121_IRQHandler
        B .

        PUBWEAK  PWM120_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM120_IRQHandler
        B .

        PUBWEAK  SPIS120_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPIS120_IRQHandler
        B .

        PUBWEAK  SPIM120_UARTE120_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPIM120_UARTE120_IRQHandler
        B .

        PUBWEAK  SPIM121_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SPIM121_IRQHandler
        B .

        PUBWEAK  VPR130_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
VPR130_IRQHandler
        B .

        PUBWEAK  IPCT130_0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
IPCT130_0_IRQHandler
        B .

        PUBWEAK  RTC130_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RTC130_IRQHandler
        B .

        PUBWEAK  RTC131_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
RTC131_IRQHandler
        B .

        PUBWEAK  WDT131_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT131_IRQHandler
        B .

        PUBWEAK  WDT132_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
WDT132_IRQHandler
        B .

        PUBWEAK  EGU130_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
EGU130_IRQHandler
        B .

        PUBWEAK  SAADC_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SAADC_IRQHandler
        B .

        PUBWEAK  COMP_LPCOMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
COMP_LPCOMP_IRQHandler
        B .

        PUBWEAK  TEMP_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TEMP_IRQHandler
        B .

        PUBWEAK  I2S130_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2S130_IRQHandler
        B .

        PUBWEAK  PDM_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PDM_IRQHandler
        B .

        PUBWEAK  QDEC130_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QDEC130_IRQHandler
        B .

        PUBWEAK  QDEC131_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
QDEC131_IRQHandler
        B .

        PUBWEAK  I2S131_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
I2S131_IRQHandler
        B .

        PUBWEAK  TIMER130_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER130_IRQHandler
        B .

        PUBWEAK  TIMER131_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER131_IRQHandler
        B .

        PUBWEAK  PWM130_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM130_IRQHandler
        B .

        PUBWEAK  SERIAL0_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL0_IRQHandler
        B .

        PUBWEAK  SERIAL1_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL1_IRQHandler
        B .

        PUBWEAK  TIMER132_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER132_IRQHandler
        B .

        PUBWEAK  TIMER133_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER133_IRQHandler
        B .

        PUBWEAK  PWM131_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM131_IRQHandler
        B .

        PUBWEAK  SERIAL2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL2_IRQHandler
        B .

        PUBWEAK  SERIAL3_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL3_IRQHandler
        B .

        PUBWEAK  TIMER134_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER134_IRQHandler
        B .

        PUBWEAK  TIMER135_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER135_IRQHandler
        B .

        PUBWEAK  PWM132_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM132_IRQHandler
        B .

        PUBWEAK  SERIAL4_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL4_IRQHandler
        B .

        PUBWEAK  SERIAL5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL5_IRQHandler
        B .

        PUBWEAK  TIMER136_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER136_IRQHandler
        B .

        PUBWEAK  TIMER137_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
TIMER137_IRQHandler
        B .

        PUBWEAK  PWM133_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
PWM133_IRQHandler
        B .

        PUBWEAK  SERIAL6_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL6_IRQHandler
        B .

        PUBWEAK  SERIAL7_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)
SERIAL7_IRQHandler
        B .

        END


