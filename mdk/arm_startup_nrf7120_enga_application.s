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

                IF :DEF: __STARTUP_CONFIG
#ifdef  __STARTUP_CONFIG
#include "startup_config.h"
#ifndef __STARTUP_CONFIG_STACK_ALIGNEMENT
#define __STARTUP_CONFIG_STACK_ALIGNEMENT 3
#endif
#endif
                ENDIF

                IF :DEF: __STARTUP_CONFIG
Stack_Size      EQU __STARTUP_CONFIG_STACK_SIZE
                ELIF :DEF: __STACK_SIZE
Stack_Size      EQU __STACK_SIZE
                ELSE
Stack_Size      EQU 57344
                ENDIF
                
                IF :DEF: __STARTUP_CONFIG
Stack_Align     EQU __STARTUP_CONFIG_STACK_ALIGNEMENT
                ELSE
Stack_Align     EQU 3
                ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=Stack_Align
Stack_Mem       SPACE   Stack_Size
__initial_sp

                IF :DEF: __STARTUP_CONFIG
Heap_Size       EQU __STARTUP_CONFIG_HEAP_SIZE
                ELIF :DEF: __HEAP_SIZE
Heap_Size       EQU __HEAP_SIZE
                ELSE
Heap_Size       EQU 57344
                ENDIF

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
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
                DCD     BELLBOARD_0_IRQHandler
                DCD     BELLBOARD_1_IRQHandler
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
                DCD     AUXPLL_AUXPM_IRQHandler
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
                DCD     0                         ; Reserved
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
                DCD     HVBUCK_IRQHandler

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset Handler


Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main


                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemoryManagement_Handler\
                PROC
                EXPORT  MemoryManagement_Handler  [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SecureFault_Handler\
                PROC
                EXPORT  SecureFault_Handler       [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT   SWI00_IRQHandler [WEAK]
                EXPORT   SWI01_IRQHandler [WEAK]
                EXPORT   SWI02_IRQHandler [WEAK]
                EXPORT   SWI03_IRQHandler [WEAK]
                EXPORT   SPU00_IRQHandler [WEAK]
                EXPORT   MPC00_IRQHandler [WEAK]
                EXPORT   AAR00_CCM00_IRQHandler [WEAK]
                EXPORT   ECB00_IRQHandler [WEAK]
                EXPORT   VPR00_IRQHandler [WEAK]
                EXPORT   SERIAL00_IRQHandler [WEAK]
                EXPORT   MRAMC_IRQHandler [WEAK]
                EXPORT   CTRLAP_IRQHandler [WEAK]
                EXPORT   CM33SS_IRQHandler [WEAK]
                EXPORT   TIMER00_IRQHandler [WEAK]
                EXPORT   EGU00_IRQHandler [WEAK]
                EXPORT   CRACEN_IRQHandler [WEAK]
                EXPORT   USBHS_IRQHandler [WEAK]
                EXPORT   QSPI00_IRQHandler [WEAK]
                EXPORT   QSPI01_IRQHandler [WEAK]
                EXPORT   SERIAL01_IRQHandler [WEAK]
                EXPORT   BELLBOARD_0_IRQHandler [WEAK]
                EXPORT   BELLBOARD_1_IRQHandler [WEAK]
                EXPORT   SPU10_IRQHandler [WEAK]
                EXPORT   TIMER10_IRQHandler [WEAK]
                EXPORT   EGU10_IRQHandler [WEAK]
                EXPORT   RADIO_0_IRQHandler [WEAK]
                EXPORT   RADIO_1_IRQHandler [WEAK]
                EXPORT   IPCT10_0_IRQHandler [WEAK]
                EXPORT   IPCT10_1_IRQHandler [WEAK]
                EXPORT   IPCT10_2_IRQHandler [WEAK]
                EXPORT   IPCT10_3_IRQHandler [WEAK]
                EXPORT   SPU20_IRQHandler [WEAK]
                EXPORT   SERIAL20_IRQHandler [WEAK]
                EXPORT   SERIAL21_IRQHandler [WEAK]
                EXPORT   SERIAL22_IRQHandler [WEAK]
                EXPORT   EGU20_IRQHandler [WEAK]
                EXPORT   TIMER20_IRQHandler [WEAK]
                EXPORT   TIMER21_IRQHandler [WEAK]
                EXPORT   TIMER22_IRQHandler [WEAK]
                EXPORT   TIMER23_IRQHandler [WEAK]
                EXPORT   TIMER24_IRQHandler [WEAK]
                EXPORT   PDM20_IRQHandler [WEAK]
                EXPORT   PDM21_IRQHandler [WEAK]
                EXPORT   PWM20_IRQHandler [WEAK]
                EXPORT   PWM21_IRQHandler [WEAK]
                EXPORT   PWM22_IRQHandler [WEAK]
                EXPORT   SAADC_IRQHandler [WEAK]
                EXPORT   NFCT_IRQHandler [WEAK]
                EXPORT   TEMP_IRQHandler [WEAK]
                EXPORT   GPIOTE20_0_IRQHandler [WEAK]
                EXPORT   GPIOTE20_1_IRQHandler [WEAK]
                EXPORT   QDEC20_IRQHandler [WEAK]
                EXPORT   QDEC21_IRQHandler [WEAK]
                EXPORT   GRTC_0_IRQHandler [WEAK]
                EXPORT   GRTC_1_IRQHandler [WEAK]
                EXPORT   GRTC_2_IRQHandler [WEAK]
                EXPORT   GRTC_3_IRQHandler [WEAK]
                EXPORT   GRTC_4_IRQHandler [WEAK]
                EXPORT   GRTC_5_IRQHandler [WEAK]
                EXPORT   TDM_IRQHandler [WEAK]
                EXPORT   AUXPLL_AUXPM_IRQHandler [WEAK]
                EXPORT   SERIAL23_IRQHandler [WEAK]
                EXPORT   SERIAL24_IRQHandler [WEAK]
                EXPORT   TAMPC_IRQHandler [WEAK]
                EXPORT   SPU30_IRQHandler [WEAK]
                EXPORT   SERIAL30_IRQHandler [WEAK]
                EXPORT   COMP_LPCOMP_IRQHandler [WEAK]
                EXPORT   WDT30_IRQHandler [WEAK]
                EXPORT   WDT31_IRQHandler [WEAK]
                EXPORT   GPIOTE30_0_IRQHandler [WEAK]
                EXPORT   GPIOTE30_1_IRQHandler [WEAK]
                EXPORT   CLOCK_POWER_IRQHandler [WEAK]
                EXPORT   LFXO_IRQHandler [WEAK]
                EXPORT   LFRC_IRQHandler [WEAK]
                EXPORT   HFXO64M_IRQHandler [WEAK]
                EXPORT   HVBUCK_IRQHandler [WEAK]
SWI00_IRQHandler
SWI01_IRQHandler
SWI02_IRQHandler
SWI03_IRQHandler
SPU00_IRQHandler
MPC00_IRQHandler
AAR00_CCM00_IRQHandler
ECB00_IRQHandler
VPR00_IRQHandler
SERIAL00_IRQHandler
MRAMC_IRQHandler
CTRLAP_IRQHandler
CM33SS_IRQHandler
TIMER00_IRQHandler
EGU00_IRQHandler
CRACEN_IRQHandler
USBHS_IRQHandler
QSPI00_IRQHandler
QSPI01_IRQHandler
SERIAL01_IRQHandler
BELLBOARD_0_IRQHandler
BELLBOARD_1_IRQHandler
SPU10_IRQHandler
TIMER10_IRQHandler
EGU10_IRQHandler
RADIO_0_IRQHandler
RADIO_1_IRQHandler
IPCT10_0_IRQHandler
IPCT10_1_IRQHandler
IPCT10_2_IRQHandler
IPCT10_3_IRQHandler
SPU20_IRQHandler
SERIAL20_IRQHandler
SERIAL21_IRQHandler
SERIAL22_IRQHandler
EGU20_IRQHandler
TIMER20_IRQHandler
TIMER21_IRQHandler
TIMER22_IRQHandler
TIMER23_IRQHandler
TIMER24_IRQHandler
PDM20_IRQHandler
PDM21_IRQHandler
PWM20_IRQHandler
PWM21_IRQHandler
PWM22_IRQHandler
SAADC_IRQHandler
NFCT_IRQHandler
TEMP_IRQHandler
GPIOTE20_0_IRQHandler
GPIOTE20_1_IRQHandler
QDEC20_IRQHandler
QDEC21_IRQHandler
GRTC_0_IRQHandler
GRTC_1_IRQHandler
GRTC_2_IRQHandler
GRTC_3_IRQHandler
GRTC_4_IRQHandler
GRTC_5_IRQHandler
TDM_IRQHandler
AUXPLL_AUXPM_IRQHandler
SERIAL23_IRQHandler
SERIAL24_IRQHandler
TAMPC_IRQHandler
SPU30_IRQHandler
SERIAL30_IRQHandler
COMP_LPCOMP_IRQHandler
WDT30_IRQHandler
WDT31_IRQHandler
GPIOTE30_0_IRQHandler
GPIOTE30_1_IRQHandler
CLOCK_POWER_IRQHandler
LFXO_IRQHandler
LFRC_IRQHandler
HFXO64M_IRQHandler
HVBUCK_IRQHandler
                B .
                ENDP
                ALIGN

; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC

                LDR     R0, = Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem + Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF

                END
