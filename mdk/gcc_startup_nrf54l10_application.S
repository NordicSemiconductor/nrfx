/*
 
Copyright (c) 2009-2025 ARM Limited. All rights reserved.

    SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the License); you may
not use this file except in compliance with the License.
You may obtain a copy of the License at

    www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an AS IS BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

NOTICE: This file has been modified by Nordic Semiconductor ASA.

*/

    .syntax unified
    .arch armv8-m.main

#ifdef __STARTUP_CONFIG
#include "startup_config.h"
#ifndef __STARTUP_CONFIG_STACK_ALIGNEMENT
#define __STARTUP_CONFIG_STACK_ALIGNEMENT 3
#endif
#endif

    .section .stack
#if defined(__STARTUP_CONFIG)
    .align __STARTUP_CONFIG_STACK_ALIGNEMENT
    .equ    Stack_Size, __STARTUP_CONFIG_STACK_SIZE
#elif defined(__STACK_SIZE)
    .align 3
    .equ    Stack_Size, __STACK_SIZE
#else
    .align 3
    .equ    Stack_Size, 8192
#endif
    .globl __StackTop
    .globl __StackLimit
__StackLimit:
    .space Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#if defined(__STARTUP_CONFIG)
    .equ Heap_Size, __STARTUP_CONFIG_HEAP_SIZE
#elif defined(__HEAP_SIZE)
    .equ Heap_Size, __HEAP_SIZE
#else
    .equ Heap_Size, 8192
#endif
    .globl __HeapBase
    .globl __HeapLimit
__HeapBase:
    .if Heap_Size
    .space Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector, "ax"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                  /* Top of Stack */
    .long   Reset_Handler
    .long   NMI_Handler
    .long   HardFault_Handler
    .long   MemoryManagement_Handler
    .long   BusFault_Handler
    .long   UsageFault_Handler
    .long   SecureFault_Handler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SVC_Handler
    .long   DebugMon_Handler
    .long   0                           /*Reserved */
    .long   PendSV_Handler
    .long   SysTick_Handler

  /* External Interrupts */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SWI00_IRQHandler
    .long   SWI01_IRQHandler
    .long   SWI02_IRQHandler
    .long   SWI03_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SPU00_IRQHandler
    .long   MPC00_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   AAR00_CCM00_IRQHandler
    .long   ECB00_IRQHandler
    .long   CRACEN_IRQHandler
    .long   0                           /*Reserved */
    .long   SERIAL00_IRQHandler
    .long   RRAMC_IRQHandler
    .long   VPR00_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   CTRLAP_IRQHandler
    .long   0                           /*Reserved */
    .long   CM33SS_IRQHandler
    .long   TIMER00_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SPU10_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   TIMER10_IRQHandler
    .long   0                           /*Reserved */
    .long   EGU10_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   RADIO_0_IRQHandler
    .long   RADIO_1_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SPU20_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SERIAL20_IRQHandler
    .long   SERIAL21_IRQHandler
    .long   SERIAL22_IRQHandler
    .long   EGU20_IRQHandler
    .long   TIMER20_IRQHandler
    .long   TIMER21_IRQHandler
    .long   TIMER22_IRQHandler
    .long   TIMER23_IRQHandler
    .long   TIMER24_IRQHandler
    .long   0                           /*Reserved */
    .long   PDM20_IRQHandler
    .long   PDM21_IRQHandler
    .long   PWM20_IRQHandler
    .long   PWM21_IRQHandler
    .long   PWM22_IRQHandler
    .long   SAADC_IRQHandler
    .long   NFCT_IRQHandler
    .long   TEMP_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   GPIOTE20_0_IRQHandler
    .long   GPIOTE20_1_IRQHandler
    .long   TAMPC_IRQHandler
    .long   I2S20_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   QDEC20_IRQHandler
    .long   QDEC21_IRQHandler
    .long   GRTC_0_IRQHandler
    .long   GRTC_1_IRQHandler
    .long   GRTC_2_IRQHandler
    .long   GRTC_3_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SPU30_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   SERIAL30_IRQHandler
    .long   CLOCK_POWER_IRQHandler
    .long   COMP_LPCOMP_IRQHandler
    .long   0                           /*Reserved */
    .long   WDT30_IRQHandler
    .long   WDT31_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   GPIOTE30_0_IRQHandler
    .long   GPIOTE30_1_IRQHandler

    .size __isr_vector, . - __isr_vector

/* Reset Handler */


    .text
    .thumb
    .thumb_func
    .align 1
    .globl Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:


/* Loop to copy data from read only memory to RAM.
 * The ranges of copy from/to are specified by following symbols:
 *      __etext: LMA of start of the section to copy from. Usually end of text
 *      __data_start: VMA of start of the section to copy to.
 *      __data_end: VMA of end of the section to copy to. 
 *
 * All addresses must be aligned to 4 bytes boundary.
 */
#ifndef __STARTUP_SKIP_ETEXT

/* Load .data */
    ldr r1, =__data_start
    ldr r2, =__data_end
    ldr r3, =__data_load_start
    bl copy_region

/* Load .sdata */
    ldr r1, =__sdata_start
    ldr r2, =__sdata_end
    ldr r3, =__sdata_load_start
    bl copy_region

/* Load .tdata */
    ldr r1, =__tdata_start
    ldr r2, =__tdata_end
    ldr r3, =__tdata_load_start
    bl copy_region

/* Load .fast */
    ldr r1, =__fast_start
    ldr r2, =__fast_end
    ldr r3, =__fast_load_start
    bl copy_region

    b copy_etext_done

/* Method that loads data from nvm to ram */
copy_region:
    subs r2, r2, r1
    ble L_copy_region_done

L_copy_region:
    subs r2, r2, #4
    ldr r0, [r3,r2]
    str r0, [r1,r2]
    bgt L_copy_region

L_copy_region_done:

    bx lr

copy_etext_done:


#endif

/* This part of work usually is done in C library startup code. Otherwise,
 * define __STARTUP_CLEAR_BSS to enable it in this startup. This section
 * clears the RAM where BSS data is located.
 *
 * The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 * All addresses must be aligned to 4 bytes boundary.
 */
#ifdef __STARTUP_CLEAR_BSS
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__
    bl clear_region

    ldr r1, =__tbss_start__
    ldr r2, =__tbss_end__
    bl clear_region

    ldr r1, =__sbss_start__
    ldr r2, =__sbss_end__
    bl clear_region

    b clear_bss_done

/* Method that clears default-0 registers */
clear_region:
    movs r0, 0

    subs r2, r2, r1
    ble .L_clear_region_done

.L_clear_region:
    subs r2, r2, #4
    str r0, [r1, r2]
    bgt .L_clear_region

.L_clear_region_done:

    bx lr

clear_bss_done:

#endif /* __STARTUP_CLEAR_BSS */

/* Execute SystemInit function. */
    bl SystemInit

/* Call _start function provided by libraries.
 * If those libraries are not accessible, define __START as your entry point.
 */
#ifndef __START
#define __START _start
#endif
    bl __START

    .pool
    .size   Reset_Handler,.-Reset_Handler

    .section ".text"


/* Dummy Exception Handlers (infinite loops which can be modified) */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    b       .
    .size   NMI_Handler, . - NMI_Handler


    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    b       .
    .size   HardFault_Handler, . - HardFault_Handler


    .weak   MemoryManagement_Handler
    .type   MemoryManagement_Handler, %function
MemoryManagement_Handler:
    b       .
    .size   MemoryManagement_Handler, . - MemoryManagement_Handler


    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    b       .
    .size   BusFault_Handler, . - BusFault_Handler


    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    b       .
    .size   UsageFault_Handler, . - UsageFault_Handler


    .weak   SecureFault_Handler
    .type   SecureFault_Handler, %function
SecureFault_Handler:
    b       .
    .size   SecureFault_Handler, . - SecureFault_Handler


    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    b       .
    .size   SVC_Handler, . - SVC_Handler


    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    b       .
    .size   DebugMon_Handler, . - DebugMon_Handler


    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    b       .
    .size   PendSV_Handler, . - PendSV_Handler


    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    b       .
    .size   SysTick_Handler, . - SysTick_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ  SWI00_IRQHandler
    IRQ  SWI01_IRQHandler
    IRQ  SWI02_IRQHandler
    IRQ  SWI03_IRQHandler
    IRQ  SPU00_IRQHandler
    IRQ  MPC00_IRQHandler
    IRQ  AAR00_CCM00_IRQHandler
    IRQ  ECB00_IRQHandler
    IRQ  CRACEN_IRQHandler
    IRQ  SERIAL00_IRQHandler
    IRQ  RRAMC_IRQHandler
    IRQ  VPR00_IRQHandler
    IRQ  CTRLAP_IRQHandler
    IRQ  CM33SS_IRQHandler
    IRQ  TIMER00_IRQHandler
    IRQ  SPU10_IRQHandler
    IRQ  TIMER10_IRQHandler
    IRQ  EGU10_IRQHandler
    IRQ  RADIO_0_IRQHandler
    IRQ  RADIO_1_IRQHandler
    IRQ  SPU20_IRQHandler
    IRQ  SERIAL20_IRQHandler
    IRQ  SERIAL21_IRQHandler
    IRQ  SERIAL22_IRQHandler
    IRQ  EGU20_IRQHandler
    IRQ  TIMER20_IRQHandler
    IRQ  TIMER21_IRQHandler
    IRQ  TIMER22_IRQHandler
    IRQ  TIMER23_IRQHandler
    IRQ  TIMER24_IRQHandler
    IRQ  PDM20_IRQHandler
    IRQ  PDM21_IRQHandler
    IRQ  PWM20_IRQHandler
    IRQ  PWM21_IRQHandler
    IRQ  PWM22_IRQHandler
    IRQ  SAADC_IRQHandler
    IRQ  NFCT_IRQHandler
    IRQ  TEMP_IRQHandler
    IRQ  GPIOTE20_0_IRQHandler
    IRQ  GPIOTE20_1_IRQHandler
    IRQ  TAMPC_IRQHandler
    IRQ  I2S20_IRQHandler
    IRQ  QDEC20_IRQHandler
    IRQ  QDEC21_IRQHandler
    IRQ  GRTC_0_IRQHandler
    IRQ  GRTC_1_IRQHandler
    IRQ  GRTC_2_IRQHandler
    IRQ  GRTC_3_IRQHandler
    IRQ  SPU30_IRQHandler
    IRQ  SERIAL30_IRQHandler
    IRQ  CLOCK_POWER_IRQHandler
    IRQ  COMP_LPCOMP_IRQHandler
    IRQ  WDT30_IRQHandler
    IRQ  WDT31_IRQHandler
    IRQ  GPIOTE30_0_IRQHandler
    IRQ  GPIOTE30_1_IRQHandler

  .end
