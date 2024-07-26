/*
 
Copyright (c) 2009-2024 ARM Limited. All rights reserved.

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

/*

This file was modified by Nordic Semiconductor to support RISC-V devices.

*/

/* ============================ *
 *     Stack configuration      *
 * ============================ */

#ifndef __ASSEMBLY__
    #define __ASSEMBLY__
#endif
#include "core_vpr.h"

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
    .equ    Stack_Size, 2048
#endif
    .globl __StackTop
    .globl __StackLimit
__StackLimit:
    .space Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop


/* ============================ *
 *      Heap configuration      *
 * ============================ */
    .section .heap
    .align 3
#if defined(__STARTUP_CONFIG)
    .equ Heap_Size, __STARTUP_CONFIG_HEAP_SIZE
#elif defined(__HEAP_SIZE)
    .equ Heap_Size, __HEAP_SIZE
#else
    .equ Heap_Size, 2048
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

/* ============================ *
 *         Startup code         *
 * ============================ */
/* Reset Handler */


    .section .startup, "ax"
    .align 1
    .globl Reset_Handler
    .type Reset_Handler, %function
Reset_Handler:
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

#ifndef NO_STACK_INIT
    /* Initialise main stack */
    la sp, __StackTop
    add s0, sp, zero
#endif


#if !defined(NO_MTVT_CONFIG) && defined(__MTVT_PRESENT) && __MTVT_PRESENT
    /* Configure machine trap vector table register */
    la t0, __isr_vector
    csrw CSR_MTVT, t0

    /* Setup machine trap vector in CSR and clear cause register (hardfault exceptions) */
    .weak Trap_Handler
    la t0, Trap_Handler
    csrw CSR_MTVEC, t0
    csrw CSR_MCAUSE, x0
#endif

/* Loop to copy data from read only memory to RAM.
 * The ranges of copy from/to are specified by following symbols:
 *      __data_load_start: LMA of start of the section to copy from. Usually end of text
 *      __data_start: VMA of start of the section to copy to.
 *      __data_end: VMA of end of the section to copy to. 
 *
 * All addresses must be aligned to 4 bytes boundary.
 */
#ifndef __STARTUP_SKIP_ETEXT

/* Load .data */
    la t0, __data_start
    la t1, __data_end
    la t2, __data_load_start
    jal copy_region

/* Load .sdata */
    la t0, __sdata_start
    la t1, __sdata_end
    la t2, __sdata_load_start
    jal copy_region

/* Load .tdata */
    la t0, __tdata_start
    la t1, __tdata_end
    la t2, __tdata_load_start
    jal copy_region

/* Load .fast */
    la t0, __fast_start
    la t1, __fast_end
    la t2, __fast_load_start
    jal copy_region

    j copy_etext_done

/* Method that loads data from nvm to ram */
copy_region:
    beq t0, t1, L_copy_region_done

L_copy_region:
    lw a0,0(t2)
    sw a0,0(t0)
    addi t0, t0, 4
    addi t2, t2, 4
    bne  t0, t1, L_copy_region

L_copy_region_done:

    ret

copy_etext_done:

#endif

/* This part of work usually is done in C library startup code. Otherwise,
 * define __STARTUP_CLEAR_BSS to enable it in this startup. This section
 * clears the RAM where BSS data is located.
 *
 * The BSS section is specified by following symbols
 *    __bss_start: start of the BSS section.
 *    __bss_end:
 *    __sbss_start
 *    __sbss_end
 *    __tbss_start
 *    __tbss_end : end of the BSS section.
 *
 * All addresses must be aligned to 4 bytes boundary.
 */
#ifdef __STARTUP_CLEAR_BSS
    la t0, __bss_start
    la t1, __bss_end
    jal zero_region

    la t0, __sbss_start
    la t1, __sbss_end
    jal zero_region

    la t0, __tbss_start
    la t1, __tbss_end
    jal zero_region
    
    j clear_bss_done

/* Method that clears ram region */
zero_region:
    beq t0, t1, L_zero_region_done

L_zero_region:
    sw   zero, 0(t0)
    addi t0, t0, 4
    bne  t0, t1, L_zero_region

L_zero_region_done:

    ret

clear_bss_done:

#endif /* __STARTUP_CLEAR_BSS */

/* Execute SystemInit function. */
    jal SystemInit

/* Call _start function provided by libraries.
 * If those libraries are not accessible, define __START as your entry point.
 */
#ifndef __START
#define __START _start
#endif
    jal __START

    .size   Reset_Handler,.-Reset_Handler

/* ============================ *
 *  VPR interrupt return block  *
 * ============================ */
    .section .isr_return, "ax"
    .globl __isr_return
__isr_return:
    mret

/* ============================ *
 *  HardFault trap              *
 * ============================ */
.globl Trap_Handler
.type Trap_Handler, %function
.balign 8
Trap_Handler:
    j Trap_Handler

/* ============================ *
 *         Vector table         *
 * ============================ */
    .section .isr_vector, "ax"
    .balign 64
    .globl __isr_vector
__isr_vector:
    .long   UserSoftware_Handler
    .long   SuperVisorSoftware_Handler
    .long   MachineSoftware_Handler
    .long   0                           /*Reserved */
    .long   UserTimer_Handler
    .long   SuperVisorTimer_Handler
    .long   0                           /*Reserved */
    .long   MachineTimer_Handler
    .long   UserExternal_Handler
    .long   SuperVisorExternal_Handler
    .long   0                           /*Reserved */
    .long   MachineExternal_Handler
    .long   CLICSoftware_Handler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */

  /* External Interrupts */
    .long   VPRCLIC_0_IRQHandler
    .long   VPRCLIC_1_IRQHandler
    .long   VPRCLIC_2_IRQHandler
    .long   VPRCLIC_3_IRQHandler
    .long   VPRCLIC_4_IRQHandler
    .long   VPRCLIC_5_IRQHandler
    .long   VPRCLIC_6_IRQHandler
    .long   VPRCLIC_7_IRQHandler
    .long   VPRCLIC_8_IRQHandler
    .long   VPRCLIC_9_IRQHandler
    .long   VPRCLIC_10_IRQHandler
    .long   VPRCLIC_11_IRQHandler
    .long   VPRCLIC_12_IRQHandler
    .long   VPRCLIC_13_IRQHandler
    .long   VPRCLIC_14_IRQHandler
    .long   VPRCLIC_15_IRQHandler
    .long   VPRTIM_IRQHandler
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
    .long   GPIOTE130_0_IRQHandler
    .long   GPIOTE130_1_IRQHandler
    .long   GPIOTE131_0_IRQHandler
    .long   GPIOTE131_1_IRQHandler
    .long   GRTC_0_IRQHandler
    .long   GRTC_1_IRQHandler
    .long   GRTC_2_IRQHandler
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
    .long   TBM_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   USBHS_IRQHandler
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
    .long   EXMIF_IRQHandler
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
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   IPCT120_0_IRQHandler
    .long   0                           /*Reserved */
    .long   I3C120_IRQHandler
    .long   VPR121_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   CAN120_IRQHandler
    .long   MVDMA120_IRQHandler
    .long   0                           /*Reserved */
    .long   CAN121_IRQHandler
    .long   MVDMA121_IRQHandler
    .long   0                           /*Reserved */
    .long   I3C121_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   TIMER120_IRQHandler
    .long   TIMER121_IRQHandler
    .long   PWM120_IRQHandler
    .long   SPIS120_IRQHandler
    .long   SPIM120_UARTE120_IRQHandler
    .long   SPIM121_IRQHandler
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
    .long   VPR130_IRQHandler
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
    .long   IPCT130_0_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   RTC130_IRQHandler
    .long   RTC131_IRQHandler
    .long   0                           /*Reserved */
    .long   WDT131_IRQHandler
    .long   WDT132_IRQHandler
    .long   EGU130_IRQHandler
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
    .long   SAADC_IRQHandler
    .long   COMP_LPCOMP_IRQHandler
    .long   TEMP_IRQHandler
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
    .long   I2S130_IRQHandler
    .long   PDM_IRQHandler
    .long   QDEC130_IRQHandler
    .long   QDEC131_IRQHandler
    .long   0                           /*Reserved */
    .long   I2S131_IRQHandler
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
    .long   TIMER130_IRQHandler
    .long   TIMER131_IRQHandler
    .long   PWM130_IRQHandler
    .long   SERIAL0_IRQHandler
    .long   SERIAL1_IRQHandler
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
    .long   TIMER132_IRQHandler
    .long   TIMER133_IRQHandler
    .long   PWM131_IRQHandler
    .long   SERIAL2_IRQHandler
    .long   SERIAL3_IRQHandler
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
    .long   TIMER134_IRQHandler
    .long   TIMER135_IRQHandler
    .long   PWM132_IRQHandler
    .long   SERIAL4_IRQHandler
    .long   SERIAL5_IRQHandler
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
    .long   TIMER136_IRQHandler
    .long   TIMER137_IRQHandler
    .long   PWM133_IRQHandler
    .long   SERIAL6_IRQHandler
    .long   SERIAL7_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */

    .size __isr_vector, . - __isr_vector

.section .text

/* Dummy Exception Handlers (infinite loops which can be modified) */

    .weak   UserSoftware_Handler
    .type   UserSoftware_Handler, %function
UserSoftware_Handler:
    j       .
    .size   UserSoftware_Handler, . - UserSoftware_Handler


    .weak   SuperVisorSoftware_Handler
    .type   SuperVisorSoftware_Handler, %function
SuperVisorSoftware_Handler:
    j       .
    .size   SuperVisorSoftware_Handler, . - SuperVisorSoftware_Handler


    .weak   MachineSoftware_Handler
    .type   MachineSoftware_Handler, %function
MachineSoftware_Handler:
    j       .
    .size   MachineSoftware_Handler, . - MachineSoftware_Handler


    .weak   UserTimer_Handler
    .type   UserTimer_Handler, %function
UserTimer_Handler:
    j       .
    .size   UserTimer_Handler, . - UserTimer_Handler


    .weak   SuperVisorTimer_Handler
    .type   SuperVisorTimer_Handler, %function
SuperVisorTimer_Handler:
    j       .
    .size   SuperVisorTimer_Handler, . - SuperVisorTimer_Handler


    .weak   MachineTimer_Handler
    .type   MachineTimer_Handler, %function
MachineTimer_Handler:
    j       .
    .size   MachineTimer_Handler, . - MachineTimer_Handler


    .weak   UserExternal_Handler
    .type   UserExternal_Handler, %function
UserExternal_Handler:
    j       .
    .size   UserExternal_Handler, . - UserExternal_Handler


    .weak   SuperVisorExternal_Handler
    .type   SuperVisorExternal_Handler, %function
SuperVisorExternal_Handler:
    j       .
    .size   SuperVisorExternal_Handler, . - SuperVisorExternal_Handler


    .weak   MachineExternal_Handler
    .type   MachineExternal_Handler, %function
MachineExternal_Handler:
    j       .
    .size   MachineExternal_Handler, . - MachineExternal_Handler


    .weak   CLICSoftware_Handler
    .type   CLICSoftware_Handler, %function
CLICSoftware_Handler:
    j       .
    .size   CLICSoftware_Handler, . - CLICSoftware_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    j       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ  VPRCLIC_0_IRQHandler
    IRQ  VPRCLIC_1_IRQHandler
    IRQ  VPRCLIC_2_IRQHandler
    IRQ  VPRCLIC_3_IRQHandler
    IRQ  VPRCLIC_4_IRQHandler
    IRQ  VPRCLIC_5_IRQHandler
    IRQ  VPRCLIC_6_IRQHandler
    IRQ  VPRCLIC_7_IRQHandler
    IRQ  VPRCLIC_8_IRQHandler
    IRQ  VPRCLIC_9_IRQHandler
    IRQ  VPRCLIC_10_IRQHandler
    IRQ  VPRCLIC_11_IRQHandler
    IRQ  VPRCLIC_12_IRQHandler
    IRQ  VPRCLIC_13_IRQHandler
    IRQ  VPRCLIC_14_IRQHandler
    IRQ  VPRCLIC_15_IRQHandler
    IRQ  VPRTIM_IRQHandler
    IRQ  GPIOTE130_0_IRQHandler
    IRQ  GPIOTE130_1_IRQHandler
    IRQ  GPIOTE131_0_IRQHandler
    IRQ  GPIOTE131_1_IRQHandler
    IRQ  GRTC_0_IRQHandler
    IRQ  GRTC_1_IRQHandler
    IRQ  GRTC_2_IRQHandler
    IRQ  TBM_IRQHandler
    IRQ  USBHS_IRQHandler
    IRQ  EXMIF_IRQHandler
    IRQ  IPCT120_0_IRQHandler
    IRQ  I3C120_IRQHandler
    IRQ  VPR121_IRQHandler
    IRQ  CAN120_IRQHandler
    IRQ  MVDMA120_IRQHandler
    IRQ  CAN121_IRQHandler
    IRQ  MVDMA121_IRQHandler
    IRQ  I3C121_IRQHandler
    IRQ  TIMER120_IRQHandler
    IRQ  TIMER121_IRQHandler
    IRQ  PWM120_IRQHandler
    IRQ  SPIS120_IRQHandler
    IRQ  SPIM120_UARTE120_IRQHandler
    IRQ  SPIM121_IRQHandler
    IRQ  VPR130_IRQHandler
    IRQ  IPCT130_0_IRQHandler
    IRQ  RTC130_IRQHandler
    IRQ  RTC131_IRQHandler
    IRQ  WDT131_IRQHandler
    IRQ  WDT132_IRQHandler
    IRQ  EGU130_IRQHandler
    IRQ  SAADC_IRQHandler
    IRQ  COMP_LPCOMP_IRQHandler
    IRQ  TEMP_IRQHandler
    IRQ  I2S130_IRQHandler
    IRQ  PDM_IRQHandler
    IRQ  QDEC130_IRQHandler
    IRQ  QDEC131_IRQHandler
    IRQ  I2S131_IRQHandler
    IRQ  TIMER130_IRQHandler
    IRQ  TIMER131_IRQHandler
    IRQ  PWM130_IRQHandler
    IRQ  SERIAL0_IRQHandler
    IRQ  SERIAL1_IRQHandler
    IRQ  TIMER132_IRQHandler
    IRQ  TIMER133_IRQHandler
    IRQ  PWM131_IRQHandler
    IRQ  SERIAL2_IRQHandler
    IRQ  SERIAL3_IRQHandler
    IRQ  TIMER134_IRQHandler
    IRQ  TIMER135_IRQHandler
    IRQ  PWM132_IRQHandler
    IRQ  SERIAL4_IRQHandler
    IRQ  SERIAL5_IRQHandler
    IRQ  TIMER136_IRQHandler
    IRQ  TIMER137_IRQHandler
    IRQ  PWM133_IRQHandler
    IRQ  SERIAL6_IRQHandler
    IRQ  SERIAL7_IRQHandler

  .end
