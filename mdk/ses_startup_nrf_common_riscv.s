/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2020 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File      : ses_startup_nrf_common.s
Purpose   : Startup and exception handlers for Nordic Semi devices.

Additional information:
  Preprocessor Definitions
    __NO_SYSTEM_INIT
      If defined,
        SystemInit is not called.
      If not defined,
        SystemInit is called.
        SystemInit is usually supplied by the CMSIS files.
        This file declares a weak implementation as fallback.

    __MEMORY_INIT
      If defined,
        MemoryInit is called after SystemInit.
        void MemoryInit(void) can be implemented to enable external
        memory controllers.

    __VECTORS_IN_RAM
      If defined,
        the vector table will be copied from Flash to RAM,
        and the vector table offset register is adjusted.

    __NO_STACK_INIT
      If defined,
        the stack pointer is not initialized.

    __NO_MTVT_CONFIG
      If defined,
        the mtvt register is not initialized.

*/

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
/*********************************************************************
*
*       Reset_Handler
*
*  Function description
*    Exception handler for reset.
*    Generic bringup of a RISC-V system.
*/
        .global reset_handler
        .global Reset_Handler
        .equ reset_handler, Reset_Handler
        .extern nRFInitialize
        .global afterInitialize

        .section .init.reset, "ax"
        .balign 2

Reset_Handler:

        /* Perform init tasks that must be done in Assembly. */
        jal ra, nRFInitialize

#ifndef __NO_STACK_INIT
        #ifndef STACK_INIT_VAL
            #define STACK_INIT_VAL __stack_end__
        #endif
        //
        // Initialise main stack
        //
        la t0, STACK_INIT_VAL
        li t1, 0xFFFFFFF8
        and sp, t0, t1
        add s0, sp, zero
#endif

#ifndef __NO_SYSTEM_INIT
        //
        // Call SystemInit
        //
        jal ra, SystemInit
#endif
#ifdef __MEMORY_INIT
        //
        // Call MemoryInit
        //
        jal ra, MemoryInit
#endif

#ifdef __VECTORS_IN_RAM
        //
        // Copy vector table (from Flash) to RAM
        //
        la t0, __vectors_start__
        la t1, __vectors_end__
        la t2, __vectors_ram_start__
1:
        beq t0, t1, 2f
        lw a0, 0(t0)
        sw a0, 0(t2)
        add t0, t0, 4
        add t2, t2, 4
        j 1b
2:
#endif

#ifndef __NO_MTVT_CONFIG
        //
        // Configure machine trap vector table register
        //
#ifdef __VECTORS_IN_RAM
        la t0, _vectors_ram
#else
        la t0, _vectors
#endif
        lw t0, 0(t0)
        csrw 0x307, t0
#endif

        //
        // Call runtime initialization, which calls main().
        //
        j _start

        //
        // Weak only declaration of SystemInit enables Linker to replace bl SystemInit with a NOP,
        // when there is no strong definition of SystemInit.
        //
        .weak SystemInit
        //
        // Place SystemCoreClockUpdate in .init_array
        // to be called after runtime initialization
        //
#ifndef __NO_SYSTEM_INIT
        .section .init_array, "aw"
        .balign 4
        .word   SystemCoreClockUpdate
#endif

// Interrupt return handling
    .section .init.isr_return, "ax"
_return:
    mret
#ifdef INITIALIZE_USER_SECTIONS
    .global InitializeUserMemorySections
    .section .init, "ax"
InitializeUserMemorySections:
    la t0, __start_nrf_sections
    la t1, __start_nrf_sections_run
    la t2, __end_nrf_sections_run

1:
    beq t0, t1, 2f
    lw a0, 0(t0)
    sw a0, 0(t2)
    add t0, t0, 4
    add t2, t2, 4
    j 1b
2:

  cmp r0, r1
  beq 2f
  subs r2, r2, r1
  beq 2f
1:
  ldrb r3, [r0]
  adds r0, r0, #1
  strb r3, [r1]
  adds r1, r1, #1
  subs r2, r2, #1
  bne 1b
2:
  bx lr
#endif
