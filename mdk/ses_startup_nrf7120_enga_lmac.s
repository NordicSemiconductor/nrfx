/***********************************************************************************
 *                           SEGGER Microcontroller GmbH                           *
 *                               The Embedded Experts                              *
 ***********************************************************************************
 *                                                                                 *
 *                   (c) 2014 - 2018 SEGGER Microcontroller GmbH                   *
 *                                                                                 *
 *                  www.segger.com     Support: support@segger.com                 *
 *                                                                                 *
 ***********************************************************************************
 *                                                                                 *
 *        All rights reserved.                                                     *
 *                                                                                 *
 *        Redistribution and use in source and binary forms, with or               *
 *        without modification, are permitted provided that the following          *
 *        conditions are met:                                                      *
 *                                                                                 *
 *        - Redistributions of source code must retain the above copyright         *
 *          notice, this list of conditions and the following disclaimer.          *
 *                                                                                 *
 *        - Neither the name of SEGGER Microcontroller GmbH                        *
 *          nor the names of its contributors may be used to endorse or            *
 *          promote products derived from this software without specific           *
 *          prior written permission.                                              *
 *                                                                                 *
 *        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND                   *
 *        CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,              *
 *        INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF                 *
 *        MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                 *
 *        DISCLAIMED.                                                              *
 *        IN NO EVENT SHALL SEGGER Microcontroller GmbH BE LIABLE FOR              *
 *        ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR                 *
 *        CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT        *
 *        OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;          *
 *        OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF            *
 *        LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                *
 *        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE        *
 *        USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH         *
 *        DAMAGE.                                                                  *
 *                                                                                 *
 ***********************************************************************************/

/************************************************************************************
 *                         Preprocessor Definitions                                 *
 *                         ------------------------                                 *
 * VECTORS_IN_RAM                                                                   *
 *                                                                                  *
 *   If defined, an area of RAM will large enough to store the vector table         *
 *   will be reserved.                                                              *
 *                                                                                  *
 ************************************************************************************/

  .section .init, "ax"
  .align 0


/************************************************************************************
 * Macros                                                                           *
 ************************************************************************************/

// Declare an exception handler with a weak definition
.macro EXC_HANDLER Name=
        // Insert vector in vector table
        .section .vectors, "ax"
        .word \Name
        // Insert dummy handler in init section
        .section .init.\Name, "ax"
        .weak \Name
        .balign 2
\Name:
        j .   // Endless loop
.endm

// Declare an interrupt handler with a weak definition
.macro ISR_HANDLER Name=
        // Insert vector in vector table
        .section .vectors, "ax"
        .word \Name
        // Insert dummy handler in init section
#if defined(__OPTIMIZATION_SMALL)
        .section .init, "ax"
        .weak \Name
        .thumb_set \Name,Dummy_Handler
#else
        .section .init.\Name, "ax"
        .weak \Name
        .balign 2
\Name:
        j .   // Endless loop
#endif
.endm

// Place a reserved vector in vector table
.macro ISR_RESERVED
        .section .vectors, "ax"
        .word 0
.endm

// Place a reserved vector in vector table
.macro ISR_RESERVED_DUMMY
        .section .vectors, "ax"
        .word Dummy_Handler
.endm

/************************************************************************************
 * Reset Handler Extensions                                                         *
 ************************************************************************************/

  .extern Reset_Handler
  .global nRFInitialize
  .extern afterInitialize

nRFInitialize:
    ret
 
 
/************************************************************************************
 * Vector Table                                                                     *
 ************************************************************************************/

  .section .vectors, "ax"
  .balign 8
  .global _vectors

_vectors:
  EXC_HANDLER   UserSoftware_Handler
  EXC_HANDLER   SuperVisorSoftware_Handler
  EXC_HANDLER   MachineSoftware_Handler
  ISR_RESERVED                           /* Reserved */
  EXC_HANDLER   UserTimer_Handler
  EXC_HANDLER   SuperVisorTimer_Handler
  ISR_RESERVED                           /* Reserved */
  EXC_HANDLER   MachineTimer_Handler
  EXC_HANDLER   UserExternal_Handler
  EXC_HANDLER   SuperVisorExternal_Handler
  ISR_RESERVED                           /* Reserved */
  EXC_HANDLER   MachineExternal_Handler
  EXC_HANDLER   CLICSoftware_Handler
  ISR_RESERVED                           /* Reserved */
  ISR_RESERVED                           /* Reserved */
  ISR_RESERVED                           /* Reserved */

/* External Interrupts */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   VPRCLIC_16_IRQHandler
  ISR_HANDLER   VPRCLIC_17_IRQHandler
  ISR_HANDLER   VPRCLIC_18_IRQHandler
  ISR_HANDLER   VPRCLIC_19_IRQHandler
  ISR_HANDLER   VPRCLIC_20_IRQHandler
  ISR_HANDLER   VPRCLIC_21_IRQHandler
  ISR_HANDLER   VPRCLIC_22_IRQHandler
  ISR_HANDLER   VPRCLIC_23_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   UMAC_VPR_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   MVDMA_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   SERIAL00_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   SERIAL01_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   SERIAL20_IRQHandler
  ISR_HANDLER   SERIAL21_IRQHandler
  ISR_HANDLER   SERIAL22_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   GRTC_4_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   SERIAL23_IRQHandler
  ISR_HANDLER   SERIAL24_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_HANDLER   SERIAL30_IRQHandler
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
  ISR_RESERVED_DUMMY                           /* Reserved */
_vectors_end:

#ifdef VECTORS_IN_RAM
  .section .vectors_ram, "ax"
  .align 0
  .global _vectors_ram

_vectors_ram:
  .space _vectors_end - _vectors, 0
#endif

/*********************************************************************
*
*  Dummy handler to be used for reserved interrupt vectors
*  and weak implementation of interrupts.
*
*/
        .section .init.Dummy_Handler, "ax"
        .weak Dummy_Handler
        .balign 2
Dummy_Handler:
        j .   // Endless loop

