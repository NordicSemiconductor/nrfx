/*
 * Copyright (c) 2026, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <nrfx.h>

#if defined(NRF_APPLICATION)
#define FICR_MEMCONF_BLOCKTYPE NRF_FICR->TRIM.APPLICATION.MEMCONF.BLOCKTYPE
#elif defined(NRF_RADIOCORE)
#define FICR_MEMCONF_BLOCKTYPE NRF_FICR->TRIM.RADIOCORE.MEMCONF.BLOCKTYPE
#else
#error "Unsupported core."
#endif /* defined(NRF_APPLICATION) || defined(NRF_RADIOCORE) */

#define TRIM_STRIDE     sizeof(FICR_MEMCONF_BLOCKTYPE[0])
#define TRIM_END_OFFSET sizeof(FICR_MEMCONF_BLOCKTYPE)

/* Value of FICR trim register indicating empty value. */
#define FICR_TRIM_EMPTY 0xffffffff

/* nrfx_memconf_trim_apply assumes registers are of equal size. */
NRFX_STATIC_ASSERT(sizeof(NRF_MEMCONF->BLOCKTYPE[0]) == sizeof(FICR_MEMCONF_BLOCKTYPE[0]));

#if defined (__GNUC__) && defined(ISA_ARM)
/* Extended inline assembly is forbidden in naked functions and in the file scope so to pass addresses
 * of registers to the assembly code, there needs to be a separate function that is never called and contains
 * extended inline assembly where the addresses of registers are defined with .equ directive.
 */
__USED static void nrfx_memconf_trim_equs(void)
{
    __ASM volatile(
        ".equ FICR_MEMCONF_BLOCKTYPE0_TRIM_ADDR, %c0 \n"
        ".equ MEMCONF_BLOCKTYPE0_TRIM_ADDR,      %c1 \n"
        ".equ TRIM_STRIDE,                       %c2 \n"
        ".equ TRIM_END_OFFSET,                   %c3 \n"
        ".equ FICR_TRIM_EMPTY,                   %c4 \n"
        ::
        "i" (&FICR_MEMCONF_BLOCKTYPE[0].TRIM),
        "i" (&NRF_MEMCONF->BLOCKTYPE[0].TRIM),
        "i" (TRIM_STRIDE),
        "i" (TRIM_END_OFFSET),
        "i" (FICR_TRIM_EMPTY));
}

/* To avoid accessing RAM, the function is implemented as a naked function, and it uses only
 * registers r0–r3 to store variables.
 */
__attribute__((naked, noinline)) void nrfx_memconf_trim_apply(void)
{
    __ASM volatile(
        ".syntax unified                            \n"
        /* Load value corresponding to empty trim (0xFFFFFFFF) to r0. */
        "ldr r0, =FICR_TRIM_EMPTY                   \n"
        /* r1 will be used as the offset for the trim registers, starting from 0. */
        "mov r1, #0                                 \n"
        /* Loop over all trim registers. */
        "nrfx_memconf_trim_loop:                    \n"
        /* Load the address of the register holding the MEMCONF trim in FICR to r2. */
        "ldr r2, =FICR_MEMCONF_BLOCKTYPE0_TRIM_ADDR \n"
        "add r2, r2, r1                             \n"
        /* Load the trim value from FICR to r3. */
        "ldr r3, [r2]                               \n"
        /* Compare the trim value with empty value. */
        "cmp r3, r0                                 \n"
        /* If the trim value is empty, skip writing to MEMCONF. */
        "beq nrfx_memconf_trim_continue             \n"
        /* Otherwise, load the address of the MEMCONF trim register to r2. */
        "ldr r2, =MEMCONF_BLOCKTYPE0_TRIM_ADDR      \n"
        "add r2, r2, r1                             \n"
        /* Write the trim value to the MEMCONF trim register. */
        "str r3, [r2]                               \n"
        "nrfx_memconf_trim_continue:                \n"
        /* Move to the next trim register. */
        "add r1, r1, TRIM_STRIDE                    \n"
        /* Check if all trim registers have been processed. */
        "cmp r1, TRIM_END_OFFSET                    \n"
        "blt nrfx_memconf_trim_loop                 \n"
        "bx lr                                      \n"
    );
}
#else
#error "Unsupported compiler or architecture."
#endif /* defined (__GNUC__) && defined(ISA_ARM) */
