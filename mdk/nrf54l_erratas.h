#ifndef NRF54L_ERRATAS_H
#define NRF54L_ERRATAS_H

/*

Copyright (c) 2010 - 2024, Nordic Semiconductor ASA All rights reserved.

SPDX-License-Identifier: BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of Nordic Semiconductor ASA nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/

#include <stdint.h>
#include <stdbool.h>
#include "compiler_abstraction.h"

static bool nrf54l_errata_1(void) __UNUSED;
static bool nrf54l_errata_2(void) __UNUSED;
static bool nrf54l_errata_3(void) __UNUSED;
static bool nrf54l_errata_4(void) __UNUSED;
static bool nrf54l_errata_6(void) __UNUSED;
static bool nrf54l_errata_7(void) __UNUSED;
static bool nrf54l_errata_8(void) __UNUSED;
static bool nrf54l_errata_9(void) __UNUSED;
static bool nrf54l_errata_10(void) __UNUSED;
static bool nrf54l_errata_13(void) __UNUSED;
static bool nrf54l_errata_16(void) __UNUSED;
static bool nrf54l_errata_17(void) __UNUSED;
static bool nrf54l_errata_18(void) __UNUSED;
static bool nrf54l_errata_19(void) __UNUSED;
static bool nrf54l_errata_20(void) __UNUSED;
static bool nrf54l_errata_21(void) __UNUSED;
static bool nrf54l_errata_22(void) __UNUSED;
static bool nrf54l_errata_23(void) __UNUSED;
static bool nrf54l_errata_24(void) __UNUSED;
static bool nrf54l_errata_25(void) __UNUSED;
static bool nrf54l_errata_26(void) __UNUSED;
static bool nrf54l_errata_27(void) __UNUSED;
static bool nrf54l_errata_30(void) __UNUSED;
static bool nrf54l_errata_31(void) __UNUSED;
static bool nrf54l_errata_32(void) __UNUSED;

/* ========= Errata 1 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_1_PRESENT 1
#else
    #define NRF54L_ERRATA_1_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_1_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_1_ENABLE_WORKAROUND NRF54L_ERRATA_1_PRESENT
#endif

static bool nrf54l_errata_1(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 2 ========= */
#define NRF54L_ERRATA_2_PRESENT 0

#ifndef NRF54L_ERRATA_2_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_2_ENABLE_WORKAROUND NRF54L_ERRATA_2_PRESENT
#endif

static bool nrf54l_errata_2(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 3 ========= */
#define NRF54L_ERRATA_3_PRESENT 0

#ifndef NRF54L_ERRATA_3_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_3_ENABLE_WORKAROUND NRF54L_ERRATA_3_PRESENT
#endif

static bool nrf54l_errata_3(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 4 ========= */
#define NRF54L_ERRATA_4_PRESENT 0

#ifndef NRF54L_ERRATA_4_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_4_ENABLE_WORKAROUND NRF54L_ERRATA_4_PRESENT
#endif

static bool nrf54l_errata_4(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 6 ========= */
#define NRF54L_ERRATA_6_PRESENT 0

#ifndef NRF54L_ERRATA_6_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_6_ENABLE_WORKAROUND NRF54L_ERRATA_6_PRESENT
#endif

static bool nrf54l_errata_6(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 7 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_7_PRESENT 1
#else
    #define NRF54L_ERRATA_7_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_7_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_7_ENABLE_WORKAROUND NRF54L_ERRATA_7_PRESENT
#endif

static bool nrf54l_errata_7(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 8 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_8_PRESENT 1
#else
    #define NRF54L_ERRATA_8_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_8_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_8_ENABLE_WORKAROUND NRF54L_ERRATA_8_PRESENT
#endif

static bool nrf54l_errata_8(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 9 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_9_PRESENT 1
#else
    #define NRF54L_ERRATA_9_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_9_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_9_ENABLE_WORKAROUND NRF54L_ERRATA_9_PRESENT
#endif

static bool nrf54l_errata_9(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 10 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_10_PRESENT 1
#else
    #define NRF54L_ERRATA_10_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_10_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_10_ENABLE_WORKAROUND NRF54L_ERRATA_10_PRESENT
#endif

static bool nrf54l_errata_10(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 13 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_13_PRESENT 1
#else
    #define NRF54L_ERRATA_13_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_13_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_13_ENABLE_WORKAROUND NRF54L_ERRATA_13_PRESENT
#endif

static bool nrf54l_errata_13(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 16 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_16_PRESENT 1
#else
    #define NRF54L_ERRATA_16_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_16_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_16_ENABLE_WORKAROUND NRF54L_ERRATA_16_PRESENT
#endif

static bool nrf54l_errata_16(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 17 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_17_PRESENT 1
#else
    #define NRF54L_ERRATA_17_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_17_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_17_ENABLE_WORKAROUND NRF54L_ERRATA_17_PRESENT
#endif

static bool nrf54l_errata_17(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 18 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_18_PRESENT 1
#else
    #define NRF54L_ERRATA_18_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_18_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_18_ENABLE_WORKAROUND NRF54L_ERRATA_18_PRESENT
#endif

static bool nrf54l_errata_18(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 19 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_19_PRESENT 1
#else
    #define NRF54L_ERRATA_19_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_19_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_19_ENABLE_WORKAROUND NRF54L_ERRATA_19_PRESENT
#endif

static bool nrf54l_errata_19(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 20 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_20_PRESENT 1
#else
    #define NRF54L_ERRATA_20_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_20_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_20_ENABLE_WORKAROUND NRF54L_ERRATA_20_PRESENT
#endif

static bool nrf54l_errata_20(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 21 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_21_PRESENT 1
#else
    #define NRF54L_ERRATA_21_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_21_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_21_ENABLE_WORKAROUND NRF54L_ERRATA_21_PRESENT
#endif

static bool nrf54l_errata_21(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 22 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_22_PRESENT 1
#else
    #define NRF54L_ERRATA_22_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_22_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_22_ENABLE_WORKAROUND NRF54L_ERRATA_22_PRESENT
#endif

static bool nrf54l_errata_22(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 23 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_23_PRESENT 1
#else
    #define NRF54L_ERRATA_23_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_23_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_23_ENABLE_WORKAROUND NRF54L_ERRATA_23_PRESENT
#endif

static bool nrf54l_errata_23(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 24 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_24_PRESENT 1
#else
    #define NRF54L_ERRATA_24_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_24_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_24_ENABLE_WORKAROUND NRF54L_ERRATA_24_PRESENT
#endif

static bool nrf54l_errata_24(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 25 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_25_PRESENT 1
#else
    #define NRF54L_ERRATA_25_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_25_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_25_ENABLE_WORKAROUND NRF54L_ERRATA_25_PRESENT
#endif

static bool nrf54l_errata_25(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 26 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_26_PRESENT 1
#else
    #define NRF54L_ERRATA_26_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_26_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_26_ENABLE_WORKAROUND NRF54L_ERRATA_26_PRESENT
#endif

static bool nrf54l_errata_26(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 27 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_27_PRESENT 1
#else
    #define NRF54L_ERRATA_27_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_27_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_27_ENABLE_WORKAROUND NRF54L_ERRATA_27_PRESENT
#endif

static bool nrf54l_errata_27(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 30 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_30_PRESENT 1
#else
    #define NRF54L_ERRATA_30_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_30_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_30_ENABLE_WORKAROUND NRF54L_ERRATA_30_PRESENT
#endif

static bool nrf54l_errata_30(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 31 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_31_PRESENT 1
#else
    #define NRF54L_ERRATA_31_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_31_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_31_ENABLE_WORKAROUND NRF54L_ERRATA_31_PRESENT
#endif

static bool nrf54l_errata_31(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 32 ========= */
#if    defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
    #define NRF54L_ERRATA_32_PRESENT 1
#else
    #define NRF54L_ERRATA_32_PRESENT 0
#endif

#ifndef NRF54L_ERRATA_32_ENABLE_WORKAROUND
    #define NRF54L_ERRATA_32_ENABLE_WORKAROUND NRF54L_ERRATA_32_PRESENT
#endif

static bool nrf54l_errata_32(void)
{
    #ifndef NRF54L_SERIES
        return false;
    #else
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF54L15_XXAA) || defined (DEVELOP_IN_NRF54L15)
            if (var1 == 0x1C)
            {
                switch(var2)
                {
                    case 0x01ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

#endif /* NRF54L_ERRATAS_H */
