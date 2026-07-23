#ifndef NRF71_ERRATAS_H
#define NRF71_ERRATAS_H

/*

Copyright (c) 2010 - 2026, Nordic Semiconductor ASA All rights reserved.

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
#include "../common/compiler_abstraction.h"

static inline bool nrf71_errata_1(void);
static inline bool nrf71_errata_2(void);
static inline bool nrf71_errata_3(void);
static inline bool nrf71_errata_4(void);
static inline bool nrf71_errata_6(void);
static inline bool nrf71_errata_7(void);
static inline bool nrf71_errata_8(void);
static inline bool nrf71_errata_9(void);
static inline bool nrf71_errata_10(void);
static inline bool nrf71_errata_13(void);
static inline bool nrf71_errata_15(void);
static inline bool nrf71_errata_16(void);
static inline bool nrf71_errata_17(void);
static inline bool nrf71_errata_18(void);
static inline bool nrf71_errata_19(void);
static inline bool nrf71_errata_20(void);
static inline bool nrf71_errata_21(void);
static inline bool nrf71_errata_22(void);
static inline bool nrf71_errata_23(void);
static inline bool nrf71_errata_24(void);
static inline bool nrf71_errata_25(void);
static inline bool nrf71_errata_26(void);
static inline bool nrf71_errata_27(void);
static inline bool nrf71_errata_28(void);
static inline bool nrf71_errata_30(void);
static inline bool nrf71_errata_31(void);
static inline bool nrf71_errata_32(void);
static inline bool nrf71_errata_33(void);
static inline bool nrf71_errata_35(void);
static inline bool nrf71_errata_37(void);
static inline bool nrf71_errata_38(void);
static inline bool nrf71_errata_39(void);
static inline bool nrf71_errata_40(void);
static inline bool nrf71_errata_41(void);
static inline bool nrf71_errata_42(void);
static inline bool nrf71_errata_43(void);
static inline bool nrf71_errata_44(void);
static inline bool nrf71_errata_45(void);
static inline bool nrf71_errata_46(void);
static inline bool nrf71_errata_47(void);
static inline bool nrf71_errata_48(void);
static inline bool nrf71_errata_49(void);
static inline bool nrf71_errata_50(void);
static inline bool nrf71_errata_54(void);
static inline bool nrf71_errata_55(void);
static inline bool nrf71_configuration_56(void);
static inline bool nrf71_errata_57(void);
static inline bool nrf71_errata_58(void);
static inline bool nrf71_errata_59(void);
static inline bool nrf71_errata_60(void);
static inline bool nrf71_errata_63(void);
static inline bool nrf71_errata_64(void);
static inline bool nrf71_errata_65(void);
static inline bool nrf71_errata_66(void);
static inline bool nrf71_errata_68(void);
static inline bool nrf71_errata_69(void);
static inline bool nrf71_errata_70(void);
static inline bool nrf71_errata_71(void);
static inline bool nrf71_errata_72(void);
static inline bool nrf71_errata_73(void);
static inline bool nrf71_errata_74(void);
static inline bool nrf71_errata_78(void);
static inline bool nrf71_errata_79(void);
static inline bool nrf71_errata_81(void);
static inline bool nrf71_errata_83(void);
static inline bool nrf71_errata_84(void);
static inline bool nrf71_errata_85(void);
static inline bool nrf71_errata_86(void);
static inline bool nrf71_errata_87(void);
static inline bool nrf71_errata_88(void);
static inline bool nrf71_errata_89(void);
static inline bool nrf71_errata_90(void);
static inline bool nrf71_errata_91(void);
static inline bool nrf71_errata_92(void);
static inline bool nrf71_errata_93(void);
static inline bool nrf71_errata_94(void);
static inline bool nrf71_errata_95(void);
static inline bool nrf71_errata_100(void);
static inline bool nrf71_errata_101(void);
static inline bool nrf71_errata_102(void);
static inline bool nrf71_errata_103(void);
static inline bool nrf71_errata_104(void);
static inline bool nrf71_errata_105(void);
static inline bool nrf71_errata_106(void);
static inline bool nrf71_errata_109(void);
static inline bool nrf71_errata_111(void);
static inline bool nrf71_errata_112(void);
static inline bool nrf71_errata_114(void);
static inline bool nrf71_errata_115(void);
static inline bool nrf71_errata_116(void);
static inline bool nrf71_errata_121(void);
static inline bool nrf71_errata_126(void);
static inline bool nrf71_errata_132(void);

/* ========= Errata 1 ========= */
#define NRF71_ERRATA_1_PRESENT 0

#ifndef NRF71_ERRATA_1_ENABLE_WORKAROUND
    #define NRF71_ERRATA_1_ENABLE_WORKAROUND NRF71_ERRATA_1_PRESENT
#endif

static inline bool nrf71_errata_1(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 2 ========= */
#define NRF71_ERRATA_2_PRESENT 0

#ifndef NRF71_ERRATA_2_ENABLE_WORKAROUND
    #define NRF71_ERRATA_2_ENABLE_WORKAROUND NRF71_ERRATA_2_PRESENT
#endif

static inline bool nrf71_errata_2(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 3 ========= */
#define NRF71_ERRATA_3_PRESENT 0

#ifndef NRF71_ERRATA_3_ENABLE_WORKAROUND
    #define NRF71_ERRATA_3_ENABLE_WORKAROUND NRF71_ERRATA_3_PRESENT
#endif

static inline bool nrf71_errata_3(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 4 ========= */
#define NRF71_ERRATA_4_PRESENT 0

#ifndef NRF71_ERRATA_4_ENABLE_WORKAROUND
    #define NRF71_ERRATA_4_ENABLE_WORKAROUND NRF71_ERRATA_4_PRESENT
#endif

static inline bool nrf71_errata_4(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 6 ========= */
#if    defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
    #define NRF71_ERRATA_6_PRESENT 1
#else
    #define NRF71_ERRATA_6_PRESENT 0
#endif

#ifndef NRF71_ERRATA_6_ENABLE_WORKAROUND
    #define NRF71_ERRATA_6_ENABLE_WORKAROUND NRF71_ERRATA_6_PRESENT
#endif

static inline bool nrf71_errata_6(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            if (var1 == 0x2C)
            {
                switch(var2)
                {
                    case 0x00ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 7 ========= */
#if    defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
    #define NRF71_ERRATA_7_PRESENT 1
#else
    #define NRF71_ERRATA_7_PRESENT 0
#endif

#ifndef NRF71_ERRATA_7_ENABLE_WORKAROUND
    #define NRF71_ERRATA_7_ENABLE_WORKAROUND NRF71_ERRATA_7_PRESENT
#endif

static inline bool nrf71_errata_7(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            if (var1 == 0x2C)
            {
                switch(var2)
                {
                    case 0x00ul:
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
#define NRF71_ERRATA_8_PRESENT 0

#ifndef NRF71_ERRATA_8_ENABLE_WORKAROUND
    #define NRF71_ERRATA_8_ENABLE_WORKAROUND NRF71_ERRATA_8_PRESENT
#endif

static inline bool nrf71_errata_8(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 9 ========= */
#define NRF71_ERRATA_9_PRESENT 0

#ifndef NRF71_ERRATA_9_ENABLE_WORKAROUND
    #define NRF71_ERRATA_9_ENABLE_WORKAROUND NRF71_ERRATA_9_PRESENT
#endif

static inline bool nrf71_errata_9(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 10 ========= */
#define NRF71_ERRATA_10_PRESENT 0

#ifndef NRF71_ERRATA_10_ENABLE_WORKAROUND
    #define NRF71_ERRATA_10_ENABLE_WORKAROUND NRF71_ERRATA_10_PRESENT
#endif

static inline bool nrf71_errata_10(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 13 ========= */
#define NRF71_ERRATA_13_PRESENT 0

#ifndef NRF71_ERRATA_13_ENABLE_WORKAROUND
    #define NRF71_ERRATA_13_ENABLE_WORKAROUND NRF71_ERRATA_13_PRESENT
#endif

static inline bool nrf71_errata_13(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 15 ========= */
#define NRF71_ERRATA_15_PRESENT 0

#ifndef NRF71_ERRATA_15_ENABLE_WORKAROUND
    #define NRF71_ERRATA_15_ENABLE_WORKAROUND NRF71_ERRATA_15_PRESENT
#endif

static inline bool nrf71_errata_15(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 16 ========= */
#define NRF71_ERRATA_16_PRESENT 0

#ifndef NRF71_ERRATA_16_ENABLE_WORKAROUND
    #define NRF71_ERRATA_16_ENABLE_WORKAROUND NRF71_ERRATA_16_PRESENT
#endif

static inline bool nrf71_errata_16(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 17 ========= */
#define NRF71_ERRATA_17_PRESENT 0

#ifndef NRF71_ERRATA_17_ENABLE_WORKAROUND
    #define NRF71_ERRATA_17_ENABLE_WORKAROUND NRF71_ERRATA_17_PRESENT
#endif

static inline bool nrf71_errata_17(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 18 ========= */
#define NRF71_ERRATA_18_PRESENT 0

#ifndef NRF71_ERRATA_18_ENABLE_WORKAROUND
    #define NRF71_ERRATA_18_ENABLE_WORKAROUND NRF71_ERRATA_18_PRESENT
#endif

static inline bool nrf71_errata_18(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 19 ========= */
#define NRF71_ERRATA_19_PRESENT 0

#ifndef NRF71_ERRATA_19_ENABLE_WORKAROUND
    #define NRF71_ERRATA_19_ENABLE_WORKAROUND NRF71_ERRATA_19_PRESENT
#endif

static inline bool nrf71_errata_19(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 20 ========= */
#define NRF71_ERRATA_20_PRESENT 0

#ifndef NRF71_ERRATA_20_ENABLE_WORKAROUND
    #define NRF71_ERRATA_20_ENABLE_WORKAROUND NRF71_ERRATA_20_PRESENT
#endif

static inline bool nrf71_errata_20(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 21 ========= */
#define NRF71_ERRATA_21_PRESENT 0

#ifndef NRF71_ERRATA_21_ENABLE_WORKAROUND
    #define NRF71_ERRATA_21_ENABLE_WORKAROUND NRF71_ERRATA_21_PRESENT
#endif

static inline bool nrf71_errata_21(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 22 ========= */
#define NRF71_ERRATA_22_PRESENT 0

#ifndef NRF71_ERRATA_22_ENABLE_WORKAROUND
    #define NRF71_ERRATA_22_ENABLE_WORKAROUND NRF71_ERRATA_22_PRESENT
#endif

static inline bool nrf71_errata_22(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 23 ========= */
#define NRF71_ERRATA_23_PRESENT 0

#ifndef NRF71_ERRATA_23_ENABLE_WORKAROUND
    #define NRF71_ERRATA_23_ENABLE_WORKAROUND NRF71_ERRATA_23_PRESENT
#endif

static inline bool nrf71_errata_23(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 24 ========= */
#define NRF71_ERRATA_24_PRESENT 0

#ifndef NRF71_ERRATA_24_ENABLE_WORKAROUND
    #define NRF71_ERRATA_24_ENABLE_WORKAROUND NRF71_ERRATA_24_PRESENT
#endif

static inline bool nrf71_errata_24(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 25 ========= */
#define NRF71_ERRATA_25_PRESENT 0

#ifndef NRF71_ERRATA_25_ENABLE_WORKAROUND
    #define NRF71_ERRATA_25_ENABLE_WORKAROUND NRF71_ERRATA_25_PRESENT
#endif

static inline bool nrf71_errata_25(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 26 ========= */
#define NRF71_ERRATA_26_PRESENT 0

#ifndef NRF71_ERRATA_26_ENABLE_WORKAROUND
    #define NRF71_ERRATA_26_ENABLE_WORKAROUND NRF71_ERRATA_26_PRESENT
#endif

static inline bool nrf71_errata_26(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 27 ========= */
#define NRF71_ERRATA_27_PRESENT 0

#ifndef NRF71_ERRATA_27_ENABLE_WORKAROUND
    #define NRF71_ERRATA_27_ENABLE_WORKAROUND NRF71_ERRATA_27_PRESENT
#endif

static inline bool nrf71_errata_27(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 28 ========= */
#define NRF71_ERRATA_28_PRESENT 0

#ifndef NRF71_ERRATA_28_ENABLE_WORKAROUND
    #define NRF71_ERRATA_28_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_28(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 30 ========= */
#define NRF71_ERRATA_30_PRESENT 0

#ifndef NRF71_ERRATA_30_ENABLE_WORKAROUND
    #define NRF71_ERRATA_30_ENABLE_WORKAROUND NRF71_ERRATA_30_PRESENT
#endif

static inline bool nrf71_errata_30(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 31 ========= */
#define NRF71_ERRATA_31_PRESENT 0

#ifndef NRF71_ERRATA_31_ENABLE_WORKAROUND
    #define NRF71_ERRATA_31_ENABLE_WORKAROUND NRF71_ERRATA_31_PRESENT
#endif

static inline bool nrf71_errata_31(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 32 ========= */
#define NRF71_ERRATA_32_PRESENT 0

#ifndef NRF71_ERRATA_32_ENABLE_WORKAROUND
    #define NRF71_ERRATA_32_ENABLE_WORKAROUND NRF71_ERRATA_32_PRESENT
#endif

static inline bool nrf71_errata_32(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 33 ========= */
#define NRF71_ERRATA_33_PRESENT 0

#ifndef NRF71_ERRATA_33_ENABLE_WORKAROUND
    #define NRF71_ERRATA_33_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_33(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 35 ========= */
#define NRF71_ERRATA_35_PRESENT 0

#ifndef NRF71_ERRATA_35_ENABLE_WORKAROUND
    #define NRF71_ERRATA_35_ENABLE_WORKAROUND NRF71_ERRATA_35_PRESENT
#endif

static inline bool nrf71_errata_35(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 37 ========= */
#define NRF71_ERRATA_37_PRESENT 0

#ifndef NRF71_ERRATA_37_ENABLE_WORKAROUND
    #define NRF71_ERRATA_37_ENABLE_WORKAROUND NRF71_ERRATA_37_PRESENT
#endif

static inline bool nrf71_errata_37(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 38 ========= */
#define NRF71_ERRATA_38_PRESENT 0

#ifndef NRF71_ERRATA_38_ENABLE_WORKAROUND
    #define NRF71_ERRATA_38_ENABLE_WORKAROUND NRF71_ERRATA_38_PRESENT
#endif

static inline bool nrf71_errata_38(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 39 ========= */
#define NRF71_ERRATA_39_PRESENT 0

#ifndef NRF71_ERRATA_39_ENABLE_WORKAROUND
    #define NRF71_ERRATA_39_ENABLE_WORKAROUND NRF71_ERRATA_39_PRESENT
#endif

static inline bool nrf71_errata_39(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 40 ========= */
#define NRF71_ERRATA_40_PRESENT 0

#ifndef NRF71_ERRATA_40_ENABLE_WORKAROUND
    #define NRF71_ERRATA_40_ENABLE_WORKAROUND NRF71_ERRATA_40_PRESENT
#endif

static inline bool nrf71_errata_40(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 41 ========= */
#define NRF71_ERRATA_41_PRESENT 0

#ifndef NRF71_ERRATA_41_ENABLE_WORKAROUND
    #define NRF71_ERRATA_41_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_41(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 42 ========= */
#define NRF71_ERRATA_42_PRESENT 0

#ifndef NRF71_ERRATA_42_ENABLE_WORKAROUND
    #define NRF71_ERRATA_42_ENABLE_WORKAROUND NRF71_ERRATA_42_PRESENT
#endif

static inline bool nrf71_errata_42(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 43 ========= */
#define NRF71_ERRATA_43_PRESENT 0

#ifndef NRF71_ERRATA_43_ENABLE_WORKAROUND
    #define NRF71_ERRATA_43_ENABLE_WORKAROUND NRF71_ERRATA_43_PRESENT
#endif

static inline bool nrf71_errata_43(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 44 ========= */
#if    defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
    #define NRF71_ERRATA_44_PRESENT 1
#else
    #define NRF71_ERRATA_44_PRESENT 0
#endif

#ifndef NRF71_ERRATA_44_ENABLE_WORKAROUND
    #define NRF71_ERRATA_44_ENABLE_WORKAROUND NRF71_ERRATA_44_PRESENT
#endif

static inline bool nrf71_errata_44(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            if (var1 == 0x2C)
            {
                switch(var2)
                {
                    case 0x00ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 45 ========= */
#define NRF71_ERRATA_45_PRESENT 0

#ifndef NRF71_ERRATA_45_ENABLE_WORKAROUND
    #define NRF71_ERRATA_45_ENABLE_WORKAROUND NRF71_ERRATA_45_PRESENT
#endif

static inline bool nrf71_errata_45(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 46 ========= */
#define NRF71_ERRATA_46_PRESENT 0

#ifndef NRF71_ERRATA_46_ENABLE_WORKAROUND
    #define NRF71_ERRATA_46_ENABLE_WORKAROUND NRF71_ERRATA_46_PRESENT
#endif

static inline bool nrf71_errata_46(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 47 ========= */
#define NRF71_ERRATA_47_PRESENT 0

#ifndef NRF71_ERRATA_47_ENABLE_WORKAROUND
    #define NRF71_ERRATA_47_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_47(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 48 ========= */
#define NRF71_ERRATA_48_PRESENT 0

#ifndef NRF71_ERRATA_48_ENABLE_WORKAROUND
    #define NRF71_ERRATA_48_ENABLE_WORKAROUND NRF71_ERRATA_48_PRESENT
#endif

static inline bool nrf71_errata_48(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 49 ========= */
#define NRF71_ERRATA_49_PRESENT 0

#ifndef NRF71_ERRATA_49_ENABLE_WORKAROUND
    #define NRF71_ERRATA_49_ENABLE_WORKAROUND NRF71_ERRATA_49_PRESENT
#endif

static inline bool nrf71_errata_49(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 50 ========= */
#define NRF71_ERRATA_50_PRESENT 0

#ifndef NRF71_ERRATA_50_ENABLE_WORKAROUND
    #define NRF71_ERRATA_50_ENABLE_WORKAROUND NRF71_ERRATA_50_PRESENT
#endif

static inline bool nrf71_errata_50(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 54 ========= */
#define NRF71_ERRATA_54_PRESENT 0

#ifndef NRF71_ERRATA_54_ENABLE_WORKAROUND
    #define NRF71_ERRATA_54_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_54(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 55 ========= */
#if    defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
    #define NRF71_ERRATA_55_PRESENT 1
#else
    #define NRF71_ERRATA_55_PRESENT 0
#endif

#ifndef NRF71_ERRATA_55_ENABLE_WORKAROUND
    #define NRF71_ERRATA_55_ENABLE_WORKAROUND NRF71_ERRATA_55_PRESENT
#endif

static inline bool nrf71_errata_55(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            if (var1 == 0x2C)
            {
                switch(var2)
                {
                    case 0x00ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 56 ========= */
#define NRF71_CONFIGURATION_56_PRESENT 0

#ifndef NRF71_CONFIGURATION_56_ENABLE
    #define NRF71_CONFIGURATION_56_ENABLE NRF71_CONFIGURATION_56_PRESENT
#endif

static inline bool nrf71_configuration_56(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 57 ========= */
#define NRF71_ERRATA_57_PRESENT 0

#ifndef NRF71_ERRATA_57_ENABLE_WORKAROUND
    #define NRF71_ERRATA_57_ENABLE_WORKAROUND NRF71_ERRATA_57_PRESENT
#endif

static inline bool nrf71_errata_57(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 58 ========= */
#define NRF71_ERRATA_58_PRESENT 0

#ifndef NRF71_ERRATA_58_ENABLE_WORKAROUND
    #define NRF71_ERRATA_58_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_58(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 59 ========= */
#define NRF71_ERRATA_59_PRESENT 0

#ifndef NRF71_ERRATA_59_ENABLE_WORKAROUND
    #define NRF71_ERRATA_59_ENABLE_WORKAROUND NRF71_ERRATA_59_PRESENT
#endif

static inline bool nrf71_errata_59(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 60 ========= */
#define NRF71_ERRATA_60_PRESENT 0

#ifndef NRF71_ERRATA_60_ENABLE_WORKAROUND
    #define NRF71_ERRATA_60_ENABLE_WORKAROUND NRF71_ERRATA_60_PRESENT
#endif

static inline bool nrf71_errata_60(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 63 ========= */
#define NRF71_ERRATA_63_PRESENT 0

#ifndef NRF71_ERRATA_63_ENABLE_WORKAROUND
    #define NRF71_ERRATA_63_ENABLE_WORKAROUND NRF71_ERRATA_63_PRESENT
#endif

static inline bool nrf71_errata_63(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 64 ========= */
#define NRF71_ERRATA_64_PRESENT 0

#ifndef NRF71_ERRATA_64_ENABLE_WORKAROUND
    #define NRF71_ERRATA_64_ENABLE_WORKAROUND NRF71_ERRATA_64_PRESENT
#endif

static inline bool nrf71_errata_64(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 65 ========= */
#define NRF71_ERRATA_65_PRESENT 0

#ifndef NRF71_ERRATA_65_ENABLE_WORKAROUND
    #define NRF71_ERRATA_65_ENABLE_WORKAROUND NRF71_ERRATA_65_PRESENT
#endif

static inline bool nrf71_errata_65(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 66 ========= */
#define NRF71_ERRATA_66_PRESENT 0

#ifndef NRF71_ERRATA_66_ENABLE_WORKAROUND
    #define NRF71_ERRATA_66_ENABLE_WORKAROUND NRF71_ERRATA_66_PRESENT
#endif

static inline bool nrf71_errata_66(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 68 ========= */
#define NRF71_ERRATA_68_PRESENT 0

#ifndef NRF71_ERRATA_68_ENABLE_WORKAROUND
    #define NRF71_ERRATA_68_ENABLE_WORKAROUND NRF71_ERRATA_68_PRESENT
#endif

static inline bool nrf71_errata_68(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 69 ========= */
#define NRF71_ERRATA_69_PRESENT 0

#ifndef NRF71_ERRATA_69_ENABLE_WORKAROUND
    #define NRF71_ERRATA_69_ENABLE_WORKAROUND NRF71_ERRATA_69_PRESENT
#endif

static inline bool nrf71_errata_69(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 70 ========= */
#define NRF71_ERRATA_70_PRESENT 0

#ifndef NRF71_ERRATA_70_ENABLE_WORKAROUND
    #define NRF71_ERRATA_70_ENABLE_WORKAROUND NRF71_ERRATA_70_PRESENT
#endif

static inline bool nrf71_errata_70(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 71 ========= */
#define NRF71_ERRATA_71_PRESENT 0

#ifndef NRF71_ERRATA_71_ENABLE_WORKAROUND
    #define NRF71_ERRATA_71_ENABLE_WORKAROUND NRF71_ERRATA_71_PRESENT
#endif

static inline bool nrf71_errata_71(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 72 ========= */
#if    defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
    #define NRF71_ERRATA_72_PRESENT 1
#else
    #define NRF71_ERRATA_72_PRESENT 0
#endif

#ifndef NRF71_ERRATA_72_ENABLE_WORKAROUND
    #define NRF71_ERRATA_72_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_72(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            if (var1 == 0x2C)
            {
                switch(var2)
                {
                    case 0x00ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 73 ========= */
#if    defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
    #define NRF71_ERRATA_73_PRESENT 1
#else
    #define NRF71_ERRATA_73_PRESENT 0
#endif

#ifndef NRF71_ERRATA_73_ENABLE_WORKAROUND
    #define NRF71_ERRATA_73_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_73(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            if (var1 == 0x2C)
            {
                switch(var2)
                {
                    case 0x00ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 74 ========= */
#define NRF71_ERRATA_74_PRESENT 0

#ifndef NRF71_ERRATA_74_ENABLE_WORKAROUND
    #define NRF71_ERRATA_74_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_74(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 78 ========= */
#define NRF71_ERRATA_78_PRESENT 0

#ifndef NRF71_ERRATA_78_ENABLE_WORKAROUND
    #define NRF71_ERRATA_78_ENABLE_WORKAROUND NRF71_ERRATA_78_PRESENT
#endif

static inline bool nrf71_errata_78(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 79 ========= */
#define NRF71_ERRATA_79_PRESENT 0

#ifndef NRF71_ERRATA_79_ENABLE_WORKAROUND
    #define NRF71_ERRATA_79_ENABLE_WORKAROUND NRF71_ERRATA_79_PRESENT
#endif

static inline bool nrf71_errata_79(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 81 ========= */
#define NRF71_ERRATA_81_PRESENT 0

#ifndef NRF71_ERRATA_81_ENABLE_WORKAROUND
    #define NRF71_ERRATA_81_ENABLE_WORKAROUND NRF71_ERRATA_81_PRESENT
#endif

static inline bool nrf71_errata_81(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 83 ========= */
#define NRF71_ERRATA_83_PRESENT 0

#ifndef NRF71_ERRATA_83_ENABLE_WORKAROUND
    #define NRF71_ERRATA_83_ENABLE_WORKAROUND NRF71_ERRATA_83_PRESENT
#endif

static inline bool nrf71_errata_83(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 84 ========= */
#define NRF71_ERRATA_84_PRESENT 0

#ifndef NRF71_ERRATA_84_ENABLE_WORKAROUND
    #define NRF71_ERRATA_84_ENABLE_WORKAROUND NRF71_ERRATA_84_PRESENT
#endif

static inline bool nrf71_errata_84(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 85 ========= */
#define NRF71_ERRATA_85_PRESENT 0

#ifndef NRF71_ERRATA_85_ENABLE_WORKAROUND
    #define NRF71_ERRATA_85_ENABLE_WORKAROUND NRF71_ERRATA_85_PRESENT
#endif

static inline bool nrf71_errata_85(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 86 ========= */
#define NRF71_ERRATA_86_PRESENT 0

#ifndef NRF71_ERRATA_86_ENABLE_WORKAROUND
    #define NRF71_ERRATA_86_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_86(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 87 ========= */
#define NRF71_ERRATA_87_PRESENT 0

#ifndef NRF71_ERRATA_87_ENABLE_WORKAROUND
    #define NRF71_ERRATA_87_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_87(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 88 ========= */
#if    defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
    #define NRF71_ERRATA_88_PRESENT 1
#else
    #define NRF71_ERRATA_88_PRESENT 0
#endif

#ifndef NRF71_ERRATA_88_ENABLE_WORKAROUND
    #define NRF71_ERRATA_88_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_88(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            uint32_t var1 = *(uint32_t *)0x00FFC340ul;
            uint32_t var2 = *(uint32_t *)0x00FFC344ul;
        #endif
        #if defined (NRF7120E_ENGA_XXAA) || defined (DEVELOP_IN_NRF7120E_ENGA)
            if (var1 == 0x2C)
            {
                switch(var2)
                {
                    case 0x00ul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 89 ========= */
#define NRF71_ERRATA_89_PRESENT 0

#ifndef NRF71_ERRATA_89_ENABLE_WORKAROUND
    #define NRF71_ERRATA_89_ENABLE_WORKAROUND NRF71_ERRATA_89_PRESENT
#endif

static inline bool nrf71_errata_89(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 90 ========= */
#define NRF71_ERRATA_90_PRESENT 0

#ifndef NRF71_ERRATA_90_ENABLE_WORKAROUND
    #define NRF71_ERRATA_90_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_90(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 91 ========= */
#define NRF71_ERRATA_91_PRESENT 0

#ifndef NRF71_ERRATA_91_ENABLE_WORKAROUND
    #define NRF71_ERRATA_91_ENABLE_WORKAROUND NRF71_ERRATA_91_PRESENT
#endif

static inline bool nrf71_errata_91(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 92 ========= */
#define NRF71_ERRATA_92_PRESENT 0

#ifndef NRF71_ERRATA_92_ENABLE_WORKAROUND
    #define NRF71_ERRATA_92_ENABLE_WORKAROUND NRF71_ERRATA_92_PRESENT
#endif

static inline bool nrf71_errata_92(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 93 ========= */
#define NRF71_ERRATA_93_PRESENT 0

#ifndef NRF71_ERRATA_93_ENABLE_WORKAROUND
    #define NRF71_ERRATA_93_ENABLE_WORKAROUND NRF71_ERRATA_93_PRESENT
#endif

static inline bool nrf71_errata_93(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 94 ========= */
#define NRF71_ERRATA_94_PRESENT 0

#ifndef NRF71_ERRATA_94_ENABLE_WORKAROUND
    #define NRF71_ERRATA_94_ENABLE_WORKAROUND NRF71_ERRATA_94_PRESENT
#endif

static inline bool nrf71_errata_94(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 95 ========= */
#define NRF71_ERRATA_95_PRESENT 0

#ifndef NRF71_ERRATA_95_ENABLE_WORKAROUND
    #define NRF71_ERRATA_95_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_95(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 100 ========= */
#define NRF71_ERRATA_100_PRESENT 0

#ifndef NRF71_ERRATA_100_ENABLE_WORKAROUND
    #define NRF71_ERRATA_100_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_100(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 101 ========= */
#define NRF71_ERRATA_101_PRESENT 0

#ifndef NRF71_ERRATA_101_ENABLE_WORKAROUND
    #define NRF71_ERRATA_101_ENABLE_WORKAROUND NRF71_ERRATA_101_PRESENT
#endif

static inline bool nrf71_errata_101(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 102 ========= */
#define NRF71_ERRATA_102_PRESENT 0

#ifndef NRF71_ERRATA_102_ENABLE_WORKAROUND
    #define NRF71_ERRATA_102_ENABLE_WORKAROUND NRF71_ERRATA_102_PRESENT
#endif

static inline bool nrf71_errata_102(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 103 ========= */
#define NRF71_ERRATA_103_PRESENT 0

#ifndef NRF71_ERRATA_103_ENABLE_WORKAROUND
    #define NRF71_ERRATA_103_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_103(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 104 ========= */
#define NRF71_ERRATA_104_PRESENT 0

#ifndef NRF71_ERRATA_104_ENABLE_WORKAROUND
    #define NRF71_ERRATA_104_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_104(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 105 ========= */
#define NRF71_ERRATA_105_PRESENT 0

#ifndef NRF71_ERRATA_105_ENABLE_WORKAROUND
    #define NRF71_ERRATA_105_ENABLE_WORKAROUND NRF71_ERRATA_105_PRESENT
#endif

static inline bool nrf71_errata_105(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 106 ========= */
#define NRF71_ERRATA_106_PRESENT 0

#ifndef NRF71_ERRATA_106_ENABLE_WORKAROUND
    #define NRF71_ERRATA_106_ENABLE_WORKAROUND NRF71_ERRATA_106_PRESENT
#endif

static inline bool nrf71_errata_106(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 109 ========= */
#define NRF71_ERRATA_109_PRESENT 0

#ifndef NRF71_ERRATA_109_ENABLE_WORKAROUND
    #define NRF71_ERRATA_109_ENABLE_WORKAROUND NRF71_ERRATA_109_PRESENT
#endif

static inline bool nrf71_errata_109(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 111 ========= */
#define NRF71_ERRATA_111_PRESENT 0

#ifndef NRF71_ERRATA_111_ENABLE_WORKAROUND
    #define NRF71_ERRATA_111_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_111(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 112 ========= */
#define NRF71_ERRATA_112_PRESENT 0

#ifndef NRF71_ERRATA_112_ENABLE_WORKAROUND
    #define NRF71_ERRATA_112_ENABLE_WORKAROUND NRF71_ERRATA_112_PRESENT
#endif

static inline bool nrf71_errata_112(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 114 ========= */
#define NRF71_ERRATA_114_PRESENT 0

#ifndef NRF71_ERRATA_114_ENABLE_WORKAROUND
    #define NRF71_ERRATA_114_ENABLE_WORKAROUND NRF71_ERRATA_114_PRESENT
#endif

static inline bool nrf71_errata_114(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 115 ========= */
#define NRF71_ERRATA_115_PRESENT 0

#ifndef NRF71_ERRATA_115_ENABLE_WORKAROUND
    #define NRF71_ERRATA_115_ENABLE_WORKAROUND NRF71_ERRATA_115_PRESENT
#endif

static inline bool nrf71_errata_115(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 116 ========= */
#define NRF71_ERRATA_116_PRESENT 0

#ifndef NRF71_ERRATA_116_ENABLE_WORKAROUND
    #define NRF71_ERRATA_116_ENABLE_WORKAROUND NRF71_ERRATA_116_PRESENT
#endif

static inline bool nrf71_errata_116(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 121 ========= */
#define NRF71_ERRATA_121_PRESENT 0

#ifndef NRF71_ERRATA_121_ENABLE_WORKAROUND
    #define NRF71_ERRATA_121_ENABLE_WORKAROUND NRF71_ERRATA_121_PRESENT
#endif

static inline bool nrf71_errata_121(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 126 ========= */
#define NRF71_ERRATA_126_PRESENT 0

#ifndef NRF71_ERRATA_126_ENABLE_WORKAROUND
    #define NRF71_ERRATA_126_ENABLE_WORKAROUND 0
#endif

static inline bool nrf71_errata_126(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 132 ========= */
#define NRF71_ERRATA_132_PRESENT 0

#ifndef NRF71_ERRATA_132_ENABLE_WORKAROUND
    #define NRF71_ERRATA_132_ENABLE_WORKAROUND NRF71_ERRATA_132_PRESENT
#endif

static inline bool nrf71_errata_132(void)
{
    #ifndef NRF71_SERIES
        return false;
    #else
        return false;
    #endif
}

#endif /* NRF71_ERRATAS_H */
