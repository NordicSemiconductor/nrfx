#ifndef NRF92_ERRATAS_H
#define NRF92_ERRATAS_H

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

static inline bool nrf92_errata_8(void);
static inline bool nrf92_errata_12(void);
static inline bool nrf92_errata_13(void);
static inline bool nrf92_errata_14(void);
static inline bool nrf92_errata_19(void);
static inline bool nrf92_errata_20(void);
static inline bool nrf92_errata_21(void);
static inline bool nrf92_errata_23(void);
static inline bool nrf92_errata_26(void);
static inline bool nrf92_errata_31(void);
static inline bool nrf92_errata_32(void);
static inline bool nrf92_errata_34(void);
static inline bool nrf92_errata_35(void);
static inline bool nrf92_errata_36(void);
static inline bool nrf92_errata_37(void);
static inline bool nrf92_errata_39(void);
static inline bool nrf92_errata_40(void);
static inline bool nrf92_errata_41(void);
static inline bool nrf92_errata_42(void);
static inline bool nrf92_errata_43(void);
static inline bool nrf92_errata_44(void);
static inline bool nrf92_errata_45(void);
static inline bool nrf92_errata_46(void);
static inline bool nrf92_errata_47(void);
static inline bool nrf92_errata_50(void);
static inline bool nrf92_errata_51(void);
static inline bool nrf92_errata_53(void);
static inline bool nrf92_errata_55(void);
static inline bool nrf92_errata_57(void);
static inline bool nrf92_errata_58(void);
static inline bool nrf92_errata_62(void);
static inline bool nrf92_errata_63(void);
static inline bool nrf92_errata_64(void);
static inline bool nrf92_errata_65(void);
static inline bool nrf92_errata_66(void);
static inline bool nrf92_errata_67(void);
static inline bool nrf92_errata_69(void);
static inline bool nrf92_errata_72(void);
static inline bool nrf92_errata_73(void);
static inline bool nrf92_errata_74(void);
static inline bool nrf92_errata_75(void);
static inline bool nrf92_errata_76(void);
static inline bool nrf92_errata_77(void);
static inline bool nrf92_errata_78(void);
static inline bool nrf92_errata_84(void);
static inline bool nrf92_errata_92(void);
static inline bool nrf92_errata_93(void);
static inline bool nrf92_errata_103(void);
static inline bool nrf92_errata_104(void);
static inline bool nrf92_errata_105(void);
static inline bool nrf92_errata_106(void);
static inline bool nrf92_errata_107(void);
static inline bool nrf92_errata_109(void);
static inline bool nrf92_errata_110(void);
static inline bool nrf92_errata_111(void);
static inline bool nrf92_errata_112(void);
static inline bool nrf92_errata_115(void);
static inline bool nrf92_errata_116(void);
static inline bool nrf92_errata_117(void);
static inline bool nrf92_errata_118(void);
static inline bool nrf92_errata_119(void);
static inline bool nrf92_errata_120(void);
static inline bool nrf92_errata_121(void);
static inline bool nrf92_errata_122(void);
static inline bool nrf92_errata_123(void);
static inline bool nrf92_errata_124(void);
static inline bool nrf92_errata_125(void);
static inline bool nrf92_errata_126(void);
static inline bool nrf92_errata_127(void);
static inline bool nrf92_errata_128(void);
static inline bool nrf92_errata_129(void);
static inline bool nrf92_errata_130(void);
static inline bool nrf92_errata_131(void);
static inline bool nrf92_errata_132(void);
static inline bool nrf92_errata_133(void);
static inline bool nrf92_errata_134(void);
static inline bool nrf92_errata_135(void);
static inline bool nrf92_errata_136(void);
static inline bool nrf92_errata_137(void);
static inline bool nrf92_errata_138(void);
static inline bool nrf92_errata_139(void);
static inline bool nrf92_errata_140(void);
static inline bool nrf92_errata_141(void);
static inline bool nrf92_errata_142(void);
static inline bool nrf92_errata_143(void);
static inline bool nrf92_errata_146(void);
static inline bool nrf92_errata_148(void);
static inline bool nrf92_errata_149(void);
static inline bool nrf92_errata_151(void);
static inline bool nrf92_errata_152(void);
static inline bool nrf92_errata_156(void);
static inline bool nrf92_errata_157(void);
static inline bool nrf92_errata_158(void);
static inline bool nrf92_errata_159(void);
static inline bool nrf92_errata_160(void);
static inline bool nrf92_errata_161(void);
static inline bool nrf92_errata_162(void);
static inline bool nrf92_errata_163(void);
static inline bool nrf92_errata_165(void);
static inline bool nrf92_errata_166(void);
static inline bool nrf92_errata_169(void);
static inline bool nrf92_errata_171(void);
static inline bool nrf92_errata_173(void);
static inline bool nrf92_errata_175(void);
static inline bool nrf92_errata_177(void);
static inline bool nrf92_errata_178(void);
static inline bool nrf92_errata_179(void);
static inline bool nrf92_errata_182(void);
static inline bool nrf92_errata_184(void);
static inline bool nrf92_errata_185(void);
static inline bool nrf92_errata_186(void);
static inline bool nrf92_errata_187(void);
static inline bool nrf92_errata_190(void);
static inline bool nrf92_errata_192(void);
static inline bool nrf92_errata_193(void);
static inline bool nrf92_errata_194(void);
static inline bool nrf92_errata_195(void);
static inline bool nrf92_errata_196(void);
static inline bool nrf92_errata_198(void);
static inline bool nrf92_errata_200(void);
static inline bool nrf92_errata_201(void);
static inline bool nrf92_errata_202(void);
static inline bool nrf92_errata_205(void);
static inline bool nrf92_errata_206(void);
static inline bool nrf92_errata_207(void);
static inline bool nrf92_errata_208(void);
static inline bool nrf92_errata_209(void);
static inline bool nrf92_errata_210(void);
static inline bool nrf92_errata_212(void);
static inline bool nrf92_errata_213(void);
static inline bool nrf92_errata_216(void);
static inline bool nrf92_errata_217(void);
static inline bool nrf92_errata_218(void);
static inline bool nrf92_errata_229(void);
static inline bool nrf92_errata_230(void);
static inline bool nrf92_errata_231(void);
static inline bool nrf92_errata_233(void);
static inline bool nrf92_errata_234(void);
static inline bool nrf92_errata_235(void);
static inline bool nrf92_errata_236(void);
static inline bool nrf92_errata_237(void);
static inline bool nrf92_errata_238(void);
static inline bool nrf92_errata_239(void);
static inline bool nrf92_errata_241(void);
static inline bool nrf92_errata_242(void);
static inline bool nrf92_errata_243(void);
static inline bool nrf92_errata_244(void);
static inline bool nrf92_errata_245(void);
static inline bool nrf92_errata_253(void);
static inline bool nrf92_errata_254(void);
static inline bool nrf92_errata_256(void);
static inline bool nrf92_errata_271(void);
static inline bool nrf92_errata_272(void);
static inline bool nrf92_errata_273(void);
static inline bool nrf92_errata_274(void);
static inline bool nrf92_errata_275(void);
static inline bool nrf92_errata_277(void);
static inline bool nrf92_errata_278(void);
static inline bool nrf92_errata_279(void);
static inline bool nrf92_errata_280(void);
static inline bool nrf92_errata_281(void);

/* ========= Errata 8 ========= */
#define NRF92_ERRATA_8_PRESENT 0

#ifndef NRF92_ERRATA_8_ENABLE_WORKAROUND
    #define NRF92_ERRATA_8_ENABLE_WORKAROUND NRF92_ERRATA_8_PRESENT
#endif

static inline bool nrf92_errata_8(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 12 ========= */
#define NRF92_ERRATA_12_PRESENT 0

#ifndef NRF92_ERRATA_12_ENABLE_WORKAROUND
    #define NRF92_ERRATA_12_ENABLE_WORKAROUND NRF92_ERRATA_12_PRESENT
#endif

static inline bool nrf92_errata_12(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 13 ========= */
#define NRF92_ERRATA_13_PRESENT 0

#ifndef NRF92_ERRATA_13_ENABLE_WORKAROUND
    #define NRF92_ERRATA_13_ENABLE_WORKAROUND NRF92_ERRATA_13_PRESENT
#endif

static inline bool nrf92_errata_13(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 14 ========= */
#define NRF92_ERRATA_14_PRESENT 0

#ifndef NRF92_ERRATA_14_ENABLE_WORKAROUND
    #define NRF92_ERRATA_14_ENABLE_WORKAROUND NRF92_ERRATA_14_PRESENT
#endif

static inline bool nrf92_errata_14(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 19 ========= */
#define NRF92_ERRATA_19_PRESENT 0

#ifndef NRF92_ERRATA_19_ENABLE_WORKAROUND
    #define NRF92_ERRATA_19_ENABLE_WORKAROUND NRF92_ERRATA_19_PRESENT
#endif

static inline bool nrf92_errata_19(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 20 ========= */
#define NRF92_ERRATA_20_PRESENT 0

#ifndef NRF92_ERRATA_20_ENABLE_WORKAROUND
    #define NRF92_ERRATA_20_ENABLE_WORKAROUND NRF92_ERRATA_20_PRESENT
#endif

static inline bool nrf92_errata_20(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 21 ========= */
#define NRF92_ERRATA_21_PRESENT 0

#ifndef NRF92_ERRATA_21_ENABLE_WORKAROUND
    #define NRF92_ERRATA_21_ENABLE_WORKAROUND NRF92_ERRATA_21_PRESENT
#endif

static inline bool nrf92_errata_21(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 23 ========= */
#define NRF92_ERRATA_23_PRESENT 0

#ifndef NRF92_ERRATA_23_ENABLE_WORKAROUND
    #define NRF92_ERRATA_23_ENABLE_WORKAROUND NRF92_ERRATA_23_PRESENT
#endif

static inline bool nrf92_errata_23(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 26 ========= */
#define NRF92_ERRATA_26_PRESENT 0

#ifndef NRF92_ERRATA_26_ENABLE_WORKAROUND
    #define NRF92_ERRATA_26_ENABLE_WORKAROUND NRF92_ERRATA_26_PRESENT
#endif

static inline bool nrf92_errata_26(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 31 ========= */
#define NRF92_ERRATA_31_PRESENT 0

#ifndef NRF92_ERRATA_31_ENABLE_WORKAROUND
    #define NRF92_ERRATA_31_ENABLE_WORKAROUND NRF92_ERRATA_31_PRESENT
#endif

static inline bool nrf92_errata_31(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 32 ========= */
#define NRF92_ERRATA_32_PRESENT 0

#ifndef NRF92_ERRATA_32_ENABLE_WORKAROUND
    #define NRF92_ERRATA_32_ENABLE_WORKAROUND NRF92_ERRATA_32_PRESENT
#endif

static inline bool nrf92_errata_32(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 34 ========= */
#define NRF92_ERRATA_34_PRESENT 0

#ifndef NRF92_ERRATA_34_ENABLE_WORKAROUND
    #define NRF92_ERRATA_34_ENABLE_WORKAROUND NRF92_ERRATA_34_PRESENT
#endif

static inline bool nrf92_errata_34(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 35 ========= */
#define NRF92_ERRATA_35_PRESENT 0

#ifndef NRF92_ERRATA_35_ENABLE_WORKAROUND
    #define NRF92_ERRATA_35_ENABLE_WORKAROUND NRF92_ERRATA_35_PRESENT
#endif

static inline bool nrf92_errata_35(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 36 ========= */
#define NRF92_ERRATA_36_PRESENT 0

#ifndef NRF92_ERRATA_36_ENABLE_WORKAROUND
    #define NRF92_ERRATA_36_ENABLE_WORKAROUND NRF92_ERRATA_36_PRESENT
#endif

static inline bool nrf92_errata_36(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 37 ========= */
#define NRF92_ERRATA_37_PRESENT 0

#ifndef NRF92_ERRATA_37_ENABLE_WORKAROUND
    #define NRF92_ERRATA_37_ENABLE_WORKAROUND NRF92_ERRATA_37_PRESENT
#endif

static inline bool nrf92_errata_37(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 39 ========= */
#define NRF92_ERRATA_39_PRESENT 0

#ifndef NRF92_ERRATA_39_ENABLE_WORKAROUND
    #define NRF92_ERRATA_39_ENABLE_WORKAROUND NRF92_ERRATA_39_PRESENT
#endif

static inline bool nrf92_errata_39(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 40 ========= */
#define NRF92_ERRATA_40_PRESENT 0

#ifndef NRF92_ERRATA_40_ENABLE_WORKAROUND
    #define NRF92_ERRATA_40_ENABLE_WORKAROUND NRF92_ERRATA_40_PRESENT
#endif

static inline bool nrf92_errata_40(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 41 ========= */
#define NRF92_ERRATA_41_PRESENT 0

#ifndef NRF92_ERRATA_41_ENABLE_WORKAROUND
    #define NRF92_ERRATA_41_ENABLE_WORKAROUND NRF92_ERRATA_41_PRESENT
#endif

static inline bool nrf92_errata_41(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 42 ========= */
#define NRF92_ERRATA_42_PRESENT 0

#ifndef NRF92_ERRATA_42_ENABLE_WORKAROUND
    #define NRF92_ERRATA_42_ENABLE_WORKAROUND NRF92_ERRATA_42_PRESENT
#endif

static inline bool nrf92_errata_42(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 43 ========= */
#define NRF92_ERRATA_43_PRESENT 0

#ifndef NRF92_ERRATA_43_ENABLE_WORKAROUND
    #define NRF92_ERRATA_43_ENABLE_WORKAROUND NRF92_ERRATA_43_PRESENT
#endif

static inline bool nrf92_errata_43(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 44 ========= */
#define NRF92_ERRATA_44_PRESENT 0

#ifndef NRF92_ERRATA_44_ENABLE_WORKAROUND
    #define NRF92_ERRATA_44_ENABLE_WORKAROUND NRF92_ERRATA_44_PRESENT
#endif

static inline bool nrf92_errata_44(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 45 ========= */
#define NRF92_ERRATA_45_PRESENT 0

#ifndef NRF92_ERRATA_45_ENABLE_WORKAROUND
    #define NRF92_ERRATA_45_ENABLE_WORKAROUND NRF92_ERRATA_45_PRESENT
#endif

static inline bool nrf92_errata_45(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 46 ========= */
#define NRF92_ERRATA_46_PRESENT 0

#ifndef NRF92_ERRATA_46_ENABLE_WORKAROUND
    #define NRF92_ERRATA_46_ENABLE_WORKAROUND NRF92_ERRATA_46_PRESENT
#endif

static inline bool nrf92_errata_46(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 47 ========= */
#define NRF92_ERRATA_47_PRESENT 0

#ifndef NRF92_ERRATA_47_ENABLE_WORKAROUND
    #define NRF92_ERRATA_47_ENABLE_WORKAROUND NRF92_ERRATA_47_PRESENT
#endif

static inline bool nrf92_errata_47(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 50 ========= */
#define NRF92_ERRATA_50_PRESENT 0

#ifndef NRF92_ERRATA_50_ENABLE_WORKAROUND
    #define NRF92_ERRATA_50_ENABLE_WORKAROUND NRF92_ERRATA_50_PRESENT
#endif

static inline bool nrf92_errata_50(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 51 ========= */
#define NRF92_ERRATA_51_PRESENT 0

#ifndef NRF92_ERRATA_51_ENABLE_WORKAROUND
    #define NRF92_ERRATA_51_ENABLE_WORKAROUND NRF92_ERRATA_51_PRESENT
#endif

static inline bool nrf92_errata_51(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 53 ========= */
#define NRF92_ERRATA_53_PRESENT 0

#ifndef NRF92_ERRATA_53_ENABLE_WORKAROUND
    #define NRF92_ERRATA_53_ENABLE_WORKAROUND NRF92_ERRATA_53_PRESENT
#endif

static inline bool nrf92_errata_53(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 55 ========= */
#define NRF92_ERRATA_55_PRESENT 0

#ifndef NRF92_ERRATA_55_ENABLE_WORKAROUND
    #define NRF92_ERRATA_55_ENABLE_WORKAROUND NRF92_ERRATA_55_PRESENT
#endif

static inline bool nrf92_errata_55(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 57 ========= */
#define NRF92_ERRATA_57_PRESENT 0

#ifndef NRF92_ERRATA_57_ENABLE_WORKAROUND
    #define NRF92_ERRATA_57_ENABLE_WORKAROUND NRF92_ERRATA_57_PRESENT
#endif

static inline bool nrf92_errata_57(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 58 ========= */
#define NRF92_ERRATA_58_PRESENT 0

#ifndef NRF92_ERRATA_58_ENABLE_WORKAROUND
    #define NRF92_ERRATA_58_ENABLE_WORKAROUND NRF92_ERRATA_58_PRESENT
#endif

static inline bool nrf92_errata_58(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 62 ========= */
#define NRF92_ERRATA_62_PRESENT 0

#ifndef NRF92_ERRATA_62_ENABLE_WORKAROUND
    #define NRF92_ERRATA_62_ENABLE_WORKAROUND NRF92_ERRATA_62_PRESENT
#endif

static inline bool nrf92_errata_62(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 63 ========= */
#define NRF92_ERRATA_63_PRESENT 0

#ifndef NRF92_ERRATA_63_ENABLE_WORKAROUND
    #define NRF92_ERRATA_63_ENABLE_WORKAROUND NRF92_ERRATA_63_PRESENT
#endif

static inline bool nrf92_errata_63(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 64 ========= */
#define NRF92_ERRATA_64_PRESENT 0

#ifndef NRF92_ERRATA_64_ENABLE_WORKAROUND
    #define NRF92_ERRATA_64_ENABLE_WORKAROUND NRF92_ERRATA_64_PRESENT
#endif

static inline bool nrf92_errata_64(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 65 ========= */
#define NRF92_ERRATA_65_PRESENT 0

#ifndef NRF92_ERRATA_65_ENABLE_WORKAROUND
    #define NRF92_ERRATA_65_ENABLE_WORKAROUND NRF92_ERRATA_65_PRESENT
#endif

static inline bool nrf92_errata_65(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 66 ========= */
#define NRF92_ERRATA_66_PRESENT 0

#ifndef NRF92_ERRATA_66_ENABLE_WORKAROUND
    #define NRF92_ERRATA_66_ENABLE_WORKAROUND NRF92_ERRATA_66_PRESENT
#endif

static inline bool nrf92_errata_66(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 67 ========= */
#define NRF92_ERRATA_67_PRESENT 0

#ifndef NRF92_ERRATA_67_ENABLE_WORKAROUND
    #define NRF92_ERRATA_67_ENABLE_WORKAROUND NRF92_ERRATA_67_PRESENT
#endif

static inline bool nrf92_errata_67(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 69 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_69_PRESENT 1
#else
    #define NRF92_ERRATA_69_PRESENT 0
#endif

#ifndef NRF92_ERRATA_69_ENABLE_WORKAROUND
    #define NRF92_ERRATA_69_ENABLE_WORKAROUND NRF92_ERRATA_69_PRESENT
#endif

static inline bool nrf92_errata_69(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 72 ========= */
#define NRF92_ERRATA_72_PRESENT 0

#ifndef NRF92_ERRATA_72_ENABLE_WORKAROUND
    #define NRF92_ERRATA_72_ENABLE_WORKAROUND NRF92_ERRATA_72_PRESENT
#endif

static inline bool nrf92_errata_72(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 73 ========= */
#define NRF92_ERRATA_73_PRESENT 0

#ifndef NRF92_ERRATA_73_ENABLE_WORKAROUND
    #define NRF92_ERRATA_73_ENABLE_WORKAROUND NRF92_ERRATA_73_PRESENT
#endif

static inline bool nrf92_errata_73(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 74 ========= */
#define NRF92_ERRATA_74_PRESENT 0

#ifndef NRF92_ERRATA_74_ENABLE_WORKAROUND
    #define NRF92_ERRATA_74_ENABLE_WORKAROUND NRF92_ERRATA_74_PRESENT
#endif

static inline bool nrf92_errata_74(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 75 ========= */
#define NRF92_ERRATA_75_PRESENT 0

#ifndef NRF92_ERRATA_75_ENABLE_WORKAROUND
    #define NRF92_ERRATA_75_ENABLE_WORKAROUND NRF92_ERRATA_75_PRESENT
#endif

static inline bool nrf92_errata_75(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 76 ========= */
#define NRF92_ERRATA_76_PRESENT 0

#ifndef NRF92_ERRATA_76_ENABLE_WORKAROUND
    #define NRF92_ERRATA_76_ENABLE_WORKAROUND NRF92_ERRATA_76_PRESENT
#endif

static inline bool nrf92_errata_76(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 77 ========= */
#define NRF92_ERRATA_77_PRESENT 0

#ifndef NRF92_ERRATA_77_ENABLE_WORKAROUND
    #define NRF92_ERRATA_77_ENABLE_WORKAROUND NRF92_ERRATA_77_PRESENT
#endif

static inline bool nrf92_errata_77(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 78 ========= */
#define NRF92_ERRATA_78_PRESENT 0

#ifndef NRF92_ERRATA_78_ENABLE_WORKAROUND
    #define NRF92_ERRATA_78_ENABLE_WORKAROUND NRF92_ERRATA_78_PRESENT
#endif

static inline bool nrf92_errata_78(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 84 ========= */
#define NRF92_ERRATA_84_PRESENT 0

#ifndef NRF92_ERRATA_84_ENABLE_WORKAROUND
    #define NRF92_ERRATA_84_ENABLE_WORKAROUND NRF92_ERRATA_84_PRESENT
#endif

static inline bool nrf92_errata_84(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 92 ========= */
#define NRF92_ERRATA_92_PRESENT 0

#ifndef NRF92_ERRATA_92_ENABLE_WORKAROUND
    #define NRF92_ERRATA_92_ENABLE_WORKAROUND NRF92_ERRATA_92_PRESENT
#endif

static inline bool nrf92_errata_92(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 93 ========= */
#define NRF92_ERRATA_93_PRESENT 0

#ifndef NRF92_ERRATA_93_ENABLE_WORKAROUND
    #define NRF92_ERRATA_93_ENABLE_WORKAROUND NRF92_ERRATA_93_PRESENT
#endif

static inline bool nrf92_errata_93(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 103 ========= */
#define NRF92_ERRATA_103_PRESENT 0

#ifndef NRF92_ERRATA_103_ENABLE_WORKAROUND
    #define NRF92_ERRATA_103_ENABLE_WORKAROUND NRF92_ERRATA_103_PRESENT
#endif

static inline bool nrf92_errata_103(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 104 ========= */
#define NRF92_ERRATA_104_PRESENT 0

#ifndef NRF92_ERRATA_104_ENABLE_WORKAROUND
    #define NRF92_ERRATA_104_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_104(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 105 ========= */
#define NRF92_ERRATA_105_PRESENT 0

#ifndef NRF92_ERRATA_105_ENABLE_WORKAROUND
    #define NRF92_ERRATA_105_ENABLE_WORKAROUND NRF92_ERRATA_105_PRESENT
#endif

static inline bool nrf92_errata_105(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 106 ========= */
#define NRF92_ERRATA_106_PRESENT 0

#ifndef NRF92_ERRATA_106_ENABLE_WORKAROUND
    #define NRF92_ERRATA_106_ENABLE_WORKAROUND NRF92_ERRATA_106_PRESENT
#endif

static inline bool nrf92_errata_106(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 107 ========= */
#define NRF92_ERRATA_107_PRESENT 0

#ifndef NRF92_ERRATA_107_ENABLE_WORKAROUND
    #define NRF92_ERRATA_107_ENABLE_WORKAROUND NRF92_ERRATA_107_PRESENT
#endif

static inline bool nrf92_errata_107(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 109 ========= */
#define NRF92_ERRATA_109_PRESENT 0

#ifndef NRF92_ERRATA_109_ENABLE_WORKAROUND
    #define NRF92_ERRATA_109_ENABLE_WORKAROUND NRF92_ERRATA_109_PRESENT
#endif

static inline bool nrf92_errata_109(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 110 ========= */
#define NRF92_ERRATA_110_PRESENT 0

#ifndef NRF92_ERRATA_110_ENABLE_WORKAROUND
    #define NRF92_ERRATA_110_ENABLE_WORKAROUND NRF92_ERRATA_110_PRESENT
#endif

static inline bool nrf92_errata_110(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 111 ========= */
#define NRF92_ERRATA_111_PRESENT 0

#ifndef NRF92_ERRATA_111_ENABLE_WORKAROUND
    #define NRF92_ERRATA_111_ENABLE_WORKAROUND NRF92_ERRATA_111_PRESENT
#endif

static inline bool nrf92_errata_111(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 112 ========= */
#define NRF92_ERRATA_112_PRESENT 0

#ifndef NRF92_ERRATA_112_ENABLE_WORKAROUND
    #define NRF92_ERRATA_112_ENABLE_WORKAROUND NRF92_ERRATA_112_PRESENT
#endif

static inline bool nrf92_errata_112(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 115 ========= */
#define NRF92_ERRATA_115_PRESENT 0

#ifndef NRF92_ERRATA_115_ENABLE_WORKAROUND
    #define NRF92_ERRATA_115_ENABLE_WORKAROUND NRF92_ERRATA_115_PRESENT
#endif

static inline bool nrf92_errata_115(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 116 ========= */
#define NRF92_ERRATA_116_PRESENT 0

#ifndef NRF92_ERRATA_116_ENABLE_WORKAROUND
    #define NRF92_ERRATA_116_ENABLE_WORKAROUND NRF92_ERRATA_116_PRESENT
#endif

static inline bool nrf92_errata_116(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 117 ========= */
#define NRF92_ERRATA_117_PRESENT 0

#ifndef NRF92_ERRATA_117_ENABLE_WORKAROUND
    #define NRF92_ERRATA_117_ENABLE_WORKAROUND NRF92_ERRATA_117_PRESENT
#endif

static inline bool nrf92_errata_117(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 118 ========= */
#define NRF92_ERRATA_118_PRESENT 0

#ifndef NRF92_ERRATA_118_ENABLE_WORKAROUND
    #define NRF92_ERRATA_118_ENABLE_WORKAROUND NRF92_ERRATA_118_PRESENT
#endif

static inline bool nrf92_errata_118(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 119 ========= */
#define NRF92_ERRATA_119_PRESENT 0

#ifndef NRF92_ERRATA_119_ENABLE_WORKAROUND
    #define NRF92_ERRATA_119_ENABLE_WORKAROUND NRF92_ERRATA_119_PRESENT
#endif

static inline bool nrf92_errata_119(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 120 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_120_PRESENT 1
#else
    #define NRF92_ERRATA_120_PRESENT 0
#endif

#ifndef NRF92_ERRATA_120_ENABLE_WORKAROUND
    #define NRF92_ERRATA_120_ENABLE_WORKAROUND NRF92_ERRATA_120_PRESENT
#endif

static inline bool nrf92_errata_120(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return false;
                    case 0x13ul:
                        return false;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 121 ========= */
#define NRF92_ERRATA_121_PRESENT 0

#ifndef NRF92_ERRATA_121_ENABLE_WORKAROUND
    #define NRF92_ERRATA_121_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_121(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 122 ========= */
#define NRF92_ERRATA_122_PRESENT 0

#ifndef NRF92_ERRATA_122_ENABLE_WORKAROUND
    #define NRF92_ERRATA_122_ENABLE_WORKAROUND NRF92_ERRATA_122_PRESENT
#endif

static inline bool nrf92_errata_122(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 123 ========= */
#define NRF92_ERRATA_123_PRESENT 0

#ifndef NRF92_ERRATA_123_ENABLE_WORKAROUND
    #define NRF92_ERRATA_123_ENABLE_WORKAROUND NRF92_ERRATA_123_PRESENT
#endif

static inline bool nrf92_errata_123(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 124 ========= */
#define NRF92_ERRATA_124_PRESENT 0

#ifndef NRF92_ERRATA_124_ENABLE_WORKAROUND
    #define NRF92_ERRATA_124_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_124(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 125 ========= */
#define NRF92_ERRATA_125_PRESENT 0

#ifndef NRF92_ERRATA_125_ENABLE_WORKAROUND
    #define NRF92_ERRATA_125_ENABLE_WORKAROUND NRF92_ERRATA_125_PRESENT
#endif

static inline bool nrf92_errata_125(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 126 ========= */
#define NRF92_ERRATA_126_PRESENT 0

#ifndef NRF92_ERRATA_126_ENABLE_WORKAROUND
    #define NRF92_ERRATA_126_ENABLE_WORKAROUND NRF92_ERRATA_126_PRESENT
#endif

static inline bool nrf92_errata_126(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 127 ========= */
#define NRF92_ERRATA_127_PRESENT 0

#ifndef NRF92_ERRATA_127_ENABLE_WORKAROUND
    #define NRF92_ERRATA_127_ENABLE_WORKAROUND NRF92_ERRATA_127_PRESENT
#endif

static inline bool nrf92_errata_127(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 128 ========= */
#define NRF92_ERRATA_128_PRESENT 0

#ifndef NRF92_ERRATA_128_ENABLE_WORKAROUND
    #define NRF92_ERRATA_128_ENABLE_WORKAROUND NRF92_ERRATA_128_PRESENT
#endif

static inline bool nrf92_errata_128(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 129 ========= */
#define NRF92_ERRATA_129_PRESENT 0

#ifndef NRF92_ERRATA_129_ENABLE_WORKAROUND
    #define NRF92_ERRATA_129_ENABLE_WORKAROUND NRF92_ERRATA_129_PRESENT
#endif

static inline bool nrf92_errata_129(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 130 ========= */
#define NRF92_ERRATA_130_PRESENT 0

#ifndef NRF92_ERRATA_130_ENABLE_WORKAROUND
    #define NRF92_ERRATA_130_ENABLE_WORKAROUND NRF92_ERRATA_130_PRESENT
#endif

static inline bool nrf92_errata_130(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 131 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_131_PRESENT 1
#else
    #define NRF92_ERRATA_131_PRESENT 0
#endif

#ifndef NRF92_ERRATA_131_ENABLE_WORKAROUND
    #define NRF92_ERRATA_131_ENABLE_WORKAROUND NRF92_ERRATA_131_PRESENT
#endif

static inline bool nrf92_errata_131(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 132 ========= */
#define NRF92_ERRATA_132_PRESENT 0

#ifndef NRF92_ERRATA_132_ENABLE_WORKAROUND
    #define NRF92_ERRATA_132_ENABLE_WORKAROUND NRF92_ERRATA_132_PRESENT
#endif

static inline bool nrf92_errata_132(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 133 ========= */
#define NRF92_ERRATA_133_PRESENT 0

#ifndef NRF92_ERRATA_133_ENABLE_WORKAROUND
    #define NRF92_ERRATA_133_ENABLE_WORKAROUND NRF92_ERRATA_133_PRESENT
#endif

static inline bool nrf92_errata_133(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 134 ========= */
#define NRF92_ERRATA_134_PRESENT 0

#ifndef NRF92_ERRATA_134_ENABLE_WORKAROUND
    #define NRF92_ERRATA_134_ENABLE_WORKAROUND NRF92_ERRATA_134_PRESENT
#endif

static inline bool nrf92_errata_134(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 135 ========= */
#define NRF92_ERRATA_135_PRESENT 0

#ifndef NRF92_ERRATA_135_ENABLE_WORKAROUND
    #define NRF92_ERRATA_135_ENABLE_WORKAROUND NRF92_ERRATA_135_PRESENT
#endif

static inline bool nrf92_errata_135(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 136 ========= */
#define NRF92_ERRATA_136_PRESENT 0

#ifndef NRF92_ERRATA_136_ENABLE_WORKAROUND
    #define NRF92_ERRATA_136_ENABLE_WORKAROUND NRF92_ERRATA_136_PRESENT
#endif

static inline bool nrf92_errata_136(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 137 ========= */
#define NRF92_ERRATA_137_PRESENT 0

#ifndef NRF92_ERRATA_137_ENABLE_WORKAROUND
    #define NRF92_ERRATA_137_ENABLE_WORKAROUND NRF92_ERRATA_137_PRESENT
#endif

static inline bool nrf92_errata_137(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 138 ========= */
#define NRF92_ERRATA_138_PRESENT 0

#ifndef NRF92_ERRATA_138_ENABLE_WORKAROUND
    #define NRF92_ERRATA_138_ENABLE_WORKAROUND NRF92_ERRATA_138_PRESENT
#endif

static inline bool nrf92_errata_138(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 139 ========= */
#define NRF92_ERRATA_139_PRESENT 0

#ifndef NRF92_ERRATA_139_ENABLE_WORKAROUND
    #define NRF92_ERRATA_139_ENABLE_WORKAROUND NRF92_ERRATA_139_PRESENT
#endif

static inline bool nrf92_errata_139(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 140 ========= */
#define NRF92_ERRATA_140_PRESENT 0

#ifndef NRF92_ERRATA_140_ENABLE_WORKAROUND
    #define NRF92_ERRATA_140_ENABLE_WORKAROUND NRF92_ERRATA_140_PRESENT
#endif

static inline bool nrf92_errata_140(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 141 ========= */
#define NRF92_ERRATA_141_PRESENT 0

#ifndef NRF92_ERRATA_141_ENABLE_WORKAROUND
    #define NRF92_ERRATA_141_ENABLE_WORKAROUND NRF92_ERRATA_141_PRESENT
#endif

static inline bool nrf92_errata_141(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 142 ========= */
#define NRF92_ERRATA_142_PRESENT 0

#ifndef NRF92_ERRATA_142_ENABLE_WORKAROUND
    #define NRF92_ERRATA_142_ENABLE_WORKAROUND NRF92_ERRATA_142_PRESENT
#endif

static inline bool nrf92_errata_142(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 143 ========= */
#define NRF92_ERRATA_143_PRESENT 0

#ifndef NRF92_ERRATA_143_ENABLE_WORKAROUND
    #define NRF92_ERRATA_143_ENABLE_WORKAROUND NRF92_ERRATA_143_PRESENT
#endif

static inline bool nrf92_errata_143(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 146 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_146_PRESENT 1
#else
    #define NRF92_ERRATA_146_PRESENT 0
#endif

#ifndef NRF92_ERRATA_146_ENABLE_WORKAROUND
    #define NRF92_ERRATA_146_ENABLE_WORKAROUND NRF92_ERRATA_146_PRESENT
#endif

static inline bool nrf92_errata_146(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 148 ========= */
#define NRF92_ERRATA_148_PRESENT 0

#ifndef NRF92_ERRATA_148_ENABLE_WORKAROUND
    #define NRF92_ERRATA_148_ENABLE_WORKAROUND NRF92_ERRATA_148_PRESENT
#endif

static inline bool nrf92_errata_148(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 149 ========= */
#define NRF92_ERRATA_149_PRESENT 0

#ifndef NRF92_ERRATA_149_ENABLE_WORKAROUND
    #define NRF92_ERRATA_149_ENABLE_WORKAROUND NRF92_ERRATA_149_PRESENT
#endif

static inline bool nrf92_errata_149(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 151 ========= */
#define NRF92_ERRATA_151_PRESENT 0

#ifndef NRF92_ERRATA_151_ENABLE_WORKAROUND
    #define NRF92_ERRATA_151_ENABLE_WORKAROUND NRF92_ERRATA_151_PRESENT
#endif

static inline bool nrf92_errata_151(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 152 ========= */
#define NRF92_ERRATA_152_PRESENT 0

#ifndef NRF92_ERRATA_152_ENABLE_WORKAROUND
    #define NRF92_ERRATA_152_ENABLE_WORKAROUND NRF92_ERRATA_152_PRESENT
#endif

static inline bool nrf92_errata_152(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 156 ========= */
#define NRF92_ERRATA_156_PRESENT 0

#ifndef NRF92_ERRATA_156_ENABLE_WORKAROUND
    #define NRF92_ERRATA_156_ENABLE_WORKAROUND NRF92_ERRATA_156_PRESENT
#endif

static inline bool nrf92_errata_156(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 157 ========= */
#define NRF92_ERRATA_157_PRESENT 0

#ifndef NRF92_ERRATA_157_ENABLE_WORKAROUND
    #define NRF92_ERRATA_157_ENABLE_WORKAROUND NRF92_ERRATA_157_PRESENT
#endif

static inline bool nrf92_errata_157(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 158 ========= */
#define NRF92_ERRATA_158_PRESENT 0

#ifndef NRF92_ERRATA_158_ENABLE_WORKAROUND
    #define NRF92_ERRATA_158_ENABLE_WORKAROUND NRF92_ERRATA_158_PRESENT
#endif

static inline bool nrf92_errata_158(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 159 ========= */
#define NRF92_ERRATA_159_PRESENT 0

#ifndef NRF92_ERRATA_159_ENABLE_WORKAROUND
    #define NRF92_ERRATA_159_ENABLE_WORKAROUND NRF92_ERRATA_159_PRESENT
#endif

static inline bool nrf92_errata_159(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 160 ========= */
#define NRF92_ERRATA_160_PRESENT 0

#ifndef NRF92_ERRATA_160_ENABLE_WORKAROUND
    #define NRF92_ERRATA_160_ENABLE_WORKAROUND NRF92_ERRATA_160_PRESENT
#endif

static inline bool nrf92_errata_160(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 161 ========= */
#define NRF92_ERRATA_161_PRESENT 0

#ifndef NRF92_ERRATA_161_ENABLE_WORKAROUND
    #define NRF92_ERRATA_161_ENABLE_WORKAROUND NRF92_ERRATA_161_PRESENT
#endif

static inline bool nrf92_errata_161(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 162 ========= */
#define NRF92_ERRATA_162_PRESENT 0

#ifndef NRF92_ERRATA_162_ENABLE_WORKAROUND
    #define NRF92_ERRATA_162_ENABLE_WORKAROUND NRF92_ERRATA_162_PRESENT
#endif

static inline bool nrf92_errata_162(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 163 ========= */
#define NRF92_ERRATA_163_PRESENT 0

#ifndef NRF92_ERRATA_163_ENABLE_WORKAROUND
    #define NRF92_ERRATA_163_ENABLE_WORKAROUND NRF92_ERRATA_163_PRESENT
#endif

static inline bool nrf92_errata_163(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 165 ========= */
#define NRF92_ERRATA_165_PRESENT 0

#ifndef NRF92_ERRATA_165_ENABLE_WORKAROUND
    #define NRF92_ERRATA_165_ENABLE_WORKAROUND NRF92_ERRATA_165_PRESENT
#endif

static inline bool nrf92_errata_165(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 166 ========= */
#define NRF92_ERRATA_166_PRESENT 0

#ifndef NRF92_ERRATA_166_ENABLE_WORKAROUND
    #define NRF92_ERRATA_166_ENABLE_WORKAROUND NRF92_ERRATA_166_PRESENT
#endif

static inline bool nrf92_errata_166(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 169 ========= */
#define NRF92_ERRATA_169_PRESENT 0

#ifndef NRF92_ERRATA_169_ENABLE_WORKAROUND
    #define NRF92_ERRATA_169_ENABLE_WORKAROUND NRF92_ERRATA_169_PRESENT
#endif

static inline bool nrf92_errata_169(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 171 ========= */
#define NRF92_ERRATA_171_PRESENT 0

#ifndef NRF92_ERRATA_171_ENABLE_WORKAROUND
    #define NRF92_ERRATA_171_ENABLE_WORKAROUND NRF92_ERRATA_171_PRESENT
#endif

static inline bool nrf92_errata_171(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 173 ========= */
#define NRF92_ERRATA_173_PRESENT 0

#ifndef NRF92_ERRATA_173_ENABLE_WORKAROUND
    #define NRF92_ERRATA_173_ENABLE_WORKAROUND NRF92_ERRATA_173_PRESENT
#endif

static inline bool nrf92_errata_173(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 175 ========= */
#define NRF92_ERRATA_175_PRESENT 0

#ifndef NRF92_ERRATA_175_ENABLE_WORKAROUND
    #define NRF92_ERRATA_175_ENABLE_WORKAROUND NRF92_ERRATA_175_PRESENT
#endif

static inline bool nrf92_errata_175(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 177 ========= */
#define NRF92_ERRATA_177_PRESENT 0

#ifndef NRF92_ERRATA_177_ENABLE_WORKAROUND
    #define NRF92_ERRATA_177_ENABLE_WORKAROUND NRF92_ERRATA_177_PRESENT
#endif

static inline bool nrf92_errata_177(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 178 ========= */
#define NRF92_ERRATA_178_PRESENT 0

#ifndef NRF92_ERRATA_178_ENABLE_WORKAROUND
    #define NRF92_ERRATA_178_ENABLE_WORKAROUND NRF92_ERRATA_178_PRESENT
#endif

static inline bool nrf92_errata_178(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 179 ========= */
#define NRF92_ERRATA_179_PRESENT 0

#ifndef NRF92_ERRATA_179_ENABLE_WORKAROUND
    #define NRF92_ERRATA_179_ENABLE_WORKAROUND NRF92_ERRATA_179_PRESENT
#endif

static inline bool nrf92_errata_179(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 182 ========= */
#define NRF92_ERRATA_182_PRESENT 0

#ifndef NRF92_ERRATA_182_ENABLE_WORKAROUND
    #define NRF92_ERRATA_182_ENABLE_WORKAROUND NRF92_ERRATA_182_PRESENT
#endif

static inline bool nrf92_errata_182(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 184 ========= */
#define NRF92_ERRATA_184_PRESENT 0

#ifndef NRF92_ERRATA_184_ENABLE_WORKAROUND
    #define NRF92_ERRATA_184_ENABLE_WORKAROUND NRF92_ERRATA_184_PRESENT
#endif

static inline bool nrf92_errata_184(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 185 ========= */
#define NRF92_ERRATA_185_PRESENT 0

#ifndef NRF92_ERRATA_185_ENABLE_WORKAROUND
    #define NRF92_ERRATA_185_ENABLE_WORKAROUND NRF92_ERRATA_185_PRESENT
#endif

static inline bool nrf92_errata_185(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 186 ========= */
#define NRF92_ERRATA_186_PRESENT 0

#ifndef NRF92_ERRATA_186_ENABLE_WORKAROUND
    #define NRF92_ERRATA_186_ENABLE_WORKAROUND NRF92_ERRATA_186_PRESENT
#endif

static inline bool nrf92_errata_186(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 187 ========= */
#define NRF92_ERRATA_187_PRESENT 0

#ifndef NRF92_ERRATA_187_ENABLE_WORKAROUND
    #define NRF92_ERRATA_187_ENABLE_WORKAROUND NRF92_ERRATA_187_PRESENT
#endif

static inline bool nrf92_errata_187(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 190 ========= */
#define NRF92_ERRATA_190_PRESENT 0

#ifndef NRF92_ERRATA_190_ENABLE_WORKAROUND
    #define NRF92_ERRATA_190_ENABLE_WORKAROUND NRF92_ERRATA_190_PRESENT
#endif

static inline bool nrf92_errata_190(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 192 ========= */
#define NRF92_ERRATA_192_PRESENT 0

#ifndef NRF92_ERRATA_192_ENABLE_WORKAROUND
    #define NRF92_ERRATA_192_ENABLE_WORKAROUND NRF92_ERRATA_192_PRESENT
#endif

static inline bool nrf92_errata_192(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 193 ========= */
#define NRF92_ERRATA_193_PRESENT 0

#ifndef NRF92_ERRATA_193_ENABLE_WORKAROUND
    #define NRF92_ERRATA_193_ENABLE_WORKAROUND NRF92_ERRATA_193_PRESENT
#endif

static inline bool nrf92_errata_193(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 194 ========= */
#define NRF92_ERRATA_194_PRESENT 0

#ifndef NRF92_ERRATA_194_ENABLE_WORKAROUND
    #define NRF92_ERRATA_194_ENABLE_WORKAROUND NRF92_ERRATA_194_PRESENT
#endif

static inline bool nrf92_errata_194(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 195 ========= */
#define NRF92_ERRATA_195_PRESENT 0

#ifndef NRF92_ERRATA_195_ENABLE_WORKAROUND
    #define NRF92_ERRATA_195_ENABLE_WORKAROUND NRF92_ERRATA_195_PRESENT
#endif

static inline bool nrf92_errata_195(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 196 ========= */
#define NRF92_ERRATA_196_PRESENT 0

#ifndef NRF92_ERRATA_196_ENABLE_WORKAROUND
    #define NRF92_ERRATA_196_ENABLE_WORKAROUND NRF92_ERRATA_196_PRESENT
#endif

static inline bool nrf92_errata_196(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 198 ========= */
#define NRF92_ERRATA_198_PRESENT 0

#ifndef NRF92_ERRATA_198_ENABLE_WORKAROUND
    #define NRF92_ERRATA_198_ENABLE_WORKAROUND NRF92_ERRATA_198_PRESENT
#endif

static inline bool nrf92_errata_198(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 200 ========= */
#define NRF92_ERRATA_200_PRESENT 0

#ifndef NRF92_ERRATA_200_ENABLE_WORKAROUND
    #define NRF92_ERRATA_200_ENABLE_WORKAROUND NRF92_ERRATA_200_PRESENT
#endif

static inline bool nrf92_errata_200(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 201 ========= */
#define NRF92_ERRATA_201_PRESENT 0

#ifndef NRF92_ERRATA_201_ENABLE_WORKAROUND
    #define NRF92_ERRATA_201_ENABLE_WORKAROUND NRF92_ERRATA_201_PRESENT
#endif

static inline bool nrf92_errata_201(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 202 ========= */
#define NRF92_ERRATA_202_PRESENT 0

#ifndef NRF92_ERRATA_202_ENABLE_WORKAROUND
    #define NRF92_ERRATA_202_ENABLE_WORKAROUND NRF92_ERRATA_202_PRESENT
#endif

static inline bool nrf92_errata_202(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 205 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_205_PRESENT 1
#else
    #define NRF92_ERRATA_205_PRESENT 0
#endif

#ifndef NRF92_ERRATA_205_ENABLE_WORKAROUND
    #define NRF92_ERRATA_205_ENABLE_WORKAROUND NRF92_ERRATA_205_PRESENT
#endif

static inline bool nrf92_errata_205(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 206 ========= */
#define NRF92_ERRATA_206_PRESENT 0

#ifndef NRF92_ERRATA_206_ENABLE_WORKAROUND
    #define NRF92_ERRATA_206_ENABLE_WORKAROUND NRF92_ERRATA_206_PRESENT
#endif

static inline bool nrf92_errata_206(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 207 ========= */
#define NRF92_ERRATA_207_PRESENT 0

#ifndef NRF92_ERRATA_207_ENABLE_WORKAROUND
    #define NRF92_ERRATA_207_ENABLE_WORKAROUND NRF92_ERRATA_207_PRESENT
#endif

static inline bool nrf92_errata_207(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 208 ========= */
#define NRF92_ERRATA_208_PRESENT 0

#ifndef NRF92_ERRATA_208_ENABLE_WORKAROUND
    #define NRF92_ERRATA_208_ENABLE_WORKAROUND NRF92_ERRATA_208_PRESENT
#endif

static inline bool nrf92_errata_208(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 209 ========= */
#define NRF92_ERRATA_209_PRESENT 0

#ifndef NRF92_ERRATA_209_ENABLE_WORKAROUND
    #define NRF92_ERRATA_209_ENABLE_WORKAROUND NRF92_ERRATA_209_PRESENT
#endif

static inline bool nrf92_errata_209(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 210 ========= */
#define NRF92_ERRATA_210_PRESENT 0

#ifndef NRF92_ERRATA_210_ENABLE_WORKAROUND
    #define NRF92_ERRATA_210_ENABLE_WORKAROUND NRF92_ERRATA_210_PRESENT
#endif

static inline bool nrf92_errata_210(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 212 ========= */
#define NRF92_ERRATA_212_PRESENT 0

#ifndef NRF92_ERRATA_212_ENABLE_WORKAROUND
    #define NRF92_ERRATA_212_ENABLE_WORKAROUND NRF92_ERRATA_212_PRESENT
#endif

static inline bool nrf92_errata_212(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 213 ========= */
#define NRF92_ERRATA_213_PRESENT 0

#ifndef NRF92_ERRATA_213_ENABLE_WORKAROUND
    #define NRF92_ERRATA_213_ENABLE_WORKAROUND NRF92_ERRATA_213_PRESENT
#endif

static inline bool nrf92_errata_213(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 216 ========= */
#define NRF92_ERRATA_216_PRESENT 0

#ifndef NRF92_ERRATA_216_ENABLE_WORKAROUND
    #define NRF92_ERRATA_216_ENABLE_WORKAROUND NRF92_ERRATA_216_PRESENT
#endif

static inline bool nrf92_errata_216(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 217 ========= */
#define NRF92_ERRATA_217_PRESENT 0

#ifndef NRF92_ERRATA_217_ENABLE_WORKAROUND
    #define NRF92_ERRATA_217_ENABLE_WORKAROUND NRF92_ERRATA_217_PRESENT
#endif

static inline bool nrf92_errata_217(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 218 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_218_PRESENT 1
#else
    #define NRF92_ERRATA_218_PRESENT 0
#endif

#ifndef NRF92_ERRATA_218_ENABLE_WORKAROUND
    #define NRF92_ERRATA_218_ENABLE_WORKAROUND NRF92_ERRATA_218_PRESENT
#endif

static inline bool nrf92_errata_218(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 229 ========= */
#define NRF92_ERRATA_229_PRESENT 0

#ifndef NRF92_ERRATA_229_ENABLE_WORKAROUND
    #define NRF92_ERRATA_229_ENABLE_WORKAROUND NRF92_ERRATA_229_PRESENT
#endif

static inline bool nrf92_errata_229(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 230 ========= */
#define NRF92_ERRATA_230_PRESENT 0

#ifndef NRF92_ERRATA_230_ENABLE_WORKAROUND
    #define NRF92_ERRATA_230_ENABLE_WORKAROUND NRF92_ERRATA_230_PRESENT
#endif

static inline bool nrf92_errata_230(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 231 ========= */
#define NRF92_ERRATA_231_PRESENT 0

#ifndef NRF92_ERRATA_231_ENABLE_WORKAROUND
    #define NRF92_ERRATA_231_ENABLE_WORKAROUND NRF92_ERRATA_231_PRESENT
#endif

static inline bool nrf92_errata_231(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 233 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_233_PRESENT 1
#else
    #define NRF92_ERRATA_233_PRESENT 0
#endif

#ifndef NRF92_ERRATA_233_ENABLE_WORKAROUND
    #define NRF92_ERRATA_233_ENABLE_WORKAROUND NRF92_ERRATA_233_PRESENT
#endif

static inline bool nrf92_errata_233(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 234 ========= */
#define NRF92_ERRATA_234_PRESENT 0

#ifndef NRF92_ERRATA_234_ENABLE_WORKAROUND
    #define NRF92_ERRATA_234_ENABLE_WORKAROUND NRF92_ERRATA_234_PRESENT
#endif

static inline bool nrf92_errata_234(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 235 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_235_PRESENT 1
#else
    #define NRF92_ERRATA_235_PRESENT 0
#endif

#ifndef NRF92_ERRATA_235_ENABLE_WORKAROUND
    #define NRF92_ERRATA_235_ENABLE_WORKAROUND NRF92_ERRATA_235_PRESENT
#endif

static inline bool nrf92_errata_235(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 236 ========= */
#define NRF92_ERRATA_236_PRESENT 0

#ifndef NRF92_ERRATA_236_ENABLE_WORKAROUND
    #define NRF92_ERRATA_236_ENABLE_WORKAROUND NRF92_ERRATA_236_PRESENT
#endif

static inline bool nrf92_errata_236(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 237 ========= */
#define NRF92_ERRATA_237_PRESENT 0

#ifndef NRF92_ERRATA_237_ENABLE_WORKAROUND
    #define NRF92_ERRATA_237_ENABLE_WORKAROUND NRF92_ERRATA_237_PRESENT
#endif

static inline bool nrf92_errata_237(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 238 ========= */
#define NRF92_ERRATA_238_PRESENT 0

#ifndef NRF92_ERRATA_238_ENABLE_WORKAROUND
    #define NRF92_ERRATA_238_ENABLE_WORKAROUND NRF92_ERRATA_238_PRESENT
#endif

static inline bool nrf92_errata_238(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 239 ========= */
#define NRF92_ERRATA_239_PRESENT 0

#ifndef NRF92_ERRATA_239_ENABLE_WORKAROUND
    #define NRF92_ERRATA_239_ENABLE_WORKAROUND NRF92_ERRATA_239_PRESENT
#endif

static inline bool nrf92_errata_239(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 241 ========= */
#define NRF92_ERRATA_241_PRESENT 0

#ifndef NRF92_ERRATA_241_ENABLE_WORKAROUND
    #define NRF92_ERRATA_241_ENABLE_WORKAROUND NRF92_ERRATA_241_PRESENT
#endif

static inline bool nrf92_errata_241(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 242 ========= */
#define NRF92_ERRATA_242_PRESENT 0

#ifndef NRF92_ERRATA_242_ENABLE_WORKAROUND
    #define NRF92_ERRATA_242_ENABLE_WORKAROUND NRF92_ERRATA_242_PRESENT
#endif

static inline bool nrf92_errata_242(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 243 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_243_PRESENT 1
#else
    #define NRF92_ERRATA_243_PRESENT 0
#endif

#ifndef NRF92_ERRATA_243_ENABLE_WORKAROUND
    #define NRF92_ERRATA_243_ENABLE_WORKAROUND NRF92_ERRATA_243_PRESENT
#endif

static inline bool nrf92_errata_243(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 244 ========= */
#define NRF92_ERRATA_244_PRESENT 0

#ifndef NRF92_ERRATA_244_ENABLE_WORKAROUND
    #define NRF92_ERRATA_244_ENABLE_WORKAROUND NRF92_ERRATA_244_PRESENT
#endif

static inline bool nrf92_errata_244(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 245 ========= */
#define NRF92_ERRATA_245_PRESENT 0

#ifndef NRF92_ERRATA_245_ENABLE_WORKAROUND
    #define NRF92_ERRATA_245_ENABLE_WORKAROUND NRF92_ERRATA_245_PRESENT
#endif

static inline bool nrf92_errata_245(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 253 ========= */
#define NRF92_ERRATA_253_PRESENT 0

#ifndef NRF92_ERRATA_253_ENABLE_WORKAROUND
    #define NRF92_ERRATA_253_ENABLE_WORKAROUND NRF92_ERRATA_253_PRESENT
#endif

static inline bool nrf92_errata_253(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 254 ========= */
#define NRF92_ERRATA_254_PRESENT 0

#ifndef NRF92_ERRATA_254_ENABLE_WORKAROUND
    #define NRF92_ERRATA_254_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_254(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 256 ========= */
#define NRF92_ERRATA_256_PRESENT 0

#ifndef NRF92_ERRATA_256_ENABLE_WORKAROUND
    #define NRF92_ERRATA_256_ENABLE_WORKAROUND NRF92_ERRATA_256_PRESENT
#endif

static inline bool nrf92_errata_256(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 271 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_271_PRESENT 1
#else
    #define NRF92_ERRATA_271_PRESENT 0
#endif

#ifndef NRF92_ERRATA_271_ENABLE_WORKAROUND
    #define NRF92_ERRATA_271_ENABLE_WORKAROUND NRF92_ERRATA_271_PRESENT
#endif

static inline bool nrf92_errata_271(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 272 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_272_PRESENT 1
#else
    #define NRF92_ERRATA_272_PRESENT 0
#endif

#ifndef NRF92_ERRATA_272_ENABLE_WORKAROUND
    #define NRF92_ERRATA_272_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_272(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 273 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_273_PRESENT 1
#else
    #define NRF92_ERRATA_273_PRESENT 0
#endif

#ifndef NRF92_ERRATA_273_ENABLE_WORKAROUND
    #define NRF92_ERRATA_273_ENABLE_WORKAROUND NRF92_ERRATA_273_PRESENT
#endif

static inline bool nrf92_errata_273(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 274 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_274_PRESENT 1
#else
    #define NRF92_ERRATA_274_PRESENT 0
#endif

#ifndef NRF92_ERRATA_274_ENABLE_WORKAROUND
    #define NRF92_ERRATA_274_ENABLE_WORKAROUND NRF92_ERRATA_274_PRESENT
#endif

static inline bool nrf92_errata_274(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 275 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_275_PRESENT 1
#else
    #define NRF92_ERRATA_275_PRESENT 0
#endif

#ifndef NRF92_ERRATA_275_ENABLE_WORKAROUND
    #define NRF92_ERRATA_275_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_275(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 277 ========= */
#define NRF92_ERRATA_277_PRESENT 0

#ifndef NRF92_ERRATA_277_ENABLE_WORKAROUND
    #define NRF92_ERRATA_277_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_277(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 278 ========= */
#define NRF92_ERRATA_278_PRESENT 0

#ifndef NRF92_ERRATA_278_ENABLE_WORKAROUND
    #define NRF92_ERRATA_278_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_278(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        return false;
    #endif
}

/* ========= Errata 279 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_279_PRESENT 1
#else
    #define NRF92_ERRATA_279_PRESENT 0
#endif

#ifndef NRF92_ERRATA_279_ENABLE_WORKAROUND
    #define NRF92_ERRATA_279_ENABLE_WORKAROUND 0
#endif

static inline bool nrf92_errata_279(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 280 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_280_PRESENT 1
#else
    #define NRF92_ERRATA_280_PRESENT 0
#endif

#ifndef NRF92_ERRATA_280_ENABLE_WORKAROUND
    #define NRF92_ERRATA_280_ENABLE_WORKAROUND NRF92_ERRATA_280_PRESENT
#endif

static inline bool nrf92_errata_280(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

/* ========= Errata 281 ========= */
#if    defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
    #define NRF92_ERRATA_281_PRESENT 1
#else
    #define NRF92_ERRATA_281_PRESENT 0
#endif

#ifndef NRF92_ERRATA_281_ENABLE_WORKAROUND
    #define NRF92_ERRATA_281_ENABLE_WORKAROUND NRF92_ERRATA_281_PRESENT
#endif

static inline bool nrf92_errata_281(void)
{
    #ifndef NRF92_SERIES
        return false;
    #else
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            uint32_t var1 = *(uint32_t *)0x0FFFE000ul;
            uint32_t var2 = *(uint32_t *)0x0FFFE008ul;
        #endif
        #if defined (NRF9220_XXAA) || defined (DEVELOP_IN_NRF9220)
            if (var1 == 0x2D)
            {
                switch(var2)
                {
                    case 0x12ul:
                        return true;
                    case 0x13ul:
                        return true;
                    case 0xFFFFFFFFul:
                        return true;
                    default:
                        return true;
                }
            }
        #endif
        return false;
    #endif
}

#endif /* NRF92_ERRATAS_H */
