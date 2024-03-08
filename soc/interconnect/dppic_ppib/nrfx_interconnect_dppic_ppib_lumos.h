/*
 * Copyright (c) 2023 - 2024, Nordic Semiconductor ASA
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

#ifndef NRFX_INTERCONNECT_DPPIC_PPIB_LUMOS_H__
#define NRFX_INTERCONNECT_DPPIC_PPIB_LUMOS_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRFX_INTERCONNECT_PPIB_TASKS_GET(PPIB_INDEX) \
    (NRFX_CONCAT(PPIB, PPIB_INDEX, _NTASKSEVENTS_MAX) + 1UL)

#define NRFX_INTERCONNECT_PPIB(FIRST_PPIB_INDEX, SECOND_PPIB_INDEX)                                \
{                                                                                                  \
    .p_ppib1 = NRFX_CONCAT(NRF_PPIB, FIRST_PPIB_INDEX),                                            \
    .p_ppib2 = NRFX_CONCAT(NRF_PPIB, SECOND_PPIB_INDEX),                                           \
    .channels_mask = NRFX_BIT_MASK(NRFX_MIN(NRFX_INTERCONNECT_PPIB_TASKS_GET(FIRST_PPIB_INDEX),    \
                                            NRFX_INTERCONNECT_PPIB_TASKS_GET(SECOND_PPIB_INDEX))), \
}

#define NRFX_INTERCONNECT_PPIB_MAP  \
{                                   \
    NRFX_INTERCONNECT_PPIB(00, 10), \
    NRFX_INTERCONNECT_PPIB(11, 21), \
    NRFX_INTERCONNECT_PPIB(22, 30), \
    NRFX_INTERCONNECT_PPIB(20, 01), \
}

#define NRFX_INTERCONNECT_DPPIC_PPIB_MAP \
{                                        \
    {                                    \
        .dppic = NRF_DPPIC00,            \
        .ppib  = NRF_PPIB00_S,           \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC00,            \
        .ppib  = NRF_PPIB01_S,           \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC10,            \
        .ppib  = NRF_PPIB10_S,           \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC10,            \
        .ppib  = NRF_PPIB11_S,           \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC20,            \
        .ppib  = NRF_PPIB20_S,           \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC20,            \
        .ppib  = NRF_PPIB21_S,           \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC20,            \
        .ppib  = NRF_PPIB22_S,           \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC30,            \
        .ppib  = NRF_PPIB30_S,           \
    },                                   \
}

#define NRFX_INTERCONNECT_DPPIC_MAP                     \
{                                                       \
    {                                                   \
        .apb_index     = NRF_APB_INDEX_MCU,             \
        .dppic         = NRF_DPPIC00,                   \
        .channels_mask = NRFX_BIT_MASK(DPPIC00_CH_NUM), \
        .apb_size      = 0x40000                        \
    },                                                  \
    {                                                   \
        .apb_index     = NRF_APB_INDEX_RADIO,           \
        .dppic         = NRF_DPPIC10,                   \
        .channels_mask = NRFX_BIT_MASK(DPPIC10_CH_NUM), \
        .apb_size      = 0x40000                        \
    },                                                  \
    {                                                   \
        .apb_index     = NRF_APB_INDEX_PERI,            \
        .dppic         = NRF_DPPIC20,                   \
        .channels_mask = NRFX_BIT_MASK(DPPIC20_CH_NUM), \
        .apb_size      = 0x40000                        \
    },                                                  \
    {                                                   \
        .apb_index     = NRF_APB_INDEX_LP,              \
        .dppic         = NRF_DPPIC30,                   \
        .channels_mask = NRFX_BIT_MASK(DPPIC30_CH_NUM), \
        .apb_size      = 0x40000                        \
    },                                                  \
}

#define NRFX_INTERCONNECT_DPPIC_COUNT      DPPIC_COUNT
#define NRFX_INTERCONNECT_DPPIC_PPIB_COUNT PPIB_COUNT
#define NRFX_INTERCONNECT_PPIB_COUNT       (NRFX_INTERCONNECT_DPPIC_PPIB_COUNT/2)

#ifdef __cplusplus
}
#endif

#endif // NRFX_INTERCONNECT_DPPIC_PPIB_LUMOS_H__
