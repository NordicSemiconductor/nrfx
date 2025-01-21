/*
 * Copyright (c) 2023 - 2025, Nordic Semiconductor ASA
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
#include <nrfx_ppib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRFX_INTERCONNECT_PPIB_TASKS_GET(PPIB_INDEX) \
    (NRFX_CONCAT(PPIB, PPIB_INDEX, _NTASKSEVENTS_MAX) + 1UL)

#define NRFX_INTERCONNECT_PPIB(FIRST_PPIB_INDEX, SECOND_PPIB_INDEX)                                \
{                                                                                                  \
    .ppib = NRFX_PPIB_INTERCONNECT_INSTANCE(FIRST_PPIB_INDEX, SECOND_PPIB_INDEX),                  \
}

#if NRFX_API_VER_AT_LEAST(3, 8, 0)
#define DPPI_INSTANCE(idx) .dppic = NRFX_DPPI_INSTANCE(idx)
#else
#define DPPI_INSTANCE(idx)                                          \
    .dppic         = NRFX_CONCAT(NRF_DPPIC, idx),                   \
    .channels_mask = NRFX_BIT_MASK(NRFX_CONCAT(DPPIC, idx, _CH_NUM))
#endif

#if defined(NRF54L_SERIES) || defined(NRF7120_ENGA_XXAA)

#define NRFX_INTERCONNECT_PPIB_MAP  \
{                                   \
    NRFX_INTERCONNECT_PPIB(00, 10), \
    NRFX_INTERCONNECT_PPIB(11, 21), \
    NRFX_INTERCONNECT_PPIB(22, 30), \
    NRFX_INTERCONNECT_PPIB(01, 20), \
}

#define NRFX_INTERCONNECT_DPPIC_PPIB_MAP \
{                                        \
    {                                    \
        .dppic = NRF_DPPIC00,            \
        .ppib  = NRF_PPIB00,             \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC00,            \
        .ppib  = NRF_PPIB01,             \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC10,            \
        .ppib  = NRF_PPIB10,             \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC10,            \
        .ppib  = NRF_PPIB11,             \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC20,            \
        .ppib  = NRF_PPIB20,             \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC20,            \
        .ppib  = NRF_PPIB21,             \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC20,            \
        .ppib  = NRF_PPIB22,             \
    },                                   \
    {                                    \
        .dppic = NRF_DPPIC30,            \
        .ppib  = NRF_PPIB30,             \
    },                                   \
}

#define NRFX_INTERCONNECT_DPPIC_MAP                        \
{                                                          \
    {                                                      \
        .apb_index     = NRF_APB_INDEX_MCU,                \
        DPPI_INSTANCE(00),                                 \
        .apb_size      = 0x40000                           \
    },                                                     \
    {                                                      \
        .apb_index     = NRF_APB_INDEX_RADIO,              \
        DPPI_INSTANCE(10),                                 \
        .apb_size      = 0x40000                           \
    },                                                     \
    {                                                      \
        .apb_index     = NRF_APB_INDEX_PERI,               \
        DPPI_INSTANCE(20),                                 \
        .apb_size      = 0x40000                           \
    },                                                     \
    {                                                      \
        .apb_index     = NRF_APB_INDEX_LP,                 \
        DPPI_INSTANCE(30),                                 \
        .apb_size      = 0x40000                           \
    },                                                     \
}

#endif

#define NRFX_INTERCONNECT_DPPIC_COUNT      DPPIC_COUNT
#define NRFX_INTERCONNECT_DPPIC_PPIB_COUNT PPIB_COUNT
#define NRFX_INTERCONNECT_PPIB_COUNT       (NRFX_INTERCONNECT_DPPIC_PPIB_COUNT / 2)

#ifdef __cplusplus
}
#endif

#endif // NRFX_INTERCONNECT_DPPIC_PPIB_LUMOS_H__
