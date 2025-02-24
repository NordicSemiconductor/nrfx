/*
 * Copyright (c) 2024 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_PPIB_ENABLED)

#include <nrfx_ppib.h>
#include <helpers/nrfx_flag32_allocator.h>

#define NRFX_LOG_MODULE PPIB
#include <nrfx_log.h>

#if !defined(__NRFX_DOXYGEN__)

#if defined(NRF54L_SERIES) || defined(NRF7120_ENGA_XXAA)

#if !defined(NRFX_PPIB_INTERCONNECT_00_10_CHANNELS_USED)
/**
 * Bitmask that defines PPIB00 and PPIB10 channels that are
 * reserved for use outside of the nrfx library.
 */
#define NRFX_PPIB_INTERCONNECT_00_10_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_PPIB_INTERCONNECT_01_20_CHANNELS_USED)
/**
 * Bitmask that defines PPIB01 and PPIB20 channels that are
 * reserved for use outside of the nrfx library.
 */
#define NRFX_PPIB_INTERCONNECT_01_20_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_PPIB_INTERCONNECT_11_21_CHANNELS_USED)
/**
 * Bitmask that defines PPIB11 and PPIB21 channels that are
 * reserved for use outside of the nrfx library.
 */
#define NRFX_PPIB_INTERCONNECT_11_21_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_PPIB_INTERCONNECT_22_30_CHANNELS_USED)
/**
 * Bitmask that defines PPIB022 and PPIB30 channels that are
 * reserved for use outside of the nrfx library.
 */
#define NRFX_PPIB_INTERCONNECT_22_30_CHANNELS_USED 0UL
#endif

#endif

#if defined(NRF54LM20A_ENGA_XXAA)

#if !defined(NRFX_PPIB_INTERCONNECT_02_03_CHANNELS_USED)
/**
 * Bitmask that defines PPIB022 and PPIB30 channels that are
 * reserved for use outside of the nrfx library.
 */
#define NRFX_PPIB_INTERCONNECT_02_03_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_PPIB_INTERCONNECT_04_12_CHANNELS_USED)
/**
 * Bitmask that defines PPIB022 and PPIB30 channels that are
 * reserved for use outside of the nrfx library.
 */
#define NRFX_PPIB_INTERCONNECT_04_12_CHANNELS_USED 0UL
#endif

#endif

#if defined(HALTIUM_XXAA)

#if !defined(NRFX_PPIB_INTERCONNECT_020_030_CHANNELS_USED)
/**
 * Bitmask that defines PPIB020 and PPIB030 channels that are
 * reserved for use outside of the nrfx library.
 */
#define NRFX_PPIB_INTERCONNECT_020_030_CHANNELS_USED 0UL
#endif

#endif

#endif // !defined(__NRFX_DOXYGEN__)

#define PPIB_CHANNELS_NUM(idx)  (NRFX_CONCAT(PPIB, idx, _NTASKSEVENTS_MAX) + 1UL)
#define PPIB_CHANNELS_MASK(left, right)                                       \
    NRFX_BIT_MASK(NRFX_MIN(PPIB_CHANNELS_NUM(left), PPIB_CHANNELS_NUM(right))
#define PPIB_CHANNELS_USED(left, right)                                 \
    NRFX_CONCAT(NRFX_PPIB_INTERCONNECT_, left, _, right, _CHANNELS_USED)
#define PPIB_AVAILABLE_CHANNELS_MASK(left, right) \
    ((uint32_t)(PPIB_CHANNELS_MASK(left, right)) & ~(PPIB_CHANNELS_USED(left, right))))

/* Structure holding state of the PPIB instance. */
typedef struct
{
    /**< Bitmap representing channels availability. */
    nrfx_atomic_t allocated_channels;
    /**< Bitmap representing available channels. */
    const uint32_t available_channels;
} ppib_control_block_t;

#define _NRFX_PPIBC_CB_INITIALIZER(left_idx, right_idx)                                         \
    [NRFX_CONCAT(NRFX_PPIB_INTERCONNECT_, left_idx, _, right_idx, _INST_IDX)] = {               \
        .allocated_channels = (nrfx_atomic_t)PPIB_AVAILABLE_CHANNELS_MASK(left_idx, right_idx), \
        .available_channels = PPIB_AVAILABLE_CHANNELS_MASK(left_idx, right_idx),                \
    },

static ppib_control_block_t m_cb[NRFX_PPIB_INTERCONNECT_COUNT] = {
#if defined(NRF54L_SERIES) || defined(NRF7120_ENGA_XXAA)
#if NRFX_CHECK(NRFX_PPIB00_ENABLED) && NRFX_CHECK(NRFX_PPIB10_ENABLED)
    _NRFX_PPIBC_CB_INITIALIZER(00, 10)
#endif
#if NRFX_CHECK(NRFX_PPIB01_ENABLED) && NRFX_CHECK(NRFX_PPIB20_ENABLED)
    _NRFX_PPIBC_CB_INITIALIZER(01, 20)
#endif
#if NRFX_CHECK(NRFX_PPIB11_ENABLED) && NRFX_CHECK(NRFX_PPIB21_ENABLED)
    _NRFX_PPIBC_CB_INITIALIZER(11, 21)
#endif
#if NRFX_CHECK(NRFX_PPIB22_ENABLED) && NRFX_CHECK(NRFX_PPIB30_ENABLED)
    _NRFX_PPIBC_CB_INITIALIZER(22, 30)
#endif
#endif
#if defined(NRF54LM20A_ENGA_XXAA)
#if NRFX_CHECK(NRFX_PPIB02_ENABLED) && NRFX_CHECK(NRFX_PPIB03_ENABLED)
    _NRFX_PPIBC_CB_INITIALIZER(02, 03)
#endif
#if NRFX_CHECK(NRFX_PPIB04_ENABLED) && NRFX_CHECK(NRFX_PPIB12_ENABLED)
    _NRFX_PPIBC_CB_INITIALIZER(04, 12)
#endif
#endif
#if defined(HALTIUM_XXAA)
#if NRFX_CHECK(NRFX_PPIB020_ENABLED) && NRFX_CHECK(NRFX_PPIB030_ENABLED)
    _NRFX_PPIBC_CB_INITIALIZER(020, 030)
#endif
#endif
};

void nrfx_ppib_free(nrfx_ppib_interconnect_t const * p_instance)
{
    ppib_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    uint32_t mask = p_cb->available_channels & ~p_cb->allocated_channels;
    uint8_t channel_idx = 0;

    // Clear all channel configurations
    while (mask)
    {
        if (mask & NRFX_BIT(channel_idx))
        {
            nrfx_ppib_channel_free(p_instance, channel_idx);
            mask &= ~NRFX_BIT(channel_idx);
        }
        channel_idx++;
    }
}

nrfx_err_t nrfx_ppib_channel_alloc(nrfx_ppib_interconnect_t const * p_instance, uint8_t * p_channel)
{
    ppib_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    return nrfx_flag32_alloc(&p_cb->allocated_channels, p_channel);
}

nrfx_err_t nrfx_ppib_channel_free(nrfx_ppib_interconnect_t const * p_instance, uint8_t channel)
{
    ppib_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if ((p_cb->available_channels & NRFX_BIT(channel)) == 0)
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    nrf_ppib_subscribe_clear(p_instance->left.p_reg, nrf_ppib_send_task_get(channel));
    nrf_ppib_subscribe_clear(p_instance->right.p_reg, nrf_ppib_send_task_get(channel));
    nrf_ppib_publish_clear(p_instance->left.p_reg, nrf_ppib_receive_event_get(channel));
    nrf_ppib_publish_clear(p_instance->right.p_reg, nrf_ppib_receive_event_get(channel));

    return nrfx_flag32_free(&p_cb->allocated_channels, channel);
}

#endif // defined(PPIB_PRESENT)
