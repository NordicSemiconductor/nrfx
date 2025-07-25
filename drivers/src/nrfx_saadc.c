/*
 * Copyright (c) 2015 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_SAADC_ENABLED)
#include <nrfx_saadc.h>

#define NRFX_LOG_MODULE SAADC
#include <nrfx_log.h>

#if NRF_SAADC_HAS_CAL || NRF_SAADC_HAS_CALREF ||  NRF_SAADC_HAS_LIN_CAL
#include <hal/nrf_ficr.h>
#endif

#if defined(NRF52_SERIES) && !defined(USE_WORKAROUND_FOR_ANOMALY_212)
    // ANOMALY 212 - SAADC events are missing when switching from single channel
    //               to multi channel configuration with burst enabled.
    #define USE_WORKAROUND_FOR_ANOMALY_212 1
#endif

#if defined(NRF53_SERIES) || defined(NRF91_SERIES)
    // Make sure that SAADC is stopped before channel configuration.
    #define STOP_SAADC_ON_CHANNEL_CONFIG 1
#endif

/** @brief Bitmask of all available SAADC channels. */
#define SAADC_ALL_CHANNELS_MASK ((1UL << SAADC_CH_NUM) - 1UL)

/** @brief SAADC driver states.*/
typedef enum
{
    NRF_SAADC_STATE_UNINITIALIZED = 0,
    NRF_SAADC_STATE_IDLE,
    NRF_SAADC_STATE_SIMPLE_MODE,
    NRF_SAADC_STATE_SIMPLE_MODE_SAMPLE,
    NRF_SAADC_STATE_ADV_MODE,
    NRF_SAADC_STATE_ADV_MODE_SAMPLE,
    NRF_SAADC_STATE_ADV_MODE_SAMPLE_STARTED,
    NRF_SAADC_STATE_CALIBRATION
} nrf_saadc_state_t;

/** @brief SAADC control block.*/
typedef struct
{
    nrfx_saadc_event_handler_t event_handler;                ///< Event handler function pointer.
    nrfx_saadc_event_handler_t calib_event_handler;          ///< Event handler function pointer for calibration event.
    nrfy_saadc_buffer_t        buffer_primary;               ///< Primary buffer description structure.
    nrfy_saadc_buffer_t        buffer_secondary;             ///< Secondary buffer description structure.
    uint16_t                   calib_samples[2];             ///< Scratch buffer for post-calibration samples.
    uint16_t                   samples_converted;            ///< Number of samples present in result buffer when in the blocking mode.
    nrfy_saadc_channel_input_t channels_input[SAADC_CH_NUM]; ///< Array holding input of each of the channels.
    nrf_saadc_state_t          saadc_state;                  ///< State of the SAADC driver.
    nrf_saadc_state_t          saadc_state_prev;             ///< Previous state of the SAADC driver.
    uint8_t                    channels_configured;          ///< Bitmask of the configured channels.
    uint8_t                    channels_activated;           ///< Bitmask of the activated channels.
    uint8_t                    channels_activated_count;     ///< Number of the activated channels.
    uint8_t                    limits_low_activated;         ///< Bitmask of the activated low limits.
    uint8_t                    limits_high_activated;        ///< Bitmask of the activated high limits.
    bool                       start_on_end;                 ///< Flag indicating if the START task is to be triggered on the END event.
    bool                       oversampling_without_burst;   ///< Flag indicating whether oversampling without burst is configured.
} nrfx_saadc_cb_t;

static nrfx_saadc_cb_t m_cb;

#if NRFX_CHECK(USE_WORKAROUND_FOR_ANOMALY_212)
static void saadc_anomaly_212_workaround_apply(void)
{
    uint32_t c[SAADC_CH_NUM];
    uint32_t l[SAADC_CH_NUM];

    for (uint32_t i = 0; i < SAADC_CH_NUM; i++)
    {
        c[i] = NRF_SAADC->CH[i].CONFIG;
        l[i] = NRF_SAADC->CH[i].LIMIT;
    }
    nrf_saadc_resolution_t resolution = nrfy_saadc_resolution_get(NRF_SAADC);
    uint32_t u640 = *(volatile uint32_t *)0x40007640;
    uint32_t u644 = *(volatile uint32_t *)0x40007644;
    uint32_t u648 = *(volatile uint32_t *)0x40007648;

    *(volatile uint32_t *)0x40007FFC = 0;
    *(volatile uint32_t *)0x40007FFC = 1;

    for (uint32_t i = 0; i < SAADC_CH_NUM; i++)
    {
        NRF_SAADC->CH[i].CONFIG = c[i];
        NRF_SAADC->CH[i].LIMIT = l[i];
    }
    *(volatile uint32_t *)0x40007640 = u640;
    *(volatile uint32_t *)0x40007644 = u644;
    *(volatile uint32_t *)0x40007648 = u648;
    nrfy_saadc_resolution_set(NRF_SAADC, resolution);
}
#endif // NRFX_CHECK(USE_WORKAROUND_FOR_ANOMALY_212)

static nrfx_err_t saadc_channel_count_get(uint32_t  ch_to_activate_mask,
                                          uint8_t * p_active_ch_count)
{
    NRFX_ASSERT(ch_to_activate_mask);
    NRFX_ASSERT(ch_to_activate_mask < (1 << SAADC_CH_NUM));

    uint8_t active_ch_count = 0;
    for (uint32_t ch_mask = 1; ch_mask < (1 << SAADC_CH_NUM); ch_mask <<= 1)
    {
        if (ch_to_activate_mask & ch_mask)
        {
            // Check if requested channels are configured.
            if (!(m_cb.channels_configured & ch_mask))
            {
                return NRFX_ERROR_INVALID_PARAM;
            }
            active_ch_count++;
        }
    }

    *p_active_ch_count = active_ch_count;
    return NRFX_SUCCESS;
}

static void saadc_channel_config(nrfx_saadc_channel_t const * p_channel)
{
    NRFX_ASSERT(p_channel->pin_p != NRF_SAADC_INPUT_DISABLED);

    uint8_t channel_index = p_channel->channel_index;
    nrfy_saadc_channel_configure(NRF_SAADC, channel_index, &p_channel->channel_config, NULL);
    m_cb.channels_input[channel_index].input_p = p_channel->pin_p;
    m_cb.channels_input[channel_index].input_n = p_channel->pin_n;
    m_cb.channels_configured |= (uint8_t)(1U << channel_index);
}

static void saadc_channels_deconfig(uint32_t channel_mask)
{
    while (channel_mask)
    {
        uint8_t channel = (uint8_t)NRF_CTZ(channel_mask);

        channel_mask             &= ~(1UL << channel);
        m_cb.channels_configured &= (uint8_t)~(1UL << channel);

        m_cb.channels_input[channel].input_p = NRF_SAADC_INPUT_DISABLED;
        m_cb.channels_input[channel].input_n = NRF_SAADC_INPUT_DISABLED;
    }
}

static void saadc_channels_disable(uint32_t channel_mask)
{
    while (channel_mask)
    {
        uint8_t channel = (uint8_t)NRF_CTZ(channel_mask);
        channel_mask &= ~(1UL << channel);
        nrfy_saadc_channel_input_set(NRF_SAADC, channel,
                                     NRF_SAADC_INPUT_DISABLED, NRF_SAADC_INPUT_DISABLED);
    }
}

static bool saadc_busy_check(void)
{
    if ((m_cb.saadc_state == NRF_SAADC_STATE_IDLE)     ||
        (m_cb.saadc_state == NRF_SAADC_STATE_ADV_MODE) ||
        (m_cb.saadc_state == NRF_SAADC_STATE_SIMPLE_MODE))
    {
        return false;
    }
    else
    {
        return true;
    }
}

static void saadc_generic_mode_set(uint32_t                   ch_to_activate_mask,
                                   nrf_saadc_resolution_t     resolution,
                                   nrf_saadc_oversample_t     oversampling,
                                   nrf_saadc_burst_t          burst,
                                   nrfx_saadc_event_handler_t event_handler)
{
#if NRFX_CHECK(USE_WORKAROUND_FOR_ANOMALY_212)
    saadc_anomaly_212_workaround_apply();
#endif

#if NRFX_CHECK(STOP_SAADC_ON_CHANNEL_CONFIG)
    nrfy_saadc_int_disable(NRF_SAADC, NRF_SAADC_INT_STOPPED);
    nrfy_saadc_stop(NRF_SAADC, true);
#endif

    m_cb.limits_low_activated  = 0;
    m_cb.limits_high_activated = 0;

    m_cb.buffer_primary.p_buffer   = NULL;
    m_cb.buffer_secondary.p_buffer = NULL;
    m_cb.event_handler             = event_handler;
    m_cb.channels_activated        = (uint8_t)ch_to_activate_mask;
    m_cb.samples_converted         = 0;

    nrfy_saadc_config_t config = {.resolution = resolution,
                                  .oversampling = oversampling,
                                   NRFX_COND_CODE_1(NRF_SAADC_HAS_BURST,
                                 (.burst = burst), ())};

    nrfy_saadc_periph_configure(NRF_SAADC, &config);
    if (event_handler)
    {
        nrfy_saadc_int_set(NRF_SAADC,
                           NRF_SAADC_INT_STARTED |
                           NRF_SAADC_INT_STOPPED |
                           NRF_SAADC_INT_END);
    }
    else
    {
        nrfy_saadc_int_set(NRF_SAADC, 0);
    }

    for (uint32_t ch_pos = 0; ch_pos < SAADC_CH_NUM; ch_pos++)
    {
        nrfy_saadc_channel_input_t input = {.input_p = NRF_SAADC_INPUT_DISABLED,
                                            .input_n = NRF_SAADC_INPUT_DISABLED};
        if (ch_to_activate_mask & (1 << ch_pos))
        {
            input = m_cb.channels_input[ch_pos];
        }
        nrfy_saadc_channel_configure(NRF_SAADC, (uint8_t)ch_pos, NULL, &input);

#if NRF_SAADC_HAS_CH_BURST
        nrf_saadc_burst_t burst_to_set = NRF_SAADC_BURST_DISABLED;
        if (ch_to_activate_mask & (1 << ch_pos))
        {
            burst_to_set = burst;
        }
        nrfy_saadc_channel_burst_set(NRF_SAADC, (uint8_t)ch_pos, burst_to_set);
#endif
    }
}

nrfx_err_t nrfx_saadc_init(uint8_t interrupt_priority)
{
    nrfx_err_t err_code;
    if (m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED)
    {
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
        err_code = NRFX_ERROR_ALREADY;
#else
        err_code = NRFX_ERROR_INVALID_STATE;
#endif
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    m_cb.saadc_state = NRF_SAADC_STATE_IDLE;

    saadc_channels_deconfig(SAADC_ALL_CHANNELS_MASK);
    uint32_t mask = NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_STARTED) |
                    NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_STOPPED) |
                    NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_END) |
                    NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_CALIBRATEDONE);

    nrfy_saadc_int_init(NRF_SAADC, mask, interrupt_priority, false);
    m_cb.event_handler = NULL;

#if NRF_SAADC_HAS_CALREF && NRF_FICR_HAS_GLOBAL_SAADC_CALREF
    uint32_t trim = nrf_ficr_global_saadc_calref_get(NRF_FICR);
    nrfy_saadc_calref_set(NRF_SAADC, trim);
#endif

#if NRF_SAADC_HAS_CAL && NRF_FICR_HAS_GLOBAL_SAADC_CAL
    trim = nrf_ficr_global_saadc_cal_get(NRF_FICR, 0);
    nrfy_saadc_cal_set(NRF_SAADC, trim);
#endif

#if NRF_SAADC_HAS_LIN_CAL && NRF_FICR_HAS_GLOBAL_SAADC_LINCALCOEFF
    for (uint8_t i = 0; i < FICR_TRIM_GLOBAL_SAADC_LINCALCOEFF_MaxIndex; i++)
    {
        uint32_t val = nrf_ficr_global_saadc_lincalcoeff_get(NRF_FICR, i);
        nrfy_saadc_linearity_calibration_coeff_set(NRF_SAADC, i, val);
    }
#endif

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));

    return err_code;
}

void nrfx_saadc_uninit(void)
{
    nrfx_saadc_abort();

    nrfy_saadc_int_uninit(NRF_SAADC);
    nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
    nrfy_saadc_disable(NRF_SAADC);
    saadc_channels_disable(m_cb.channels_configured | m_cb.channels_activated);
    m_cb.saadc_state = NRF_SAADC_STATE_UNINITIALIZED;
}

bool nrfx_saadc_init_check(void)
{
    return (m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);
}

nrfx_err_t nrfx_saadc_channels_config(nrfx_saadc_channel_t const * p_channels,
                                      uint32_t                     channel_count)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_channels);
    NRFX_ASSERT(channel_count <= SAADC_CH_NUM);

    if (saadc_busy_check())
    {
        return NRFX_ERROR_BUSY;
    }

    saadc_channels_deconfig(SAADC_ALL_CHANNELS_MASK);
    for (uint8_t i = 0; i < channel_count; i++)
    {
        if (m_cb.channels_configured & (1 << p_channels[i].channel_index))
        {
            // This channel is already configured!
            return NRFX_ERROR_INVALID_PARAM;
        }

        saadc_channel_config(&p_channels[i]);
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_saadc_channel_config(nrfx_saadc_channel_t const * p_channel)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_channel);

    if (saadc_busy_check())
    {
        return NRFX_ERROR_BUSY;
    }

    saadc_channel_config(p_channel);

    return NRFX_SUCCESS;
}

uint32_t nrfx_saadc_channels_configured_get(void)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);

    return m_cb.channels_configured;
}

nrfx_err_t nrfx_saadc_channels_deconfig(uint32_t channel_mask)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);

    if (saadc_busy_check())
    {
        return NRFX_ERROR_BUSY;
    }

    saadc_channels_deconfig(channel_mask);

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_saadc_simple_mode_set(uint32_t                   channel_mask,
                                      nrf_saadc_resolution_t     resolution,
                                      nrf_saadc_oversample_t     oversampling,
                                      nrfx_saadc_event_handler_t event_handler)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);

    if (saadc_busy_check())
    {
        return NRFX_ERROR_BUSY;
    }

    uint8_t active_ch_count;
    nrfx_err_t err = saadc_channel_count_get(channel_mask, &active_ch_count);
    if (err != NRFX_SUCCESS)
    {
        return err;
    }

    nrf_saadc_burst_t burst;
    if (oversampling == NRF_SAADC_OVERSAMPLE_DISABLED)
    {
        burst = NRF_SAADC_BURST_DISABLED;
    }
    else
    {
        // Burst is implicitly enabled if oversampling is enabled.
        burst = NRF_SAADC_BURST_ENABLED;
    }

    saadc_generic_mode_set(channel_mask,
                           resolution,
                           oversampling,
                           burst,
                           event_handler);

    m_cb.channels_activated_count = active_ch_count;
    m_cb.saadc_state = NRF_SAADC_STATE_SIMPLE_MODE;

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_saadc_advanced_mode_set(uint32_t                        channel_mask,
                                        nrf_saadc_resolution_t          resolution,
                                        nrfx_saadc_adv_config_t const * p_config,
                                        nrfx_saadc_event_handler_t      event_handler)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_config);

    if (saadc_busy_check())
    {
        return NRFX_ERROR_BUSY;
    }

    uint8_t active_ch_count;
    nrfx_err_t err = saadc_channel_count_get(channel_mask, &active_ch_count);
    if (err != NRFX_SUCCESS)
    {
        return err;
    }

    if ((p_config->internal_timer_cc) && ((active_ch_count > 1) || (!event_handler)))
    {
        return NRFX_ERROR_NOT_SUPPORTED;
    }

    bool oversampling_without_burst = false;
    if ((p_config->oversampling != NRF_SAADC_OVERSAMPLE_DISABLED) &&
        (p_config->burst == NRF_SAADC_BURST_DISABLED))
    {
        if (active_ch_count > 1)
        {
            // Oversampling without burst is possible only on single channel.
            return NRFX_ERROR_NOT_SUPPORTED;
        }
        else
        {
            oversampling_without_burst = true;
        }
    }

    saadc_generic_mode_set(channel_mask,
                           resolution,
                           p_config->oversampling,
                           p_config->burst,
                           event_handler);

    if (p_config->internal_timer_cc)
    {
        nrfy_saadc_continuous_mode_enable(NRF_SAADC, p_config->internal_timer_cc);
    }
    else
    {
        nrfy_saadc_continuous_mode_disable(NRF_SAADC);
    }

    m_cb.channels_activated_count = active_ch_count;
    m_cb.start_on_end = p_config->start_on_end;
    m_cb.oversampling_without_burst = oversampling_without_burst;

    m_cb.saadc_state = NRF_SAADC_STATE_ADV_MODE;

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_saadc_buffer_set(nrf_saadc_value_t * p_buffer, uint16_t size)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);

    if (m_cb.buffer_secondary.p_buffer)
    {
        return NRFX_ERROR_ALREADY;
    }

    if (!nrfx_is_in_ram(p_buffer))
    {
        return NRFX_ERROR_INVALID_ADDR;
    }

    if ((size % m_cb.channels_activated_count != 0) ||
        (size > SAADC_RESULT_MAXCNT_MAXCNT_Msk)  ||
        (!size))
    {
        return NRFX_ERROR_INVALID_LENGTH;
    }

    nrfy_saadc_buffer_t buffer = {.p_buffer = p_buffer, .length = size};
    switch (m_cb.saadc_state)
    {
        case NRF_SAADC_STATE_SIMPLE_MODE:
            if (m_cb.channels_activated_count != size)
            {
                return NRFX_ERROR_INVALID_LENGTH;
            }
            m_cb.buffer_primary = buffer;
            break;

        case NRF_SAADC_STATE_ADV_MODE_SAMPLE_STARTED:
            nrfy_saadc_buffer_set(NRF_SAADC, &buffer, false, false);
            /* FALLTHROUGH */

        case NRF_SAADC_STATE_ADV_MODE:
            /* FALLTHROUGH */

        case NRF_SAADC_STATE_ADV_MODE_SAMPLE:
            if (m_cb.buffer_primary.p_buffer)
            {
                m_cb.buffer_secondary = buffer;
            }
            else
            {
                m_cb.buffer_primary = buffer;
            }
            break;

        default:
            return NRFX_ERROR_INVALID_STATE;
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_saadc_mode_trigger(void)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_IDLE);

    if (!m_cb.buffer_primary.p_buffer)
    {
        return NRFX_ERROR_NO_MEM;
    }

    nrfx_err_t result = NRFX_SUCCESS;
    switch (m_cb.saadc_state)
    {
        case NRF_SAADC_STATE_SIMPLE_MODE:
        {
            nrfy_saadc_enable(NRF_SAADC);
            // When in simple blocking or non-blocking mode, buffer size is equal to activated channel count.
            // Single SAMPLE task is enough to obtain one sample on each activated channel.
            // This will result in buffer being filled with samples and therefore END event will appear.

            if (m_cb.event_handler)
            {
                m_cb.saadc_state = NRF_SAADC_STATE_SIMPLE_MODE_SAMPLE;
                nrfy_saadc_buffer_set(NRF_SAADC, &m_cb.buffer_primary, true, false);
            }
            else
            {
                nrfy_saadc_buffer_set(NRF_SAADC, &m_cb.buffer_primary, true, true);
                nrfy_saadc_sample_start(NRF_SAADC, &m_cb.buffer_primary);
                nrfy_saadc_disable(NRF_SAADC);
            }
            break;
        }

        case NRF_SAADC_STATE_ADV_MODE:
        {
            nrfy_saadc_enable(NRF_SAADC);
            if (m_cb.event_handler)
            {
                // When in advanced non-blocking mode, latch whole buffer in EasyDMA.
                // END event will arrive when whole buffer is filled with samples.

                m_cb.saadc_state = NRF_SAADC_STATE_ADV_MODE_SAMPLE;
                nrfy_saadc_buffer_set(NRF_SAADC, &m_cb.buffer_primary, true, false);
                break;
            }
            // When in advanced blocking mode, latch single chunk of buffer in EasyDMA.
            // Each chunk consists of single sample from each activated channels.
            // END event will arrive when single chunk is filled with samples.
            nrfy_saadc_buffer_t chunk = { .length = m_cb.channels_activated_count};

            chunk.p_buffer = (nrf_saadc_value_t *)
                &((uint16_t *)m_cb.buffer_primary.p_buffer)[m_cb.samples_converted];

            nrfy_saadc_buffer_set(NRF_SAADC, &chunk, true, true);
            if (m_cb.oversampling_without_burst)
            {
                // Oversampling without burst is possible only on single channel.
                // In this configuration more than one SAMPLE task is needed to obtain single sample.
                uint32_t samples_to_take =
                    nrfy_saadc_oversample_sample_count_get(nrfy_saadc_oversample_get(NRF_SAADC));

                for (uint32_t sample_idx = 0; sample_idx < samples_to_take - 1; sample_idx++)
                {
                    nrfy_saadc_sample_start(NRF_SAADC, NULL);
                    uint32_t evt_mask = NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_DONE);
                    while (!nrfy_saadc_events_process(NRF_SAADC, evt_mask, NULL))
                    {}
                }
            }
            // Single SAMPLE task is enough to obtain one sample on each activated channel.
            // This will result in chunk being filled with samples and therefore END event will appear.
            nrfy_saadc_sample_start(NRF_SAADC, &chunk);

            m_cb.samples_converted += m_cb.channels_activated_count;
            if (m_cb.samples_converted < m_cb.buffer_primary.length)
            {
                result = NRFX_ERROR_BUSY;
            }
            else
            {
                m_cb.samples_converted         = 0;
                m_cb.buffer_primary            = m_cb.buffer_secondary;
                m_cb.buffer_secondary.p_buffer = NULL;
            }
            nrfy_saadc_disable(NRF_SAADC);
            break;
        }

        default:
            result = NRFX_ERROR_INVALID_STATE;
            break;
    }

    return result;
}

void nrfx_saadc_abort(void)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);

    if (m_cb.saadc_state == NRF_SAADC_STATE_CALIBRATION ? m_cb.calib_event_handler :
                                                          m_cb.event_handler)
    {
        nrfy_saadc_abort(NRF_SAADC, NULL);
    }
    else
    {
        m_cb.buffer_primary.p_buffer   = NULL;
        m_cb.buffer_secondary.p_buffer = NULL;
        m_cb.samples_converted         = 0;
    }
}

nrfx_err_t nrfx_saadc_limits_set(uint8_t channel, int16_t limit_low, int16_t limit_high)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);
    NRFX_ASSERT(limit_high >= limit_low);

    if (!m_cb.event_handler)
    {
        return NRFX_ERROR_FORBIDDEN;
    }

    if ((m_cb.saadc_state == NRF_SAADC_STATE_IDLE) ||
        (m_cb.saadc_state == NRF_SAADC_STATE_CALIBRATION))
    {
        return NRFX_ERROR_INVALID_STATE;
    }

    if (!(m_cb.channels_activated & (1 << channel)))
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    nrfy_saadc_channel_limits_set(NRF_SAADC, channel, limit_low, limit_high);

    uint32_t int_mask = nrfy_saadc_limit_int_get(channel, NRF_SAADC_LIMIT_LOW);
    if (limit_low == INT16_MIN)
    {
        m_cb.limits_low_activated &= (uint8_t)~(1UL << channel);
        nrfy_saadc_int_disable(NRF_SAADC, int_mask);
    }
    else
    {
        m_cb.limits_low_activated |= (uint8_t)(1UL << channel);
        nrfy_saadc_int_enable(NRF_SAADC, int_mask);
    }

    int_mask = nrfy_saadc_limit_int_get(channel, NRF_SAADC_LIMIT_HIGH);
    if (limit_high == INT16_MAX)
    {
        m_cb.limits_high_activated &= (uint8_t)~(1UL << channel);
        nrfy_saadc_int_disable(NRF_SAADC, int_mask);
    }
    else
    {
        m_cb.limits_high_activated |= (uint8_t)(1UL << channel);
        nrfy_saadc_int_enable(NRF_SAADC, int_mask);
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_saadc_offset_calibrate(nrfx_saadc_event_handler_t calib_event_handler)
{
    NRFX_ASSERT(m_cb.saadc_state != NRF_SAADC_STATE_UNINITIALIZED);

    if (saadc_busy_check())
    {
        return NRFX_ERROR_BUSY;
    }

    m_cb.saadc_state_prev = m_cb.saadc_state;
    m_cb.saadc_state = NRF_SAADC_STATE_CALIBRATION;
    m_cb.calib_event_handler = calib_event_handler;

    nrfy_saadc_enable(NRF_SAADC);

    uint32_t int_mask = nrfy_saadc_int_enable_check(NRF_SAADC, ~0UL);
    nrfy_saadc_int_set(NRF_SAADC, 0);
    if (calib_event_handler)
    {
        nrfy_saadc_calibrate(NRF_SAADC, false);
        // Make sure that LIMIT feature is disabled before offset calibration.
        int_mask &= ~(uint32_t)(NRF_SAADC_INT_CH0LIMITL | NRF_SAADC_INT_CH0LIMITH);
        nrfy_saadc_int_set(NRF_SAADC, int_mask | NRF_SAADC_INT_STARTED | NRF_SAADC_INT_STOPPED |
                                      NRF_SAADC_INT_END | NRF_SAADC_INT_CALIBRATEDONE);
    }
    else
    {
        nrfy_saadc_calibrate(NRF_SAADC, true);

        nrfy_saadc_buffer_t calib_buffer = {.p_buffer = m_cb.calib_samples,
                                            .length   = NRFX_ARRAY_SIZE(m_cb.calib_samples)};
        nrfy_saadc_buffer_set(NRF_SAADC, &calib_buffer, true, true);

        nrfy_saadc_stop(NRF_SAADC, true);
        nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_END);
        nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
        nrfy_saadc_disable(NRF_SAADC);
        m_cb.saadc_state = m_cb.saadc_state_prev;

        nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_CH0_LIMITL);
        nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_CH0_LIMITH);
        nrfy_saadc_int_set(NRF_SAADC, int_mask);
    }

    return NRFX_SUCCESS;
}

static void saadc_pre_calibration_state_restore(void)
{
    nrfy_saadc_disable(NRF_SAADC);
    uint32_t int_mask = nrfy_saadc_int_enable_check(NRF_SAADC, ~0UL) &
                        (uint32_t)(~(NRF_SAADC_INT_STARTED | NRF_SAADC_INT_STOPPED |
                                     NRF_SAADC_INT_END | NRF_SAADC_INT_CALIBRATEDONE));
    m_cb.saadc_state = m_cb.saadc_state_prev;
    if (m_cb.event_handler)
    {
        // Restore interrupts that are used in sampling if user provided event handler
        // during mode configuration.
        int_mask |= NRF_SAADC_INT_STARTED | NRF_SAADC_INT_STOPPED | NRF_SAADC_INT_END;
    }
    nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_CH0_LIMITL);
    nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_CH0_LIMITH);
    nrfy_saadc_event_clear(NRF_SAADC, NRF_SAADC_EVENT_DONE);
    if (m_cb.limits_low_activated & 0x1UL)
    {
        int_mask |= NRF_SAADC_INT_CH0LIMITL;
    }
    if (m_cb.limits_high_activated & 0x1UL)
    {
        int_mask |= NRF_SAADC_INT_CH0LIMITH;
    }
    nrfy_saadc_int_set(NRF_SAADC, int_mask);
}

static void saadc_event_started_handle(void)
{
    nrfx_saadc_evt_t evt_data;

    switch (m_cb.saadc_state)
    {
        case NRF_SAADC_STATE_ADV_MODE_SAMPLE:
            evt_data.type = NRFX_SAADC_EVT_READY;
            m_cb.event_handler(&evt_data);

            if (nrfy_saadc_continuous_mode_enable_check(NRF_SAADC))
            {
                // Trigger internal timer
                nrfy_saadc_sample_start(NRF_SAADC, NULL);
            }

            m_cb.saadc_state = NRF_SAADC_STATE_ADV_MODE_SAMPLE_STARTED;
            if (m_cb.buffer_secondary.p_buffer)
            {
                nrfy_saadc_buffer_set(NRF_SAADC, &m_cb.buffer_secondary, false, false);
            }
            /* FALLTHROUGH */

        case NRF_SAADC_STATE_ADV_MODE_SAMPLE_STARTED:
            if (!m_cb.buffer_secondary.p_buffer)
            {
                // Send next buffer request only if it was not provided earlier,
                // before conversion start or outside of user's callback context.
                evt_data.type = NRFX_SAADC_EVT_BUF_REQ;
                m_cb.event_handler(&evt_data);
            }
            break;

        case NRF_SAADC_STATE_SIMPLE_MODE_SAMPLE:
            nrfy_saadc_sample_start(NRF_SAADC, NULL);
            break;

        case NRF_SAADC_STATE_CALIBRATION:
            // Stop the SAADC immediately after the temporary buffer is latched to drop spurious samples.
            // This will cause STOPPED and END events to arrive.
            nrfy_saadc_stop(NRF_SAADC, false);
            break;

        default:
            break;
    }
}

static void saadc_event_end_handle(void)
{
    nrfx_saadc_evt_t evt_data;
    evt_data.type = NRFX_SAADC_EVT_DONE;
    evt_data.data.done.p_buffer = m_cb.buffer_primary.p_buffer;
    evt_data.data.done.size = (uint16_t)m_cb.buffer_primary.length;

    switch (m_cb.saadc_state)
    {
        case NRF_SAADC_STATE_SIMPLE_MODE_SAMPLE:
            nrfy_saadc_disable(NRF_SAADC);
            m_cb.saadc_state = NRF_SAADC_STATE_SIMPLE_MODE;
            /* In the simple, non-blocking mode the event handler must be
             * called after the internal driver state is updated. This will
             * allow starting a new conversion from the event handler context.
             */
            m_cb.event_handler(&evt_data);
            break;

        case NRF_SAADC_STATE_ADV_MODE_SAMPLE_STARTED:
            if (m_cb.start_on_end && m_cb.buffer_secondary.p_buffer)
            {
                nrfy_saadc_buffer_latch(NRF_SAADC, false);
            }
            m_cb.event_handler(&evt_data);
            m_cb.buffer_primary = m_cb.buffer_secondary;
            m_cb.buffer_secondary.p_buffer = NULL;
            if (!m_cb.buffer_primary.p_buffer)
            {
                nrfy_saadc_disable(NRF_SAADC);
                m_cb.saadc_state = NRF_SAADC_STATE_ADV_MODE;
                evt_data.type = NRFX_SAADC_EVT_FINISHED;
                m_cb.event_handler(&evt_data);
            }
            break;

        case NRF_SAADC_STATE_CALIBRATION:
            // Spurious samples were successfully dropped and they won't affect next conversion.
            saadc_pre_calibration_state_restore();
            evt_data.type = NRFX_SAADC_EVT_CALIBRATEDONE;
            m_cb.calib_event_handler(&evt_data);
            break;

        default:
            break;
    }
}

static void saadc_event_limits_handle(void)
{
    uint32_t limits_activated = nrfy_saadc_int_enable_check(NRF_SAADC,
                                                            NRF_SAADC_ALL_CHANNELS_LIMITS_INT_MASK);
    uint32_t limits_triggered = nrfy_saadc_events_process(NRF_SAADC, limits_activated, NULL) >>
                                NRF_SAADC_LIMITS_INT_OFFSET;

    while (limits_triggered)
    {
        uint8_t limit = (uint8_t)NRF_CTZ((uint32_t)limits_triggered);
        limits_triggered &= ~(1UL << limit);

         // There are two limits per channel.
        uint8_t channel = limit / 2;

        // Limits are organised into single pair (high limit and low limit) per channel.
        // Do not assume whether high limit or low limit is first in the bitmask.
        nrf_saadc_limit_t limit_type =
            ((limit & 0x1) == (NRFY_EVENT_TO_INT_BITPOS(NRF_SAADC_EVENT_CH0_LIMITH) & 0x1)) ?
            NRF_SAADC_LIMIT_HIGH : NRF_SAADC_LIMIT_LOW;

        nrfx_saadc_evt_t evt_data;
        evt_data.type = NRFX_SAADC_EVT_LIMIT;
        evt_data.data.limit.channel = channel;
        evt_data.data.limit.limit_type = limit_type;
        m_cb.event_handler(&evt_data);
    }
}

void nrfx_saadc_irq_handler(void)
{
    uint32_t evt_mask = NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_STARTED) |
                        NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_STOPPED) |
                        NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_END) |
                        NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_CALIBRATEDONE);
    evt_mask = nrfy_saadc_events_process(NRF_SAADC, evt_mask, &m_cb.buffer_primary);

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_CALIBRATEDONE))
    {
        nrfy_saadc_int_disable(NRF_SAADC, NRF_SAADC_INT_CALIBRATEDONE);
        // Latch the temporary buffer to intercept any spurious samples that may appear after calibration.
        nrfy_saadc_buffer_t calib_buffer = {.p_buffer = m_cb.calib_samples,
                                            .length  = NRFX_ARRAY_SIZE(m_cb.calib_samples)};
        nrfy_saadc_buffer_set(NRF_SAADC, &calib_buffer, true, false);
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_STOPPED))
    {
        if (m_cb.saadc_state != NRF_SAADC_STATE_CALIBRATION)
        {
            // If there was ongoing conversion the STOP task also triggers the END event
            m_cb.buffer_primary.length = nrfy_saadc_amount_get(NRF_SAADC);
            m_cb.buffer_secondary.p_buffer = NULL;
        }

        if (nrfy_saadc_int_enable_check(NRF_SAADC, NRF_SAADC_INT_CALIBRATEDONE))
        {
            // If STOP event arrived before CALIBRATEDONE then the calibration was aborted
            // and END event will not appear.
            // Calibration procedure was not completed and user handler will not be called.
            saadc_pre_calibration_state_restore();
        }
        /* fall-through to the END event handler */
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_END))
    {
        saadc_event_end_handle();
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_SAADC_EVENT_STARTED))
    {
        saadc_event_started_handle();
    }

    if (m_cb.saadc_state != NRF_SAADC_STATE_CALIBRATION)
    {
        saadc_event_limits_handle();
    }
}

#endif // NRFX_CHECK(NRFX_SAADC_ENABLED)
