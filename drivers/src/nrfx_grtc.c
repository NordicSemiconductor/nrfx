/*
 * Copyright (c) 2021 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_GRTC_ENABLED)

#include <nrfx_grtc.h>
#include <soc/nrfx_coredep.h>
#include <helpers/nrfx_flag32_allocator.h>

#define NRFX_LOG_MODULE GRTC
#include <nrfx_log.h>

#if NRFY_GRTC_HAS_EXTENDED
#define GRTC_ACTION_TO_STR(action)                                                     \
    (action == NRFX_GRTC_ACTION_START ? "NRFX_GRTC_ACTION_START" : \
    (action == NRFX_GRTC_ACTION_STOP  ? "NRFX_GRTC_ACTION_STOP"  : \
    (action == NRFX_GRTC_ACTION_CLEAR ? "NRFX_GRTC_ACTION_CLEAR" : \
                                        "UNKNOWN ACTION")))
#endif

#define GRTC_CHANNEL_TO_BITMASK(chan)          NRFX_BIT(chan)
#define GRTC_CHANNEL_MASK_TO_INT_MASK(ch_mask) ((ch_mask) << GRTC_INTEN0_COMPARE0_Pos)

#if !defined(NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK)
#error "Channels mask for GRTC must be defined."
#endif

#if !defined(NRFX_GRTC_CONFIG_NUM_OF_CC_CHANNELS)
#error "Number of channels for GRTC must be defined."
#endif

#if NRF_GRTC_HAS_RTCOUNTER
#define GRTC_NON_SYSCOMPARE_INT_MASK   (NRF_GRTC_INT_RTCOMPARE_MASK     | \
                                        NRF_GRTC_INT_RTCOMPARESYNC_MASK | \
                                        NRF_GRTC_INT_SYSCOUNTERVALID_MASK)
#define GRTC_ALL_INT_MASK              (NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK | \
                                        GRTC_NON_SYSCOMPARE_INT_MASK)
#define GRTC_RTCOUNTER_CC_HANDLER_IDX  NRFX_GRTC_CONFIG_NUM_OF_CC_CHANNELS
#define GRTC_RTCOUNTER_COMPARE_CHANNEL NRF_GRTC_SYSCOUNTER_CC_COUNT
#else
#define GRTC_ALL_INT_MASK              (NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK)
#endif // NRF_GRTC_HAS_RTCOUNTER

#if !(defined(NRF_SECURE) && NRFX_IS_ENABLED(NRFY_GRTC_HAS_EXTENDED))
    #define MAIN_GRTC_CC_CHANNEL NRF_GRTC_MAIN_CC_CHANNEL
    #if NRFX_IS_ENABLED(NRFY_GRTC_HAS_EXTENDED)
        /* Verify that the GRTC owner possesses the main capture/compare channel. */
        NRFX_STATIC_ASSERT(NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK &
                           GRTC_CHANNEL_TO_BITMASK(MAIN_GRTC_CC_CHANNEL));
    #else
        /* Any other domain which is not an owner of GRTC shouldn't have an access to
           the main capture/compare channel. */
        NRFX_STATIC_ASSERT(!(NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK &
                           GRTC_CHANNEL_TO_BITMASK(MAIN_GRTC_CC_CHANNEL)));
    #endif //NRFX_IS_ENABLED(NRFY_GRTC_HAS_EXTENDED)
#else
    /* Change the main capture/compare channel to allow Secdom to start GRTC in extended mode. */
    #define MAIN_GRTC_CC_CHANNEL (m_cb.channel_data[0].channel)
#endif // !(defined(NRF_SECURE) && NRFY_GRTC_HAS_EXTENDED)

/* The maximum SYSCOUNTERVALID settling time equals 1x32k cycles + 20x16MHz cycles. */
#define GRTC_SYSCOUNTERVALID_SETTLE_MAX_TIME_US 33

/* The timeout for the SYSCOUNTER's ready state after starting it. */
#define STARTUP_TIMEOUT ((SystemCoreClock / 1000000U) * GRTC_SYSCOUNTERVALID_SETTLE_MAX_TIME_US)

typedef struct
{
    nrfx_drv_state_t                    state;                                                 /**< Driver state. */
    nrfx_atomic_t                       available_channels;                                    /**< Bitmask of available channels. */
    uint32_t                            used_channels;                                         /**< Bitmask of channels used by the driver. */
    nrfx_grtc_channel_t                 channel_data[NRFX_GRTC_CONFIG_NUM_OF_CC_CHANNELS + 1]; /**< Channel specific data. */
    uint8_t                             ch_to_data[NRF_GRTC_SYSCOUNTER_CC_COUNT];              /**< Mapping of channel index to channel_data index. */
    uint64_t                            cc_value[NRFX_GRTC_CONFIG_NUM_OF_CC_CHANNELS];         /**< Last CC value. */
    nrfx_atomic_t                       read_cc_mask;                                          /**< Indicating if CC value must be passed to the handler. */
#if NRF_GRTC_HAS_RTCOUNTER
    nrfx_grtc_rtcomparesync_handler_t   rtcomparesync_handler;                                 /**< User handler corresponding to rtcomparesync event.*/
    void *                              rtcomparesync_context;                                 /**< User context for rtcomparesync event handler. */
#endif
#if NRFY_GRTC_HAS_EXTENDED && NRFY_GRTC_HAS_SYSCOUNTERVALID
    nrfx_grtc_syscountervalid_handler_t syscountervalid_handler;                               /**< User handler corresponding to syscountervalid event. */
    void *                              syscountervalid_context;                               /**< User context for syscountervalid event handler. */
#endif
} nrfx_grtc_cb_t;

static nrfx_grtc_cb_t m_cb =
{
    // At the initialization only channels assigned by configuration are available.
    .available_channels = (nrfx_atomic_t)NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK
};

static uint8_t num_of_channels_get(uint32_t mask)
{
    uint8_t ch_count = 0;

    while (mask)
    {
        // Calculating number of channels by counting ones inside given mask.
        ch_count += mask & 0x1;
        mask >>= 1;
    }
    return ch_count;
}

static uint32_t allocated_channels_mask_get(void)
{
    return NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK & ~m_cb.available_channels;
}

static uint32_t used_channels_mask_get(void)
{
    return NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK & m_cb.used_channels;
}

#if NRFY_GRTC_HAS_EXTENDED
static bool is_syscounter_running(void)
{
    return nrfy_grtc_sys_counter_check(NRF_GRTC);
}
#endif

static bool is_channel_used(uint8_t channel)
{
    return (GRTC_CHANNEL_TO_BITMASK(channel) & used_channels_mask_get());
}

static bool is_channel_allocated(uint8_t channel)
{
    return (GRTC_CHANNEL_TO_BITMASK(channel) & allocated_channels_mask_get());
}

static void channel_used_mark(uint8_t channel)
{
    m_cb.used_channels |= GRTC_CHANNEL_TO_BITMASK(channel);
}

static void channel_used_unmark(uint8_t channel)
{
    m_cb.used_channels &= ~GRTC_CHANNEL_TO_BITMASK(channel);
}

static bool is_channel_available(uint8_t channel)
{
    return (GRTC_CHANNEL_TO_BITMASK(channel) & NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK);
}

static nrfx_err_t syscounter_check(uint8_t channel)
{
    if (!is_channel_available(channel))
    {
        return NRFX_ERROR_FORBIDDEN;
    }
    if (!is_channel_allocated(channel))
    {
        return NRFX_ERROR_INVALID_PARAM;
    }
    return NRFX_SUCCESS;
}

static uint8_t get_channel_for_ch_data_idx(uint8_t idx)
{
    uint32_t ch_mask = NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK;

    for (uint8_t i = 0; i < idx; i++)
    {
        ch_mask &= ~(1UL << NRF_CTZ(ch_mask));
    }
    return (uint8_t)NRF_CTZ(ch_mask);
}

static void cc_channel_prepare(nrfx_grtc_channel_t * p_chan_data)
{
    NRFX_ASSERT(p_chan_data);
    uint8_t ch_data_idx = m_cb.ch_to_data[p_chan_data->channel];

    nrfy_grtc_sys_counter_compare_event_disable(NRF_GRTC, p_chan_data->channel);

    m_cb.channel_data[ch_data_idx].handler   = p_chan_data->handler;
    m_cb.channel_data[ch_data_idx].p_context = p_chan_data->p_context;
    m_cb.channel_data[ch_data_idx].channel   = p_chan_data->channel;
    channel_used_mark(p_chan_data->channel);
}

#if NRFY_GRTC_HAS_EXTENDED
static void sleep_configure(nrfx_grtc_sleep_config_t const * p_sleep_cfg)
{
    nrfy_grtc_sys_counter_auto_mode_set(NRF_GRTC, p_sleep_cfg->auto_mode);
    nrfy_grtc_timeout_set(NRF_GRTC, p_sleep_cfg->timeout);
    nrfy_grtc_waketime_set(NRF_GRTC, p_sleep_cfg->waketime);
}

static void sleep_configuration_get(nrfx_grtc_sleep_config_t * p_sleep_cfg)
{
    p_sleep_cfg->auto_mode = nrfy_grtc_sys_counter_auto_mode_check(NRF_GRTC);
    p_sleep_cfg->timeout = nrfy_grtc_timeout_get(NRF_GRTC);
    p_sleep_cfg->waketime = nrfy_grtc_waketime_get(NRF_GRTC);
}
#endif /* NRFY_GRTC_HAS_EXTENDED */

static inline bool active_check(void)
{
#if NRFY_GRTC_HAS_SYSCOUNTER_ARRAY
    return nrfy_grtc_sys_counter_active_check(NRF_GRTC);
#else
    return nrfy_grtc_sys_counter_active_state_request_check(NRF_GRTC);
#endif
}

static inline void active_set(bool active)
{
#if defined(NRF_GRTC_HAS_SYSCOUNTER_ARRAY) && (NRF_GRTC_HAS_SYSCOUNTER_ARRAY == 1)
    nrfy_grtc_sys_counter_active_set(NRF_GRTC, active);
#else
    nrfy_grtc_sys_counter_active_state_request_set(NRF_GRTC, active);
#endif
}

static inline bool ready_check(void)
{
    return nrfy_grtc_sys_counter_ready_check(NRF_GRTC);
}

bool nrfx_grtc_active_request_check(void)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    return active_check();
}

void nrfx_grtc_active_request_set(bool active)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    active_set(active);
}

bool nrfx_grtc_ready_check(void)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    return ready_check();
}

nrfx_err_t nrfx_grtc_syscounter_get(uint64_t * p_counter)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_counter);

    NRFX_CRITICAL_SECTION_ENTER();
    *p_counter = nrfy_grtc_sys_counter_get(NRF_GRTC);
    NRFX_CRITICAL_SECTION_EXIT();

    return NRFX_SUCCESS;
}

void nrfx_grtc_channel_callback_set(uint8_t                channel,
                                    nrfx_grtc_cc_handler_t handler,
                                    void *                 p_context)
{
    uint8_t ch_data_idx = m_cb.ch_to_data[channel];

    m_cb.channel_data[ch_data_idx].handler = handler;
    m_cb.channel_data[ch_data_idx].p_context = p_context;
    m_cb.channel_data[ch_data_idx].channel = channel;
    nrfy_grtc_int_enable(NRF_GRTC, GRTC_CHANNEL_TO_BITMASK(channel));
}

nrfx_err_t nrfx_grtc_channel_alloc(uint8_t * p_channel)
{
    NRFX_ASSERT(p_channel);
    nrfx_err_t err_code = nrfx_flag32_alloc(&m_cb.available_channels, p_channel);

    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
    }

    NRFX_LOG_INFO("GRTC channel %u allocated.", *p_channel);
    return err_code;
}

nrfx_err_t nrfx_grtc_channel_free(uint8_t channel)
{
    NRFX_ASSERT(channel < NRF_GRTC_SYSCOUNTER_CC_COUNT);
    nrfx_err_t err_code;

    channel_used_unmark(channel);
    if (!is_channel_available(channel))
    {
        err_code = NRFX_ERROR_FORBIDDEN;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    err_code = nrfx_flag32_free(&m_cb.available_channels, channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    NRFX_LOG_INFO("GRTC channel %u freed.", channel);
    return err_code;
}

bool nrfx_grtc_is_channel_used(uint8_t channel)
{
    return is_channel_used(channel);
}

nrfx_err_t nrfx_grtc_init(uint8_t interrupt_priority)
{
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED)
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

#if NRFY_GRTC_HAS_EXTENDED && NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOSTART)
    nrfx_grtc_sleep_config_t sleep_cfg = NRFX_GRTC_SLEEP_DEFAULT_CONFIG;
#if !NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOEN)
    sleep_cfg.auto_mode = false;
#endif

    nrfy_grtc_sys_counter_set(NRF_GRTC, false);
    sleep_configure(&sleep_cfg);
#endif

    if ((num_of_channels_get(NRFX_GRTC_CONFIG_ALLOWED_CC_CHANNELS_MASK) !=
         NRFX_GRTC_CONFIG_NUM_OF_CC_CHANNELS) || (NRFX_GRTC_CONFIG_NUM_OF_CC_CHANNELS == 0))
    {
        err_code = NRFX_ERROR_INTERNAL;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    for (uint8_t i = 0; i < NRFX_GRTC_CONFIG_NUM_OF_CC_CHANNELS; i++)
    {
        uint8_t ch = get_channel_for_ch_data_idx(i);

        m_cb.channel_data[i].channel = ch;
        m_cb.ch_to_data[ch] = i;
    }

    nrfy_grtc_int_init(NRF_GRTC, GRTC_ALL_INT_MASK, interrupt_priority, false);

#if NRFY_GRTC_HAS_EXTENDED && NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOSTART)
    nrfy_grtc_prepare(NRF_GRTC, true);

#endif /* NRFY_GRTC_HAS_EXTENDED && NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOSTART) */

    m_cb.state = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("GRTC initialized.");
    return err_code;
}

#if NRFY_GRTC_HAS_EXTENDED
nrfx_err_t nrfx_grtc_sleep_configure(nrfx_grtc_sleep_config_t const * p_sleep_cfg)
{
    NRFX_ASSERT(p_sleep_cfg);
    bool is_active;

    is_active = nrfy_grtc_sys_counter_check(NRF_GRTC);
    if (is_active)
    {
        nrfy_grtc_sys_counter_set(NRF_GRTC, false);
    }
    sleep_configure(p_sleep_cfg);
    if (is_active)
    {
        nrfy_grtc_sys_counter_set(NRF_GRTC, true);
    }
    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_grtc_sleep_configuration_get(nrfx_grtc_sleep_config_t * p_sleep_cfg)
{
    NRFX_ASSERT(p_sleep_cfg);
    sleep_configuration_get(p_sleep_cfg);
    return NRFX_SUCCESS;
}
#endif // NRFY_GRTC_HAS_EXTENDED

#if NRF_GRTC_HAS_RTCOUNTER
nrfx_err_t nrfx_grtc_rtcounter_cc_disable(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code = NRFX_SUCCESS;
    uint32_t   int_mask = NRF_GRTC_INT_RTCOMPARE_MASK | NRF_GRTC_INT_RTCOMPARESYNC_MASK;

    if (is_syscounter_running())
    {
        err_code = NRFX_ERROR_INTERNAL;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (nrfy_grtc_int_enable_check(NRF_GRTC, int_mask))
    {
        nrfy_grtc_int_disable(NRF_GRTC, int_mask);
        if (nrfy_grtc_event_check(NRF_GRTC, NRF_GRTC_EVENT_RTCOMPARE) ||
            nrfy_grtc_event_check(NRF_GRTC, NRF_GRTC_EVENT_RTCOMPARESYNC))
        {
            nrfy_grtc_event_clear(NRF_GRTC, NRF_GRTC_EVENT_RTCOMPARE);
            nrfy_grtc_event_clear(NRF_GRTC, NRF_GRTC_EVENT_RTCOMPARESYNC);
            err_code = NRFX_ERROR_TIMEOUT;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }

    NRFX_LOG_INFO("GRTC RTCOUNTER compare disabled.");
    return err_code;
}

void nrfx_grtc_rtcomparesync_int_enable(nrfx_grtc_rtcomparesync_handler_t handler, void * p_context)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    m_cb.rtcomparesync_handler = handler;
    m_cb.rtcomparesync_context = p_context;
    nrfy_grtc_event_clear(NRF_GRTC, NRF_GRTC_EVENT_RTCOMPARESYNC);
    nrfy_grtc_int_enable(NRF_GRTC, NRFY_EVENT_TO_INT_BITMASK(NRF_GRTC_EVENT_RTCOMPARESYNC));
    NRFX_LOG_INFO("GRTC RTCOMPARESYNC interrupt enabled.");
}

void nrfx_grtc_rtcomparesync_int_disable(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_grtc_int_disable(NRF_GRTC, NRF_GRTC_INT_RTCOMPARESYNC_MASK);
    NRFX_LOG_INFO("GRTC RTCOMPARESYNC interrupt disabled.");
}

nrfx_err_t nrfx_grtc_rtcounter_cc_absolute_set(nrfx_grtc_rtcounter_handler_data_t * p_handler_data,
                                               uint64_t                             val,
                                               bool                                 enable_irq,
                                               bool                                 sync)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_handler_data);
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (is_syscounter_running())
    {
        err_code = NRFX_ERROR_INTERNAL;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    nrfx_grtc_channel_t * p_chan_data = &m_cb.channel_data[GRTC_RTCOUNTER_CC_HANDLER_IDX];

    p_chan_data->handler   = p_handler_data->handler;
    p_chan_data->p_context = p_handler_data->p_context;
    p_chan_data->channel   = GRTC_RTCOUNTER_COMPARE_CHANNEL;

    NRFX_CRITICAL_SECTION_ENTER();
    nrfy_grtc_rt_counter_cc_set(NRF_GRTC, val, sync);
    NRFX_CRITICAL_SECTION_EXIT();

    nrf_grtc_event_t event = NRF_GRTC_EVENT_RTCOMPARE;

    nrfy_grtc_event_clear(NRF_GRTC, event);
    if (enable_irq)
    {
        nrfy_grtc_int_enable(NRF_GRTC, NRFY_EVENT_TO_INT_BITMASK(event));
    }

    NRFX_LOG_INFO("GRTC RTCOUNTER compare set to %llu.", val);
    return err_code;
}
#endif // NRF_GRTC_HAS_RTCOUNTER

#if NRFY_GRTC_HAS_EXTENDED
nrfx_err_t nrfx_grtc_syscounter_start(bool busy_wait, uint8_t * p_main_cc_channel)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_main_cc_channel);
    NRFX_ASSERT(m_cb.channel_data[0].channel == MAIN_GRTC_CC_CHANNEL);
    nrfx_err_t    err_code  = NRFX_SUCCESS;
    nrfx_atomic_t init_mask = GRTC_CHANNEL_TO_BITMASK(MAIN_GRTC_CC_CHANNEL) &
                              m_cb.available_channels;

    err_code = nrfx_flag32_alloc(&init_mask, &m_cb.channel_data[0].channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    *p_main_cc_channel       = MAIN_GRTC_CC_CHANNEL;
    m_cb.available_channels &= ~GRTC_CHANNEL_TO_BITMASK(MAIN_GRTC_CC_CHANNEL);
    channel_used_mark(MAIN_GRTC_CC_CHANNEL);
    NRFX_LOG_INFO("GRTC channel %u allocated.", m_cb.channel_data[0].channel);

    if (is_syscounter_running())
    {
        err_code = NRFX_ERROR_ALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    nrfy_grtc_sys_counter_start(NRF_GRTC, busy_wait);
#if NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOEN)
    uint32_t startup_timeout = STARTUP_TIMEOUT;

    while ((startup_timeout > 0) && (!ready_check()))
    {
        startup_timeout--;
    }
    if (startup_timeout == 0)
    {
        return NRFX_ERROR_TIMEOUT;
    }
#endif /* NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOEN) */
    NRFX_LOG_INFO("GRTC SYSCOUNTER started.");
    return err_code;
}

nrfx_err_t nrfx_grtc_action_perform(nrfx_grtc_action_t action)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (is_syscounter_running())
    {
        err_code = NRFX_ERROR_INTERNAL;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));

        return err_code;
    }

    nrf_grtc_task_t task = (nrf_grtc_task_t)action;
    nrfy_grtc_task_trigger(NRF_GRTC, task);

    NRFX_LOG_INFO("GRTC %s action.", GRTC_ACTION_TO_STR(action));
    return err_code;
}
#endif // NRFY_GRTC_HAS_EXTENDED

void nrfx_grtc_uninit(void)
{
    uint32_t ch_mask = allocated_channels_mask_get();

    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_grtc_int_disable(NRF_GRTC, GRTC_ALL_INT_MASK);

    for (uint8_t chan = 0; ch_mask; chan++, ch_mask >>= 1)
    {
        if (is_channel_used(chan))
        {
            channel_used_unmark(chan);
            if (is_channel_allocated(chan))
            {
                nrfy_grtc_sys_counter_compare_event_disable(NRF_GRTC, chan);
                if (ch_mask & 0x1)
                {
                    (void)nrfx_flag32_free(&m_cb.available_channels, chan);
                }
            }
        }
    }
    nrfy_grtc_int_uninit(NRF_GRTC);

#if NRFY_GRTC_HAS_SYSCOUNTER_ARRAY
    nrfy_grtc_sys_counter_active_set(NRF_GRTC, false);
#else
    nrfy_grtc_sys_counter_active_state_request_set(NRF_GRTC, false);
#endif

#if NRFY_GRTC_HAS_EXTENDED && NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOSTART)
    nrfy_grtc_sys_counter_auto_mode_set(NRF_GRTC, false);
    nrfy_grtc_sys_counter_set(NRF_GRTC, false);
    nrf_grtc_task_trigger(NRF_GRTC, NRF_GRTC_TASK_STOP);
    nrf_grtc_task_trigger(NRF_GRTC, NRF_GRTC_TASK_CLEAR);
#endif // NRFY_GRTC_HAS_EXTENDED && NRFX_IS_ENABLED(NRFX_GRTC_CONFIG_AUTOSTART)

    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("GRTC uninitialized.");
}

bool nrfx_grtc_init_check(void)
{
    return (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

#if NRF_GRTC_HAS_RTCOUNTER
void nrfx_grtc_rtcounter_cc_int_enable(bool sync)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    nrf_grtc_event_t event = sync ? NRF_GRTC_EVENT_RTCOMPARE : NRF_GRTC_EVENT_RTCOMPARESYNC;

    nrfy_grtc_event_clear(NRF_GRTC, event);
    nrfy_grtc_int_enable(NRF_GRTC, NRFY_EVENT_TO_INT_BITMASK(event));
    NRFX_LOG_INFO("GRTC RTCOMPARE%s interrupt enabled.", sync ? "SYNC" : "");
}

void nrfx_grtc_rtcounter_cc_int_disable(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_grtc_int_disable(NRF_GRTC, NRF_GRTC_INT_RTCOMPARE_MASK | NRF_GRTC_INT_RTCOMPARESYNC_MASK);
    NRFX_LOG_INFO("GRTC RTCOMPARE/RTCOMPARESYNC interrupt disabled.");
}
#endif // NRF_GRTC_HAS_RTCOUNTER

#if NRFY_GRTC_HAS_EXTENDED && NRFY_GRTC_HAS_SYSCOUNTERVALID
void nrfx_grtc_syscountervalid_int_enable(nrfx_grtc_syscountervalid_handler_t handler,
                                          void *                              p_context)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    m_cb.syscountervalid_handler = handler;
    m_cb.syscountervalid_context = p_context;
    nrfy_grtc_int_enable(NRF_GRTC, NRFY_EVENT_TO_INT_BITMASK(NRF_GRTC_EVENT_SYSCOUNTERVALID));
    NRFX_LOG_INFO("GRTC SYSCOUNTERVALID interrupt enabled.");
}

void nrfx_grtc_syscountervalid_int_disable(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_grtc_int_disable(NRF_GRTC, NRF_GRTC_INT_SYSCOUNTERVALID_MASK);
    NRFX_LOG_INFO("GRTC SYSCOUNTERVALID interrupt disabled.");
}
#endif // NRFY_GRTC_HAS_EXTENDED && NRFY_GRTC_HAS_SYSCOUNTERVALID

nrfx_err_t nrfx_grtc_syscounter_cc_disable(uint8_t channel)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    uint32_t   int_mask = NRF_GRTC_CHANNEL_INT_MASK(channel);
    nrfx_err_t err_code = syscounter_check(channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    if (!is_channel_used(channel))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    channel_used_unmark(channel);

    nrfy_grtc_sys_counter_compare_event_disable(NRF_GRTC, channel);

    if (nrfy_grtc_int_enable_check(NRF_GRTC, int_mask))
    {
        nrfy_grtc_int_disable(NRF_GRTC, int_mask);
        if (nrfy_grtc_sys_counter_compare_event_check(NRF_GRTC, channel))
        {
            nrfy_grtc_sys_counter_compare_event_clear(NRF_GRTC, channel);
            err_code = NRFX_ERROR_TIMEOUT;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }
    NRFX_LOG_INFO("GRTC SYSCOUNTER compare for channel %u disabled.", (uint32_t)channel);
    return err_code;
}

void nrfx_grtc_syscounter_cc_abs_set(uint8_t channel, uint64_t val, bool safe_setting)
{
    NRFX_ASSERT(syscounter_check(channel) == NRFX_SUCCESS);

    m_cb.cc_value[m_cb.ch_to_data[channel]] = val;
    if (safe_setting)
    {
        nrfy_grtc_sys_counter_cc_set(NRF_GRTC, channel, val);
        if (nrfy_grtc_sys_counter_compare_event_check(NRF_GRTC, channel))
        {
            uint64_t now;

            nrfx_grtc_syscounter_get(&now);
            if (val > now)
            {
                nrfy_grtc_sys_counter_compare_event_clear(NRF_GRTC, channel);
            }
        }
    }
    else
    {
        nrfy_grtc_sys_counter_cc_set(NRF_GRTC, channel, val);
    }
}

nrfx_err_t nrfx_grtc_syscounter_cc_absolute_set(nrfx_grtc_channel_t * p_chan_data,
                                                uint64_t              val,
                                                bool                  enable_irq)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_chan_data);
    nrfx_err_t err_code = syscounter_check(p_chan_data->channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    cc_channel_prepare(p_chan_data);
    NRFX_CRITICAL_SECTION_ENTER();
    nrfy_grtc_sys_counter_compare_event_clear(NRF_GRTC, p_chan_data->channel);
    nrfy_grtc_sys_counter_cc_set(NRF_GRTC, p_chan_data->channel, val);
    NRFX_CRITICAL_SECTION_EXIT();

    if (enable_irq)
    {
        NRFX_ATOMIC_FETCH_OR(&m_cb.read_cc_mask, NRFX_BIT(p_chan_data->channel));
        nrfy_grtc_int_enable(NRF_GRTC, GRTC_CHANNEL_TO_BITMASK(p_chan_data->channel));
    }

    NRFX_LOG_INFO("GRTC SYSCOUNTER absolute compare for channel %u set to %u.",
                  (uint32_t)p_chan_data->channel,
                  (uint32_t)nrfy_grtc_sys_counter_cc_get(NRF_GRTC, p_chan_data->channel));
    return err_code;
}

void nrfx_grtc_syscounter_cc_rel_set(uint8_t channel,
                                     uint32_t val,
                                     nrfx_grtc_cc_relative_reference_t reference)
{
    NRFX_ASSERT(syscounter_check(channel) == NRFX_SUCCESS);

    m_cb.cc_value[m_cb.ch_to_data[channel]] += val;
    nrfy_grtc_sys_counter_cc_add_set(NRF_GRTC,
                                     channel,
                                     val,
                                     (nrf_grtc_cc_add_reference_t)reference);
}

nrfx_err_t nrfx_grtc_syscounter_cc_relative_set(nrfx_grtc_channel_t *             p_chan_data,
                                                uint32_t                          val,
                                                bool                              enable_irq,
                                                nrfx_grtc_cc_relative_reference_t reference)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_chan_data);
    nrfx_err_t err_code = syscounter_check(p_chan_data->channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    cc_channel_prepare(p_chan_data);
    NRFX_CRITICAL_SECTION_ENTER();
    nrfy_grtc_sys_counter_compare_event_clear(NRF_GRTC, p_chan_data->channel);
    {
        nrfy_grtc_sys_counter_cc_add_set(NRF_GRTC,
                                         p_chan_data->channel,
                                         val,
                                         (nrf_grtc_cc_add_reference_t)reference);
    }
    NRFX_CRITICAL_SECTION_EXIT();

    if (enable_irq)
    {
        NRFX_ATOMIC_FETCH_OR(&m_cb.read_cc_mask, NRFX_BIT(p_chan_data->channel));
        nrfy_grtc_int_enable(NRF_GRTC, GRTC_CHANNEL_TO_BITMASK(p_chan_data->channel));
    }

    NRFX_LOG_INFO("GRTC SYSCOUNTER compare for channel %u set to %u.",
                  (uint32_t)p_chan_data->channel,
                  (uint32_t)val);
    return err_code;
}

nrfx_err_t nrfx_grtc_syscounter_cc_int_disable(uint8_t channel)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code = syscounter_check(channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    if (!is_channel_used(channel))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    nrfy_grtc_int_disable(NRF_GRTC, NRF_GRTC_CHANNEL_INT_MASK(channel));
    NRFX_LOG_INFO("GRTC SYSCOUNTER compare interrupt for channel %u disabled.", (uint32_t)channel);
    return err_code;
}

nrfx_err_t nrfx_grtc_syscounter_cc_int_enable(uint8_t channel)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code = syscounter_check(channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    channel_used_mark(channel);
    nrfy_grtc_int_enable(NRF_GRTC, GRTC_CHANNEL_TO_BITMASK(channel));
    NRFX_LOG_INFO("GRTC SYSCOUNTER compare interrupt for channel %u enabled.", (uint32_t)channel);
    return err_code;
}

bool nrfx_grtc_syscounter_cc_int_enable_check(uint8_t channel)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(channel < NRF_GRTC_SYSCOUNTER_CC_COUNT);
    return nrfy_grtc_int_enable_check(NRF_GRTC, GRTC_CHANNEL_TO_BITMASK(channel));
}

nrfx_err_t nrfx_grtc_syscounter_capture(uint8_t channel)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code = syscounter_check(channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    channel_used_mark(channel);
    nrfy_grtc_task_trigger(NRF_GRTC, nrfy_grtc_sys_counter_capture_task_get(channel));

    NRFX_LOG_INFO("GRTC SYSCOUNTER capture for channel %u triggered.", (uint32_t)channel);
    return err_code;
}

nrfx_err_t nrfx_grtc_syscounter_cc_value_read(uint8_t channel, uint64_t * p_val)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_val);
    nrfx_err_t err_code = syscounter_check(channel);
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    if (!is_channel_used(channel))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    *p_val = nrfy_grtc_sys_counter_cc_get(NRF_GRTC, channel);

    NRFX_LOG_INFO("GRTC SYSCOUNTER capture for channel %u read: %llu.", (uint32_t)channel, *p_val);
    return err_code;
}

#if NRF_GRTC_HAS_RTCOUNTER || (NRFY_GRTC_HAS_EXTENDED && NRFY_GRTC_HAS_SYSCOUNTERVALID)
#define GRTC_EXT 1
#endif

static void grtc_irq_handler(void)
{
    uint32_t intpend = nrfy_grtc_int_pending_get(NRF_GRTC);

    while (intpend)
    {
        uint8_t idx = (uint8_t)NRFX_CTZ(intpend);

        intpend &= ~NRFX_BIT(idx);

        if (!NRFX_IS_ENABLED(GRTC_EXT) || idx < NRF_GRTC_SYSCOUNTER_CC_COUNT)
        {
            uint32_t i = m_cb.ch_to_data[idx];

            NRFX_ASSERT(m_cb.channel_data[i].channel == idx);

            if (m_cb.channel_data[i].handler)
            {
                uint64_t cc_value;

                if (NRFX_ATOMIC_FETCH_AND(&m_cb.read_cc_mask, ~NRFX_BIT(idx)) & NRFX_BIT(idx))
                {
                    /* Read CC value only if channel was set using legacy functions. It is done
                     * for API backward compatibility. Reading 64 bit value from GRTC is costly
                     * and it is avoided if possible.
                     */
                    cc_value = nrfy_grtc_sys_counter_cc_get(NRF_GRTC, idx);
                }
                else
                {
                    /* If CC was set using optimized API then CC is stored in RAM (much faster
                     * access).
                     */
                    cc_value = m_cb.cc_value[i];
                }

                /* Check event again (initially checked via INTPEND). It is possible that
                 * CC is reconfigured from higher priority context. In that case event
                 * might be cleared.
                 */
                if (!nrf_grtc_event_check(NRF_GRTC, NRFY_INT_BITPOS_TO_EVENT(idx)))
                {
                    continue;
                }

                nrf_grtc_event_clear(NRF_GRTC, NRFY_INT_BITPOS_TO_EVENT(idx));

                m_cb.channel_data[i].handler(idx, cc_value, m_cb.channel_data[i].p_context);
            }
            else if (nrf_grtc_event_check(NRF_GRTC, NRFY_INT_BITPOS_TO_EVENT(idx)))
            {
                nrf_grtc_event_clear(NRF_GRTC, NRFY_INT_BITPOS_TO_EVENT(idx));
            }
        }
#if NRF_GRTC_HAS_RTCOUNTER
        if (idx == NRFY_EVENT_TO_INT_BITPOS(NRF_GRTC_EVENT_RTCOMPARE))
        {
            nrf_grtc_event_clear(NRF_GRTC, NRFY_INT_BITPOS_TO_EVENT(idx));
            nrfx_grtc_channel_t const * p_channel =
                                &m_cb.channel_data[GRTC_RTCOUNTER_CC_HANDLER_IDX];
            if (p_channel->handler)
            {
                p_channel->handler((int32_t)GRTC_RTCOUNTER_COMPARE_CHANNEL,
                                   nrfy_grtc_rt_counter_cc_get(NRF_GRTC),
                                   p_channel->p_context);
            }
         }
#endif // NRF_GRTC_HAS_RTCOUNTER
#if NRFY_GRTC_HAS_EXTENDED && NRFY_GRTC_HAS_SYSCOUNTERVALID
        if (idx == NRFY_EVENT_TO_INT_BITPOS(NRF_GRTC_EVENT_SYSCOUNTERVALID))
        {
            /* The SYSCOUNTERVALID bit is automatically cleared when GRTC goes into sleep state
             * and set when returning from this state. It can't be cleared inside the ISR
             * procedure because we rely on it during SYSCOUNTER value reading procedure. */
            NRFX_LOG_INFO("Event: NRF_GRTC_EVENT_SYSCOUNTERVALID.");
            nrf_grtc_event_clear(NRF_GRTC, NRFY_INT_BITPOS_TO_EVENT(idx));
            if (m_cb.syscountervalid_handler)
            {
                m_cb.syscountervalid_handler(m_cb.syscountervalid_context);
            }
        }
#endif // NRFY_GRTC_HAS_EXTENDED && NRFY_GRTC_HAS_SYSCOUNTERVALID
    }
}

void nrfx_grtc_irq_handler(void)
{
    grtc_irq_handler();
}

#endif // NRFX_CHECK(NRFX_GRTC_ENABLED)
