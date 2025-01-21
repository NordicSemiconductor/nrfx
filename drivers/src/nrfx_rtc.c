/*
 * Copyright (c) 2014 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_RTC_ENABLED)

#if !NRFX_FEATURE_PRESENT(NRFX_RTC, _ENABLED)
#error "No enabled RTC instances. Check <nrfx_config.h>."
#endif

#include <nrfx_rtc.h>

#define NRFX_LOG_MODULE RTC
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                           \
    (event == NRF_RTC_EVENT_TICK      ? "NRF_RTC_EVENT_TICK"      : \
    (event == NRF_RTC_EVENT_OVERFLOW  ? "NRF_RTC_EVENT_OVERFLOW"  : \
    (event == NRF_RTC_EVENT_COMPARE_0 ? "NRF_RTC_EVENT_COMPARE_0" : \
    (event == NRF_RTC_EVENT_COMPARE_1 ? "NRF_RTC_EVENT_COMPARE_1" : \
    (event == NRF_RTC_EVENT_COMPARE_2 ? "NRF_RTC_EVENT_COMPARE_2" : \
    (event == NRF_RTC_EVENT_COMPARE_3 ? "NRF_RTC_EVENT_COMPARE_3" : \
                                        "UNKNOWN EVENT"))))))

/** @brief RTC driver instance control block structure. */
typedef struct
{
    nrfx_rtc_handler_t handler;      /**< Instance event handler. */
    nrfx_drv_state_t   state;        /**< Instance state. */
    bool               reliable;     /**< Reliable mode flag. */
    uint8_t            tick_latency; /**< Maximum length of interrupt handler in ticks (max 7.7 ms). */
} nrfx_rtc_cb_t;

// User callbacks local storage.
static nrfx_rtc_cb_t      m_cb[NRFX_RTC_ENABLED_COUNT];

nrfx_err_t nrfx_rtc_init(nrfx_rtc_t const *        p_instance,
                         nrfx_rtc_config_t const * p_config,
                         nrfx_rtc_handler_t        handler)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(handler);
    nrfx_err_t err_code;

    m_cb[p_instance->instance_id].handler = handler;

    if (m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED)
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

    nrfy_rtc_int_init(p_instance->p_reg, 0, p_config->interrupt_priority, false);

    nrfy_rtc_config_t config =
    {
        .prescaler = p_config->prescaler
    };

    nrfy_rtc_periph_configure(p_instance->p_reg, &config);
    m_cb[p_instance->instance_id].reliable     = p_config->reliable;
    m_cb[p_instance->instance_id].tick_latency = p_config->tick_latency;
    m_cb[p_instance->instance_id].state        = NRFX_DRV_STATE_INITIALIZED;

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

void nrfx_rtc_uninit(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);

    uint32_t mask = NRF_RTC_INT_TICK_MASK     |
                    NRF_RTC_INT_OVERFLOW_MASK |
                    NRF_RTC_ALL_CHANNELS_INT_MASK;

    nrfy_rtc_int_uninit(p_instance->p_reg);
    nrfy_rtc_stop(p_instance->p_reg, mask);

    m_cb[p_instance->instance_id].state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_rtc_init_check(nrfx_rtc_t const * p_instance)
{
    return (m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_rtc_enable(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_rtc_task_trigger(p_instance->p_reg, NRF_RTC_TASK_START);
    m_cb[p_instance->instance_id].state = NRFX_DRV_STATE_POWERED_ON;
    NRFX_LOG_INFO("Enabled.");
}

void nrfx_rtc_disable(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_task_trigger(p_instance->p_reg, NRF_RTC_TASK_STOP);
    m_cb[p_instance->instance_id].state = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("Disabled.");
}

nrfx_err_t nrfx_rtc_cc_disable(nrfx_rtc_t const * p_instance, uint32_t channel)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(channel < p_instance->cc_channel_count);

    nrfx_err_t err_code;
    uint32_t int_mask = NRF_RTC_CHANNEL_INT_MASK(channel);
    nrf_rtc_event_t event = NRF_RTC_CHANNEL_EVENT_ADDR(channel);

    nrfy_rtc_event_disable(p_instance->p_reg, int_mask);
    if (nrfy_rtc_int_enable_check(p_instance->p_reg, int_mask))
    {
        nrfy_rtc_int_disable(p_instance->p_reg, int_mask);
        if (nrfy_rtc_event_check(p_instance->p_reg, event))
        {
            nrfy_rtc_event_clear(p_instance->p_reg, event);
            err_code = NRFX_ERROR_TIMEOUT;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }
    NRFX_LOG_INFO("RTC id: %d, channel disabled: %lu.",
                  p_instance->instance_id,
                  (unsigned long)channel);
    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_rtc_cc_set(nrfx_rtc_t const * p_instance,
                           uint32_t           channel,
                           uint32_t           val,
                           bool               enable_irq)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(channel < p_instance->cc_channel_count);

    nrfx_err_t err_code;
    uint32_t int_mask = NRF_RTC_CHANNEL_INT_MASK(channel);
    nrf_rtc_event_t event = NRF_RTC_CHANNEL_EVENT_ADDR(channel);

    nrfy_rtc_event_int_disable(p_instance->p_reg, int_mask);

    val = NRF_RTC_WRAP(val);
    if (m_cb[p_instance->instance_id].reliable)
    {
        nrfy_rtc_cc_set(p_instance->p_reg, channel, val);
        uint32_t cnt = nrfy_rtc_counter_get(p_instance->p_reg);
        int32_t diff = (int32_t)(cnt - val);
        if (cnt < val)
        {
            diff += (int32_t)NRF_RTC_COUNTER_MAX;
        }
        if (diff < m_cb[p_instance->instance_id].tick_latency)
        {
            err_code = NRFX_ERROR_TIMEOUT;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }
    else
    {
        nrfy_rtc_cc_set(p_instance->p_reg, channel, val);
    }

    nrfy_rtc_event_int_clear_enable(p_instance->p_reg, event, enable_irq);

    NRFX_LOG_INFO("RTC id: %d, channel enabled: %lu, compare value: %lu.",
                  p_instance->instance_id,
                  (unsigned long)channel,
                  (unsigned long)val);
    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

void nrfx_rtc_tick_enable(nrfx_rtc_t const * p_instance, bool enable_irq)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_clear_enable(p_instance->p_reg,
                                    NRF_RTC_EVENT_TICK,
                                    enable_irq);
    NRFX_LOG_INFO("Tick events enabled.");
}

void nrfx_rtc_tick_disable(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_disable(p_instance->p_reg,
                               NRF_RTC_INT_TICK_MASK);
    NRFX_LOG_INFO("Tick events disabled.");
}

void nrfx_rtc_overflow_enable(nrfx_rtc_t const * p_instance, bool enable_irq)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_clear_enable(p_instance->p_reg,
                                    NRF_RTC_EVENT_OVERFLOW,
                                    enable_irq);
}

void nrfx_rtc_overflow_disable(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_disable(p_instance->p_reg,
                               NRF_RTC_INT_OVERFLOW_MASK);
}

uint32_t nrfx_rtc_max_ticks_get(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(m_cb[p_instance->instance_id].state != NRFX_DRV_STATE_UNINITIALIZED);

    uint32_t ticks;
    if (m_cb[p_instance->instance_id].reliable)
    {
        ticks = NRF_RTC_COUNTER_MAX - m_cb[p_instance->instance_id].tick_latency;
    }
    else
    {
        ticks = NRF_RTC_COUNTER_MAX;
    }
    return ticks;
}

static void irq_handler(NRF_RTC_Type  * p_reg,
                        nrfx_rtc_cb_t * p_cb,
                        uint32_t        channel_count)
{
    uint32_t evt_to_process = NRFY_EVENT_TO_INT_BITMASK(NRF_RTC_EVENT_TICK)     |
                              NRFY_EVENT_TO_INT_BITMASK(NRF_RTC_EVENT_OVERFLOW) |
                              NRF_RTC_ALL_CHANNELS_INT_MASK;

    uint32_t event_mask = nrfy_rtc_events_process(p_reg, evt_to_process);

    uint32_t active_cc_mask = nrfy_rtc_int_enable_check(p_reg, NRF_RTC_ALL_CHANNELS_INT_MASK);

    for (uint8_t i = 0; i < channel_count; i++)
    {
        nrf_rtc_event_t event = nrf_rtc_compare_event_get(i);
        if ((active_cc_mask & NRFY_EVENT_TO_INT_BITMASK(event)) &&
            (event_mask & NRFY_EVENT_TO_INT_BITMASK(event)))
        {
            nrfy_rtc_event_disable(p_reg, NRFY_EVENT_TO_INT_BITMASK(event));
            NRFX_LOG_DEBUG("Event: %s, reg: %p.", EVT_TO_STR(event), p_reg);
            p_cb->handler((nrfx_rtc_int_type_t)i);
        }
    }

    if (event_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_RTC_EVENT_TICK))
    {
        NRFX_LOG_DEBUG("Event: %s, reg: %p.", EVT_TO_STR(NRF_RTC_EVENT_TICK), p_reg);
        p_cb->handler(NRFX_RTC_INT_TICK);
    }

    if (event_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_RTC_EVENT_OVERFLOW))
    {
        NRFX_LOG_DEBUG("Event: %s, reg: %p.", EVT_TO_STR(NRF_RTC_EVENT_OVERFLOW), p_reg);
        p_cb->handler(NRFX_RTC_INT_OVERFLOW);
    }
}

NRFX_INSTANCE_IRQ_HANDLERS_EXT(RTC, rtc, NRF_RTC_CC_CHANNEL_COUNT)

#endif // NRFX_CHECK(NRFX_RTC_ENABLED)
