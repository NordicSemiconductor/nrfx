/*
 * Copyright (c) 2014 - 2026, Nordic Semiconductor ASA
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

#include <nrfx_rtc.h>

#if !NRFX_API_VER_AT_LEAST(4, 1, 0)
#include "nrfx_rtc_legacy.c"
#else

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

int nrfx_rtc_init(nrfx_rtc_t *              p_instance,
                  nrfx_rtc_config_t const * p_config,
                  nrfx_rtc_handler_t        handler)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(handler);
    int err_code;

    p_instance->cb.handler = handler;

    if (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    nrfy_rtc_int_init(p_instance->p_reg, 0, p_config->interrupt_priority, false);

    nrfy_rtc_config_t config =
    {
        .prescaler = p_config->prescaler
    };

    nrfy_rtc_periph_configure(p_instance->p_reg, &config);
    p_instance->cb.reliable     = p_config->reliable;
    p_instance->cb.tick_latency = p_config->tick_latency;
    p_instance->cb.p_context    = p_config->p_context;
    p_instance->cb.state        = NRFX_DRV_STATE_INITIALIZED;

    err_code = 0;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

void nrfx_rtc_uninit(nrfx_rtc_t * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    uint32_t mask = NRF_RTC_INT_TICK_MASK     |
                    NRF_RTC_INT_OVERFLOW_MASK |
                    NRF_RTC_ALL_CHANNELS_INT_MASK;

    nrfy_rtc_int_uninit(p_instance->p_reg);
    nrfy_rtc_stop(p_instance->p_reg, mask);

    p_instance->cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_rtc_init_check(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(p_instance);
    return (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_rtc_enable(nrfx_rtc_t * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_rtc_task_trigger(p_instance->p_reg, NRF_RTC_TASK_START);
    p_instance->cb.state = NRFX_DRV_STATE_POWERED_ON;
    NRFX_LOG_INFO("Enabled.");
}

void nrfx_rtc_disable(nrfx_rtc_t * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_task_trigger(p_instance->p_reg, NRF_RTC_TASK_STOP);
    p_instance->cb.state = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("Disabled.");
}

int nrfx_rtc_cc_disable(nrfx_rtc_t const * p_instance, uint32_t channel)
{
    NRFX_ASSERT(p_instance && (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED) &&
                (channel < NRF_RTC_CC_COUNT_MAX));

    uint32_t int_mask = NRF_RTC_CHANNEL_INT_MASK(channel);
    nrf_rtc_event_t event = NRF_RTC_CHANNEL_EVENT_ADDR(channel);

    nrfy_rtc_event_disable(p_instance->p_reg, int_mask);
    if (nrfy_rtc_int_enable_check(p_instance->p_reg, int_mask))
    {
        nrfy_rtc_int_disable(p_instance->p_reg, int_mask);
        if (nrfy_rtc_event_check(p_instance->p_reg, event))
        {
            int err_code = -ETIMEDOUT;

            nrfy_rtc_event_clear(p_instance->p_reg, event);
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }
    NRFX_LOG_INFO("RTC: %p, channel disabled: %lu.",
                  p_instance->p_reg, (unsigned long)channel);

    return 0;
}

int nrfx_rtc_cc_set(nrfx_rtc_t const * p_instance,
                    uint32_t           channel,
                    uint32_t           val,
                    bool               enable_irq)
{
    NRFX_ASSERT(p_instance && (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED) &&
                (channel < NRF_RTC_CC_COUNT_MAX));

    int err_code;
    uint32_t int_mask = NRF_RTC_CHANNEL_INT_MASK(channel);
    nrf_rtc_event_t event = NRF_RTC_CHANNEL_EVENT_ADDR(channel);

    nrfy_rtc_event_int_disable(p_instance->p_reg, int_mask);

    val = NRF_RTC_WRAP(val);
    if (p_instance->cb.reliable)
    {
        nrfy_rtc_cc_set(p_instance->p_reg, channel, val);
        uint32_t cnt = nrfy_rtc_counter_get(p_instance->p_reg);
        int32_t diff = (int32_t)(cnt - val);
        if (cnt < val)
        {
            diff += (int32_t)NRF_RTC_COUNTER_MAX;
        }
        if (diff < p_instance->cb.tick_latency)
        {
            err_code = -ETIMEDOUT;
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

    NRFX_LOG_INFO("RTC: %p, channel enabled: %lu, compare value: %lu.",
                  p_instance->p_reg, (unsigned long)channel, (unsigned long)val);
    return 0;
}

void nrfx_rtc_tick_enable(nrfx_rtc_t const * p_instance, bool enable_irq)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_clear_enable(p_instance->p_reg, NRF_RTC_EVENT_TICK, enable_irq);
    NRFX_LOG_INFO("Tick events enabled.");
}

void nrfx_rtc_tick_disable(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_disable(p_instance->p_reg, NRF_RTC_INT_TICK_MASK);
    NRFX_LOG_INFO("Tick events disabled.");
}

void nrfx_rtc_overflow_enable(nrfx_rtc_t const * p_instance, bool enable_irq)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_clear_enable(p_instance->p_reg, NRF_RTC_EVENT_OVERFLOW, enable_irq);
}

void nrfx_rtc_overflow_disable(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rtc_event_int_disable(p_instance->p_reg, NRF_RTC_INT_OVERFLOW_MASK);
}

uint32_t nrfx_rtc_max_ticks_get(nrfx_rtc_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    uint32_t ticks;
    if (p_instance->cb.reliable)
    {
        ticks = NRF_RTC_COUNTER_MAX - p_instance->cb.tick_latency;
    }
    else
    {
        ticks = NRF_RTC_COUNTER_MAX;
    }
    return ticks;
}

void nrfx_rtc_irq_handler(nrfx_rtc_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    uint32_t active_int_mask = nrfy_rtc_int_enable_check(p_instance->p_reg, UINT32_MAX);

    while (active_int_mask)
    {
        uint32_t idx = NRFX_CTZ(active_int_mask);
        nrf_rtc_event_t event = (nrf_rtc_event_t)NRFY_INT_BITPOS_TO_EVENT(idx);

        active_int_mask &= ~NRFX_BIT(idx);
        if (nrfy_rtc_event_check(p_instance->p_reg, event))
        {
            nrfy_rtc_event_clear(p_instance->p_reg, event);
            if (event >= NRF_RTC_EVENT_COMPARE_0)
            {
                nrfy_rtc_event_int_disable(p_instance->p_reg, NRFX_BIT(idx));
            }
            p_instance->cb.handler(event, p_instance->cb.p_context);
        }
    }
}
#endif // !NRFX_API_VER_AT_LEAST(4, 1, 0)
