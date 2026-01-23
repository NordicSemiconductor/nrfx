/*
 * Copyright (c) 2015 - 2026, Nordic Semiconductor ASA
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
#include <nrfx_timer.h>

#define PRESCALER_INVALID UINT32_MAX

#define TIMER_FREQUENCY_VALID_CHECK(p_instance, frequency)                                 \
        ((NRFX_TIMER_BASE_FREQUENCY_GET(p_instance) % (frequency) == 0) &&                 \
         NRFX_IS_POWER_OF_TWO(NRFX_TIMER_BASE_FREQUENCY_GET(p_instance) / (frequency)) && \
         ((NRFX_TIMER_BASE_FREQUENCY_GET(p_instance) / (frequency)) <=                     \
          (1 << NRF_TIMER_PRESCALER_MAX)))

#define NRFX_LOG_MODULE TIMER
#include <nrfx_log.h>


static uint32_t prescaler_calculate(nrfx_timer_t const * p_instance, uint32_t frequency)
{
    (void)p_instance;
    uint32_t base_frequency = NRFX_TIMER_BASE_FREQUENCY_GET(p_instance);

    if (!TIMER_FREQUENCY_VALID_CHECK(p_instance, frequency))
    {
        return PRESCALER_INVALID;
    }
    return NRF_CTZ(base_frequency / frequency);
}

static int timer_configure(nrfx_timer_t const *        p_instance,
                           nrfx_timer_config_t const * p_config)
{
    uint32_t prescaler;

    prescaler = prescaler_calculate(p_instance, p_config->frequency);
    if (prescaler == PRESCALER_INVALID)
    {
        NRFX_LOG_WARNING("Specified frequency is not supported by the TIMER instance.");
        return -EINVAL;
    }

    nrfy_timer_config_t nrfy_config =
    {
        .prescaler = prescaler,
        .mode      = p_config->mode,
        .bit_width = p_config->bit_width
    };

    nrfy_timer_periph_configure(p_instance->p_reg, &nrfy_config);
    nrfy_timer_int_init(p_instance->p_reg,
                        NRF_TIMER_ALL_CHANNELS_INT_MASK,
                        p_config->interrupt_priority,
                        false);
    return 0;
}

int nrfx_timer_init(nrfx_timer_t *              p_instance,
                    nrfx_timer_config_t const * p_config,
                    nrfx_timer_event_handler_t  timer_event_handler)
{
    NRFX_ASSERT(p_instance);

    nrfx_timer_control_block_t * p_cb = &p_instance->cb;

    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    p_cb->handler = timer_event_handler;

    if (p_config)
    {
        p_cb->context = p_config->p_context;
        NRFX_ASSERT(NRF_TIMER_IS_BIT_WIDTH_VALID(p_instance->p_reg, p_config->bit_width));

        err_code = timer_configure(p_instance, p_config);
    }
    else
    {
        err_code = 0;
    }

    p_cb->state = err_code == 0 ?
                NRFX_DRV_STATE_INITIALIZED : NRFX_DRV_STATE_UNINITIALIZED;

    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));

    return err_code;
}

int nrfx_timer_reconfigure(nrfx_timer_t *              p_instance,
                           nrfx_timer_config_t const * p_config)
{
    NRFX_ASSERT(p_instance && p_config &&
                NRF_TIMER_IS_BIT_WIDTH_VALID(p_instance->p_reg, p_config->bit_width));

    nrfx_timer_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return -EINPROGRESS;
    }
    if (p_cb->state == NRFX_DRV_STATE_POWERED_ON)
    {
        return -EBUSY;
    }

    p_cb->context = p_config->p_context;
    int err_code = timer_configure(p_instance, p_config);
    return err_code;
}

void nrfx_timer_uninit(nrfx_timer_t * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_timer_int_uninit(p_instance->p_reg);

    nrfy_timer_shorts_disable(p_instance->p_reg, ~0UL);
    nrfy_timer_int_disable(p_instance->p_reg, ~0UL);

    nrfx_timer_disable(p_instance);

    p_instance->cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized instance: %p.", p_instance->p_reg);
}

bool nrfx_timer_init_check(nrfx_timer_t const * p_instance)
{
    NRFX_ASSERT(p_instance);

    return (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_timer_enable(nrfx_timer_t * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_START);
    p_instance->cb.state = NRFX_DRV_STATE_POWERED_ON;
    NRFX_LOG_INFO("Enabled instance: %p.", p_instance->p_reg);
}

void nrfx_timer_disable(nrfx_timer_t * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

#if NRF_TIMER_HAS_SHUTDOWN
    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_SHUTDOWN);
#else
    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_STOP);
    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_CLEAR);
#endif
    p_instance->cb.state = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("Disabled instance: %p.", p_instance->p_reg);
}

bool nrfx_timer_is_enabled(nrfx_timer_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    return (p_instance->cb.state == NRFX_DRV_STATE_POWERED_ON);
}

void nrfx_timer_resume(nrfx_timer_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_START);
    NRFX_LOG_INFO("Resumed instance: %p.", p_instance->p_reg);
}

void nrfx_timer_pause(nrfx_timer_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_STOP);
    NRFX_LOG_INFO("Paused instance: %p.", p_instance->p_reg);
}

void nrfx_timer_clear(nrfx_timer_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_CLEAR);
}

void nrfx_timer_increment(nrfx_timer_t const * p_instance)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(nrfy_timer_mode_get(p_instance->p_reg) != NRF_TIMER_MODE_TIMER);

    nrfy_timer_task_trigger(p_instance->p_reg, NRF_TIMER_TASK_COUNT);
}

uint32_t nrfx_timer_capture(nrfx_timer_t const * p_instance, nrf_timer_cc_channel_t cc_channel)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    return nrfy_timer_capture_get(p_instance->p_reg, cc_channel);
}

uint32_t nrfx_timer_us_to_ticks(nrfx_timer_t const * p_instance, uint32_t time_us)
{
    NRFX_ASSERT(p_instance);

    uint32_t prescaler = nrfy_timer_prescaler_get(p_instance->p_reg);
    uint32_t freq_base_mhz = NRFX_TIMER_BASE_FREQUENCY_GET(p_instance) / 1000000;
    uint64_t ticks = (((uint64_t)time_us * freq_base_mhz) >> prescaler);
    return (uint32_t)ticks;
}

uint32_t nrfx_timer_ms_to_ticks(nrfx_timer_t const * p_instance, uint32_t time_ms)
{
    NRFX_ASSERT(p_instance);

    uint32_t prescaler = nrfy_timer_prescaler_get(p_instance->p_reg);
    uint32_t freq_base_khz = NRFX_TIMER_BASE_FREQUENCY_GET(p_instance) / 1000;
    uint64_t ticks = (((uint64_t)time_ms * freq_base_khz) >> prescaler);
    return (uint32_t)ticks;
}

void nrfx_timer_compare(nrfx_timer_t const *   p_instance,
                        nrf_timer_cc_channel_t cc_channel,
                        uint32_t               cc_value,
                        bool                   enable_int)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrf_timer_int_mask_t timer_int = nrfy_timer_compare_int_get(cc_channel);

    if (enable_int)
    {
        nrfy_timer_event_clear(p_instance->p_reg, nrfy_timer_compare_event_get(cc_channel));
        nrfy_timer_int_enable(p_instance->p_reg, timer_int);
    }
    else
    {
        nrfy_timer_int_disable(p_instance->p_reg, timer_int);
    }

    nrfy_timer_cc_set(p_instance->p_reg, cc_channel, cc_value);
    NRFX_LOG_INFO("Timer address: %p, capture value set: %lu, channel: %d.",
                  p_instance->p_reg,
                  (unsigned long)cc_value,
                  cc_channel);
}

void nrfx_timer_extended_compare(nrfx_timer_t const *   p_instance,
                                 nrf_timer_cc_channel_t cc_channel,
                                 uint32_t               cc_value,
                                 nrf_timer_short_mask_t timer_short_mask,
                                 bool                   enable_int)
{
    NRFX_ASSERT(p_instance && p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_timer_shorts_disable(p_instance->p_reg,
        (uint32_t)(NRF_TIMER_SHORT_COMPARE0_STOP_MASK  << cc_channel) |
        (uint32_t)(NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK << cc_channel));

    nrfy_timer_shorts_enable(p_instance->p_reg, timer_short_mask);

    nrfx_timer_compare(p_instance,
                       cc_channel,
                       cc_value,
                       enable_int);
    NRFX_LOG_INFO("Timer address: %p, capture value set: %lu, channel: %d.",
                  p_instance->p_reg,
                  (unsigned long)cc_value,
                  cc_channel);
}

void nrfx_timer_compare_int_enable(nrfx_timer_t const * p_instance, uint32_t channel)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_timer_event_clear(p_instance->p_reg, nrfy_timer_compare_event_get((uint8_t)channel));
    nrfy_timer_int_enable(p_instance->p_reg, nrfy_timer_compare_int_get((uint8_t)channel));
}

void nrfx_timer_compare_int_disable(nrfx_timer_t const * p_instance, uint32_t channel)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_timer_int_disable(p_instance->p_reg, nrfy_timer_compare_int_get((uint8_t)channel));
}

void nrfx_timer_irq_handler(nrfx_timer_t const * p_instance)
{
    NRFX_ASSERT(p_instance);
    uint32_t active_int_mask = nrfy_timer_int_enable_check(p_instance->p_reg, UINT32_MAX);

    while (active_int_mask)
    {
        uint32_t idx = NRFX_CTZ(active_int_mask);
        nrf_timer_event_t event = (nrf_timer_event_t)NRFY_INT_BITPOS_TO_EVENT(idx);

        active_int_mask &= ~NRFX_BIT(idx);
        if (nrfy_timer_event_check(p_instance->p_reg, event))
        {
            nrfy_timer_event_clear(p_instance->p_reg, event);
            if (p_instance->cb.handler)
            {
                p_instance->cb.handler(event, p_instance->cb.context);
            }
        }
    }
}
