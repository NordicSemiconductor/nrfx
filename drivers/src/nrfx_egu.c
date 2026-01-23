/*
 * Copyright (c) 2019 - 2026, Nordic Semiconductor ASA
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
#include <nrfx_egu.h>

/*
 * `-Warray-bounds` warning is disabled for the `egu_event_mask_get_and_clear`
 * function because GCC 12 and above may report a false positive due to accessing
 * event registers.
 */
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif

static uint32_t egu_event_mask_get_and_clear(NRF_EGU_Type * p_reg, uint32_t int_mask)
{
    uint32_t event_mask = 0;
    while (int_mask)
    {
        uint8_t event_idx = (uint8_t)NRF_CTZ(int_mask);
        int_mask &= ~(1UL << event_idx);

        nrf_egu_event_t event = nrf_egu_triggered_event_get(event_idx);
        if (nrf_egu_event_check(p_reg, event))
        {
            nrf_egu_event_clear(p_reg, event);
            event_mask |= (1UL << event_idx);
        }
    }
    return event_mask;
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

int nrfx_egu_init(nrfx_egu_t *             p_instance,
                  uint8_t                  interrupt_priority,
                  nrfx_egu_event_handler_t event_handler,
                  void *                   p_context)
{
    NRFX_ASSERT(p_instance);
    nrfx_egu_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        return -EALREADY;
    }

    p_cb->state     = NRFX_DRV_STATE_INITIALIZED;
    p_cb->p_context = p_context;
    p_cb->handler   = event_handler;
    if (event_handler)
    {
        NRFX_IRQ_ENABLE(nrfx_get_irq_number(p_instance->p_reg));
        NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(p_instance->p_reg), interrupt_priority);
    }

    return 0;
}

void nrfx_egu_int_enable(nrfx_egu_t * p_instance, uint32_t mask)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_instance->cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_instance->cb.handler);

    (void)egu_event_mask_get_and_clear(p_instance->p_reg, mask);
    nrf_egu_int_enable(p_instance->p_reg, mask);
}

void nrfx_egu_int_disable(nrfx_egu_t * p_instance, uint32_t mask)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_instance->cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrf_egu_int_disable(p_instance->p_reg, mask);
}

void nrfx_egu_trigger(nrfx_egu_t * p_instance, uint8_t event_idx)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_instance->cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(event_idx < nrf_egu_channel_count(p_instance->p_reg));

    nrf_egu_task_trigger(p_instance->p_reg, nrf_egu_trigger_task_get(event_idx));
}

void nrfx_egu_uninit(nrfx_egu_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_egu_control_block_t * p_cb = &p_instance->cb;

    nrf_egu_int_disable(p_instance->p_reg, ~0UL);
    NRFX_IRQ_DISABLE(nrfx_get_irq_number(p_instance->p_reg));

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
}

bool nrfx_egu_init_check(nrfx_egu_t const * p_instance)
{
    NRFX_ASSERT(p_instance);

    return (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_egu_irq_handler(nrfx_egu_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_egu_control_block_t * p_cb = &p_instance->cb;

    uint32_t int_mask = nrf_egu_int_enable_check(p_instance->p_reg, ~0UL);

    /* Check (and clear) only the events that are set to generate interrupts.
       Leave the other ones untouched. */
    uint32_t event_mask = egu_event_mask_get_and_clear(p_instance->p_reg, int_mask);
    while (event_mask)
    {
        uint8_t event_idx = (uint8_t)NRF_CTZ(event_mask);
        event_mask &= ~(1UL << event_idx);
        p_cb->handler(event_idx, p_cb->p_context);
    }
}
