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

#if NRFX_CHECK(NRFX_BELLBOARD_ENABLED)

#include <nrfx_bellboard.h>
#include <nrf_bitmask.h>

#define NRFX_LOG_MODULE BELLBOARD
#include <nrfx_log.h>

typedef struct
{
    nrfx_bellboard_event_handler_t handler;
    void * context;
    uint32_t int_pend;
    uint8_t int_idx;
    nrfx_drv_state_t state;
} nrfx_bellboard_cb_t;

static nrfx_bellboard_cb_t m_cb[NRFX_BELLBOARD_ENABLED_COUNT];

nrfx_err_t nrfx_bellboard_init(nrfx_bellboard_t const *       p_instance,
                               uint8_t                        interrupt_priority,
                               nrfx_bellboard_event_handler_t event_handler,
                               void *                         p_context)
{
    nrfx_bellboard_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (p_cb->state == NRFX_DRV_STATE_INITIALIZED)
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

    p_cb->state   = NRFX_DRV_STATE_INITIALIZED;
    p_cb->handler = event_handler;
    p_cb->context = p_context;
    p_cb->int_idx = p_instance->int_idx;

    nrfy_bellboard_int_init(NRF_BELLBOARD,
                            0,
                            interrupt_priority,
                            false,
                            p_instance->int_idx);

    NRFX_LOG_INFO("Initialized.");
    return err_code;
}

void nrfx_bellboard_uninit(nrfx_bellboard_t const * p_instance)
{
    NRFX_ASSERT(m_cb[p_instance->drv_inst_idx].state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_bellboard_int_uninit(p_instance->int_idx);

    nrfx_bellboard_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    p_cb->handler = NULL;
    p_cb->int_idx = 0;
    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_bellboard_init_check(nrfx_bellboard_t const * p_instance)
{
    nrfx_bellboard_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    return (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_bellboard_int_enable(nrfx_bellboard_t const * p_instance, uint32_t mask)
{
    NRFX_ASSERT(m_cb[p_instance->drv_inst_idx].state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_bellboard_int_enable(NRF_BELLBOARD, p_instance->int_idx, mask);
}

void nrfx_bellboard_int_disable(nrfx_bellboard_t const * p_instance, uint32_t mask)
{
    NRFX_ASSERT(m_cb[p_instance->drv_inst_idx].state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_bellboard_int_disable(NRF_BELLBOARD, p_instance->int_idx, mask);
}

static void irq_handler(void * unused, nrfx_bellboard_cb_t * p_cb)
{
    (void)unused;
    /* Pending interrupts registers are cleared when event is cleared.
     * Add current pending interrupts to be processed later.
     */
    for (uint8_t i = 0; i < NRFX_BELLBOARD_ENABLED_COUNT; i++)
    {
        if (m_cb[i].state == NRFX_DRV_STATE_INITIALIZED)
        {
            m_cb[i].int_pend |= nrfy_bellboard_int_pending_get(NRF_BELLBOARD, m_cb[i].int_idx);
        }
    }

    uint32_t int_pend = p_cb->int_pend;
    p_cb->int_pend = 0;

    (void)nrfy_bellboard_events_process(NRF_BELLBOARD, int_pend);

    if (p_cb->handler != NULL)
    {
        while (int_pend)
        {
            uint8_t event_no = (uint8_t)NRF_CTZ(int_pend);
            p_cb->handler(event_no, p_cb->context);
            nrf_bitmask_bit_clear(event_no, &int_pend);
        }
    }
}

NRFX_INSTANCE_IRQ_HANDLERS(BELLBOARD, bellboard)

#endif // NRFX_CHECK(NRFX_BELLBOARD_ENABLED)
