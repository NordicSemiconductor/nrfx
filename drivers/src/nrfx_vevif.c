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

#if NRFX_CHECK(NRFX_VEVIF_ENABLED)

#include <nrfx_vevif.h>
#include <nrf_bitmask.h>
#include <hal/nrf_vpr.h>
#include <hal/nrf_vpr_csr.h>
#include <hal/nrf_vpr_csr_vevif.h>

#define NRFX_LOG_MODULE VEVIF
#include <nrfx_log.h>

#define NRFX_VEVIF_IRQ_HANDLER_DEFINE(idx, _) \
void nrfx_vevif_##idx##_irq_handler(void)     \
{                                             \
    nrfx_vevif_irq_handler(idx);              \
}

typedef struct
{
    nrfx_vevif_event_handler_t handler;
    void *                     p_context;
    nrfx_drv_state_t           state;
} nrfx_vevif_cb_t;

static nrfx_vevif_cb_t m_cb;

nrfx_err_t nrfx_vevif_init(uint8_t                    interrupt_priority,
                           nrfx_vevif_event_handler_t event_handler,
                           void *                     p_context)
{
    NRFX_ASSERT(event_handler);

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

    m_cb.handler   = event_handler;
    m_cb.p_context = p_context;
    m_cb.state     = NRFX_DRV_STATE_INITIALIZED;

    nrf_vpr_csr_vevif_tasks_clear(NRF_VPR_TASK_TRIGGER_ALL_MASK);

    for (uint8_t i = 0; i < NRF_VPR_CSR_VEVIF_EVENT_TASK_COUNT; i++)
    {
        NRFY_IRQ_PRIORITY_SET((VPRCLIC_0_IRQn + i), interrupt_priority);
    }

    NRFX_LOG_INFO("Initialized.");
    return err_code;
}

void nrfx_vevif_uninit(void)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    for (uint8_t i = 0; i < NRF_VPR_CSR_VEVIF_EVENT_TASK_COUNT; i++)
    {
        NRFY_IRQ_DISABLE(VPRCLIC_0_IRQn + i);
    }

    m_cb.handler = NULL;
    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

void nrfx_vevif_int_enable(uint32_t mask)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    while (mask != 0)
    {
        uint32_t event_no = NRF_CTZ(mask);
        NRFY_IRQ_ENABLE(VPRCLIC_0_IRQn + event_no);
        nrf_bitmask_bit_clear(event_no, (void *)&mask);
    }
}

bool nrfx_vevif_init_check(void)
{
    return (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_vevif_int_disable(uint32_t mask)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    while (mask != 0)
    {
        uint32_t event_no = NRF_CTZ(mask);
        NRFY_IRQ_DISABLE(VPRCLIC_0_IRQn + event_no);
        nrf_bitmask_bit_clear(event_no, (void *)&mask);
    }
}

static void nrfx_vevif_irq_handler(uint8_t irq_idx)
{
    nrf_vpr_csr_vevif_tasks_clear(1UL << irq_idx);

    m_cb.handler(irq_idx, m_cb.p_context);
}

/* Define interrupt handlers for 0..31 NRF_VEVIF driver instances. */
NRFX_LISTIFY(32, NRFX_VEVIF_IRQ_HANDLER_DEFINE, (;), _)

#endif // NRFX_CHECK(NRFX_VEVIF_ENABLED)
