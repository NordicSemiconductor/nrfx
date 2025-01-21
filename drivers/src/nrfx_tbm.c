/*
 * Copyright (c) 2022 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_TBM_ENABLED)

#include <nrfx_tbm.h>
#include <haly/nrfy_tbm.h>

static nrfx_drv_state_t state;
static nrfx_tbm_event_handler_t evt_handler;
static const uint32_t m_int_flags = NRF_TBM_INT_HALFFULL_MASK |
                                    NRF_TBM_INT_FULL_MASK     |
                                    NRF_TBM_INT_FLUSH_MASK;

nrfx_err_t nrfx_tbm_init(nrfx_tbm_config_t const * p_config, nrfx_tbm_event_handler_t handler)
{
    if (state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_ALREADY;
    }

    nrfy_tbm_configure(NRF_TBM, p_config->size);

    if (handler)
    {
        evt_handler = handler;
        nrfy_tbm_int_init(NRF_TBM, m_int_flags, p_config->interrupt_priority, true);
    }

    state = NRFX_DRV_STATE_INITIALIZED;

    return NRFX_SUCCESS;
}

void nrfx_tbm_start(void)
{
    NRFX_ASSERT(state == NRFX_DRV_STATE_INITIALIZED);
    nrfy_tbm_task_trigger(NRF_TBM, NRF_TBM_TASK_START);
}

void nrfx_tbm_stop(void)
{
    NRFX_ASSERT(state == NRFX_DRV_STATE_INITIALIZED);
    nrfy_tbm_task_trigger(NRF_TBM, NRF_TBM_TASK_STOP);
}

uint32_t nrfx_tbm_count_get(void)
{
    NRFX_ASSERT(state == NRFX_DRV_STATE_INITIALIZED);
    return nrfy_tbm_count_get(NRF_TBM);
}

void nrfx_tbm_uninit(void)
{
    NRFX_ASSERT(state == NRFX_DRV_STATE_INITIALIZED);
    nrfx_tbm_stop();
    nrfy_tbm_int_uninit(NRF_TBM);

    state = NRFX_DRV_STATE_UNINITIALIZED;
}

bool nrfx_tbm_init_check(void)
{
    return (state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_tbm_irq_handler(void)
{
    uint32_t evts = nrfy_tbm_events_process(NRF_TBM, m_int_flags);

    if (evts & NRF_TBM_INT_HALFFULL_MASK)
    {
        evt_handler(NRF_TBM_EVENT_HALFFULL);
    }

    if (evts & NRF_TBM_INT_FULL_MASK)
    {
        evt_handler(NRF_TBM_EVENT_FULL);
    }

    if (evts & NRF_TBM_INT_FLUSH_MASK)
    {
        evt_handler(NRF_TBM_EVENT_FLUSH);
    }
}

#endif // NRFX_CHECK(NRFX_TBM_ENABLED)
