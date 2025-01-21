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

#if NRFX_CHECK(NRFX_LPCOMP_ENABLED)

#include <nrfx_lpcomp.h>
#include "prs/nrfx_prs.h"

#define NRFX_LOG_MODULE LPCOMP
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                         \
    (event == NRF_LPCOMP_EVENT_READY ? "NRF_LPCOMP_EVENT_READY" : \
    (event == NRF_LPCOMP_EVENT_DOWN  ? "NRF_LPCOMP_EVENT_DOWN"  : \
    (event == NRF_LPCOMP_EVENT_UP    ? "NRF_LPCOMP_EVENT_UP"    : \
    (event == NRF_LPCOMP_EVENT_CROSS ? "NRF_LPCOMP_EVENT_CROSS" : \
                                       "UNKNOWN EVENT"))))

static nrfx_lpcomp_event_handler_t  m_lpcomp_event_handler = NULL;
static nrfx_drv_state_t             m_state = NRFX_DRV_STATE_UNINITIALIZED;

static void lpcomp_configure(nrfx_lpcomp_config_t const * p_config)
{
    nrfy_lpcomp_config_t nrfy_config =
    {
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
        .reference = p_config->reference,
        .ext_ref   = p_config->ext_ref,
        .detection = p_config->detection,
        NRFX_COND_CODE_1(LPCOMP_FEATURE_HYST_PRESENT, (.hyst = p_config->config.hyst), ())
#else
        .config =
        {
            .reference = p_config->config.reference,
            .detection = p_config->config.detection,
            NRFX_COND_CODE_1(LPCOMP_FEATURE_HYST_PRESENT, (.hyst = p_config->config.hyst), ())
        },
#endif
        .input = p_config->input
    };

    nrfy_lpcomp_periph_configure(NRF_LPCOMP, &nrfy_config);

    uint32_t int_mask = 0;

#if NRFX_API_VER_AT_LEAST(3, 2, 0)
    switch (p_config->detection)
#else
    switch (p_config->config.detection)
#endif
    {
        case NRF_LPCOMP_DETECT_UP:
            int_mask = NRF_LPCOMP_INT_UP_MASK;
            break;

        case NRF_LPCOMP_DETECT_DOWN:
            int_mask = NRF_LPCOMP_INT_DOWN_MASK;
            break;

        case NRF_LPCOMP_DETECT_CROSS:
            int_mask = NRF_LPCOMP_INT_CROSS_MASK;
            break;

        default:
            break;
    }
    nrfy_lpcomp_int_init(NRF_LPCOMP, int_mask, p_config->interrupt_priority, true);
}

static void lpcomp_execute_handler(nrf_lpcomp_event_t event, uint32_t event_mask)
{
    if (event_mask & nrfy_lpcomp_int_enable_check(NRF_LPCOMP, NRFY_EVENT_TO_INT_BITMASK(event)))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(event));

        m_lpcomp_event_handler(event);
    }
}

void nrfx_lpcomp_irq_handler(void)
{
    uint32_t evt_mask = nrfy_lpcomp_events_process(NRF_LPCOMP,
                                                   NRF_LPCOMP_INT_READY_MASK |
                                                   NRF_LPCOMP_INT_DOWN_MASK |
                                                   NRF_LPCOMP_INT_UP_MASK |
                                                   NRF_LPCOMP_INT_CROSS_MASK);

    lpcomp_execute_handler(NRF_LPCOMP_EVENT_READY, evt_mask);
    lpcomp_execute_handler(NRF_LPCOMP_EVENT_DOWN,  evt_mask);
    lpcomp_execute_handler(NRF_LPCOMP_EVENT_UP,    evt_mask);
    lpcomp_execute_handler(NRF_LPCOMP_EVENT_CROSS, evt_mask);
}

nrfx_err_t nrfx_lpcomp_init(nrfx_lpcomp_config_t const * p_config,
                            nrfx_lpcomp_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(event_handler);
    nrfx_err_t err_code;

    if (m_state != NRFX_DRV_STATE_UNINITIALIZED)
    { // LPCOMP driver is already initialized
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

    m_lpcomp_event_handler = event_handler;

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    if (nrfx_prs_acquire(NRF_LPCOMP, nrfx_lpcomp_irq_handler) != NRFX_SUCCESS)
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif
    nrfy_lpcomp_task_trigger(NRF_LPCOMP, NRF_LPCOMP_TASK_STOP);
    nrfy_lpcomp_disable(NRF_LPCOMP);

    nrfy_lpcomp_shorts_disable(NRF_LPCOMP,
                               NRF_LPCOMP_SHORT_CROSS_STOP_MASK |
                               NRF_LPCOMP_SHORT_UP_STOP_MASK |
                               NRF_LPCOMP_SHORT_DOWN_STOP_MASK |
                               NRF_LPCOMP_SHORT_READY_STOP_MASK |
                               NRF_LPCOMP_SHORT_READY_SAMPLE_MASK);
    nrfy_lpcomp_int_disable(NRF_LPCOMP,
                            NRF_LPCOMP_INT_READY_MASK |
                            NRF_LPCOMP_INT_DOWN_MASK |
                            NRF_LPCOMP_INT_UP_MASK |
                            NRF_LPCOMP_INT_CROSS_MASK);

    lpcomp_configure(p_config);

    nrfy_lpcomp_enable(NRF_LPCOMP);

    nrfy_lpcomp_shorts_enable(NRF_LPCOMP, NRF_LPCOMP_SHORT_READY_SAMPLE_MASK);

    m_state = NRFX_DRV_STATE_INITIALIZED;

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_lpcomp_reconfigure(nrfx_lpcomp_config_t const * p_config)
{
    NRFX_ASSERT(p_config);
    nrfx_err_t err_code;

    if (m_state == NRFX_DRV_STATE_POWERED_ON)
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    else if (m_state == NRFX_DRV_STATE_INITIALIZED)
    {
        nrfy_lpcomp_disable(NRF_LPCOMP);
        lpcomp_configure(p_config);
        nrfy_lpcomp_enable(NRF_LPCOMP);
        return NRFX_SUCCESS;
    }
    else
    {
        NRFX_ASSERT(m_state == NRFX_DRV_STATE_UNINITIALIZED);

        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
}

void nrfx_lpcomp_uninit(void)
{
    NRFX_ASSERT(m_state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfy_lpcomp_int_uninit(NRF_LPCOMP);
    nrfx_lpcomp_disable();
#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(NRF_LPCOMP);
#endif
    m_state = NRFX_DRV_STATE_UNINITIALIZED;
    m_lpcomp_event_handler = NULL;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_lpcomp_init_check(void)
{
    return (m_state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_lpcomp_start(uint32_t lpcomp_evt_en_mask, uint32_t lpcomp_shorts_mask)
{
    NRFX_ASSERT(m_state == NRFX_DRV_STATE_INITIALIZED);
    nrfy_lpcomp_enable(NRF_LPCOMP);
    (void)nrfy_lpcomp_events_process(NRF_LPCOMP, lpcomp_evt_en_mask);
    nrfy_lpcomp_int_enable(NRF_LPCOMP, lpcomp_evt_en_mask);
    nrfy_lpcomp_shorts_enable(NRF_LPCOMP, lpcomp_shorts_mask);
    m_state = NRFX_DRV_STATE_POWERED_ON;
    nrfy_lpcomp_task_trigger(NRF_LPCOMP, NRF_LPCOMP_TASK_START);
    NRFX_LOG_INFO("Enabled.");
}

void nrfx_lpcomp_stop(void)
{
    NRFX_ASSERT(m_state == NRFX_DRV_STATE_POWERED_ON);
    nrfy_lpcomp_disable(NRF_LPCOMP);
    nrfy_lpcomp_task_trigger(NRF_LPCOMP, NRF_LPCOMP_TASK_STOP);
    nrfy_lpcomp_int_disable(NRF_LPCOMP,
                            NRF_LPCOMP_INT_READY_MASK |
                            NRF_LPCOMP_INT_DOWN_MASK |
                            NRF_LPCOMP_INT_UP_MASK |
                            NRF_LPCOMP_INT_CROSS_MASK);
    m_state = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("Disabled.");
}

uint32_t nrfx_lpcomp_sample(void)
{
    NRFX_ASSERT(m_state == NRFX_DRV_STATE_POWERED_ON);

    return nrfy_lpcomp_sample(NRF_LPCOMP);
}

#endif // NRFX_CHECK(NRFX_LPCOMP_ENABLED)
