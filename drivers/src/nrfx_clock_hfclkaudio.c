/*
 * Copyright (c) 2025 - 2026, Nordic Semiconductor ASA
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
#include <nrfx_clock_hfclkaudio.h>

#if NRF_CLOCK_HAS_HFCLKAUDIO

#define NRFX_LOG_MODULE CLOCK_HFCLKAUDIO
#include <nrfx_log.h>

/** @brief HFCLKAUDIO control block. */
typedef struct
{
    nrfx_clock_hfclkaudio_event_handler_t event_handler;
    bool                                  module_initialized; /*< Indicate the state of module */
} nrfx_clock_cb_t;

static nrfx_clock_cb_t m_clock_cb;

static void clock_stop(void)
{
    nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_HFAUDIO_STARTED_MASK);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKAUDIOSTOP);
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED);

    bool stopped;
    NRFX_WAIT_FOR(!nrfx_clock_hfclkaudio_running_check(), 10000, 1, stopped);
    if (!stopped)
    {
        NRFX_LOG_ERROR("Failed to stop clock domain: HFCLKAUDIO.");
    }
}

int nrfx_clock_hfclkaudio_init(nrfx_clock_hfclkaudio_event_handler_t event_handler)
{
    if (m_clock_cb.module_initialized)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(-EALREADY));
        return -EALREADY;
    }

    m_clock_cb.event_handler = event_handler;
    m_clock_cb.module_initialized = true;

    return 0;
}

void nrfx_clock_hfclkaudio_uninit(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

    clock_stop();

    m_clock_cb.module_initialized = false;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_clock_hfclkaudio_init_check(void)
{
    return m_clock_cb.module_initialized;
}

void nrfx_clock_hfclkaudio_start(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKAUDIOSTART);
    if (m_clock_cb.event_handler)
    {
        nrf_clock_int_enable(NRF_CLOCK, NRF_CLOCK_INT_HFAUDIO_STARTED_MASK);
    }
    else
    {
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED))
        {}
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED);
    }
}

void nrfx_clock_hfclkaudio_stop(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

    clock_stop();
}

void nrfx_clock_hfclkaudio_irq_handler(void)
{
    if (!nrf_clock_int_enable_check(NRF_CLOCK, NRF_CLOCK_INT_HFAUDIO_STARTED_MASK) ||
        !nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED))
    {
        return;
    }

    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKAUDIOSTARTED);
    nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_HFAUDIO_STARTED_MASK);

    NRFX_LOG_DEBUG("Event: HFCLKAUDIO_STARTED");

    m_clock_cb.event_handler();
}

#endif // NRF_CLOCK_HAS_HFCLKAUDIO
