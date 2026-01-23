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
#include <nrfx_clock_hfclk.h>

#if NRF_CLOCK_HAS_HFCLK

#define NRFX_LOG_MODULE CLOCK_HFCLK
#include <nrfx_log.h>

/** @brief CLOCK control block. */
typedef struct
{
    nrfx_clock_hfclk_event_handler_t event_handler;
    bool                             module_initialized; /*< Indicate the state of module */
#if NRF_ERRATA_STATIC_CHECK(52, 201)
    bool                             hfclk_started;      /*< Anomaly 201 workaround. */
#endif
} nrfx_clock_hfclk_cb_t;

static nrfx_clock_hfclk_cb_t m_clock_cb;

static void clock_stop(void)
{
    nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_HF_STARTED_MASK);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTOP);
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);

    bool stopped;
    nrf_clock_hfclk_t clk_src = NRF_CLOCK_HFCLK_HIGH_ACCURACY;
    NRFX_WAIT_FOR(!nrfx_clock_hfclk_running_check(&clk_src), 10000, 1, stopped);
    if (!stopped)
    {
        NRFX_LOG_ERROR("Failed to stop clock domain: NRF_CLOCK_DOMAIN_HFCLK.");
    }
    else
    {
#if NRF_ERRATA_STATIC_CHECK(52, 201)
        if (NRF_ERRATA_DYNAMIC_CHECK(52, 201))
        {
            m_clock_cb.hfclk_started = false;
        }
#endif
    }
}

int nrfx_clock_hfclk_init(nrfx_clock_hfclk_event_handler_t event_handler)
{
    if (m_clock_cb.module_initialized)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(-EALREADY));
        return -EALREADY;
    }
    else
    {
        m_clock_cb.event_handler = event_handler;
        m_clock_cb.module_initialized = true;
#if NRF_ERRATA_STATIC_CHECK(52, 201)
        if (NRF_ERRATA_DYNAMIC_CHECK(52, 201))
        {
            m_clock_cb.hfclk_started = false;
        }
#endif
    }
#if NRF_CLOCK_HAS_HFCLKSRC
    nrf_clock_hf_src_set(NRF_CLOCK, NRF_CLOCK_HFCLK_HIGH_ACCURACY);
#endif // NRF_CLOCK_HAS_HFCLKSRC

    return 0;
}

bool nrfx_clock_hfclk_init_check(void)
{
    return m_clock_cb.module_initialized;
}

void nrfx_clock_hfclk_uninit(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);
    clock_stop();

    m_clock_cb.module_initialized = false;
    NRFX_LOG_INFO("Uninitialized.");
}

void nrfx_clock_hfclk_start(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
    if (m_clock_cb.event_handler)
    {
        nrf_clock_int_enable(NRF_CLOCK, NRF_CLOCK_INT_HF_STARTED_MASK);
    }
    else
    {
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED))
        {}
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
    }
}

void nrfx_clock_hfclk_stop(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);
    clock_stop();
}

#if NRF_CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT
void nrfx_clock_hfclk_divider_set(nrf_clock_hfclk_div_t div)
{
    nrf_clock_hfclk_div_set(NRF_CLOCK, div);
    SystemCoreClockUpdate();
}
#endif

void nrfx_clock_hfclk_irq_handler(void)
{
    if (!nrf_clock_int_enable_check(NRF_CLOCK, NRF_CLOCK_INT_HF_STARTED_MASK) ||
        !nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED))
    {
        return;
    }

    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
    nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_HF_STARTED_MASK);

    NRFX_LOG_DEBUG("Event: HFCLK_STARTED");

#if NRF_ERRATA_STATIC_CHECK(52, 201)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 201))
    {
        if (!m_clock_cb.hfclk_started)
        {
            m_clock_cb.hfclk_started = true;
        }
        else
        {
            return;
        }
    }
#endif

    m_clock_cb.event_handler();
}

#endif // NRF_CLOCK_HAS_HFCLK
