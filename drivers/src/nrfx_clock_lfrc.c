/*
 * Copyright (c) 2026, Nordic Semiconductor ASA
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

#if NRFX_CHECK(LFRC_PRESENT)
#include <nrfx_clock_lfrc.h>

#if NRF_CLOCK_HAS_LFCLK
#include <nrfx_clock_lfclk.h>

#if NRF_CLOCK_HAS_XO
#include <nrfx_clock_xo.h>
#endif

#define NRFX_LOG_MODULE CLOCK_LFRC
#include <nrfx_log.h>

#if (NRF_LFRC_HAS_CALIBRATION && NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED) && !NRF_CLOCK_HAS_XO)
#error "Calibration requires high-frequency clock to be present in the SoC that is used."
#endif

#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
typedef enum
{
    CAL_STATE_IDLE,
    CAL_STATE_CAL
} nrfx_clock_lfrc_cal_state_t;
#endif

/** @brief CLOCK control block. */
typedef struct
{
    nrfx_clock_lfrc_event_handler_t      event_handler;
    bool                                 module_initialized; /*< Indicate the state of module */
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
    volatile nrfx_clock_lfrc_cal_state_t cal_state;
#endif
} nrfx_clock_lfrc_cb_t;

static nrfx_clock_lfrc_cb_t m_clock_cb;

int nrfx_clock_lfrc_init(nrfx_clock_lfrc_event_handler_t event_handler)
{
    if (m_clock_cb.module_initialized)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(-EALREADY));
        return -EALREADY;
    }
    else
    {
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
        m_clock_cb.cal_state = CAL_STATE_IDLE;
#endif
        m_clock_cb.event_handler = event_handler;
        m_clock_cb.module_initialized = true;
    }

    return 0;
}

void nrfx_clock_lfrc_uninit(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

    m_clock_cb.module_initialized = false;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_clock_lfrc_init_check(void)
{
    return m_clock_cb.module_initialized;
}

#if (NRF_LFRC_HAS_CALIBRATION && NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED))
int nrfx_clock_lfrc_calibration_start(void)
{
    int err_code = 0;

    nrf_clock_hfclk_t clk_src;
#if NRFX_CHECK(NRF_LFRC_HAS_CKLFRCSTARTED_EVENT)
    nrf_clock_lfclk_t lfclk_clk_src;
#endif

    if (!nrfx_clock_xo_running_check(&clk_src))
    {
        err_code = -EINPROGRESS;
    }
    else if (clk_src != NRF_CLOCK_HFCLK_HIGH_ACCURACY)
    {
        err_code = -EINPROGRESS;
    }
#if NRFX_CHECK(NRF_LFRC_HAS_CKLFRCSTARTED_EVENT)
    else if (!nrfx_clock_lfclk_running_check(&lfclk_clk_src))
    {
        err_code = -EINPROGRESS;
    }

    // Check if LFRC is running before starting calibration.
    bool lfclk_started;
    NRFX_WAIT_FOR((nrfx_clock_lfclk_running_check(&lfclk_clk_src) && \
                   lfclk_clk_src == NRF_CLOCK_LFCLK_RC), 1000, 1, lfclk_started);

    if (!lfclk_started)
    {
        err_code = -EINPROGRESS;
    }
#else
    else if (!nrfx_clock_lfclk_running_check(NULL))
    {
        err_code = -EINPROGRESS;
    }
#endif

    if (err_code != 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (m_clock_cb.cal_state == CAL_STATE_IDLE)
    {
        nrf_lfrc_event_clear(NRF_LFRC, NRF_LFRC_EVENT_CALDONE);

        m_clock_cb.cal_state = CAL_STATE_CAL;

        nrf_lfrc_task_trigger(NRF_LFRC, NRF_LFRC_TASK_CAL);
        if (m_clock_cb.event_handler)
        {
            nrf_lfrc_int_enable(NRF_LFRC, NRF_LFRC_INT_CALDONE_MASK);
        }
        else
        {
            while (!nrf_lfrc_event_check(NRF_LFRC, NRF_LFRC_EVENT_CALDONE))
            {}
            nrf_lfrc_event_clear(NRF_LFRC, NRF_LFRC_EVENT_CALDONE);
        }
    }
    else
    {
        err_code = -EBUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    NRFX_LOG_INFO("Initialized.");
    return err_code;
}

int nrfx_clock_lfrc_calibrating_check(void)
{
    if (m_clock_cb.cal_state == CAL_STATE_CAL)
    {
        return -EBUSY;
    }
    return 0;
}
#endif /* NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED) */

void nrfx_clock_lfrc_irq_handler(void)
{
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
    if (nrf_lfrc_event_check(NRF_LFRC, NRF_LFRC_EVENT_CALDONE))
    {
        nrf_lfrc_event_clear(NRF_LFRC, NRF_LFRC_EVENT_CALDONE);
        nrf_lfrc_int_disable(NRF_LFRC, NRF_LFRC_INT_CALDONE_MASK);
        m_clock_cb.cal_state = CAL_STATE_IDLE;
        m_clock_cb.event_handler(NRFX_CLOCK_LFRC_EVT_CAL_DONE);
    }
#endif
}

#endif /* NRF_CLOCK_HAS_LFCLK */

#endif /* NRFX_CHECK(LFRC_PRESENT) */
