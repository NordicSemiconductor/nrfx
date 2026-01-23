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
#include <hal/nrf_clock.h>

#if NRF_CLOCK_HAS_LFCLK
#include <nrfx_clock_lfclk.h>

#if NRF_CLOCK_HAS_HFCLK
#include <nrfx_clock_hfclk.h>
#endif

#if NRF_CLOCK_HAS_XO
#include <nrfx_clock_xo.h>
#endif

#define NRFX_LOG_MODULE CLOCK_LFCLK
#include <nrfx_log.h>

#if (NRF_CLOCK_HAS_CALIBRATION && NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED) && \
        !NRF_CLOCK_HAS_HFCLK && !NRF_CLOCK_HAS_XO)
#error "Calibration requires HFCLK or XO to be present in the SoC that is used."
#endif

#if defined(CLOCK_LFCLKSRC_SRC_RC) || defined(__NRFX_DOXYGEN__)
    #define LF_SRC_RC CLOCK_LFCLKSRC_SRC_RC
#elif defined(CLOCK_LFCLKSRC_SRC_LFRC)
    #define LF_SRC_RC CLOCK_LFCLKSRC_SRC_LFRC
#else
    #define LF_SRC_RC CLOCK_LFCLK_SRC_SRC_LFRC
#endif
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
    #if (NRF_CLOCK_HAS_CALIBRATION == 0 && NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION) == 0)
        #error "Calibration is not available in the SoC that is used."
    #endif
    #if (NRFX_CLOCK_CONFIG_LF_SRC != LF_SRC_RC)
        #error "Calibration can be performed only for the RC Oscillator."
    #endif
#endif

#if defined(CLOCK_LFCLKSRC_SRC_Xtal)
    #define LF_SRC_LFXO CLOCK_LFCLKSRC_SRC_Xtal
#elif NRF_CLOCK_HAS_LFCLK_TYPE
    #define LF_SRC_LFXO CLOCK_LFCLK_SRC_SRC_LFXO
#else
    #define LF_SRC_LFXO CLOCK_LFCLKSRC_SRC_LFXO
#endif

#if defined(NRF_CLOCK_USE_EXTERNAL_LFCLK_SOURCES)
    #define LF_SRC_XTAL_LOW  (CLOCK_LFCLKSRC_SRC_Xtal | \
                             (CLOCK_LFCLKSRC_EXTERNAL_Enabled << CLOCK_LFCLKSRC_EXTERNAL_Pos))
    #define LF_SRC_XTAL_FULL (CLOCK_LFCLKSRC_SRC_Xtal | \
                             (CLOCK_LFCLKSRC_BYPASS_Enabled   << CLOCK_LFCLKSRC_BYPASS_Pos) | \
                             (CLOCK_LFCLKSRC_EXTERNAL_Enabled << CLOCK_LFCLKSRC_EXTERNAL_Pos))
#else
    #define LF_SRC_XTAL_LOW  LF_SRC_LFXO
    #define LF_SRC_XTAL_FULL LF_SRC_LFXO
#endif

#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LFXO_TWO_STAGE_ENABLED) && \
    NRFX_CLOCK_CONFIG_LF_SRC != LF_SRC_LFXO && \
    NRFX_CLOCK_CONFIG_LF_SRC != LF_SRC_XTAL_LOW && \
    NRFX_CLOCK_CONFIG_LF_SRC != LF_SRC_XTAL_FULL
    #error "Two-stage LFXO start procedure enabled but LFCLK source is not set to LFXO!"
#endif

#if !defined(NRFX_CLOCK_CONFIG_CT_ENABLED) && NRF_CLOCK_HAS_CALIBRATION_TIMER
#define NRFX_CLOCK_CONFIG_CT_ENABLED 1
#endif

#if NRFX_CHECK(NRFX_CLOCK_CONFIG_CT_ENABLED) && !NRF_CLOCK_HAS_CALIBRATION_TIMER
    #error "Calibration timer is not available in the SoC that is used."
#endif

#define INTERRUPT_MASK (                                                                         \
    NRF_CLOCK_INT_LF_STARTED_MASK |                                                              \
    NRFX_COND_CODE_1(NRF_CLOCK_HAS_LFCLK_SRC_CHANGED, (NRF_CLOCK_INT_LF_SRC_CHANGED_MASK |), ()) \
        NRFX_COND_CODE_1(NRF_CLOCK_HAS_CALIBRATION, (NRF_CLOCK_INT_DONE_MASK |), ())             \
            NRFX_COND_CODE_1(NRF_CLOCK_HAS_CALIBRATION_TIMER, (NRF_CLOCK_INT_CTTO_MASK |), ()) 0)

#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
typedef enum
{
    CAL_STATE_IDLE,
    CAL_STATE_CAL
} nrfx_clock_lfclk_cal_state_t;
#endif

#define NRFX_CLOCK_EVT2STR(evt_type)                                                        \
    evt_type == NRFX_CLOCK_LFCLK_EVT_LFCLK_STARTED ? "LFCLK_STARTED" :                      \
    NRFX_COND_CODE_1(NRF_CLOCK_HAS_CALIBRATION_TIMER,                                       \
            (evt_type == NRFX_CLOCK_LFCLK_EVT_CTTO ? "CTTO" : ), ())                        \
    NRFX_COND_CODE_1(NRF_CLOCK_HAS_CALIBRATION,                                             \
            (evt_type == NRFX_CLOCK_LFCLK_EVT_CAL_DONE ? "CAL_DONE" : ), ()) "Unknown"

/** @brief CLOCK control block. */
typedef struct
{
    nrfx_clock_lfclk_event_handler_t        event_handler;
    bool                                    module_initialized; /*< Indicate the state of module */
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
    volatile nrfx_clock_lfclk_cal_state_t   cal_state;
#endif
} nrfx_clock_lfclk_cb_t;

static nrfx_clock_lfclk_cb_t m_clock_cb;

static void clock_stop(void)
{
    nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_LF_STARTED_MASK);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTOP);
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);

    bool stopped;
    NRFX_WAIT_FOR((!nrfx_clock_lfclk_running_check(NULL)), 10000, 1, stopped);
    if (!stopped)
    {
        NRFX_LOG_ERROR("Failed to stop clock domain: %d.", NRF_CLOCK_DOMAIN_LFCLK);
    }
}

static nrf_clock_lfclk_t clock_initial_lfclksrc_get(void)
{
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LFXO_TWO_STAGE_ENABLED)
    return NRF_CLOCK_LFCLK_RC;
#else
    return (nrf_clock_lfclk_t)NRFX_CLOCK_CONFIG_LF_SRC;
#endif
}

/**
 * @brief Function for tweaking the specified low-frequency clock source given current driver state.
 *
 * @warning This function may stop currently running low-frequency clock source.
 *
 * @param[in,out] p_lfclksrc Pointer to the variable containing low-frequency clock source.
 *                           It is set to adequate value in case of being inappropriate
 *                           for current driver configuration.
 *
 * @return True if the specified clock source was correct, false otherwise.
 */
static bool clock_lfclksrc_tweak(nrf_clock_lfclk_t * p_lfclksrc)
{
    bool is_correct_clk = (*p_lfclksrc == NRFX_CLOCK_CONFIG_LF_SRC);
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LFXO_TWO_STAGE_ENABLED)
    // In case of two-stage LFXO start procedure RC source is valid as well.
    is_correct_clk = is_correct_clk || (*p_lfclksrc == NRF_CLOCK_LFCLK_RC);
#endif
    if (!is_correct_clk)
    {
        // Inappropriate LF clock source is chosen.
        // Stop currently active LF clock source and choose the correct one to start.
        clock_stop();
        *p_lfclksrc = clock_initial_lfclksrc_get();
    }
    return is_correct_clk;
}

int nrfx_clock_lfclk_init(nrfx_clock_lfclk_event_handler_t event_handler)
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

    nrf_clock_lf_src_set(NRF_CLOCK, clock_initial_lfclksrc_get());

    return 0;
}

void nrfx_clock_lfclk_uninit(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);
    clock_stop();

    m_clock_cb.module_initialized = false;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_clock_lfclk_init_check(void)
{
    return m_clock_cb.module_initialized;
}

void nrfx_clock_lfclk_start(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

    nrf_clock_lfclk_t lfclksrc;
    if (nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK, &lfclksrc))
    {
        // LF clock is already running. Inspect its source.
        // If LF clock source is inappropriate then it will be stopped and modified.
        // Ignore return value as LF clock will be started again regardless of the result.
        (void)clock_lfclksrc_tweak(&lfclksrc);
    }
    else if (nrf_clock_start_task_check(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK))
    {
        // LF clock is not active yet but was started already. Inspect its source.
        lfclksrc = nrf_clock_lf_srccopy_get(NRF_CLOCK);
        if (clock_lfclksrc_tweak(&lfclksrc))
        {
            // LF clock was started already and the configured source
            // corresponds to the user configuration.
            // No action is needed as the chosen LF clock source will become active soon.
            if (m_clock_cb.event_handler)
            {
                nrf_clock_int_enable(NRF_CLOCK, NRF_CLOCK_INT_LF_STARTED_MASK);
            }
            else
            {
                while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED))
                {}
                nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
            }
            return;
        }
        // Otherwise LF clock was started already but with inappropriate source.
        // LF clock was stopped and modified. Now it will be restarted.
    }
    else
    {
        // LF clock not active and not started.
        lfclksrc = clock_initial_lfclksrc_get();
    }
    nrf_clock_lf_src_set(NRF_CLOCK, lfclksrc);
        
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 132))
    {
        NRFX_DELAY_US(138);
    }
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
    if (m_clock_cb.event_handler)
    {
        nrf_clock_int_enable(NRF_CLOCK, NRF_CLOCK_INT_LF_STARTED_MASK);
    }
    else
    {
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED))
        {}
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_LFCLKSTARTED);
    }
}

void nrfx_clock_lfclk_stop(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);
    clock_stop();
}

#if ((NRF_CLOCK_HAS_CALIBRATION || NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION)) && \
     NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED))
int nrfx_clock_lfclk_calibration_start(void)
{
    int err_code = 0;

    nrf_clock_hfclk_t clk_src;
#if NRF_CLOCK_HAS_HFCLK
    if (!nrfx_clock_hfclk_running_check(&clk_src))
#else
    if (!nrfx_clock_xo_running_check(&clk_src))
#endif
    {
        err_code = -EINPROGRESS;
    }
    else if (clk_src != NRF_CLOCK_HFCLK_HIGH_ACCURACY)
    {
        err_code = -EINPROGRESS;
    }
    else if (!nrfx_clock_lfclk_running_check(NULL))
    {
        err_code = -EINPROGRESS;
    }

    if (err_code != 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (m_clock_cb.cal_state == CAL_STATE_IDLE)
    {
#if NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION)
        nrf_lfrc_event_clear(NRF_LFRC, NRF_LFRC_EVENT_CALDONE);
#else
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_DONE);
#endif

        m_clock_cb.cal_state = CAL_STATE_CAL;

        if (NRF_ERRATA_DYNAMIC_CHECK(52, 192))
        {
            *(volatile uint32_t *)0x40000C34 = 0x00000002;
        }

#if NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION)
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
#else
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_CAL);
        if (m_clock_cb.event_handler)
        {
            nrf_clock_int_enable(NRF_CLOCK, NRF_CLOCK_INT_DONE_MASK);
        }
        else
        {
            while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_DONE))
            {}
            nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_DONE);
        }
#endif
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

int nrfx_clock_lfclk_calibrating_check(void)
{
    if (m_clock_cb.cal_state == CAL_STATE_CAL)
    {
        return -EBUSY;
    }
    return 0;
}

#if NRF_CLOCK_HAS_CALIBRATION_TIMER && NRFX_CHECK(NRFX_CLOCK_CONFIG_CT_ENABLED)
void nrfx_clock_lfclk_calibration_timer_start(uint8_t interval)
{
    nrf_clock_cal_timer_timeout_set(NRF_CLOCK, interval);
    nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_CTTO);

    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_CTSTART);
    if (m_clock_cb.event_handler)
    {
        nrf_clock_int_enable(NRF_CLOCK, NRF_CLOCK_INT_CTTO_MASK);
    }
    else
    {
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_CTTO))
        {}
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_CTTO);
    }
}

void nrfx_clock_lfclk_calibration_timer_stop(void)
{
    nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_CTTO_MASK);
    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_CTSTOP);
}
#endif // NRF_CLOCK_HAS_CALIBRATION_TIMER && NRFX_CHECK(NRFX_CLOCK_CONFIG_CT_ENABLED)
#endif /* ((NRF_CLOCK_HAS_CALIBRATION || NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION)) &&
            NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)) */

void nrfx_clock_lfclk_irq_handler(void)
{
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_USE_LFRC_CALIBRATION) && \
    NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
    if (nrf_lfrc_event_check(NRF_LFRC, NRF_LFRC_EVENT_CALDONE))
    {
        nrf_lfrc_event_clear(NRF_LFRC, NRF_LFRC_EVENT_CALDONE);
        nrf_lfrc_int_disable(NRF_LFRC, NRF_LFRC_INT_CALDONE_MASK);
        m_clock_cb.cal_state = CAL_STATE_IDLE;
        m_clock_cb.event_handler(NRFX_CLOCK_EVT_CAL_DONE);
    }
#endif
#if NRF_CLOCK_HAS_INTPEND
    uint32_t intpend = nrf_clock_int_pending_get(NRF_CLOCK) & INTERRUPT_MASK;
#else
    uint32_t intpend = nrf_clock_int_enable_check(NRF_CLOCK, INTERRUPT_MASK);
#endif

    while (intpend != 0)
    {
        uint32_t int_bit = 31 - NRF_CLZ(intpend);
        nrf_clock_int_mask_t int_mask = (nrf_clock_int_mask_t)NRFX_BIT(int_bit);
        nrf_clock_event_t evt = (nrf_clock_event_t)NRFY_INT_BITPOS_TO_EVENT(int_bit);
        nrfx_clock_lfclk_evt_type_t evt_type = (nrfx_clock_lfclk_evt_type_t)int_bit;
        bool call_handler = true;
#if !NRF_CLOCK_HAS_INTPEND
        intpend &= ~int_mask;
        // Check if event is set for that interrupt and if not continue.
        if (!nrf_clock_event_check(NRF_CLOCK, evt)) {
            continue;
        }
#endif

        nrf_clock_event_clear(NRF_CLOCK, evt);
        nrf_clock_int_disable(NRF_CLOCK, int_mask);

        NRFX_LOG_DEBUG("Event: %s", NRFX_CLOCK_EVT2STR(evt_type));
        switch (int_mask)
        {
            case NRF_CLOCK_INT_LF_STARTED_MASK:
            {
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LFXO_TWO_STAGE_ENABLED)
                nrf_clock_lfclk_t lfclksrc;
                (void)nrf_clock_is_running(NRF_CLOCK, NRF_CLOCK_DOMAIN_LFCLK, &lfclksrc);
                if (lfclksrc == NRF_CLOCK_LFCLK_RC)
                {
                    // After the LFRC oscillator start switch to external source.
                    nrf_clock_lf_src_set(NRF_CLOCK, (nrf_clock_lfclk_t)NRFX_CLOCK_CONFIG_LF_SRC);
                    nrf_clock_int_enable(NRF_CLOCK, int_mask);
                    nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_LFCLKSTART);
                    call_handler = false;
                }
                else
#endif
                {
                    // After the LF clock external source start invoke user callback.
                }
                break;
            }
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED) && NRF_CLOCK_HAS_CALIBRATION
#if NRF_CLOCK_HAS_CALIBRATION_TIMER && NRFX_CHECK(NRFX_CLOCK_CONFIG_CT_ENABLED)
            case NRF_CLOCK_INT_CTTO_MASK:
                break;
#endif
            case NRF_CLOCK_INT_DONE_MASK:
                if (NRF_ERRATA_DYNAMIC_CHECK(52, 192))
                {
                    *(volatile uint32_t *)0x40000C34 = 0x00000000;
                }
                m_clock_cb.cal_state = CAL_STATE_IDLE;
                break;
#endif // (NRFX_CLOCK_CONFIG_LF_CAL_ENABLED && NRF_CLOCK_HAS_CALIBRATION)
            default:
                NRFX_ASSERT(0);
                break;
        }

        if (call_handler)
        {
            m_clock_cb.event_handler(evt_type);
        }
#if NRF_CLOCK_HAS_INTPEND
        intpend = nrf_clock_int_pending_get(NRF_CLOCK) & INTERRUPT_MASK;
#endif
    }
}

#endif /* NRF_CLOCK_HAS_LFCLK */
