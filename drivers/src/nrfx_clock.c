/*
 * Copyright (c) 2016 - 2026, Nordic Semiconductor ASA
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
#include <nrfx_clock.h>

#define NRFX_LOG_MODULE CLOCK
#include <nrfx_log.h>

#if NRFX_CHECK(NRFX_POWER_ENABLED)
extern bool nrfx_power_irq_enabled;
#endif

/** @brief CLOCK control block. */
typedef struct
{
    nrfx_clock_event_handler_t event_handler;
    bool                       module_initialized; /*< Indicate the state of module */
} nrfx_clock_cb_t;

static nrfx_clock_cb_t m_clock_cb;

/**
 * This variable is used to check whether common POWER_CLOCK common interrupt
 * should be disabled or not if @ref nrfx_power tries to disable the interrupt.
 */
#if NRFX_CHECK(NRFX_POWER_ENABLED)
bool nrfx_clock_irq_enabled;
#endif

#if NRFX_CHECK(NRF_CLOCK_HAS_HFCLK)
static void hfclk_event_handler(void)
{
    m_clock_cb.event_handler(NRFX_CLOCK_EVT_HFCLK_STARTED);
}
#endif

#if NRF_CLOCK_HAS_XO
static void xo_event_handler(nrfx_clock_xo_event_type_t event)
{
    m_clock_cb.event_handler((nrfx_clock_evt_type_t)event);
}
#endif // NRF_CLOCK_HAS_XO

#if NRF_CLOCK_HAS_LFCLK
static void lfclk_event_handler(nrfx_clock_lfclk_evt_type_t event)
{
    m_clock_cb.event_handler((nrfx_clock_evt_type_t)event);
}
#endif // NRF_CLOCK_HAS_LFCLK

#if NRF_CLOCK_HAS_HFCLK192M
static void hfclk192m_event_handler(void)
{
    m_clock_cb.event_handler(NRFX_CLOCK_EVT_HFCLK192M_STARTED);
}
#endif // NRF_CLOCK_HAS_HFCLK192M

#if NRF_CLOCK_HAS_HFCLK24M
static void xo24m_event_handler(void)
{
    m_clock_cb.event_handler(NRFX_CLOCK_EVT_HFCLK24M_STARTED);
}
#endif // NRF_CLOCK_HAS_HFCLK24M

#if NRF_CLOCK_HAS_HFCLKAUDIO
static void hfclkaudio_event_handler(void)
{
    m_clock_cb.event_handler(NRFX_CLOCK_EVT_HFCLKAUDIO_STARTED);
}
#endif // NRF_CLOCK_HAS_HFCLKAUDIO

int nrfx_clock_init(nrfx_clock_event_handler_t event_handler)
{
    int err_code = 0;
    if (m_clock_cb.module_initialized)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(-EALREADY));
        return -EALREADY;
    }
    else
    {
        m_clock_cb.event_handler = event_handler;
        m_clock_cb.module_initialized = true;
    }

#if NRFX_CHECK(NRF_CLOCK_HAS_HFCLK)
    err_code = nrfx_clock_hfclk_init(m_clock_cb.event_handler ? &hfclk_event_handler : NULL);
#elif NRF_CLOCK_HAS_XO
    err_code = nrfx_clock_xo_init(m_clock_cb.event_handler ? &xo_event_handler : NULL);
#endif
    if (err_code != 0)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRF_CLOCK_HAS_HFCLK192M
    err_code = nrfx_clock_hfclk192m_init(m_clock_cb.event_handler ? &hfclk192m_event_handler :
                                                                    NULL);
    if (err_code != 0)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif

#if NRF_CLOCK_HAS_LFCLK
    err_code = nrfx_clock_lfclk_init(m_clock_cb.event_handler ? &lfclk_event_handler : NULL);
    if (err_code != 0)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRF_CLOCK_HAS_LFCLK

#if NRF_CLOCK_HAS_HFCLK24M
    err_code = nrfx_clock_xo24m_init(m_clock_cb.event_handler ? &xo24m_event_handler : NULL);
    if (err_code != 0)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRF_CLOCK_HAS_HFCLK24M

#if NRF_CLOCK_HAS_HFCLKAUDIO
    err_code = nrfx_clock_hfclkaudio_init(m_clock_cb.event_handler ? &hfclkaudio_event_handler :
                                                                     NULL);
    if (err_code != 0)
    {
        NRFX_LOG_INFO("Function: %s, error code: %s.", __func__,
                      NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRF_CLOCK_HAS_HFCLKAUDIO

    return 0;
}

void nrfx_clock_enable(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);
    if (m_clock_cb.event_handler)
    {
        nrfx_power_clock_irq_init();
    }
#if NRFX_CHECK(NRFX_POWER_ENABLED)
    nrfx_clock_irq_enabled = true;
#endif
    NRFX_LOG_INFO("Module enabled.");
}

void nrfx_clock_disable(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

    if (m_clock_cb.event_handler)
    {
#if NRFX_CHECK(NRFX_POWER_ENABLED)
        NRFX_ASSERT(nrfx_clock_irq_enabled);
        if (!nrfx_power_irq_enabled)
#endif
        {
#if defined(CLOCK_STATIC_IRQ)
            IRQn_Type irqn = CLOCK_POWER_IRQn;
#else
            IRQn_Type irqn = nrfx_get_irq_number(NRF_CLOCK);
#endif
            NRFX_IRQ_DISABLE(irqn);
        }
    }
    nrf_clock_int_disable(NRF_CLOCK, NRF_CLOCK_INT_HF_STARTED_MASK |
#if NRF_CLOCK_HAS_LFCLK
                                     NRF_CLOCK_INT_LF_STARTED_MASK |
#endif
#if NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED) && (NRF_CLOCK_HAS_CALIBRATION)
                                     NRF_CLOCK_INT_DONE_MASK |
#if NRF_CLOCK_HAS_CALIBRATION_TIMER
                                     NRF_CLOCK_INT_CTTO_MASK |
#endif
#endif // NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
                                     0);
#if NRFX_CHECK(NRFX_POWER_ENABLED)
    nrfx_clock_irq_enabled = false;
#endif
#if NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION) && NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)
    NRFX_IRQ_DISABLE(LFRC_IRQn);
    nrf_lfrc_int_disable(NRF_LFRC, NRF_LFRC_INT_CALDONE_MASK);
#endif
    NRFX_LOG_INFO("Module disabled.");
}

void nrfx_clock_uninit(void)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);

#if NRF_CLOCK_HAS_LFCLK
    nrfx_clock_lfclk_uninit();
#endif
#if NRF_CLOCK_HAS_HFCLK192M
    nrfx_clock_hfclk192m_uninit();
#endif
#if NRF_CLOCK_HAS_HFCLKAUDIO
    nrfx_clock_hfclkaudio_uninit();
#endif
#if NRF_CLOCK_HAS_HFCLK24M
    nrfx_clock_xo24m_uninit();
#endif
#if NRF_CLOCK_HAS_HFCLK
    nrfx_clock_hfclk_uninit();
#endif
#if NRF_CLOCK_HAS_XO
    nrfx_clock_xo_uninit();
#endif
    m_clock_cb.module_initialized = false;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_clock_init_check(void)
{
    return m_clock_cb.module_initialized;
}

void nrfx_clock_start(nrf_clock_domain_t domain)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);
    switch (domain)
    {
#if NRF_CLOCK_HAS_LFCLK
    case NRF_CLOCK_DOMAIN_LFCLK:
        nrfx_clock_lfclk_start();
        return;
#endif
    case NRF_CLOCK_DOMAIN_HFCLK:
#if NRF_CLOCK_HAS_XO
        nrfx_clock_xo_start();
#elif NRF_CLOCK_HAS_HFCLK
        nrfx_clock_hfclk_start();
#endif
        return;
#if NRF_CLOCK_HAS_HFCLK192M
    case NRF_CLOCK_DOMAIN_HFCLK192M:
        nrfx_clock_hfclk192m_start();
        return;
#endif
#if NRF_CLOCK_HAS_HFCLKAUDIO
    case NRF_CLOCK_DOMAIN_HFCLKAUDIO:
        nrfx_clock_hfclkaudio_start();
        return;
#endif
#if NRF_CLOCK_HAS_HFCLK24M
    case NRF_CLOCK_DOMAIN_HFCLK24M:
        nrfx_clock_xo24m_start();
        return;
#endif
    default:
        NRFX_ASSERT(0);
        return;
    }
}

void nrfx_clock_stop(nrf_clock_domain_t domain)
{
    NRFX_ASSERT(m_clock_cb.module_initialized);
    switch (domain)
    {
    case NRF_CLOCK_DOMAIN_HFCLK:
#if NRF_CLOCK_HAS_HFCLK
        nrfx_clock_hfclk_stop();
#elif NRF_CLOCK_HAS_XO
        nrfx_clock_xo_stop();
#endif // NRF_CLOCK_HAS_HFCLK
        break;
#if NRF_CLOCK_HAS_LFCLK
    case NRF_CLOCK_DOMAIN_LFCLK:
        nrfx_clock_lfclk_stop();
        break;
#endif
#if NRF_CLOCK_HAS_HFCLK192M
    case NRF_CLOCK_DOMAIN_HFCLK192M:
        nrfx_clock_hfclk192m_stop();
        break;
#endif // NRF_CLOCK_HAS_HFCLK192M
#if NRF_CLOCK_HAS_HFCLK24M
    case NRF_CLOCK_DOMAIN_HFCLK24M:
        nrfx_clock_xo24m_stop();
        break;
#endif // NRF_CLOCK_HAS_HFCLK24M
#if NRF_CLOCK_HAS_HFCLKAUDIO
    case NRF_CLOCK_DOMAIN_HFCLKAUDIO:
        nrfx_clock_hfclkaudio_stop();
        break;
#endif // NRF_CLOCK_HAS_HFCLKAUDIO
    default:
        NRFX_ASSERT(0);
        break;
    }
}

#if ((NRF_CLOCK_HAS_CALIBRATION || NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION)) && \
     NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED))
int nrfx_clock_calibration_start(void)
{
    return nrfx_clock_lfclk_calibration_start();
}

int nrfx_clock_is_calibrating(void)
{
    return nrfx_clock_lfclk_calibrating_check();
}

#if NRF_CLOCK_HAS_CALIBRATION_TIMER && NRFX_CHECK(NRFX_CLOCK_CONFIG_CT_ENABLED)
void nrfx_clock_calibration_timer_start(uint8_t interval)
{
    nrfx_clock_lfclk_calibration_timer_start(interval);
}

void nrfx_clock_calibration_timer_stop(void)
{
    nrfx_clock_lfclk_calibration_timer_stop();
}
#endif // NRF_CLOCK_HAS_CALIBRATION_TIMER && NRFX_CHECK(NRFX_CLOCK_CONFIG_CT_ENABLED)
#endif /* ((NRF_CLOCK_HAS_CALIBRATION || NRFX_CHECK(NRF_LFRC_HAS_CALIBRATION)) && \
            NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)) */

#if NRF_CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT || NRF_CLOCK_HAS_HFCLK192M
int nrfx_clock_divider_set(nrf_clock_domain_t domain,
                                  nrf_clock_hfclk_div_t div)
{
    switch (domain)
    {
#if NRF_CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT
        case NRF_CLOCK_DOMAIN_HFCLK:
            nrfx_clock_hfclk_divider_set(div);
            return 0;
#endif // NRF_CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT
#if NRF_CLOCK_HAS_HFCLK192M
        case NRF_CLOCK_DOMAIN_HFCLK192M:
            return nrfx_clock_hfclk192m_divider_set(div);
#endif
        default:
            NRFX_ASSERT(0);
            return -ENOTSUP;
    }
}
#endif

void nrfx_clock_irq_handler(void)
{
#if NRF_CLOCK_HAS_HFCLK
    nrfx_clock_hfclk_irq_handler();
#endif

#if NRF_CLOCK_HAS_XO
    nrfx_clock_xo_irq_handler();
#endif

#if NRF_CLOCK_HAS_LFCLK
    nrfx_clock_lfclk_irq_handler();
#endif

#if NRF_CLOCK_HAS_HFCLK192M
    nrfx_clock_hfclk192m_irq_handler();
#endif

#if NRF_CLOCK_HAS_HFCLK24M
    nrfx_clock_xo24m_irq_handler();
#endif

#if NRF_CLOCK_HAS_HFCLKAUDIO
    nrfx_clock_hfclkaudio_irq_handler();
#endif
}
