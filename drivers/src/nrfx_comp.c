/*
 * Copyright (c) 2015 - 2026, Nordic Semiconductor ASA
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
#include <nrfx_comp.h>
#include "nrfx_comp_common.h"
#include "prs/nrfx_prs.h"

#define NRFX_LOG_MODULE COMP
#include <nrfx_log.h>

#if NRFY_COMP_HAS_REFTRIM
#include <hal/nrf_ficr.h>
#endif

#if NRF_COMP_HAS_AIN_AS_PIN
#include <hal/nrf_gpio.h>
#endif

#define EVT_TO_STR(event)                                     \
    (event == NRF_COMP_EVENT_READY ? "NRF_COMP_EVENT_READY" : \
    (event == NRF_COMP_EVENT_DOWN  ? "NRF_COMP_EVENT_DOWN"  : \
    (event == NRF_COMP_EVENT_UP    ? "NRF_COMP_EVENT_UP"    : \
    (event == NRF_COMP_EVENT_CROSS ? "NRF_COMP_EVENT_CROSS" : \
                                     "UNKNOWN ERROR"))))


static nrfx_comp_event_handler_t    m_comp_event_handler = NULL;
static nrfx_drv_state_t             m_state = NRFX_DRV_STATE_UNINITIALIZED;

static int comp_input_convert(nrfx_analog_input_t analog_input,
                              nrf_comp_input_t *  p_comp_input)
{
    NRFX_ASSERT(p_comp_input);

    nrf_comp_input_t comp_input = nrfx_comp_ain_get(analog_input);

    if (comp_input == (nrf_comp_input_t)NRFX_COMP_INPUT_NOT_PRESENT)
    {
        return -EINVAL;
    }

#if (NRF_COMP_HAS_AIN_AS_PIN && NRF_GPIO_HAS_RETENTION_SETCLEAR)
    nrf_gpio_pin_retain_disable(comp_input);
#endif

    *p_comp_input = comp_input;
    return 0;
}

static int comp_ext_ref_input_convert(nrfx_analog_input_t  analog_input,
                                      nrf_comp_ext_ref_t * p_ext_ref_input)
{
    NRFX_ASSERT(p_ext_ref_input);

    nrf_comp_ext_ref_t ext_ref_input = nrfx_comp_ext_ref_get(analog_input);

    if (ext_ref_input == (nrf_comp_ext_ref_t)NRFX_COMP_INPUT_NOT_PRESENT)
    {
        return -EINVAL;
    }

    *p_ext_ref_input = ext_ref_input;
    return 0;
}

static int comp_configure(nrfx_comp_config_t const * p_config)
{
    int err_code;

    nrfy_comp_config_t nrfy_config =
    {
        .reference  = p_config->reference,
        .main_mode  = p_config->main_mode,
        .threshold  = p_config->threshold,
        .speed_mode = p_config->speed_mode,
        .hyst       = p_config->hyst,
        NRFX_COND_CODE_1(NRF_COMP_HAS_ISOURCE, (.isource = p_config->isource,), ())
    };

    err_code = comp_input_convert(p_config->input, &nrfy_config.input);
    if (err_code != 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    err_code = comp_ext_ref_input_convert(p_config->ext_ref, &nrfy_config.ext_ref);
    if (err_code != 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    nrfy_comp_periph_configure(NRF_COMP, &nrfy_config);
    nrfy_comp_int_init(NRF_COMP,
                       NRF_COMP_INT_READY_MASK |
                       NRF_COMP_INT_DOWN_MASK |
                       NRF_COMP_INT_UP_MASK |
                       NRF_COMP_INT_CROSS_MASK,
                       p_config->interrupt_priority,
                       false);
#if NRFY_COMP_HAS_REFTRIM && NRF_FICR_HAS_GLOBAL_COMP_REFTRIM
    uint32_t trim = nrf_ficr_global_comp_reftrim_get(NRF_FICR);
    nrfy_comp_reftrim_set(NRF_COMP, trim);
#endif

    return 0;
}

static void comp_execute_handler(nrf_comp_event_t event, uint32_t event_mask)
{
    if (event_mask & nrfy_comp_int_enable_check(NRF_COMP, NRFY_EVENT_TO_INT_BITMASK(event)))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(event));

        m_comp_event_handler(event);
    }
}

void nrfx_comp_irq_handler(void)
{
    uint32_t evt_mask = nrfy_comp_events_process(NRF_COMP,
                                                 NRF_COMP_INT_READY_MASK |
                                                 NRF_COMP_INT_DOWN_MASK |
                                                 NRF_COMP_INT_UP_MASK |
                                                 NRF_COMP_INT_CROSS_MASK);
    comp_execute_handler(NRF_COMP_EVENT_READY, evt_mask);
    comp_execute_handler(NRF_COMP_EVENT_DOWN,  evt_mask);
    comp_execute_handler(NRF_COMP_EVENT_UP,    evt_mask);
    comp_execute_handler(NRF_COMP_EVENT_CROSS, evt_mask);
}


int nrfx_comp_init(nrfx_comp_config_t const * p_config,
                   nrfx_comp_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(event_handler);
    int err_code;

    if (m_state != NRFX_DRV_STATE_UNINITIALIZED)
    { // COMP driver is already initialized
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    m_comp_event_handler = event_handler;

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    err_code = nrfx_prs_acquire(NRF_COMP, (nrfx_irq_handler_t)nrfx_comp_irq_handler, NULL);
    if (err_code < 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif
    nrfy_comp_shorts_disable(NRF_COMP,
                             NRF_COMP_SHORT_STOP_CROSS_MASK |
                             NRF_COMP_SHORT_STOP_UP_MASK |
                             NRF_COMP_SHORT_STOP_DOWN_MASK);
    nrfy_comp_int_disable(NRF_COMP,
                          NRF_COMP_INT_READY_MASK |
                          NRF_COMP_INT_DOWN_MASK |
                          NRF_COMP_INT_UP_MASK |
                          NRF_COMP_INT_CROSS_MASK);

    if (p_config)
    {
        err_code = comp_configure(p_config);
        if (err_code != 0)
        {
            return err_code;
        }
    }

    nrfy_comp_enable(NRF_COMP);
    nrfy_comp_task_trigger(NRF_COMP, NRF_COMP_TASK_STOP);

    m_state = NRFX_DRV_STATE_INITIALIZED;

    err_code = 0;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

int nrfx_comp_reconfigure(nrfx_comp_config_t const * p_config)
{
    NRFX_ASSERT(p_config);
    int err_code;

    if (m_state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EINPROGRESS;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;

    }
    else if (m_state == NRFX_DRV_STATE_POWERED_ON)
    {
        err_code = -EBUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    nrfy_comp_disable(NRF_COMP);
    err_code = comp_configure(p_config);
    if (err_code != 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    nrfy_comp_enable(NRF_COMP);
    return 0;
}

void nrfx_comp_uninit(void)
{
    NRFX_ASSERT(m_state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_comp_shorts_disable(NRF_COMP, UINT32_MAX);
    nrfy_comp_int_disable(NRF_COMP, UINT32_MAX);
    nrfy_comp_task_trigger(NRF_COMP, NRF_COMP_TASK_STOP);
    nrfy_comp_int_uninit(NRF_COMP);
    nrfy_comp_disable(NRF_COMP);
#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(NRF_COMP);
#endif
    m_state = NRFX_DRV_STATE_UNINITIALIZED;
    m_comp_event_handler = NULL;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_comp_init_check(void)
{
    return (m_state != NRFX_DRV_STATE_UNINITIALIZED);
}

int nrfx_comp_pin_select(nrfx_analog_input_t psel)
{
    NRFX_ASSERT(m_state != NRFX_DRV_STATE_UNINITIALIZED);

    int err = 0;
    bool comp_enable_state = nrfy_comp_enable_check(NRF_COMP);
    nrf_comp_input_t comp_input;

    nrfy_comp_task_trigger(NRF_COMP, NRF_COMP_TASK_STOP);
    if (m_state == NRFX_DRV_STATE_POWERED_ON)
    {
        m_state = NRFX_DRV_STATE_INITIALIZED;
    }
    nrfy_comp_disable(NRF_COMP);

    err = comp_input_convert(psel, &comp_input);
    if (err != 0)
    {
        return err;
    }
    nrfy_comp_input_select(NRF_COMP, comp_input);
    if (comp_enable_state == true)
    {
        nrfy_comp_enable(NRF_COMP);
    }

    return err;
}

void nrfx_comp_start(uint32_t comp_int_mask, uint32_t comp_shorts_mask)
{
    NRFX_ASSERT(m_state == NRFX_DRV_STATE_INITIALIZED);

    (void)nrfy_comp_events_process(NRF_COMP, comp_int_mask);
    nrfy_comp_int_enable(NRF_COMP, comp_int_mask);
    nrfy_comp_shorts_enable(NRF_COMP, comp_shorts_mask);
    m_state = NRFX_DRV_STATE_POWERED_ON;
    nrfy_comp_task_trigger(NRF_COMP, NRF_COMP_TASK_START);
    NRFX_LOG_INFO("Enabled.");
}

void nrfx_comp_stop(void)
{
    NRFX_ASSERT(m_state == NRFX_DRV_STATE_POWERED_ON);

    nrfy_comp_shorts_disable(NRF_COMP, UINT32_MAX);
    nrfy_comp_int_disable(NRF_COMP, UINT32_MAX);
    nrfy_comp_task_trigger(NRF_COMP, NRF_COMP_TASK_STOP);
    m_state = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("Disabled.");
}

uint32_t nrfx_comp_sample()
{
    NRFX_ASSERT(m_state == NRFX_DRV_STATE_POWERED_ON);

    return nrfy_comp_sample(NRF_COMP);
}
