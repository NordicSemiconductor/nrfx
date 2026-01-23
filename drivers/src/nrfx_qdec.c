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
#include <nrfx_qdec.h>

#include <haly/nrfy_gpio.h>

#define NRFX_LOG_MODULE QDEC
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                             \
    (event == NRF_QDEC_EVENT_SAMPLERDY ? "NRF_QDEC_EVENT_SAMPLERDY" : \
    (event == NRF_QDEC_EVENT_REPORTRDY ? "NRF_QDEC_EVENT_REPORTRDY" : \
    (event == NRF_QDEC_EVENT_ACCOF     ? "NRF_QDEC_EVENT_ACCOF"     : \
                                         "UNKNOWN EVENT")))

static void qdec_configure(nrfx_qdec_t const *        p_instance,
                           nrfx_qdec_config_t const * p_config)
{
    if (!p_config->skip_gpio_cfg)
    {
        nrfy_gpio_cfg_input(p_config->psela, NRF_GPIO_PIN_NOPULL);
        nrfy_gpio_cfg_input(p_config->pselb, NRF_GPIO_PIN_NOPULL);
        if (p_config->pselled != NRF_QDEC_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_input(p_config->pselled, NRF_GPIO_PIN_NOPULL);
        }
    }

    nrfy_qdec_config_t nrfy_config =
    {
        .reportper = p_config->reportper,
        .sampleper = p_config->sampleper,
        .pins = {
            .a_pin   = p_config->psela,
            .b_pin   = p_config->pselb,
            .led_pin = p_config->pselled
        },
        .ledpre    = p_config->ledpre,
        .ledpol    = p_config->ledpol,
        .dbfen     = p_config->dbfen,
        .skip_psel_cfg = p_config->skip_psel_cfg
    };

    nrfy_qdec_periph_configure(p_instance->p_reg, &nrfy_config);

    uint32_t int_mask = NRF_QDEC_INT_ACCOF_MASK;

    if (p_config->reportper_inten)
    {
        int_mask |= NRF_QDEC_INT_REPORTRDY_MASK;
        nrfy_qdec_shorts_enable(p_instance->p_reg, NRF_QDEC_SHORT_REPORTRDY_READCLRACC_MASK);
    }
    if (p_config->sample_inten)
    {
        int_mask |= NRF_QDEC_INT_SAMPLERDY_MASK;
    }
    nrfy_qdec_int_init(p_instance->p_reg, int_mask, p_config->interrupt_priority, true);
}

int nrfx_qdec_init(nrfx_qdec_t *              p_instance,
                   nrfx_qdec_config_t const * p_config,
                   nrfx_qdec_event_handler_t  handler,
                   void *                     p_context)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(handler);

    nrfx_qdec_control_block_t * const p_cb = &p_instance->cb;

    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    p_cb->handler = handler;
    p_cb->p_context = p_context;

    if (p_config)
    {
        p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
        qdec_configure(p_instance, p_config);
    }

    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    err_code = 0;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

int nrfx_qdec_reconfigure(nrfx_qdec_t *              p_instance,
                          nrfx_qdec_config_t const * p_config)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);
    nrfx_qdec_control_block_t const * p_cb = &p_instance->cb;

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return -EINPROGRESS;
    }
    if (p_cb->state == NRFX_DRV_STATE_POWERED_ON)
    {
        return -EALREADY;
    }
    qdec_configure(p_instance, p_config);
    nrfy_qdec_enable(p_instance->p_reg);
    return 0;
}

void nrfx_qdec_uninit(nrfx_qdec_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_qdec_control_block_t * const p_cb = &p_instance->cb;
    nrfy_qdec_pins_t pins;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_qdec_disable(p_instance->p_reg);
    nrfy_qdec_int_disable(p_instance->p_reg, 0xFFFFFFFF);
    nrfy_qdec_int_uninit(p_instance->p_reg);

    nrfy_qdec_shorts_disable(p_instance->p_reg, NRF_QDEC_SHORT_REPORTRDY_READCLRACC_MASK);
    if (!p_cb->skip_gpio_cfg)
    {
        nrfy_qdec_pins_get(p_instance->p_reg, &pins);
        nrfy_gpio_cfg_default(pins.a_pin);
        nrfy_gpio_cfg_default(pins.b_pin);

        uint32_t led_pin = nrfy_qdec_led_pin_get(p_instance->p_reg);
        if (led_pin != NRF_QDEC_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_default(led_pin);
        }
    }

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_qdec_init_check(nrfx_qdec_t const * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_qdec_control_block_t const * p_cb = &p_instance->cb;

    return (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_qdec_enable(nrfx_qdec_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_qdec_control_block_t * const p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_qdec_enable(p_instance->p_reg);
    nrfy_qdec_task_trigger(p_instance->p_reg, NRF_QDEC_TASK_START);
    p_cb->state = NRFX_DRV_STATE_POWERED_ON;
    NRFX_LOG_INFO("Enabled.");
}

void nrfx_qdec_disable(nrfx_qdec_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_qdec_control_block_t * const p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_POWERED_ON);
    nrfy_qdec_task_trigger(p_instance->p_reg, NRF_QDEC_TASK_STOP);
    nrfy_qdec_disable(p_instance->p_reg);
    p_cb->state = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("Disabled.");
}

void nrfx_qdec_accumulators_read(nrfx_qdec_t const * p_instance,
                                 int32_t *           p_acc,
                                 uint32_t *          p_accdbl)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_accdbl);
    NRFX_ASSERT(p_acc);
    NRFX_ASSERT(p_instance->cb.state == NRFX_DRV_STATE_POWERED_ON);

    nrfy_qdec_task_trigger(p_instance->p_reg, NRF_QDEC_TASK_READCLRACC);
    nrfy_qdec_accumulators_read(p_instance->p_reg, p_acc, p_accdbl);

    NRFX_LOG_DEBUG("Accumulators data, ACC register:");
    NRFX_LOG_HEXDUMP_DEBUG((uint8_t *)p_acc, sizeof(p_acc[0]));
    NRFX_LOG_DEBUG("Accumulators data, ACCDBL register:");
    NRFX_LOG_HEXDUMP_DEBUG((uint8_t *)p_accdbl, sizeof(p_accdbl[0]));
}

void nrfx_qdec_irq_handler(nrfx_qdec_t * p_instance)
{
    uint32_t evt_to_process;
    nrfx_qdec_event_t event;
    uint32_t evt_mask;
    uint32_t all_evt_mask;

    nrfx_qdec_control_block_t const * p_cb = &p_instance->cb;

    all_evt_mask = NRFY_EVENT_TO_INT_BITMASK(NRF_QDEC_EVENT_SAMPLERDY) |
                   NRFY_EVENT_TO_INT_BITMASK(NRF_QDEC_EVENT_REPORTRDY) |
                   NRFY_EVENT_TO_INT_BITMASK(NRF_QDEC_EVENT_ACCOF);

    evt_to_process = nrfy_qdec_int_enable_check(p_instance->p_reg, all_evt_mask);
    evt_mask = nrfy_qdec_events_process(p_instance->p_reg, evt_to_process);

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_QDEC_EVENT_SAMPLERDY))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRF_QDEC_EVENT_SAMPLERDY));

        event.type = NRF_QDEC_EVENT_SAMPLERDY;
        event.data.sample.value = (int8_t)nrfy_qdec_sample_get(p_instance->p_reg);
        p_cb->handler(event, p_cb->p_context);
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_QDEC_EVENT_REPORTRDY))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRF_QDEC_EVENT_REPORTRDY));

        event.type = NRF_QDEC_EVENT_REPORTRDY;
        nrfy_qdec_accumulators_read(p_instance->p_reg, &event.data.report.acc, &event.data.report.accdbl);
        p_cb->handler(event, p_cb->p_context);
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_QDEC_EVENT_ACCOF))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRF_QDEC_EVENT_ACCOF));

        event.type = NRF_QDEC_EVENT_ACCOF;
        p_cb->handler(event, p_cb->p_context);
    }
}

