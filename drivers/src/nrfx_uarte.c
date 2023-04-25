/*
 * Copyright (c) 2015 - 2023, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_UARTE_ENABLED)

#if !NRFX_FEATURE_PRESENT(NRFX_UARTE, _ENABLED)
#error "No enabled UARTE instances. Check <nrfx_config.h>."
#endif

#include <nrfx_uarte.h>
#include "prs/nrfx_prs.h"
#include <haly/nrfy_gpio.h>

#define NRFX_LOG_MODULE UARTE
#include <nrfx_log.h>

#define EVT_TO_STR(event) \
    (event == NRF_UARTE_EVENT_ERROR ? "NRF_UARTE_EVENT_ERROR" : \
                                      "UNKNOWN EVENT")

#define UARTEX_LENGTH_VALIDATE(periph_name, prefix, i, drv_inst_idx, len1, len2) \
    (((drv_inst_idx) == NRFX_CONCAT(NRFX_, periph_name, prefix, i, _INST_IDX)) && \
     NRFX_EASYDMA_LENGTH_VALIDATE(NRFX_CONCAT(periph_name, prefix, i), len1, len2))

#define UARTE_LENGTH_VALIDATE(drv_inst_idx, len)    \
        (NRFX_FOREACH_ENABLED(UARTE, UARTEX_LENGTH_VALIDATE, (||), (0), drv_inst_idx, len, 0))

typedef struct
{
    void                     * p_context;
    nrfx_uarte_event_handler_t handler;
    uint8_t            const * p_tx_buffer;
    uint8_t                  * p_rx_buffer;
    uint8_t                  * p_rx_secondary_buffer;
    volatile size_t            tx_buffer_length;
    size_t                     rx_buffer_length;
    size_t                     rx_secondary_buffer_length;
    nrfx_drv_state_t           state;
    bool                       rx_aborted;
    bool                       skip_gpio_cfg : 1;
    bool                       skip_psel_cfg : 1;
} uarte_control_block_t;
static uarte_control_block_t m_cb[NRFX_UARTE_ENABLED_COUNT];

static void apply_workaround_for_enable_anomaly(nrfx_uarte_t const * p_instance);

static void uarte_configure(nrfx_uarte_t        const * p_instance,
                            nrfx_uarte_config_t const * p_config)
{
    if (!p_config->skip_gpio_cfg)
    {
        if (p_config->txd_pin != NRF_UARTE_PSEL_DISCONNECTED)
        {
            nrfy_gpio_pin_set(p_config->txd_pin);
            nrfy_gpio_cfg_output(p_config->txd_pin);
        }
        if (p_config->rxd_pin != NRF_UARTE_PSEL_DISCONNECTED)
        {
            nrfy_gpio_cfg_input(p_config->rxd_pin, NRF_GPIO_PIN_NOPULL);
        }
    }

    if (p_config->config.hwfc == NRF_UARTE_HWFC_ENABLED)
    {
        if (!p_config->skip_gpio_cfg)
        {
            if (p_config->cts_pin != NRF_UARTE_PSEL_DISCONNECTED)
            {
                nrfy_gpio_cfg_input(p_config->cts_pin, NRF_GPIO_PIN_NOPULL);
            }
            if (p_config->rts_pin != NRF_UARTE_PSEL_DISCONNECTED)
            {
                nrfy_gpio_pin_set(p_config->rts_pin);
                nrfy_gpio_cfg_output(p_config->rts_pin);
#if NRF_GPIO_HAS_CLOCKPIN
                nrfy_gpio_pin_clock_set(p_config->rts_pin, true);
#endif
            }
        }
    }

    nrfy_uarte_config_t nrfy_config =
    {
        .pins =
        {
            .txd_pin = p_config->txd_pin,
            .rxd_pin = p_config->rxd_pin,
            .rts_pin = p_config->rts_pin,
            .cts_pin = p_config->cts_pin
        },
        .baudrate = p_config->baudrate,
        .skip_psel_cfg = p_config->skip_psel_cfg
    };
    nrfy_config.config = p_config->config;

    nrfy_uarte_periph_configure(p_instance->p_reg, &nrfy_config);

    apply_workaround_for_enable_anomaly(p_instance);

    if (m_cb[p_instance->drv_inst_idx].handler)
    {
        nrfy_uarte_int_init(p_instance->p_reg,
                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDRX) |
                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDTX) |
                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ERROR) |
                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXTO)  |
                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTOPPED),
                            p_config->interrupt_priority,
                            true);
    }
}

static void pins_to_default(nrfx_uarte_t const * p_instance)
{
    uarte_control_block_t const * p_cb = &m_cb[p_instance->drv_inst_idx];

    /* Reset pins to default states */
    nrfy_uarte_pins_t pins;
    nrfy_uarte_pins_get(p_instance->p_reg, &pins);
    if (!p_cb->skip_psel_cfg)
    {
        nrfy_uarte_pins_disconnect(p_instance->p_reg);
    }
    if (!p_cb->skip_gpio_cfg)
    {
        if (pins.txd_pin != NRF_UARTE_PSEL_DISCONNECTED)
        {
            nrfy_gpio_cfg_default(pins.txd_pin);
        }
        if (pins.rxd_pin != NRF_UARTE_PSEL_DISCONNECTED)
        {
            nrfy_gpio_cfg_default(pins.rxd_pin);
        }
        if (pins.cts_pin != NRF_UARTE_PSEL_DISCONNECTED)
        {
            nrfy_gpio_cfg_default(pins.cts_pin);
        }
        if (pins.rts_pin != NRF_UARTE_PSEL_DISCONNECTED)
        {
            nrfy_gpio_cfg_default(pins.rts_pin);
        }
    }
}

static void apply_workaround_for_enable_anomaly(nrfx_uarte_t const * p_instance)
{
#if defined(NRF53_SERIES) || defined(NRF91_SERIES)
    // Apply workaround for anomalies:
    // - nRF91 - anomaly 23
    // - nRF53 - anomaly 44
    volatile uint32_t const * rxenable_reg =
        (volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x564);
    volatile uint32_t const * txenable_reg =
        (volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x568);

    if (*txenable_reg == 1)
    {
        nrfy_uarte_task_trigger(p_instance->p_reg, NRF_UARTE_TASK_STOPTX);
    }

    if (*rxenable_reg == 1)
    {
        nrfy_uarte_enable(p_instance->p_reg);
        nrfy_uarte_task_trigger(p_instance->p_reg, NRF_UARTE_TASK_STOPRX);

        bool workaround_succeded;
        // The UARTE is able to receive up to four bytes after the STOPRX task has been triggered.
        // On lowest supported baud rate (1200 baud), with parity bit and two stop bits configured
        // (resulting in 12 bits per data byte sent), this may take up to 40 ms.
        NRFX_WAIT_FOR(*rxenable_reg == 0, 40000, 1, workaround_succeded);
        if (!workaround_succeded)
        {
            NRFX_LOG_ERROR("Failed to apply workaround for instance with base address: %p.",
                           (void *)p_instance->p_reg);
        }

        (void)nrfy_uarte_errorsrc_get_and_clear(p_instance->p_reg);
        nrfy_uarte_disable(p_instance->p_reg);
    }
#else
    (void)(p_instance);
#endif // defined(NRF53_SERIES) || defined(NRF91_SERIES)
}

nrfx_err_t nrfx_uarte_init(nrfx_uarte_t const *        p_instance,
                           nrfx_uarte_config_t const * p_config,
                           nrfx_uarte_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_config);
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    static nrfx_irq_handler_t const irq_handlers[NRFX_UARTE_ENABLED_COUNT] = {
        NRFX_INSTANCE_IRQ_HANDLERS_LIST(UARTE, uarte)
    };
    if (nrfx_prs_acquire(p_instance->p_reg,
            irq_handlers[p_instance->drv_inst_idx]) != NRFX_SUCCESS)
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED)

    p_cb->handler = event_handler;

    if (p_config)
    {
        p_cb->p_context = p_config->p_context;
        p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
        p_cb->skip_psel_cfg = p_config->skip_psel_cfg;
        uarte_configure(p_instance, p_config);
    }

    nrfy_uarte_enable(p_instance->p_reg);
    p_cb->rx_buffer_length           = 0;
    p_cb->rx_secondary_buffer_length = 0;
    p_cb->tx_buffer_length           = 0;
    p_cb->state                      = NRFX_DRV_STATE_INITIALIZED;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_uarte_reconfigure(nrfx_uarte_t const *        p_instance,
                                  nrfx_uarte_config_t const * p_config)
{
    NRFX_ASSERT(p_config);
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }
    if (nrfx_uarte_tx_in_progress(p_instance))
    {
        return NRFX_ERROR_BUSY;
    }
    nrfy_uarte_disable(p_instance->p_reg);
    if (p_cb->handler)
    {
        p_cb->p_context = p_config->p_context;
    }
    uarte_configure(p_instance, p_config);
    nrfy_uarte_enable(p_instance->p_reg);
    return NRFX_SUCCESS;
}

void nrfx_uarte_uninit(nrfx_uarte_t const * p_instance)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if (p_cb->handler)
    {
        nrfy_uarte_int_disable(p_instance->p_reg,
                               NRF_UARTE_INT_ENDRX_MASK |
                               NRF_UARTE_INT_ENDTX_MASK |
                               NRF_UARTE_INT_ERROR_MASK |
                               NRF_UARTE_INT_RXTO_MASK  |
                               NRF_UARTE_INT_TXSTOPPED_MASK);
        nrfy_uarte_int_uninit(p_instance->p_reg);
    }
    // Make sure all transfers are finished before UARTE is disabled
    // to achieve the lowest power consumption.
    nrfy_uarte_shorts_disable(p_instance->p_reg, NRF_UARTE_SHORT_ENDRX_STARTRX);

    nrfy_uarte_xfer_desc_t xfer_desc = {
        .p_buffer = p_cb->p_rx_buffer,
        .length   = p_cb->rx_buffer_length
    };
    nrfy_uarte_stop(p_instance->p_reg, &xfer_desc);

    nrfy_uarte_disable(p_instance->p_reg);
    pins_to_default(p_instance);

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(p_instance->p_reg);
#endif

    p_cb->state   = NRFX_DRV_STATE_UNINITIALIZED;
    p_cb->handler = NULL;
    NRFX_LOG_INFO("Instance uninitialized: %d.", p_instance->drv_inst_idx);
}

nrfx_err_t nrfx_uarte_tx(nrfx_uarte_t const * p_instance,
                         uint8_t const *      p_data,
                         size_t               length,
                         uint32_t             flags)
{
    (void)flags;
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_data);
    NRFX_ASSERT(length > 0);
    NRFX_ASSERT(UARTE_LENGTH_VALIDATE(p_instance->drv_inst_idx, length));

    nrfx_err_t err_code = NRFX_SUCCESS;

    // EasyDMA requires that transfer buffers are placed in DataRAM,
    // signal error if the are not.
    if (!nrf_dma_accessible_check(p_instance->p_reg, p_data))
    {
        err_code = NRFX_ERROR_INVALID_ADDR;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (nrfx_uarte_tx_in_progress(p_instance))
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    p_cb->tx_buffer_length = length;
    p_cb->p_tx_buffer      = p_data;

    NRFX_LOG_INFO("Transfer tx_len: %d.", p_cb->tx_buffer_length);
    NRFX_LOG_DEBUG("Tx data:");
    NRFX_LOG_HEXDUMP_DEBUG(p_cb->p_tx_buffer,
                           p_cb->tx_buffer_length * sizeof(p_cb->p_tx_buffer[0]));

    nrfy_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_ENDTX);
    nrfy_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_TXSTOPPED);
    nrfy_uarte_tx_buffer_set(p_instance->p_reg, p_cb->p_tx_buffer, p_cb->tx_buffer_length);

    uint32_t evt_mask = nrfy_uarte_tx_start(p_instance->p_reg, !p_cb->handler);
    if (p_cb->handler == NULL)
    {
        if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTOPPED))
        {
            err_code = NRFX_ERROR_FORBIDDEN;
        }
        else
        {
            // Transmitter has to be stopped by triggering the STOPTX task to achieve
            // the lowest possible level of the UARTE power consumption.
            nrfy_uarte_stop(p_instance->p_reg, NULL);
        }
        p_cb->tx_buffer_length = 0;
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

bool nrfx_uarte_tx_in_progress(nrfx_uarte_t const * p_instance)
{
    return (m_cb[p_instance->drv_inst_idx].tx_buffer_length != 0);
}

nrfx_err_t nrfx_uarte_rx(nrfx_uarte_t const * p_instance,
                         uint8_t *            p_data,
                         size_t               length)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_data);
    NRFX_ASSERT(length > 0);
    NRFX_ASSERT(UARTE_LENGTH_VALIDATE(p_instance->drv_inst_idx, length));

    nrfx_err_t err_code = NRFX_SUCCESS;

    // EasyDMA requires that transfer buffers are placed in DataRAM,
    // signal error if the are not.
    if (!nrf_dma_accessible_check(p_instance->p_reg, p_data))
    {
        err_code = NRFX_ERROR_INVALID_ADDR;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    bool second_buffer = false;

    if (p_cb->handler)
    {
        nrfy_uarte_int_disable(p_instance->p_reg, NRF_UARTE_INT_ERROR_MASK |
                                                  NRF_UARTE_INT_ENDRX_MASK);
    }
    if (p_cb->rx_buffer_length != 0)
    {
        if (p_cb->rx_secondary_buffer_length != 0)
        {
            if (p_cb->handler)
            {
                nrfy_uarte_int_enable(p_instance->p_reg, NRF_UARTE_INT_ERROR_MASK |
                                                         NRF_UARTE_INT_ENDRX_MASK);
            }
            err_code = NRFX_ERROR_BUSY;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
        second_buffer = true;
    }

    if (!second_buffer)
    {
        p_cb->rx_buffer_length = length;
        p_cb->p_rx_buffer      = p_data;
        p_cb->rx_secondary_buffer_length = 0;
    }
    else
    {
        p_cb->p_rx_secondary_buffer = p_data;
        p_cb->rx_secondary_buffer_length = length;
    }

    NRFX_LOG_INFO("Transfer rx_len: %d.", length);

    nrfy_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_ENDRX);
    nrfy_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_RXTO);
    nrfy_uarte_rx_buffer_set(p_instance->p_reg, p_data, length);
    uint32_t evt_mask = 0;
    if (second_buffer)
    {
        nrfy_uarte_shorts_enable(p_instance->p_reg, NRF_UARTE_SHORT_ENDRX_STARTRX);
    }
    else
    {
        nrfy_uarte_xfer_desc_t xfer_desc = {
            .p_buffer = p_cb->p_rx_buffer,
            .length   = p_cb->rx_buffer_length
        };

        evt_mask = nrfy_uarte_rx_start(p_instance->p_reg, !p_cb->handler ? &xfer_desc : NULL);
    }

    if (p_cb->handler == NULL)
    {
        p_cb->rx_buffer_length = 0;

        if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ERROR))
        {
            err_code = NRFX_ERROR_INTERNAL;
        }

        if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXTO))
        {
            err_code = NRFX_ERROR_FORBIDDEN;
        }
    }
    else
    {
        p_cb->rx_aborted = false;
        nrfy_uarte_int_enable(p_instance->p_reg, NRF_UARTE_INT_ERROR_MASK |
                                                 NRF_UARTE_INT_ENDRX_MASK);
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_uarte_rx_ready(nrfx_uarte_t const * p_instance, size_t * p_rx_amount)
{
    (void)p_rx_amount;

    return nrfy_uarte_event_check(p_instance->p_reg, NRF_UARTE_EVENT_ENDRX) ?
            NRFX_SUCCESS : NRFX_ERROR_BUSY;
}

uint32_t nrfx_uarte_errorsrc_get(nrfx_uarte_t const * p_instance)
{
    nrfy_uarte_event_clear(p_instance->p_reg, NRF_UARTE_EVENT_ERROR);
    return nrfy_uarte_errorsrc_get_and_clear(p_instance->p_reg);
}

static void rx_done_event(uarte_control_block_t * p_cb,
                          size_t                  bytes,
                          uint8_t *               p_data)
{
    nrfx_uarte_event_t event;
    event.type             = NRFX_UARTE_EVT_RX_DONE;
    event.data.rx.bytes  = bytes;
    event.data.rx.p_data = p_data;

    p_cb->handler(&event, p_cb->p_context);
}

static void tx_done_event(uarte_control_block_t * p_cb,
                          size_t                  bytes)
{
    nrfx_uarte_event_t event;
    event.type             = NRFX_UARTE_EVT_TX_DONE;
    event.data.tx.bytes  = bytes;
    event.data.tx.p_data = (uint8_t *)p_cb->p_tx_buffer;

    p_cb->tx_buffer_length = 0;
    p_cb->handler(&event, p_cb->p_context);
}

nrfx_err_t nrfx_uarte_tx_abort(nrfx_uarte_t const * p_instance, bool sync)
{
    (void)sync;
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfy_uarte_tx_abort(p_instance->p_reg, !p_cb->handler ? true : false);
    NRFX_LOG_INFO("TX transaction aborted.");

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_uarte_rx_abort(nrfx_uarte_t const * p_instance, bool disable_all, bool sync)
{
    (void)disable_all;
    (void)sync;
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    // Short between ENDRX event and STARTRX task must be disabled before
    // aborting transmission.
    if (p_cb->rx_secondary_buffer_length != 0)
    {
        nrfy_uarte_shorts_disable(p_instance->p_reg, NRF_UARTE_SHORT_ENDRX_STARTRX);
    }
    p_cb->rx_aborted = true;
    nrfy_uarte_task_trigger(p_instance->p_reg, NRF_UARTE_TASK_STOPRX);
    NRFX_LOG_INFO("RX transaction aborted.");

    return NRFX_SUCCESS;
}

static void irq_handler(NRF_UARTE_Type * p_reg, uarte_control_block_t * p_cb)
{
    nrfy_uarte_xfer_desc_t xfer_desc = {
        .p_buffer = p_cb->p_rx_buffer,
        .length   = p_cb->rx_buffer_length
    };
    uint32_t evt_mask = nrfy_uarte_events_process(p_reg,
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ERROR) |
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDRX) |
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDTX) |
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXTO)  |
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTOPPED),
                                            &xfer_desc);

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ERROR))
    {
        nrfx_uarte_event_t event;
        event.type                   = NRFX_UARTE_EVT_ERROR;
        event.data.error.error_mask  = nrfy_uarte_errorsrc_get_and_clear(p_reg);
        event.data.error.rx.bytes  = nrfy_uarte_rx_amount_get(p_reg);
        event.data.error.rx.p_data = p_cb->p_rx_buffer;

        // Abort transfer.
        p_cb->rx_buffer_length = 0;
        p_cb->rx_secondary_buffer_length = 0;

        p_cb->handler(&event, p_cb->p_context);
    }
    else if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDRX))
    {
        // Aborted transfers are handled in RXTO event processing.
        if (!p_cb->rx_aborted)
        {
            size_t amount = p_cb->rx_buffer_length;
            if (p_cb->rx_secondary_buffer_length != 0)
            {
                uint8_t * p_data = p_cb->p_rx_buffer;
                nrfy_uarte_shorts_disable(p_reg, NRF_UARTE_SHORT_ENDRX_STARTRX);
                p_cb->rx_buffer_length = p_cb->rx_secondary_buffer_length;
                p_cb->p_rx_buffer = p_cb->p_rx_secondary_buffer;
                p_cb->rx_secondary_buffer_length = 0;
                rx_done_event(p_cb, amount, p_data);
            }
            else
            {
                p_cb->rx_buffer_length = 0;
                rx_done_event(p_cb, amount, p_cb->p_rx_buffer);
            }
        }
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXTO))
    {
        if (p_cb->rx_buffer_length != 0)
        {
            p_cb->rx_buffer_length = 0;
            // In case of using double-buffered reception both variables storing buffer length
            // have to be cleared to prevent incorrect behaviour of the driver.
            p_cb->rx_secondary_buffer_length = 0;
            rx_done_event(p_cb, nrfy_uarte_rx_amount_get(p_reg), p_cb->p_rx_buffer);
        }
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDTX))
    {
        // Transmitter has to be stopped by triggering STOPTX task to achieve
        // the lowest possible level of the UARTE power consumption.
        nrfy_uarte_task_trigger(p_reg, NRF_UARTE_TASK_STOPTX);

        if (p_cb->tx_buffer_length != 0)
        {
            tx_done_event(p_cb, nrfy_uarte_tx_amount_get(p_reg));
        }
    }

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTOPPED))
    {
        if (p_cb->tx_buffer_length != 0)
        {
            tx_done_event(p_cb, nrfy_uarte_tx_amount_get(p_reg));
        }
    }
}

NRFX_INSTANCE_IRQ_HANDLERS(UARTE, uarte)

#endif // NRFX_CHECK(NRFX_UARTE_ENABLED)
