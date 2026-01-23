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

#include <nrfx_twim.h>
#include <haly/nrfy_gpio.h>
#include "prs/nrfx_prs.h"

#define NRFX_LOG_MODULE TWIM
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                       \
    (event == NRFX_TWIM_EVT_DONE         ? "EVT_DONE"         : \
    (event == NRFX_TWIM_EVT_ADDRESS_NACK ? "EVT_ADDRESS_NACK" : \
    (event == NRFX_TWIM_EVT_DATA_NACK    ? "EVT_DATA_NACK"    : \
    (event == NRFX_TWIM_EVT_OVERRUN      ? "EVT_OVERRUN"      : \
    (event == NRFX_TWIM_EVT_BUS_ERROR    ? "EVT_BUS_ERROR"    : \
                                           "UNKNOWN ERROR")))))

#define EVT_TO_STR_TWIM(event)                                        \
    (event == NRF_TWIM_EVENT_STOPPED   ? "NRF_TWIM_EVENT_STOPPED"   : \
    (event == NRF_TWIM_EVENT_ERROR     ? "NRF_TWIM_EVENT_ERROR"     : \
    (event == NRF_TWIM_EVENT_SUSPENDED ? "NRF_TWIM_EVENT_SUSPENDED" : \
    (event == NRF_TWIM_EVENT_RXSTARTED ? "NRF_TWIM_EVENT_RXSTARTED" : \
    (event == NRF_TWIM_EVENT_TXSTARTED ? "NRF_TWIM_EVENT_TXSTARTED" : \
    (event == NRF_TWIM_EVENT_LASTRX    ? "NRF_TWIM_EVENT_LASTRX"    : \
    (event == NRF_TWIM_EVENT_LASTTX    ? "NRF_TWIM_EVENT_LASTTX"    : \
                                         "UNKNOWN ERROR")))))))

#define TRANSFER_TO_STR(type)                    \
    (type == NRFX_TWIM_XFER_TX   ? "XFER_TX"   : \
    (type == NRFX_TWIM_XFER_RX   ? "XFER_RX"   : \
    (type == NRFX_TWIM_XFER_TXRX ? "XFER_TXRX" : \
    (type == NRFX_TWIM_XFER_TXTX ? "XFER_TXTX" : \
                                   "UNKNOWN TRANSFER TYPE"))))

#define TWIM_PIN_INIT(_pin, _drive) nrfy_gpio_cfg((_pin),                     \
                                                  NRF_GPIO_PIN_DIR_INPUT,     \
                                                  NRF_GPIO_PIN_INPUT_CONNECT, \
                                                  NRF_GPIO_PIN_PULLUP,        \
                                                  (_drive),                   \
                                                  NRF_GPIO_PIN_NOSENSE)

static int twi_process_error(uint32_t errorsrc)
{
    int ret = -ECANCELED;

    if (errorsrc & NRF_TWIM_ERROR_OVERRUN)
    {
        ret = -EOVERFLOW;
    }

    if (errorsrc & NRF_TWIM_ERROR_ADDRESS_NACK)
    {
        ret = -EFAULT;
    }

    if (errorsrc & NRF_TWIM_ERROR_DATA_NACK)
    {
        ret = -EAGAIN;
    }

    return ret;
}

static bool xfer_completeness_check(NRF_TWIM_Type * p_twim, nrfx_twim_control_block_t * p_cb)
{
    // If the actual number of transferred bytes is not equal to what was requested,
    // but there was no error signaled by the peripheral, this means that something
    // unexpected, like a premature STOP condition, was received on the bus.
    // In such case the peripheral has to be disabled and re-enabled, so that its
    // internal state machine is reinitialized.

    bool transfer_complete = true;
    switch (p_cb->xfer_type)
    {
        case NRFX_TWIM_XFER_TXTX:
            // int_mask variable is used to determine which length should be checked
            // against number of bytes latched in EasyDMA.
            // NRF_TWIM_INT_SUSPENDED_MASK is configured only in first TX of TXTX transfer.
            if (((p_cb->int_mask & NRF_TWIM_INT_SUSPENDED_MASK) &&
                 (nrfy_twim_txd_amount_get(p_twim) != p_cb->xfer_desc_primary.length)) ||
                (!(p_cb->int_mask & NRF_TWIM_INT_SUSPENDED_MASK) &&
                 (nrfy_twim_txd_amount_get(p_twim) != p_cb->xfer_desc_secondary.length)))
            {
                transfer_complete = false;
            }
            break;
        case NRFX_TWIM_XFER_TXRX:
            if ((nrfy_twim_txd_amount_get(p_twim) != p_cb->xfer_desc_primary.length) ||
                (nrfy_twim_rxd_amount_get(p_twim) != p_cb->xfer_desc_secondary.length))
            {
                transfer_complete = false;
            }
            break;
        case NRFX_TWIM_XFER_TX:
            if (nrfy_twim_txd_amount_get(p_twim) != p_cb->xfer_desc_primary.length)
            {
                transfer_complete = false;
            }
            break;
        case NRFX_TWIM_XFER_RX:
            if (nrfy_twim_rxd_amount_get(p_twim) != p_cb->xfer_desc_primary.length)
            {
                transfer_complete = false;
            }
            break;
        default:
            break;
    }

    if (!transfer_complete)
    {
        nrfy_twim_disable(p_twim);
        nrfy_twim_enable(p_twim);
    }

    return transfer_complete;
}

static void twim_configure(nrfx_twim_t *              p_instance,
                           nrfx_twim_config_t const * p_config)
{
    nrfy_twim_config_t nrfy_config =
    {
        .pins = {
            .scl_pin = p_config->scl_pin,
            .sda_pin = p_config->sda_pin
        },
        .frequency     = p_config->frequency,
        .skip_psel_cfg = p_config->skip_psel_cfg
    };

#if NRF_ERRATA_STATIC_CHECK(52, 219) || NRF_ERRATA_STATIC_CHECK(53, 47)
    if ((NRF_ERRATA_DYNAMIC_CHECK(52, 219) || NRF_ERRATA_DYNAMIC_CHECK(53, 47)) &&
        (p_config->frequency == NRF_TWIM_FREQ_400K))
    {
        nrfy_config.frequency = (nrf_twim_frequency_t) 0x06200000UL; // 390 kbps
    }
#endif

    nrfy_twim_periph_configure(p_instance->p_twim, &nrfy_config);
    if (p_instance->cb.handler)
    {
        nrfy_twim_int_init(p_instance->p_twim, 0, p_config->interrupt_priority, false);
    }
}

static bool pins_configure(nrfx_twim_config_t const * p_config)
{
    nrf_gpio_pin_drive_t pin_drive;

    if (p_config->skip_psel_cfg && p_config->skip_gpio_cfg)
    {
        return true;
    }

#if NRF_TWIM_HAS_1000_KHZ_FREQ && defined(NRF_TWIM_1MBPS_NEEDS_E0E1)
    if (p_config->frequency >= NRF_TWIM_FREQ_1000K)
    {
        /* When using 1 Mbps mode, two high-speed pins have to be used with extra high drive. */
        pin_drive = NRF_GPIO_PIN_E0E1;

        uint32_t e0e1_pin_1 = NRF_GPIO_PIN_MAP(1, 2);
        uint32_t e0e1_pin_2 = NRF_GPIO_PIN_MAP(1, 3);

        /* Check whether provided pins have the extra high drive capabilities. */
        if (((p_config->scl_pin != e0e1_pin_1) ||
             (p_config->sda_pin != e0e1_pin_2)) &&
            ((p_config->scl_pin != e0e1_pin_2) ||
             (p_config->sda_pin != e0e1_pin_1)))
        {
            return false;
        }
    }
    else
#endif
    {
        pin_drive = NRF_GPIO_PIN_S0D1;
    }

    /* To secure correct signal levels on the pins used by the TWI
       master when the system is in OFF mode, and when the TWI master is
       disabled, these pins must be configured in the GPIO peripheral.
    */
   if (!p_config->skip_gpio_cfg)
   {
        NRFX_ASSERT(p_config->scl_pin != p_config->sda_pin);
        TWIM_PIN_INIT(p_config->scl_pin, pin_drive);
        TWIM_PIN_INIT(p_config->sda_pin, pin_drive);
#if NRF_GPIO_HAS_CLOCKPIN
#if defined(NRF_TWIM_CLOCKPIN_SCL_NEEDED)
        nrfy_gpio_pin_clock_set(p_config->scl_pin, true);
#endif
#if defined(NRF_TWIM_CLOCKPIN_SDA_NEEDED)
        nrfy_gpio_pin_clock_set(p_config->sda_pin, true);
#endif
#endif
   }
    return true;
}

int nrfx_twim_init(nrfx_twim_t *              p_instance,
                   nrfx_twim_config_t const * p_config,
                   nrfx_twim_event_handler_t    event_handler,
                   void *                     p_context)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;
    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    err_code = nrfx_prs_acquire(p_instance->p_twim,
                                (nrfx_irq_handler_t)nrfx_twim_irq_handler,p_instance);
    if (err_code < 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED)

    p_cb->handler         = event_handler;
    p_cb->p_context       = p_context;
    p_cb->int_mask        = 0;
    p_cb->repeated        = false;
    p_cb->busy            = false;

    if (p_config)
    {
        p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
        if (!pins_configure(p_config))
        {
#if NRFX_CHECK(NRFX_PRS_ENABLED)
            nrfx_prs_release(p_instance->p_twim);
#endif
            return -EINVAL;
        }

        p_cb->hold_bus_uninit = p_config->hold_bus_uninit;
#if NRF_ERRATA_STATIC_CHECK(52, 109)
        if (NRF_ERRATA_DYNAMIC_CHECK(52, 109))
        {
            p_cb->bus_frequency = (nrf_twim_frequency_t)p_config->frequency;
        }
#endif

        twim_configure(p_instance, p_config);
    }

    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    err_code = 0;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

int nrfx_twim_reconfigure(nrfx_twim_t *              p_instance,
                          nrfx_twim_config_t const * p_config)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return -EINPROGRESS;
    }
    if (p_cb->busy)
    {
        return -EBUSY;
    }

    int err_code = 0;
    nrfy_twim_disable(p_instance->p_twim);
    if (pins_configure(p_config))
    {
        twim_configure(p_instance, p_config);
    }
    else
    {
        err_code = -EINVAL;
    }
    nrfy_twim_enable(p_instance->p_twim);
    return err_code;
}

void nrfx_twim_callback_get(nrfx_twim_t const *         p_instance,
                            nrfx_twim_event_handler_t * p_event_handler,
                            void **                     pp_context)
{
    NRFX_ASSERT(p_instance);

    nrfx_twim_control_block_t const * p_cb = &p_instance->cb;

    *p_event_handler = p_cb->handler;
    *pp_context      = p_cb->p_context;
}

int nrfx_twim_callback_set(nrfx_twim_t *             p_instance,
                           nrfx_twim_event_handler_t event_handler,
                           void *                    p_context)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(event_handler);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->busy)
    {
        return -EBUSY;
    }

    if (p_cb->handler == NULL)
    {
        return -EINPROGRESS;
    }

    nrfy_twim_int_disable(p_instance->p_twim, NRF_TWIM_ALL_INTS_MASK);

    p_cb->handler   = event_handler;
    p_cb->p_context = p_context;

    return 0;
}

void nrfx_twim_uninit(nrfx_twim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_twim_int_uninit(p_instance->p_twim);
    nrfx_twim_disable(p_instance);

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(p_instance->p_twim);
#endif

    if (!p_cb->skip_gpio_cfg && !p_cb->hold_bus_uninit)
    {
        nrfy_twim_pins_t pins;

        nrfy_twim_pins_get(p_instance->p_twim, &pins);
        nrfy_gpio_cfg_default(pins.scl_pin);
        nrfy_gpio_cfg_default(pins.sda_pin);
    }

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Instance uninitialized: %p.", p_instance->p_twim);
}

bool nrfx_twim_init_check(nrfx_twim_t const * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_twim_control_block_t const * p_cb = &p_instance->cb;

    return (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_twim_enable(nrfx_twim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;
    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_twim_enable(p_instance->p_twim);

    p_cb->state = NRFX_DRV_STATE_POWERED_ON;
    NRFX_LOG_INFO("Instance enabled: %p.", p_instance->p_twim);
}

void nrfx_twim_disable(nrfx_twim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    p_cb->int_mask = 0;
    nrfy_twim_stop(p_instance->p_twim);
    p_cb->state = NRFX_DRV_STATE_INITIALIZED;
    p_cb->busy = false;
    NRFX_LOG_INFO("Instance disabled: %p.", p_instance->p_twim);
}

bool nrfx_twim_is_busy(nrfx_twim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;
    return p_cb->busy;
}

static int twim_xfer(nrfx_twim_control_block_t   * p_cb,
                     NRF_TWIM_Type               * p_twim,
                     nrfx_twim_xfer_desc_t const * p_xfer_desc,
                     uint32_t                      flags)
{
    int err_code = 0;
    p_cb->error = false;

    if (p_xfer_desc->primary_length != 0 &&
        !nrf_dma_accessible_check(p_twim, p_xfer_desc->p_primary_buf))
    {
        err_code = -EACCES;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if ((p_xfer_desc->type == NRFX_TWIM_XFER_TXTX ||
         p_xfer_desc->type == NRFX_TWIM_XFER_TXRX) &&
         !nrf_dma_accessible_check(p_twim, p_xfer_desc->p_secondary_buf))
    {
            err_code = -EACCES;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
    }

#if !NRFY_TWIM_HAS_ARRAY_LIST
    if ((NRFX_TWIM_FLAG_TX_POSTINC | NRFX_TWIM_FLAG_RX_POSTINC) & flags)
    {
        err_code = -ENOTSUP;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif

    /* Block TWI interrupts to ensure that function is not interrupted by TWI interrupt. */
    nrfy_twim_int_disable(p_twim, NRF_TWIM_ALL_INTS_MASK);
    if (p_cb->busy)
    {
        nrfy_twim_int_enable(p_twim, p_cb->int_mask);
        err_code = -EBUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    else
    {
        p_cb->busy = ((NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER & flags) ||
                      (NRFX_TWIM_FLAG_REPEATED_XFER & flags)) ? false : true;
    }

    p_cb->xfer_type = p_xfer_desc->type;
    p_cb->address   = p_xfer_desc->address;
    p_cb->xfer_desc_primary.p_buffer   = p_xfer_desc->p_primary_buf;
    p_cb->xfer_desc_primary.length     = p_xfer_desc->primary_length;
    p_cb->xfer_desc_secondary.p_buffer = p_xfer_desc->p_secondary_buf;
    p_cb->xfer_desc_secondary.length   = p_xfer_desc->secondary_length;
    p_cb->repeated = (flags & NRFX_TWIM_FLAG_REPEATED_XFER) ? true : false;
    p_cb->flags = flags;
    nrfy_twim_address_set(p_twim, p_xfer_desc->address);

    nrfy_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTTX);
    nrfy_twim_event_clear(p_twim, NRF_TWIM_EVENT_SUSPENDED);
    nrfy_twim_event_clear(p_twim, NRF_TWIM_EVENT_ERROR);
    nrfy_twim_event_clear(p_twim, NRF_TWIM_EVENT_STOPPED);

#if NRFY_TWIM_HAS_ARRAY_LIST
    nrfy_twim_tx_list_set(p_twim, NRFX_TWIM_FLAG_TX_POSTINC & flags);
    nrfy_twim_rx_list_set(p_twim, NRFX_TWIM_FLAG_RX_POSTINC & flags);
#endif
    switch (p_xfer_desc->type)
    {
        case NRFX_TWIM_XFER_TXTX:
            NRFX_ASSERT(!(flags & NRFX_TWIM_FLAG_REPEATED_XFER));
            NRFX_ASSERT(!(flags & NRFX_TWIM_FLAG_HOLD_XFER));
            NRFX_ASSERT(!(flags & NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER));
            nrfy_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK);
            nrfy_twim_tx_buffer_set(p_twim, &p_cb->xfer_desc_primary);
            nrfy_twim_tx_start(p_twim, NULL);
            while (nrfy_twim_events_process(p_twim,
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_TWIM_EVENT_TXSTARTED),
                                            NULL))
            {}
            nrfy_twim_tx_buffer_set(p_twim, &p_cb->xfer_desc_secondary);
            NRFX_LOG_DEBUG("TWIM: Event: %s.", EVT_TO_STR_TWIM(NRF_TWIM_EVENT_TXSTARTED));
            p_cb->int_mask = NRF_TWIM_INT_SUSPENDED_MASK;
            break;
        case NRFX_TWIM_XFER_TXRX:
            nrfy_twim_tx_buffer_set(p_twim, &p_cb->xfer_desc_primary);
            nrfy_twim_rx_buffer_set(p_twim, &p_cb->xfer_desc_secondary);
            nrfy_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTTX_STARTRX_MASK |
                                    NRF_TWIM_SHORT_LASTRX_STOP_MASK);
            nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
            p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK;
            break;
        case NRFX_TWIM_XFER_TX:
            nrfy_twim_tx_buffer_set(p_twim, &p_cb->xfer_desc_primary);
            if (NRFX_TWIM_FLAG_TX_NO_STOP & flags)
            {
                nrfy_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTTX_SUSPEND_MASK);
                p_cb->int_mask = NRF_TWIM_INT_SUSPENDED_MASK;
            }
            else
            {
                nrfy_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTTX_STOP_MASK);
                p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK;
            }
            nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
            break;
        case NRFX_TWIM_XFER_RX:
            nrfy_twim_rx_buffer_set(p_twim, &p_cb->xfer_desc_primary);
            nrfy_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTRX_STOP_MASK);
            nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
            p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK;
            break;
        default:
            err_code = -EINVAL;
            break;
    }

    if (!(flags & NRFX_TWIM_FLAG_HOLD_XFER) && (p_xfer_desc->type != NRFX_TWIM_XFER_TXTX))
    {
        if (p_xfer_desc->type == NRFX_TWIM_XFER_RX)
        {
            nrfy_twim_rx_start(p_twim, p_cb->handler ? NULL : &p_cb->xfer_desc_primary);
        }
        else
        {
            nrfy_twim_tx_start(p_twim, p_cb->handler ? NULL : &p_cb->xfer_desc_primary);
        }
        /* Handling zero length transfers in non-blocking mode.
           In blocking mode zero length transfer is handled in
           @ref{nrfy_twim_tx_start} and @ref{nrfy_twim_rx_start}
        */
        if (p_xfer_desc->primary_length == 0 && p_cb->handler)
        {
            nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_STOP);
        }
    }

    if (p_cb->handler)
    {
        if (flags & NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER)
        {
            p_cb->int_mask = 0;
        }

        if (!(flags & NRFX_TWIM_FLAG_NO_SPURIOUS_STOP_CHECK))
        {
            p_cb->int_mask |= NRF_TWIM_INT_STOPPED_MASK;
        }

        // Interrupts for ERROR are implicitly enabled, regardless of driver configuration.
        p_cb->int_mask |= NRF_TWIM_INT_ERROR_MASK;
        nrfy_twim_int_enable(p_twim, p_cb->int_mask);

#if NRF_ERRATA_STATIC_CHECK(52, 109)
        if (NRF_ERRATA_DYNAMIC_CHECK(52, 109))
        {
            if ((flags & NRFX_TWIM_FLAG_HOLD_XFER) && (p_xfer_desc->type != NRFX_TWIM_XFER_RX))
            {
                nrfy_twim_tx_list_set(p_twim, false);
                nrfy_twim_rx_list_set(p_twim, false);
                p_twim->FREQUENCY = 0;
                nrfy_twim_event_clear(p_twim, NRF_TWIM_EVENT_TXSTARTED);
                nrfy_twim_int_enable(p_twim, NRF_TWIM_INT_TXSTARTED_MASK);
            }
            else
            {
                nrf_twim_frequency_t new_frequency = p_cb->bus_frequency;
#if NRF_ERRATA_STATIC_CHECK(52, 219) || NRF_ERRATA_STATIC_CHECK(53, 47)
                if ((NRF_ERRATA_DYNAMIC_CHECK(52, 219) || NRF_ERRATA_DYNAMIC_CHECK(53, 47)) &&
                    (p_cb->bus_frequency == NRF_TWIM_FREQ_400K))
                {
                    new_frequency = (nrf_twim_frequency_t) 0x06200000UL; // 390 kbps
                }
#endif
                nrfy_twim_frequency_set(p_twim, new_frequency);
            }
        }
#endif
    }
    else
    {
        uint32_t errorsrc = nrfy_twim_errorsrc_get_and_clear(p_twim);

        p_cb->busy = false;

        if (errorsrc)
        {
            err_code = twi_process_error(errorsrc);
        }
        else
        {
            if (!(flags & NRFX_TWIM_FLAG_NO_SPURIOUS_STOP_CHECK) &&
                !xfer_completeness_check(p_twim, p_cb))
            {
                err_code = -ECANCELED;
            }
        }
    }

    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}


int nrfx_twim_xfer(nrfx_twim_t                 * p_instance,
                   nrfx_twim_xfer_desc_t const * p_xfer_desc,
                   uint32_t                      flags)
{
    NRFX_ASSERT(p_instance);

    nrfx_twim_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_POWERED_ON);

    // TXRX and TXTX transfers are supported only in non-blocking mode.
    NRFX_ASSERT( !((p_cb->handler == NULL) && (p_xfer_desc->type == NRFX_TWIM_XFER_TXRX)));
    NRFX_ASSERT( !((p_cb->handler == NULL) && (p_xfer_desc->type == NRFX_TWIM_XFER_TXTX)));

    NRFX_LOG_INFO("Transfer type: %s.", TRANSFER_TO_STR(p_xfer_desc->type));
    NRFX_LOG_INFO("Transfer buffers length: primary: %d, secondary: %d.",
                  p_xfer_desc->primary_length,
                  p_xfer_desc->secondary_length);
    NRFX_LOG_DEBUG("Primary buffer data:");
    NRFX_LOG_HEXDUMP_DEBUG(p_xfer_desc->p_primary_buf,
                           p_xfer_desc->primary_length *
                           sizeof(p_xfer_desc->p_primary_buf[0]));
    NRFX_LOG_DEBUG("Secondary buffer data:");
    NRFX_LOG_HEXDUMP_DEBUG(p_xfer_desc->p_secondary_buf,
                           p_xfer_desc->secondary_length *
                           sizeof(p_xfer_desc->p_secondary_buf[0]));

    return twim_xfer(p_cb, p_instance->p_twim, p_xfer_desc, flags);
}

uint32_t nrfx_twim_start_task_address_get(nrfx_twim_t const *   p_instance,
                                          nrfx_twim_xfer_type_t xfer_type)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    return nrfy_twim_task_address_get(p_instance->p_twim,
        (xfer_type != NRFX_TWIM_XFER_RX) ? NRF_TWIM_TASK_STARTTX : NRF_TWIM_TASK_STARTRX);
}

uint32_t nrfx_twim_stopped_event_address_get(nrfx_twim_t const * p_instance)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    return nrfy_twim_event_address_get(p_instance->p_twim, NRF_TWIM_EVENT_STOPPED);
}

void nrfx_twim_irq_handler(nrfx_twim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    NRF_TWIM_Type * p_twim = p_instance->p_twim;
    nrfx_twim_control_block_t * p_cb = &p_instance->cb;

    nrfy_twim_xfer_desc_t * p_xfer = p_cb->xfer_type == NRFX_TWIM_XFER_RX ?
                                                        &p_cb->xfer_desc_primary :
                                                        &p_cb->xfer_desc_secondary;
#if NRF_ERRATA_STATIC_CHECK(52, 109)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 109) &&
        nrfy_twim_events_process(p_twim,
                                 NRFY_EVENT_TO_INT_BITMASK(NRF_TWIM_EVENT_TXSTARTED),
                                 NULL))
    {
        nrfy_twim_int_disable(p_twim, NRF_TWIM_INT_TXSTARTED_MASK);
        if (p_twim->FREQUENCY == 0)
        {
            // Set enable to zero to reset TWIM internal state.
            nrfy_twim_disable(p_twim);
            nrfy_twim_enable(p_twim);

            // Set proper frequency.
#if NRF_ERRATA_STATIC_CHECK(52, 219) || NRF_ERRATA_STATIC_CHECK(53, 47)
            if ((NRF_ERRATA_DYNAMIC_CHECK(52, 219) || NRF_ERRATA_DYNAMIC_CHECK(53, 47)) &&
                (p_cb->bus_frequency == NRF_TWIM_FREQ_400K))
            {
                nrfy_twim_frequency_set(p_twim, (nrf_twim_frequency_t) 0x06200000UL); // 390 kbps
            }
            else
#endif
            {
                nrfy_twim_frequency_set(p_twim, p_cb->bus_frequency);
            }
            nrfy_twim_tx_list_set(p_twim, NRFX_TWIM_FLAG_TX_POSTINC & p_cb->flags);
            nrfy_twim_rx_list_set(p_twim, NRFX_TWIM_FLAG_RX_POSTINC & p_cb->flags);
            // Start proper transmission.
            nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_STARTTX);
            return;
        }
    }
#endif
    NRFX_ASSERT(p_cb->handler);

    bool stopped = nrfy_twim_events_process(p_twim,
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_TWIM_EVENT_STOPPED),
                                            p_xfer);

    if (nrfy_twim_events_process(p_twim,
                                 NRFY_EVENT_TO_INT_BITMASK(NRF_TWIM_EVENT_ERROR),
                                 p_xfer))
    {
        NRFX_LOG_DEBUG("TWIM: Event: %s.", EVT_TO_STR_TWIM(NRF_TWIM_EVENT_ERROR));
        if (!stopped)
        {
            nrfy_twim_int_disable(p_twim, p_cb->int_mask);
            p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK;
            nrfy_twim_int_enable(p_twim, p_cb->int_mask);

            if (!(nrfy_twim_events_process(p_twim,
                                           NRFY_EVENT_TO_INT_BITMASK(NRF_TWIM_EVENT_LASTTX),
                                           NULL) &&
                 (nrfy_twim_shorts_get(p_twim) & NRF_TWIM_SHORT_LASTTX_STOP_MASK)))
            {
                nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
                nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_STOP);
            }

            p_cb->error = true;
            return;
        }
    }

    nrfx_twim_event_t event;

    if (stopped)
    {
        NRFX_LOG_DEBUG("TWIM: Event: %s.", EVT_TO_STR_TWIM(NRF_TWIM_EVENT_STOPPED));

        if (!(p_cb->flags & NRFX_TWIM_FLAG_NO_SPURIOUS_STOP_CHECK) && !p_cb->error)
        {
            p_cb->error = !xfer_completeness_check(p_twim, p_cb);
        }

        // Further processing of STOPPED event is valid only if NO_XFER_EVT_HANDLER
        // setting is not used.
        if (!(p_cb->flags & NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER))
        {
            nrfy_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTTX);
            nrfy_twim_event_clear(p_twim, NRF_TWIM_EVENT_LASTRX);
            if (!p_cb->repeated || p_cb->error)
            {
                nrfy_twim_shorts_set(p_twim, 0);
                p_cb->int_mask = 0;
                nrfy_twim_int_disable(p_twim, NRF_TWIM_ALL_INTS_MASK);

                // At this point interrupt handler should not be invoked again for current transfer.
                // If STOPPED arrived during ERROR processing,
                // its pending interrupt should be ignored.
                // Otherwise spurious NRFX_TWIM_EVT_DONE or NRFX_TWIM_EVT_BUS_ERROR
                // would be passed to user's handler.
                NRFY_IRQ_PENDING_CLEAR(nrfx_get_irq_number(p_twim));
            }

            event.xfer_desc.type    = p_cb->xfer_type;
            event.xfer_desc.address = p_cb->address;
            event.xfer_desc.p_primary_buf    = p_cb->xfer_desc_primary.p_buffer;
            event.xfer_desc.primary_length   = p_cb->xfer_desc_primary.length;
            event.xfer_desc.p_secondary_buf  = p_cb->xfer_desc_secondary.p_buffer;
            event.xfer_desc.secondary_length = p_cb->xfer_desc_secondary.length;
        }
#if NRF_ERRATA_STATIC_CHECK(52, 109)
        else if (NRF_ERRATA_DYNAMIC_CHECK(52, 109) && (p_cb->xfer_type != NRFX_TWIM_XFER_RX))
        {
            /* Add Anomaly 109 workaround for each potential repeated transfer starting from TX. */
            nrfy_twim_tx_list_set(p_twim, false);
            nrfy_twim_rx_list_set(p_twim, false);
            nrfy_twim_frequency_set(p_twim, (nrf_twim_frequency_t)0);
            nrfy_twim_int_enable(p_twim, NRF_TWIM_INT_TXSTARTED_MASK);
        }
#endif
    }
    else
    {
        (void)nrfy_twim_events_process(p_twim,
                                       NRFY_EVENT_TO_INT_BITMASK(NRF_TWIM_EVENT_SUSPENDED),
                                       p_xfer);
        NRFX_LOG_DEBUG("TWIM: Event: %s.", EVT_TO_STR_TWIM(NRF_TWIM_EVENT_SUSPENDED));
        if (p_cb->xfer_type == NRFX_TWIM_XFER_TX)
        {
            if (!p_cb->repeated)
            {
                nrfy_twim_shorts_set(p_twim, 0);
                p_cb->int_mask = 0;
                nrfy_twim_int_disable(p_twim, NRF_TWIM_ALL_INTS_MASK);

                // At this point interrupt handler should not be invoked again for current transfer.
                // If STOPPED arrived during SUSPENDED processing,
                // its pending interrupt should be ignored.
                // Otherwise spurious NRFX_TWIM_EVT_DONE or NRFX_TWIM_EVT_BUS_ERROR
                // would be passed to user's handler.
                NRFY_IRQ_PENDING_CLEAR(nrfx_get_irq_number(p_twim));
            }

            event.xfer_desc.type    = p_cb->xfer_type;
            event.xfer_desc.address = p_cb->address;
            event.xfer_desc.p_primary_buf    = p_cb->xfer_desc_primary.p_buffer;
            event.xfer_desc.primary_length   = p_cb->xfer_desc_primary.length;
            event.xfer_desc.p_secondary_buf  = p_cb->xfer_desc_secondary.p_buffer;
            event.xfer_desc.secondary_length = p_cb->xfer_desc_secondary.length;
        }
        else
        {
            nrfy_twim_shorts_set(p_twim, NRF_TWIM_SHORT_LASTTX_STOP_MASK);
            p_cb->int_mask = NRF_TWIM_INT_STOPPED_MASK | NRF_TWIM_INT_ERROR_MASK;
            nrfy_twim_int_disable(p_twim, NRF_TWIM_ALL_INTS_MASK);
            nrfy_twim_int_enable(p_twim, p_cb->int_mask);
            nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_STARTTX);
            nrfy_twim_task_trigger(p_twim, NRF_TWIM_TASK_RESUME);
            return;
        }
    }

    uint32_t errorsrc = nrfy_twim_errorsrc_get_and_clear(p_twim);
    if (errorsrc & NRF_TWIM_ERROR_ADDRESS_NACK)
    {
        event.type = NRFX_TWIM_EVT_ADDRESS_NACK;
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRFX_TWIM_EVT_ADDRESS_NACK));
    }
    else if (errorsrc & NRF_TWIM_ERROR_DATA_NACK)
    {
        event.type = NRFX_TWIM_EVT_DATA_NACK;
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRFX_TWIM_EVT_DATA_NACK));
    }
    else if (errorsrc & NRF_TWIM_ERROR_OVERRUN)
    {
        event.type = NRFX_TWIM_EVT_OVERRUN;
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRFX_TWIM_EVT_OVERRUN));
    }
    else if (p_cb->error)
    {
        event.type = NRFX_TWIM_EVT_BUS_ERROR;
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRFX_TWIM_EVT_BUS_ERROR));
    }
    else
    {
        event.type = NRFX_TWIM_EVT_DONE;
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRFX_TWIM_EVT_DONE));
    }

    if (!p_cb->repeated)
    {
        p_cb->busy = false;
    }

    if (!(p_cb->flags & NRFX_TWIM_FLAG_NO_XFER_EVT_HANDLER) || p_cb->error)
    {
        p_cb->handler(&event, p_cb->p_context);
    }
}
