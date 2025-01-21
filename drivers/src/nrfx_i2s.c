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

#if NRFX_CHECK(NRFX_I2S_ENABLED)

#include <nrfx_i2s.h>
#include <haly/nrfy_gpio.h>

#define NRFX_LOG_MODULE I2S
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                         \
    (event == NRF_I2S_EVENT_RXPTRUPD ? "NRF_I2S_EVENT_RXPTRUPD" : \
    (event == NRF_I2S_EVENT_TXPTRUPD ? "NRF_I2S_EVENT_TXPTRUPD" : \
    (event == NRF_I2S_EVENT_STOPPED  ? "NRF_I2S_EVENT_STOPPED"  : \
                                       "UNKNOWN EVENT")))

#if !defined(USE_WORKAROUND_FOR_I2S_STOP_ANOMALY) && \
    (defined(NRF52_SERIES) || defined(NRF91_SERIES))
// Enable workaround for nRF52 Series anomaly 194 / nRF9160 anomaly 1
// (STOP task does not switch off all resources).
#define USE_WORKAROUND_FOR_I2S_STOP_ANOMALY 1
#endif

#if !defined(USE_WORKAROUND_FOR_ANOMALY_170) && defined(NRF52_SERIES)
// Enable workaround for nRF52 Series anomaly 170
// (when reading the value of PSEL registers, the CONNECT field might not
//  return the same value that has been written to it).
#define USE_WORKAROUND_FOR_ANOMALY_170 1
#endif

#if !defined(USE_WORKAROUND_FOR_ANOMALY_196) && defined(NRF52_SERIES)
// Enable workaround for nRF52 Series anomaly 196
// (PSEL acquires GPIO regardless of ENABLE).
#define USE_WORKAROUND_FOR_ANOMALY_196 1
#endif

// Control block - driver instance local data.
typedef struct
{
    nrfx_i2s_data_handler_t handler;
    nrfx_drv_state_t        state;

    bool use_rx         : 1;
    bool use_tx         : 1;
    bool rx_ready       : 1;
    bool tx_ready       : 1;
    bool buffers_needed : 1;
    bool buffers_reused : 1;
    bool skip_gpio_cfg  : 1;
    bool skip_psel_cfg  : 1;

#if !NRFX_API_VER_AT_LEAST(3, 3, 0)
    uint16_t            buffer_size;
#endif
    nrfx_i2s_buffers_t  next_buffers;
    nrfx_i2s_buffers_t  current_buffers;
} nrfx_i2s_cb_t;

static nrfx_i2s_cb_t m_cb[NRFX_I2S_ENABLED_COUNT];

static void configure_pins(nrfx_i2s_config_t const * p_config)
{
    if (!p_config->skip_gpio_cfg)
    {
        // Configure pins used by the peripheral:

        // - SCK and LRCK (required) - depending on the mode of operation these
        //   pins are configured as outputs (in Master mode) or inputs (in Slave
        //   mode).
        if (p_config->mode == NRF_I2S_MODE_MASTER)
        {
            nrfy_gpio_cfg_output(p_config->sck_pin);
            nrfy_gpio_cfg_output(p_config->lrck_pin);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_I2S_CLOCKPIN_SCK_NEEDED)
            nrfy_gpio_pin_clock_set(p_config->sck_pin, true);
#endif
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_I2S_CLOCKPIN_LRCK_NEEDED)
            nrfy_gpio_pin_clock_set(p_config->lrck_pin, true);
#endif
        }
        else
        {
            nrfy_gpio_cfg_input(p_config->sck_pin,  NRF_GPIO_PIN_NOPULL);
            nrfy_gpio_cfg_input(p_config->lrck_pin, NRF_GPIO_PIN_NOPULL);
        }
        // - MCK (optional) - always output,
        if (p_config->mck_pin != NRF_I2S_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_output(p_config->mck_pin);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_I2S_CLOCKPIN_MCK_NEEDED)
            nrfy_gpio_pin_clock_set(p_config->mck_pin, true);
#endif
        }
        // - SDOUT (optional) - always output,
        if (p_config->sdout_pin != NRF_I2S_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_output(p_config->sdout_pin);
        }
        // - SDIN (optional) - always input.
        if (p_config->sdin_pin != NRF_I2S_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_input(p_config->sdin_pin, NRF_GPIO_PIN_NOPULL);
        }
    }
}

static void deconfigure_pins(nrfx_i2s_t const * p_instance)
{
    nrf_i2s_pins_t pins;

    nrfy_i2s_pins_get(p_instance->p_reg, &pins);

#if NRFX_CHECK(USE_WORKAROUND_FOR_ANOMALY_170)
    // Create bitmask for extracting pin number from PSEL register.
    uint32_t pin_mask = NRF_I2S_PSEL_SCK_PIN_MASK;
#if NRF_I2S_HAS_GPIO_PORT_SELECTION
    // If device supports more than one GPIO port, take port number into account as well.
    pin_mask |= NRF_I2S_PSEL_SCK_PORT_MASK;
#endif
#else
    uint32_t pin_mask = 0xFFFFFFFF;
#endif // USE_WORKAROUND_FOR_ANOMALY_170

    nrfy_gpio_cfg_default(pins.sck_pin & pin_mask);
    nrfy_gpio_cfg_default(pins.lrck_pin & pin_mask);

    if (pins.mck_pin != NRF_I2S_PIN_NOT_CONNECTED)
    {
        nrfy_gpio_cfg_default(pins.mck_pin & pin_mask);
    }

    if (pins.sdout_pin != NRF_I2S_PIN_NOT_CONNECTED)
    {
        nrfy_gpio_cfg_default(pins.sdout_pin & pin_mask);
    }

    if (pins.sdin_pin != NRF_I2S_PIN_NOT_CONNECTED)
    {
        nrfy_gpio_cfg_default(pins.sdin_pin & pin_mask);
    }
}

static inline bool validate_config(nrf_i2s_mode_t   mode,
                                   nrf_i2s_ratio_t  ratio,
                                   nrf_i2s_swidth_t swidth)
{
    // The MCK/LRCK ratio has to be a multiple of 2 * sample width.
    if (mode == NRF_I2S_MODE_MASTER)
    {
        if (swidth == NRF_I2S_SWIDTH_16BIT)
        {
            if (ratio == NRF_I2S_RATIO_48X)
            {
                return false;
            }
        }

        if (swidth == NRF_I2S_SWIDTH_24BIT)
        {
            if ((ratio == NRF_I2S_RATIO_32X)  ||
                (ratio == NRF_I2S_RATIO_64X)  ||
                (ratio == NRF_I2S_RATIO_128X) ||
                (ratio == NRF_I2S_RATIO_256X) ||
                (ratio == NRF_I2S_RATIO_512X))
            {
                return false;
            }
        }

#if NRF_I2S_HAS_SWIDTH_32BIT
        if (swidth == NRF_I2S_SWIDTH_32BIT)
        {
            if ((ratio == NRF_I2S_RATIO_32X) ||
                (ratio == NRF_I2S_RATIO_48X) ||
                (ratio == NRF_I2S_RATIO_96X))
            {
                return false;
            }
        }
#endif

    }

    return true;
}

nrfx_err_t nrfx_i2s_init(nrfx_i2s_t const *        p_instance,
                         nrfx_i2s_config_t const * p_config,
                         nrfx_i2s_data_handler_t   handler)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(handler);

    nrfx_err_t err_code;
    nrfx_i2s_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
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


    if (!validate_config(p_config->mode,
                         p_config->ratio,
                         p_config->sample_width))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    configure_pins(p_config);

    nrfy_i2s_config_t nrfy_config =
    {
        .config = {
            .mode         = p_config->mode,
            .format       = p_config->format,
            .alignment    = p_config->alignment,
            .sample_width = p_config->sample_width,
            .channels     = p_config->channels,
            .mck_setup    = p_config->mck_setup,
            .ratio        = p_config->ratio,
        },
        .pins = {
            .sck_pin      = p_config->sck_pin,
            .lrck_pin     = p_config->lrck_pin,
            .mck_pin      = p_config->mck_pin,
            .sdout_pin    = p_config->sdout_pin,
            .sdin_pin     = p_config->sdin_pin,
        },
#if NRF_I2S_HAS_CLKCONFIG
        .clksrc        = p_config->clksrc,
        .enable_bypass = p_config->enable_bypass,
#endif
        .skip_psel_cfg = p_config->skip_psel_cfg
    };

    nrfy_i2s_periph_configure(p_instance->p_reg, &nrfy_config);

    p_cb->handler = handler;
    p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
    p_cb->skip_psel_cfg = p_config->skip_psel_cfg;

    nrfy_i2s_int_init(p_instance->p_reg,
                      NRF_I2S_INT_RXPTRUPD_MASK |
                      NRF_I2S_INT_TXPTRUPD_MASK |
                      NRF_I2S_INT_STOPPED_MASK,
                      p_config->irq_priority,
                      false);

    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    NRFX_LOG_INFO("Initialized.");
    return NRFX_SUCCESS;
}

void nrfx_i2s_uninit(nrfx_i2s_t const * p_instance)
{
    nrfx_i2s_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfx_i2s_stop(p_instance);

    nrfy_i2s_int_uninit(p_instance->p_reg);
    nrfy_i2s_disable(p_instance->p_reg);

    if (!p_cb->skip_gpio_cfg)
    {
        deconfigure_pins(p_instance);
    }

#if NRFX_CHECK(USE_WORKAROUND_FOR_ANOMALY_196)
    if (!p_cb->skip_psel_cfg)
    {
        // Disabling I2S is insufficient to release pins acquired by the peripheral.
        // Explicit disconnect is needed.
        nrf_i2s_pins_t pins = {
            .sck_pin   = NRF_I2S_PIN_NOT_CONNECTED,
            .lrck_pin  = NRF_I2S_PIN_NOT_CONNECTED,
            .mck_pin   = NRF_I2S_PIN_NOT_CONNECTED,
            .sdout_pin = NRF_I2S_PIN_NOT_CONNECTED,
            .sdin_pin  = NRF_I2S_PIN_NOT_CONNECTED,
        };
        nrfy_i2s_pins_set(p_instance->p_reg, &pins);
    }
#endif

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_i2s_init_check(nrfx_i2s_t const * p_instance)
{
    nrfx_i2s_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    return (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
}

nrfx_err_t nrfx_i2s_start(nrfx_i2s_t const *         p_instance,
                          nrfx_i2s_buffers_t const * p_initial_buffers,
#if !NRFX_API_VER_AT_LEAST(3, 3, 0)
                          uint16_t                   buffer_size,
#endif
                          uint8_t                    flags)
{
    NRFX_ASSERT(p_initial_buffers != NULL);
    NRFX_ASSERT(p_initial_buffers->p_rx_buffer != NULL ||
                p_initial_buffers->p_tx_buffer != NULL);
    NRFX_ASSERT((p_initial_buffers->p_rx_buffer == NULL) ||
                (nrfx_is_in_ram(p_initial_buffers->p_rx_buffer) &&
                 nrfx_is_word_aligned(p_initial_buffers->p_rx_buffer)));
    NRFX_ASSERT((p_initial_buffers->p_tx_buffer == NULL) ||
                (nrfx_is_in_ram(p_initial_buffers->p_tx_buffer) &&
                 nrfx_is_word_aligned(p_initial_buffers->p_tx_buffer)));
#if NRFX_API_VER_AT_LEAST(3, 3, 0)
    NRFX_ASSERT(p_initial_buffers->buffer_size != 0);
#else
    NRFX_ASSERT(buffer_size != 0);
#endif
    (void)(flags);

    nrfx_err_t err_code;
    nrfx_i2s_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if (p_cb->state != NRFX_DRV_STATE_INITIALIZED)
    {
        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (((p_initial_buffers->p_rx_buffer != NULL)
         && !nrfx_is_in_ram(p_initial_buffers->p_rx_buffer))
        ||
        ((p_initial_buffers->p_tx_buffer != NULL)
         && !nrfx_is_in_ram(p_initial_buffers->p_tx_buffer)))
    {
        err_code = NRFX_ERROR_INVALID_ADDR;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    p_cb->use_rx         = (p_initial_buffers->p_rx_buffer != NULL);
    p_cb->use_tx         = (p_initial_buffers->p_tx_buffer != NULL);
    p_cb->rx_ready       = false;
    p_cb->tx_ready       = false;
    p_cb->buffers_needed = false;
#if !NRFX_API_VER_AT_LEAST(3, 3, 0)
    p_cb->buffer_size    = buffer_size;
#endif

    // Set the provided initial buffers as next, they will become the current
    // ones after the IRQ handler is called for the first time, what will occur
    // right after the START task is triggered.
    p_cb->next_buffers = *p_initial_buffers;
    p_cb->current_buffers.p_rx_buffer = NULL;
    p_cb->current_buffers.p_tx_buffer = NULL;


    nrfy_i2s_enable(p_instance->p_reg);

    p_cb->state = NRFX_DRV_STATE_POWERED_ON;

    /* Clear spurious RXPTRUPD and TXPTRUPD events (see nRF52 anomaly 55). */
    nrfy_i2s_event_clear(p_instance->p_reg, NRF_I2S_EVENT_RXPTRUPD);
    nrfy_i2s_event_clear(p_instance->p_reg, NRF_I2S_EVENT_TXPTRUPD);

    nrfy_i2s_int_enable(p_instance->p_reg,
                        (p_cb->use_rx ? NRF_I2S_INT_RXPTRUPD_MASK : 0UL) |
                        (p_cb->use_tx ? NRF_I2S_INT_TXPTRUPD_MASK : 0UL) |
                        NRF_I2S_INT_STOPPED_MASK);

#if NRFX_API_VER_AT_LEAST(3, 3, 0)
    nrfy_i2s_buffers_set(p_instance->p_reg, &p_cb->next_buffers);
#else
    const nrfy_i2s_xfer_desc_t xfer = {
        .p_rx_buffer = p_cb->next_buffers.p_rx_buffer,
        .p_tx_buffer = p_cb->next_buffers.p_tx_buffer,
        .buffer_size = p_cb->buffer_size,
    };

    nrfy_i2s_buffers_set(p_instance->p_reg, &xfer);
#endif
    nrfy_i2s_xfer_start(p_instance->p_reg, NULL);

    NRFX_LOG_INFO("Started.");
    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_i2s_next_buffers_set(nrfx_i2s_t const *         p_instance,
                                     nrfx_i2s_buffers_t const * p_buffers)
{
    nrfx_err_t err_code;
    nrfx_i2s_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_POWERED_ON);
    NRFX_ASSERT(p_buffers);
    NRFX_ASSERT((p_buffers->p_rx_buffer == NULL) ||
                (nrfx_is_in_ram(p_buffers->p_rx_buffer) &&
                 nrfx_is_word_aligned(p_buffers->p_rx_buffer)));
    NRFX_ASSERT((p_buffers->p_tx_buffer == NULL) ||
                (nrfx_is_in_ram(p_buffers->p_tx_buffer) &&
                 nrfx_is_word_aligned(p_buffers->p_tx_buffer)));
#if NRFX_API_VER_AT_LEAST(3, 3, 0)
    NRFX_ASSERT(p_buffers->buffer_size != 0);
#endif

    if (!p_cb->buffers_needed)
    {
        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (((p_buffers->p_rx_buffer != NULL)
         && !nrfx_is_in_ram(p_buffers->p_rx_buffer))
        ||
        ((p_buffers->p_tx_buffer != NULL)
         && !nrfx_is_in_ram(p_buffers->p_tx_buffer)))
    {
        err_code = NRFX_ERROR_INVALID_ADDR;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (p_cb->use_tx)
    {
        NRFX_ASSERT(p_buffers->p_tx_buffer != NULL);
    }
    if (p_cb->use_rx)
    {
        NRFX_ASSERT(p_buffers->p_rx_buffer != NULL);
    }

#if NRFX_API_VER_AT_LEAST(3, 3, 0)
    nrfy_i2s_buffers_set(p_instance->p_reg, p_buffers);
#else
    nrfy_i2s_xfer_desc_t xfer = {
        .p_rx_buffer = p_buffers->p_rx_buffer,
        .p_tx_buffer = p_buffers->p_tx_buffer,
        .buffer_size = p_cb->buffer_size,
    };

    nrfy_i2s_buffers_set(p_instance->p_reg, &xfer);
#endif

    p_cb->next_buffers   = *p_buffers;
    p_cb->buffers_needed = false;

    return NRFX_SUCCESS;
}

void nrfx_i2s_stop(nrfx_i2s_t const * p_instance)
{
    nrfx_i2s_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    p_cb->buffers_needed = false;

    // First disable interrupts, then trigger the STOP task, so no spurious
    // RXPTRUPD and TXPTRUPD events (see nRF52 anomaly 55) are processed.
    nrfy_i2s_int_disable(p_instance->p_reg, NRF_I2S_INT_RXPTRUPD_MASK |
                                            NRF_I2S_INT_TXPTRUPD_MASK);

    nrfy_i2s_abort(p_instance->p_reg, NULL);

#if NRFX_CHECK(USE_WORKAROUND_FOR_I2S_STOP_ANOMALY)
    *((volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x38)) = 1;
    *((volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x3C)) = 1;
#endif
}

static void irq_handler(NRF_I2S_Type * p_reg, nrfx_i2s_cb_t * p_cb)
{
    uint32_t event_mask;
    nrfy_i2s_xfer_desc_t * p_xfer;

#if NRFX_API_VER_AT_LEAST(3, 3, 0)
    p_xfer = &p_cb->current_buffers;
#else
    nrfy_i2s_xfer_desc_t xfer = {
        .p_rx_buffer = p_cb->current_buffers.p_rx_buffer,
        .p_tx_buffer = p_cb->current_buffers.p_tx_buffer,
        .buffer_size = p_cb->buffer_size,
    };
    p_xfer = &xfer;
#endif

    event_mask = nrfy_i2s_events_process(p_reg,
                                         NRFY_EVENT_TO_INT_BITMASK(NRF_I2S_EVENT_TXPTRUPD) |
                                         NRFY_EVENT_TO_INT_BITMASK(NRF_I2S_EVENT_RXPTRUPD) |
                                         NRFY_EVENT_TO_INT_BITMASK(NRF_I2S_EVENT_STOPPED),
                                         p_xfer);

    if (event_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_I2S_EVENT_TXPTRUPD))
    {
        p_cb->tx_ready = true;
        if (p_cb->use_tx && p_cb->buffers_needed)
        {
            p_cb->buffers_reused = true;
        }
    }
    if (event_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_I2S_EVENT_RXPTRUPD))
    {
        p_cb->rx_ready = true;
        if (p_cb->use_rx && p_cb->buffers_needed)
        {
            p_cb->buffers_reused = true;
        }
    }

    if (event_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_I2S_EVENT_STOPPED))
    {
        nrfy_i2s_int_disable(p_reg, NRF_I2S_INT_STOPPED_MASK);
        nrfy_i2s_disable(p_reg);

        // When stopped, release all buffers, including these scheduled for
        // the next part of the transfer, and signal that the transfer has
        // finished.

        p_cb->handler(&p_cb->current_buffers, 0);

        // Change the state of the driver before calling the handler with
        // the flag signaling that the transfer has finished, so that it is
        // possible to start a new transfer directly from the handler function.
        p_cb->state = NRFX_DRV_STATE_INITIALIZED;
        NRFX_LOG_INFO("Stopped.");

        p_cb->handler(&p_cb->next_buffers, NRFX_I2S_STATUS_TRANSFER_STOPPED);
    }
    else
    {
        // Check if the requested transfer has been completed:
        // - full-duplex mode
        if ((p_cb->use_tx && p_cb->use_rx &&
             p_cb->tx_ready && p_cb->rx_ready) ||
            // - TX only mode
            (!p_cb->use_rx && p_cb->tx_ready) ||
            // - RX only mode
            (!p_cb->use_tx && p_cb->rx_ready))
        {
            p_cb->tx_ready = false;
            p_cb->rx_ready = false;

            // If the application did not supply the buffers for the next
            // part of the transfer until this moment, the current buffers
            // cannot be released, since the I2S peripheral already started
            // using them. Signal this situation to the application by
            // passing NULL instead of the structure with released buffers.
            if (p_cb->buffers_reused)
            {
                p_cb->buffers_reused = false;
                // This will most likely be set at this point. However, there is
                // a small time window between TXPTRUPD and RXPTRUPD events,
                // and it is theoretically possible that next buffers will be
                // set in this window, so to be sure this flag is set to true,
                // set it explicitly.
                p_cb->buffers_needed = true;
                p_cb->handler(NULL, NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED);
            }
            else
            {
                // Buffers that have been used by the I2S peripheral (current)
                // are now released and will be returned to the application,
                // and the ones scheduled to be used as next become the current
                // ones.
                nrfx_i2s_buffers_t released_buffers = p_cb->current_buffers;
                p_cb->current_buffers = p_cb->next_buffers;
                p_cb->next_buffers.p_rx_buffer = NULL;
                p_cb->next_buffers.p_tx_buffer = NULL;
                p_cb->buffers_needed = true;
                p_cb->handler(&released_buffers, NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED);
            }

        }
    }
}

NRFX_INSTANCE_IRQ_HANDLERS(I2S, i2s)

#endif // NRFX_CHECK(NRFX_I2S_ENABLED)
