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
#include <nrfx_i2s.h>
#include <haly/nrfy_gpio.h>

#define NRFX_LOG_MODULE I2S
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                         \
    (event == NRF_I2S_EVENT_RXPTRUPD ? "NRF_I2S_EVENT_RXPTRUPD" : \
    (event == NRF_I2S_EVENT_TXPTRUPD ? "NRF_I2S_EVENT_TXPTRUPD" : \
    (event == NRF_I2S_EVENT_STOPPED  ? "NRF_I2S_EVENT_STOPPED"  : \
                                       "UNKNOWN EVENT")))

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

static void deconfigure_pins(nrfx_i2s_t * p_instance)
{
    nrf_i2s_pins_t pins;
    uint32_t pin_mask;

    nrfy_i2s_pins_get(p_instance->p_reg, &pins);

    if (NRF_ERRATA_DYNAMIC_CHECK(52, 170))
    {
        // Create bitmask for extracting pin number from PSEL register.
        pin_mask = NRF_I2S_PSEL_SCK_PIN_MASK;
#if NRF_I2S_HAS_GPIO_PORT_SELECTION
        // If device supports more than one GPIO port, take port number into account as well.
        pin_mask |= NRF_I2S_PSEL_SCK_PORT_MASK;
#endif
    }
    else
    {
        pin_mask = 0xFFFFFFFF;
    }

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

int nrfx_i2s_init(nrfx_i2s_t *              p_instance,
                  nrfx_i2s_config_t const * p_config,
                  nrfx_i2s_data_handler_t   handler)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(handler);

    int err_code;
    nrfx_i2s_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }


    if (!validate_config(p_config->mode,
                         p_config->prescalers.ratio,
                         p_config->sample_width))
    {
        err_code = -EINVAL;
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
            .mck_setup    = p_config->prescalers.mck_setup,
            .ratio        = p_config->prescalers.ratio,
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
        .enable_bypass = p_config->prescalers.enable_bypass,
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
    return 0;
}

void nrfx_i2s_uninit(nrfx_i2s_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_i2s_control_block_t * p_cb = &p_instance->cb;
    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfx_i2s_stop(p_instance);

    nrfy_i2s_int_uninit(p_instance->p_reg);
    nrfy_i2s_disable(p_instance->p_reg);

    if (!p_cb->skip_gpio_cfg)
    {
        deconfigure_pins(p_instance);
    }

    if (NRF_ERRATA_DYNAMIC_CHECK(52, 196) && !p_cb->skip_psel_cfg)
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

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_i2s_init_check(nrfx_i2s_t const * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_i2s_control_block_t const * p_cb = &p_instance->cb;

    return (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
}

int nrfx_i2s_start(nrfx_i2s_t *               p_instance,
                   nrfx_i2s_buffers_t const * p_initial_buffers,
                   uint8_t                    flags)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_initial_buffers != NULL);
    NRFX_ASSERT(p_initial_buffers->p_rx_buffer != NULL ||
                p_initial_buffers->p_tx_buffer != NULL);
    NRFX_ASSERT((p_initial_buffers->p_rx_buffer == NULL) ||
                (nrf_dma_accessible_check(p_instance->p_reg,
                                          p_initial_buffers->p_rx_buffer) &&
                 nrfx_is_word_aligned(p_initial_buffers->p_rx_buffer)));
    NRFX_ASSERT((p_initial_buffers->p_tx_buffer == NULL) ||
                (nrf_dma_accessible_check(p_instance->p_reg,
                                          p_initial_buffers->p_tx_buffer) &&
                 nrfx_is_word_aligned(p_initial_buffers->p_tx_buffer)));
    NRFX_ASSERT(p_initial_buffers->buffer_size != 0);
    (void)(flags);

    int err_code;
    nrfx_i2s_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->state != NRFX_DRV_STATE_INITIALIZED)
    {
        err_code = -EINPROGRESS;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (((p_initial_buffers->p_rx_buffer != NULL)
         && !nrf_dma_accessible_check(p_instance->p_reg, p_initial_buffers->p_rx_buffer))
        ||
        ((p_initial_buffers->p_tx_buffer != NULL)
         && !nrf_dma_accessible_check(p_instance->p_reg, p_initial_buffers->p_tx_buffer)))
    {
        err_code = -EACCES;
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

    // Set the provided initial buffers as next, they will become the current
    // ones after the IRQ handler is called for the first time, what will occur
    // right after the START task is triggered.
    p_cb->next_buffers = *p_initial_buffers;
    p_cb->current_buffers.p_rx_buffer = NULL;
    p_cb->current_buffers.p_tx_buffer = NULL;


    nrfy_i2s_enable(p_instance->p_reg);

    p_cb->state = NRFX_DRV_STATE_POWERED_ON;

    if (NRF_ERRATA_DYNAMIC_CHECK(52, 55))
    {
        /* Clear spurious RXPTRUPD and TXPTRUPD events */
        nrfy_i2s_event_clear(p_instance->p_reg, NRF_I2S_EVENT_RXPTRUPD);
        nrfy_i2s_event_clear(p_instance->p_reg, NRF_I2S_EVENT_TXPTRUPD);
    }

    nrfy_i2s_int_enable(p_instance->p_reg,
                        (p_cb->use_rx ? NRF_I2S_INT_RXPTRUPD_MASK : 0UL) |
                        (p_cb->use_tx ? NRF_I2S_INT_TXPTRUPD_MASK : 0UL) |
                        NRF_I2S_INT_STOPPED_MASK);

    nrfy_i2s_buffers_set(p_instance->p_reg, &p_cb->next_buffers);
    nrfy_i2s_xfer_start(p_instance->p_reg, NULL);

    NRFX_LOG_INFO("Started.");
    return 0;
}

int nrfx_i2s_next_buffers_set(nrfx_i2s_t *               p_instance,
                              nrfx_i2s_buffers_t const * p_buffers)
{
    NRFX_ASSERT(p_instance);

    int err_code;
    nrfx_i2s_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state == NRFX_DRV_STATE_POWERED_ON);
    NRFX_ASSERT(p_buffers);
    NRFX_ASSERT((p_buffers->p_rx_buffer == NULL) ||
                (nrf_dma_accessible_check(p_instance->p_reg, p_buffers->p_rx_buffer) &&
                 nrfx_is_word_aligned(p_buffers->p_rx_buffer)));
    NRFX_ASSERT((p_buffers->p_tx_buffer == NULL) ||
                (nrf_dma_accessible_check(p_instance->p_reg, p_buffers->p_tx_buffer) &&
                 nrfx_is_word_aligned(p_buffers->p_tx_buffer)));
    NRFX_ASSERT(p_buffers->buffer_size != 0);

    if (!p_cb->buffers_needed)
    {
        err_code = -EINPROGRESS;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (((p_buffers->p_rx_buffer != NULL)
         && !nrf_dma_accessible_check(p_instance->p_reg, p_buffers->p_rx_buffer))
        ||
        ((p_buffers->p_tx_buffer != NULL)
         && !nrf_dma_accessible_check(p_instance->p_reg, p_buffers->p_tx_buffer)))
    {
        err_code = -EACCES;
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

    nrfy_i2s_buffers_set(p_instance->p_reg, p_buffers);

    p_cb->next_buffers   = *p_buffers;
    p_cb->buffers_needed = false;

    return 0;
}

void nrfx_i2s_stop(nrfx_i2s_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_i2s_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    p_cb->buffers_needed = false;

    if (NRF_ERRATA_DYNAMIC_CHECK(52, 55))
    {
        // First disable interrupts, then trigger the STOP task, so no spurious
        // RXPTRUPD and TXPTRUPD events are processed.
        nrfy_i2s_int_disable(p_instance->p_reg, NRF_I2S_INT_RXPTRUPD_MASK |
                                                NRF_I2S_INT_TXPTRUPD_MASK);
    }

    nrfy_i2s_abort(p_instance->p_reg, NULL);

    if (NRF_ERRATA_DYNAMIC_CHECK(52, 194) ||
        NRF_ERRATA_DYNAMIC_CHECK(91, 1))
    {
        *((volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x38)) = 1;
        *((volatile uint32_t *)(((uint32_t)p_instance->p_reg) + 0x3C)) = 1;
    }
}

int nrfx_i2s_prescalers_calc(nrfx_i2s_clk_params_t const * clk_params,
                             nrfx_i2s_prescalers_t *       prescalers)
{
    NRFX_ASSERT(clk_params);
    NRFX_ASSERT(prescalers);

    static const struct
    {
        uint16_t        ratio_val;
        nrf_i2s_ratio_t ratio_enum;
    } ratios[] =
    {
        {  32, NRF_I2S_RATIO_32X },
        {  48, NRF_I2S_RATIO_48X },
        {  64, NRF_I2S_RATIO_64X },
        {  96, NRF_I2S_RATIO_96X },
        { 128, NRF_I2S_RATIO_128X },
        { 192, NRF_I2S_RATIO_192X },
        { 256, NRF_I2S_RATIO_256X },
        { 384, NRF_I2S_RATIO_384X },
        { 512, NRF_I2S_RATIO_512X }
	};

    uint32_t best_diff = UINT32_MAX;
    uint8_t best_r = 0;
    nrf_i2s_mck_t best_mck_cfg = NRF_I2S_MCK_DISABLED;

    if (clk_params->allow_bypass)
    {
#if NRF_I2S_HAS_CLKCONFIG
        for (uint8_t r = 0; r < NRFX_ARRAY_SIZE(ratios); r++)
        {
            if (clk_params->transfer_rate * ratios[r].ratio_val == clk_params->base_clock_freq)
            {
                best_r = r;
                best_diff = 0;
                best_mck_cfg = NRF_I2S_MCK_32MDIV8;

                prescalers->enable_bypass = true;

                break;
            }
        }
#else
        NRFX_LOG_ERROR("Bypass mode not supported.");
        return -ENOTSUP;
#endif
    }

    for (uint8_t r = 0; (best_diff != 0) && (r < NRFX_ARRAY_SIZE(ratios)); r++)
    {
        if (!validate_config(NRF_I2S_MODE_MASTER, ratios[r].ratio_enum, clk_params->swidth))
        {
			continue;
		}
#if defined(I2S_MCKFREQ_FACTOR)
        uint32_t requested_mck = clk_params->transfer_rate * ratios[r].ratio_val;

        uint32_t mck_factor = (uint32_t)(((uint64_t)requested_mck * I2S_MCKFREQ_FACTOR) /
                                        (clk_params->base_clock_freq + requested_mck / 2));

        if (mck_factor > I2S_MCKFREQ_FACTOR)
        {
            continue;
        }

        uint32_t actual_mck = clk_params->base_clock_freq / (I2S_MCKFREQ_FACTOR / mck_factor);

        uint32_t lrck_freq = actual_mck / ratios[r].ratio_val;
        uint32_t diff = NRFX_DIFF(lrck_freq, clk_params->transfer_rate);

        if (diff < best_diff)
        {
            best_mck_cfg = (nrf_i2s_mck_t)(mck_factor * 4096);
            best_r = r;
            best_diff = diff;
        }
#else
        static const struct
        {
            uint8_t       divider_val;
            nrf_i2s_mck_t divider_enum;
        } dividers[] =
        {
            {   8, NRF_I2S_MCK_32MDIV8 },
            {  10, NRF_I2S_MCK_32MDIV10 },
            {  11, NRF_I2S_MCK_32MDIV11 },
            {  15, NRF_I2S_MCK_32MDIV15 },
            {  16, NRF_I2S_MCK_32MDIV16 },
            {  21, NRF_I2S_MCK_32MDIV21 },
            {  23, NRF_I2S_MCK_32MDIV23 },
            {  30, NRF_I2S_MCK_32MDIV30 },
            {  31, NRF_I2S_MCK_32MDIV31 },
            {  32, NRF_I2S_MCK_32MDIV32 },
            {  42, NRF_I2S_MCK_32MDIV42 },
            {  63, NRF_I2S_MCK_32MDIV63 },
            { 125, NRF_I2S_MCK_32MDIV125 }
        };

        for (uint8_t d = 0; (best_diff != 0) && (d < NRFX_ARRAY_SIZE(dividers)); d++)
        {
            uint32_t mck_freq = clk_params->base_clock_freq / dividers[d].divider_val;
            uint32_t lrck_freq = mck_freq / ratios[r].ratio_val;
            uint32_t diff = NRFX_DIFF(lrck_freq, clk_params->transfer_rate);

            if (diff < best_diff)
            {
                best_mck_cfg = dividers[d].divider_enum;
                best_r = r;
                best_diff = diff;
            }

            if (lrck_freq < clk_params->transfer_rate)
            {
                break;
            }
        }
#endif
	}

    if (best_diff == UINT32_MAX)
    {
        return -EINVAL;
    }

	prescalers->mck_setup = best_mck_cfg;
	prescalers->ratio = ratios[best_r].ratio_enum;

    return 0;
}

void nrfx_i2s_irq_handler(nrfx_i2s_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    NRF_I2S_Type * p_reg = p_instance->p_reg;
    nrfx_i2s_control_block_t * p_cb = &p_instance->cb;

    uint32_t event_mask;
    nrfy_i2s_xfer_desc_t * p_xfer;

    p_xfer = &p_cb->current_buffers;

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
