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
#include <nrfx_tdm.h>
#include <haly/nrfy_gpio.h>

#define NRFX_LOG_MODULE TDM
#include <nrfx_log.h>

static void pins_configure(nrfx_tdm_t *              p_instance,
                           nrfx_tdm_config_t const * p_config)
{
    if (!p_config->skip_psel_cfg)
    {
        nrf_tdm_pins_t tdm_pins =
        {
            .sck_pin = p_config->sck_pin,
            .fsync_pin = p_config->fsync_pin,
            .mck_pin = p_config->mck_pin,
            .sdout_pin = p_config->sdout_pin,
            .sdin_pin = p_config->sdin_pin,
        };

        nrf_tdm_pins_set(p_instance->p_reg, &tdm_pins);
    }

    if (!p_config->skip_gpio_cfg)
    {
        if (p_config->mode == NRF_TDM_MODE_MASTER)
        {
            nrfy_gpio_cfg_output(p_config->sck_pin);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_TDM_CLOCKPIN_SCK_NEEDED)
            nrfy_gpio_pin_clock_set(p_config->sck_pin, true);
#endif
            nrfy_gpio_cfg_output(p_config->fsync_pin);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_TDM_CLOCKPIN_FSYNC_NEEDED)
            nrfy_gpio_pin_clock_set(p_config->fsync_pin, true);
#endif
        }
        else
        {
            nrfy_gpio_cfg_input(p_config->sck_pin, NRF_GPIO_PIN_NOPULL);
            nrfy_gpio_cfg_input(p_config->fsync_pin, NRF_GPIO_PIN_NOPULL);
        }

        if (p_config->mck_pin != NRF_TDM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_output(p_config->mck_pin);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_TDM_CLOCKPIN_MCK_NEEDED)
            nrfy_gpio_pin_clock_set(p_config->mck_pin, true);
#endif
        }

        if (p_config->sdout_pin != NRF_TDM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_output(p_config->sdout_pin);
        }

        if (p_config->sdin_pin != NRF_TDM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_input(p_config->sdin_pin, NRF_GPIO_PIN_NOPULL);
        }
    }
}

static nrf_tdm_channels_count_t chan_num_get(uint8_t chan_num)
{
	return (nrf_tdm_channels_count_t)(NRF_TDM_CHANNELS_COUNT_1 + chan_num - 1);
}

static bool tdm_configuration_verify(nrfx_tdm_config_t const * p_config)
{
#if defined(TDM_TX0_CHANNEL_NEEDED)
    if (!(p_config->tx_channel_mask & NRF_TDM_CHANNEL_TX0_MASK))
    {
        return false;
    }
#endif
    if (p_config->channel_number > NRFX_TDM_NUM_OF_CHANNELS)
    {
        return false;
    }

    return true;
}

static int tdm_configure(nrfx_tdm_t *              p_instance,
                         nrfx_tdm_config_t const * p_config)
{
    if (!tdm_configuration_verify(p_config))
    {
        return -EINVAL;
    }

    nrf_tdm_config_t tdm_cfg =
    {
        .mode = p_config->mode,
        .alignment = p_config->alignment,
        .sample_width = p_config->sample_width,
        .channels = (p_config->rx_channel_mask | p_config->tx_channel_mask),
        .num_of_channels = chan_num_get(p_config->channel_number),
        .channel_delay = p_config->channel_delay,
        .mck_setup = p_config->prescalers.mck_div,
        .sck_setup = p_config->prescalers.sck_div,
        .sck_polarity = p_config->sck_polarity,
        .fsync_polarity = p_config->fsync_polarity,
        .fsync_duration = p_config->fsync_duration
    };

    nrf_tdm_configure(p_instance->p_reg, &tdm_cfg);

    pins_configure(p_instance, p_config);

    nrf_tdm_ors_set(p_instance->p_reg, p_config->ors);

    nrf_tdm_mck_configure(p_instance->p_reg, p_config->mck_src, p_config->mck_bypass);
    nrf_tdm_mck_set(p_instance->p_reg, p_config->mck_pin != NRF_TDM_PIN_NOT_CONNECTED);

    nrf_tdm_sck_configure(p_instance->p_reg, p_config->sck_src, p_config->sck_bypass);

    return 0;
}

int nrfx_tdm_init(nrfx_tdm_t *              p_instance,
                  nrfx_tdm_config_t const * p_config,
                  nrfx_tdm_data_handler_t   handler)
{
    NRFX_ASSERT(p_instance && p_config && handler);

    nrfx_tdm_control_block_t * p_cb = &p_instance->cb;
    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    err_code = tdm_configure(p_instance, p_config);

    if (err_code < 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    nrf_tdm_event_clear(p_instance->p_reg, NRF_TDM_EVENT_MAXCNT);
    nrf_tdm_event_clear(p_instance->p_reg, NRF_TDM_EVENT_RXPTRUPD);
    nrf_tdm_event_clear(p_instance->p_reg, NRF_TDM_EVENT_TXPTRUPD);
    nrf_tdm_event_clear(p_instance->p_reg, NRF_TDM_EVENT_STOPPED);
    nrf_tdm_event_clear(p_instance->p_reg, NRF_TDM_EVENT_ABORTED);

    NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(p_instance->p_reg), p_config->irq_priority);
    NRFX_IRQ_ENABLE(nrfx_get_irq_number(p_instance->p_reg));

    p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
    p_cb->handler = handler;
    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    return 0;
}

void nrfx_tdm_uninit(nrfx_tdm_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_tdm_control_block_t * p_cb = &p_instance->cb;

    if (!p_cb->skip_gpio_cfg)
    {
        nrf_tdm_pins_t pins;
        nrf_tdm_pins_get(p_instance->p_reg, &pins);

        nrfy_gpio_cfg_default(pins.sck_pin & TDM_PSEL_MASK);
        nrfy_gpio_cfg_default(pins.fsync_pin & TDM_PSEL_MASK);
        if (pins.mck_pin != NRF_TDM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_default(pins.mck_pin & TDM_PSEL_MASK);
        }
        if (pins.sdout_pin != NRF_TDM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_default(pins.sdout_pin & TDM_PSEL_MASK);
        }
        if (pins.sdin_pin != NRF_TDM_PIN_NOT_CONNECTED)
        {
            nrfy_gpio_cfg_default(pins.sdin_pin & TDM_PSEL_MASK);
        }
    }

    nrf_tdm_disable(p_instance->p_reg);
    NRFX_IRQ_DISABLE(nrfx_get_irq_number(p_instance->p_reg));

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
}

bool nrfx_tdm_init_check(nrfx_tdm_t const * p_instance)
{
    NRFX_ASSERT(p_instance);

    return (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

int nrfx_tdm_reconfigure(nrfx_tdm_t *              p_instance,
                         nrfx_tdm_config_t const * p_config)
{
    NRFX_ASSERT(p_instance && p_config);

    nrfx_tdm_control_block_t * p_cb = &p_instance->cb;
    int err_code;

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EINPROGRESS;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    err_code = tdm_configure(p_instance, p_config);

    if (err_code < 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    return 0;
}

static void tdm_buffers_set(nrfx_tdm_t *               p_instance,
                            nrfx_tdm_buffers_t const * p_buffers)
{
    nrf_tdm_rx_buffer_set(p_instance->p_reg, p_buffers->p_rx_buffer);
    nrf_tdm_rx_count_set(p_instance->p_reg, p_buffers->rx_buffer_size);
    nrf_tdm_tx_buffer_set(p_instance->p_reg, p_buffers->p_tx_buffer);
    nrf_tdm_tx_count_set(p_instance->p_reg, p_buffers->tx_buffer_size);
}

static int tdm_verify_buffers(nrfx_tdm_buffers_t const * p_buffers,
                              NRF_TDM_Type const *       p_reg,
                              bool                       use_rx,
                              bool                       use_tx)
{
    if ((use_rx && ((p_buffers->rx_buffer_size < TDM_MIN_TRANSFER_SIZE) ||
                    (p_buffers->p_rx_buffer == NULL))) ||
        (use_tx && ((p_buffers->tx_buffer_size < TDM_MIN_TRANSFER_SIZE) ||
                    (p_buffers->p_tx_buffer == NULL))))
    {
        return -EINVAL;
    }

    if ((p_buffers->p_rx_buffer != NULL &&
        (!nrf_dma_accessible_check(p_reg, p_buffers->p_rx_buffer) ||
         !nrfx_is_word_aligned(p_buffers->p_rx_buffer))) ||
        (p_buffers->p_tx_buffer != NULL &&
        (!nrf_dma_accessible_check(p_reg, p_buffers->p_tx_buffer) ||
         !nrfx_is_word_aligned(p_buffers->p_tx_buffer))))
    {
        return -EACCES;
    }

    return 0;
}

int nrfx_tdm_start(nrfx_tdm_t *               p_instance,
                   nrfx_tdm_buffers_t const * p_initial_buffers)
{
    NRFX_ASSERT(p_instance && p_initial_buffers);

    nrfx_tdm_control_block_t * p_cb = &p_instance->cb;
    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_INITIALIZED)
    {
        err_code = (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED) ? -EINPROGRESS : -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    bool use_rx = (p_initial_buffers->p_rx_buffer != NULL);
    bool use_tx = (p_initial_buffers->p_tx_buffer != NULL);

    if (!use_rx && !use_tx)
    {
        return -EINVAL;
    }

    err_code = tdm_verify_buffers(p_initial_buffers,
                                  p_instance->p_reg,
                                  use_rx, use_tx);

    if (err_code < 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    p_cb->use_rx = use_rx;
    p_cb->use_tx = use_tx;
    p_cb->rx_ready       = false;
    p_cb->tx_ready       = false;
    p_cb->buffers_needed = false;
    p_cb->buffers_reused = false;

    p_cb->next_buffers = *p_initial_buffers;
    p_cb->current_buffers.p_rx_buffer = NULL;
    p_cb->current_buffers.p_tx_buffer = NULL;

    p_cb->state = NRFX_DRV_STATE_POWERED_ON;

    nrf_tdm_int_enable(p_instance->p_reg,
                      (p_cb->use_rx ? NRF_TDM_INT_RXPTRUPD_MASK_MASK : 0) |
                      (p_cb->use_tx ? NRF_TDM_INT_TXPTRUPD_MASK_MASK : 0) |
                       NRF_TDM_INT_STOPPED_MASK_MASK |
                       NRF_TDM_INT_ABORTED_MASK);

    tdm_buffers_set(p_instance, p_initial_buffers);

    nrf_tdm_transfer_direction_set(p_instance->p_reg,
                                   p_cb->use_rx ? (p_cb->use_tx ? NRF_TDM_RXTXEN_DUPLEX
                                                                : NRF_TDM_RXTXEN_RX)
                                                                : NRF_TDM_RXTXEN_TX);

    nrf_tdm_enable(p_instance->p_reg);

    nrf_tdm_task_trigger(p_instance->p_reg, NRF_TDM_TASK_START);

    return 0;
}

int nrfx_tdm_next_buffers_set(nrfx_tdm_t *               p_instance,
                              nrfx_tdm_buffers_t const * p_buffers)
{
    NRFX_ASSERT(p_instance && p_buffers);

    nrfx_tdm_control_block_t * p_cb = &p_instance->cb;
    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_POWERED_ON)
    {
        err_code = -EINPROGRESS;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    err_code = tdm_verify_buffers(p_buffers,
                                  p_instance->p_reg,
                                  p_cb->use_rx, p_cb->use_tx);

    if (err_code < 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    tdm_buffers_set(p_instance, p_buffers);

    p_cb->next_buffers = *p_buffers;
    p_cb->buffers_needed = false;

    return 0;
}

void nrfx_tdm_stop(nrfx_tdm_t * p_instance, bool abort)
{
    NRFX_ASSERT(p_instance);

    nrfx_tdm_control_block_t * p_cb = &p_instance->cb;

    p_cb->buffers_needed = false;
    p_cb->buffers_reused = false;
    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    nrf_tdm_int_disable(p_instance->p_reg, NRF_TDM_INT_RXPTRUPD_MASK_MASK |
                                           NRF_TDM_INT_TXPTRUPD_MASK_MASK);

    nrf_tdm_task_trigger(p_instance->p_reg, abort ? NRF_TDM_TASK_ABORT : NRF_TDM_TASK_STOP);
}

int nrfx_tdm_prescalers_calc(nrfx_tdm_clk_params_t const * clk_params,
                             nrfx_tdm_prescalers_t *       prescalers)
{
    NRFX_ASSERT(clk_params && prescalers);

    uint32_t mck_div = (uint32_t)(((uint64_t)clk_params->mck_freq * TDM_MCKCONST_FACTOR) /
                (clk_params->base_clock_freq + clk_params->mck_freq / 2)) * TDM_CK_DIV_FACTOR;

    uint32_t sck_div = (uint32_t)(((uint64_t)clk_params->sck_freq * TDM_MCKCONST_FACTOR) /
                (clk_params->base_clock_freq + clk_params->sck_freq / 2)) * TDM_CK_DIV_FACTOR;

    if ((mck_div < NRF_TDM_CK_DIV_MIN) || (mck_div > NRF_TDM_CK_DIV_MAX) ||
        (sck_div < NRF_TDM_CK_DIV_MIN) || (sck_div > NRF_TDM_CK_DIV_MAX))
    {
        return -EINVAL;
    }

    prescalers->mck_div = mck_div;
    prescalers->sck_div = sck_div;

    return 0;
}

void nrfx_tdm_irq_handler(nrfx_tdm_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    NRF_TDM_Type * p_reg = p_instance->p_reg;
    nrfx_tdm_control_block_t * p_cb = &p_instance->cb;

    if (nrf_tdm_event_check(p_reg, NRF_TDM_EVENT_MAXCNT)) {
        nrf_tdm_event_clear(p_reg, NRF_TDM_EVENT_MAXCNT);
    }

    if (nrf_tdm_event_check(p_reg, NRF_TDM_EVENT_RXPTRUPD)) {
        nrf_tdm_event_clear(p_reg, NRF_TDM_EVENT_RXPTRUPD);
        p_cb->rx_ready = true;
    }

    if (nrf_tdm_event_check(p_reg, NRF_TDM_EVENT_TXPTRUPD)) {
        nrf_tdm_event_clear(p_reg, NRF_TDM_EVENT_TXPTRUPD);
        p_cb->tx_ready = true;
    }

    if ((p_cb->rx_ready || p_cb->tx_ready) && p_cb->buffers_needed)
    {
        p_cb->buffers_reused = true;
    }

    if (nrf_tdm_event_check(p_reg, NRF_TDM_EVENT_STOPPED) ||
        nrf_tdm_event_check(p_reg, NRF_TDM_EVENT_ABORTED)) {
        nrf_tdm_event_clear(p_reg, NRF_TDM_EVENT_STOPPED);
        nrf_tdm_event_clear(p_reg, NRF_TDM_EVENT_ABORTED);

        nrf_tdm_int_disable(p_reg, NRF_TDM_INT_STOPPED_MASK_MASK |
                                   NRF_TDM_INT_ABORTED_MASK);
        nrf_tdm_disable(p_reg);

        p_cb->handler(&p_cb->current_buffers, 0);
        p_cb->handler(&p_cb->next_buffers, NRFX_TDM_STATUS_TRANSFER_STOPPED);
    }
    else if ((p_cb->use_tx && p_cb->use_rx && p_cb->tx_ready && p_cb->rx_ready) ||
            (!p_cb->use_rx && p_cb->tx_ready) ||
            (!p_cb->use_tx && p_cb->rx_ready))
    {
        p_cb->tx_ready = false;
        p_cb->rx_ready = false;

        if (p_cb->buffers_reused)
        {
            p_cb->buffers_reused = false;
            p_cb->buffers_needed = true;
            p_cb->handler(NULL, NRFX_TDM_STATUS_NEXT_BUFFERS_NEEDED);
        }
        else
        {
            nrfx_tdm_buffers_t released_buffers = p_cb->current_buffers;
            p_cb->current_buffers = p_cb->next_buffers;
            p_cb->next_buffers.p_rx_buffer = NULL;
            p_cb->next_buffers.rx_buffer_size = 0;
            p_cb->next_buffers.p_tx_buffer = NULL;
            p_cb->next_buffers.tx_buffer_size = 0;
            p_cb->buffers_needed = true;
            p_cb->handler(&released_buffers, NRFX_TDM_STATUS_NEXT_BUFFERS_NEEDED);
        }
    }
}
