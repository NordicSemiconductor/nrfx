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
#include <nrfx_pdm.h>
#include <haly/nrfy_pdm.h>
#include <haly/nrfy_gpio.h>

#define NRFX_LOG_MODULE PDM
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                       \
    (event == NRF_PDM_EVENT_STARTED ? "NRF_PDM_EVENT_STARTED" : \
    (event == NRF_PDM_EVENT_STOPPED ? "NRF_PDM_EVENT_STOPPED" : \
    (event == NRF_PDM_EVENT_END     ? "NRF_PDM_EVENT_END"     : \
                                      "UNKNOWN EVENT")))

static void pdm_configure(nrfx_pdm_t *              p_instance,
                          nrfx_pdm_config_t const * p_config)
{
    if (!p_config->skip_gpio_cfg)
    {
        nrfy_gpio_pin_clear(p_config->clk_pin);
        nrfy_gpio_cfg_output(p_config->clk_pin);
        nrfy_gpio_cfg_input(p_config->din_pin, NRF_GPIO_PIN_NOPULL);
    }
    if (!p_config->skip_psel_cfg)
    {
        nrf_pdm_psel_connect(p_instance->p_reg,
                             p_config->clk_pin,
                             p_config->din_pin);
    }

    nrfy_pdm_config_t nrfy_config =
    {
        .mode        = p_config->mode,
        .edge        = p_config->edge,
        .pins =
        {
            .clk_pin = p_config->clk_pin,
            .din_pin = p_config->din_pin,
        },
        NRFX_COND_CODE_1(NRF_PDM_HAS_PDMCLKCTRL, (.clock_freq  = p_config->prescalers.clock_freq,), ())
        NRFX_COND_CODE_1(NRF_PDM_HAS_PRESCALER, (.prescaler  = p_config->prescalers.prescaler,), ())
        .gain_l      = p_config->gain_l,
        .gain_r      = p_config->gain_r,
        NRFX_COND_CODE_1(NRF_PDM_HAS_RATIO_CONFIG, (.ratio = p_config->prescalers.ratio,), ())
        NRFX_COND_CODE_1(NRF_PDM_HAS_SELECTABLE_CLOCK, (.mclksrc = p_config->mclksrc,), ())
        .skip_psel_cfg = p_config->skip_psel_cfg
    };

    nrfy_pdm_periph_configure(p_instance->p_reg, &nrfy_config);

    nrfy_pdm_int_init(p_instance->p_reg,
                      NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED,
                      p_config->interrupt_priority,
                      true);
}

static int pdm_init(nrfx_pdm_t *              p_instance,
                    nrfx_pdm_config_t const * p_config,
                    nrfx_pdm_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(event_handler);
    int err_code;

    nrfx_pdm_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->drv_state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    p_cb->buff_address[0] = 0;
    p_cb->buff_address[1] = 0;
    p_cb->active_buffer = 0;
    p_cb->error = 0;
    p_cb->event_handler = event_handler;
    p_cb->op_state = NRFX_PDM_STATE_IDLE;

    if (p_config)
    {
        p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;

        if (p_config->gain_l > NRF_PDM_GAIN_MAXIMUM ||
            p_config->gain_r > NRF_PDM_GAIN_MAXIMUM)
        {
            err_code = -EINVAL;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
#if NRF_PDM_HAS_PRESCALER
        if (p_config->prescalers.prescaler < NRF_PDM_PRESCALER_MIN ||
            p_config->prescalers.prescaler > NRF_PDM_PRESCALER_MAX)
        {
            err_code = -EINVAL;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
#endif
        pdm_configure(p_instance, p_config);
    }

    p_cb->drv_state = NRFX_DRV_STATE_INITIALIZED;

    return 0;
}

static int pdm_reconfigure(nrfx_pdm_t *              p_instance,
                           nrfx_pdm_config_t const * p_config)
{
    NRFX_ASSERT(p_instance);
    nrfx_pdm_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_config);
    if (p_cb->drv_state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return -EINPROGRESS;
    }

    if (p_config->gain_l > NRF_PDM_GAIN_MAXIMUM ||
        p_config->gain_r > NRF_PDM_GAIN_MAXIMUM)
    {
        return -EINVAL;
    }

#if NRF_PDM_HAS_PRESCALER
    if (p_config->prescalers.prescaler < NRF_PDM_PRESCALER_MIN ||
        p_config->prescalers.prescaler > NRF_PDM_PRESCALER_MAX)
    {
        return -EINVAL;
    }
#endif

    if (p_cb->op_state != NRFX_PDM_STATE_IDLE)
    {
        return -EBUSY;
    }
    nrfy_pdm_disable(p_instance->p_reg);
    pdm_configure(p_instance, p_config);
    nrfy_pdm_enable(p_instance->p_reg);

    return 0;
}

static void pdm_uninit(nrfx_pdm_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfy_pdm_int_uninit(p_instance->p_reg);
    nrfy_pdm_disable(p_instance->p_reg);

    nrfx_pdm_control_block_t * p_cb = &p_instance->cb;

    if (!p_cb->skip_gpio_cfg)
    {
        nrfy_pdm_pins_t pins;
        nrfy_pdm_pins_get(p_instance->p_reg, &pins);
        nrfy_gpio_cfg_default(pins.clk_pin);
        nrfy_gpio_cfg_default(pins.din_pin);
    }

    p_cb->drv_state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

static bool pdm_init_check(nrfx_pdm_t const * p_instance)
{
    return (p_instance->cb.drv_state != NRFX_DRV_STATE_UNINITIALIZED);
}

static void pdm_buf_request(nrfx_pdm_t * p_instance)
{
    p_instance->cb.irq_buff_request = 1;
    NRFY_IRQ_PENDING_SET(nrfx_get_irq_number(p_instance->p_reg));
}

static int pdm_start(nrfx_pdm_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_pdm_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->drv_state != NRFX_DRV_STATE_UNINITIALIZED);
    int err_code;

    if (p_cb->op_state != NRFX_PDM_STATE_IDLE)
    {
        if (p_cb->op_state == NRFX_PDM_STATE_RUNNING)
        {
            err_code = 0;
            NRFX_LOG_INFO("Function: %s, error code: %s.",
                          __func__,
                          NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
        err_code = -EBUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    p_cb->op_state = NRFX_PDM_STATE_STARTING;
    pdm_buf_request(p_instance);

    return 0;
}

static int pdm_buffer_set(nrfx_pdm_t * p_instance,
                          int16_t *    buffer,
                          uint16_t     buffer_length)
{
    NRFX_ASSERT(p_instance);
    nrfx_pdm_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->drv_state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return -EINPROGRESS;
    }
    if (p_cb->op_state == NRFX_PDM_STATE_STOPPING)
    {
        return -EBUSY;
    }
    if ((buffer == NULL) || (buffer_length > NRFX_PDM_MAX_BUFFER_SIZE))
    {
        return -EINVAL;
    }

    int err_code = 0;

    // Enter the PDM critical section.
    NRFY_IRQ_DISABLE(nrfx_get_irq_number(p_instance->p_reg));

    uint8_t next_buffer = (~p_cb->active_buffer) & 0x01;
    if (p_cb->op_state == NRFX_PDM_STATE_STARTING)
    {
        next_buffer = 0;
    }

    if (p_cb->buff_address[next_buffer])
    {
        // Buffer already set.
        err_code = -EBUSY;
    }
    else
    {
        p_cb->buff_address[next_buffer] = buffer;
        p_cb->buff_length[next_buffer] = buffer_length;

        nrfy_pdm_buffer_t nrfy_buffer =
        {
            .p_buff = buffer,
            .length = buffer_length,
        };
        nrfy_pdm_buffer_set(p_instance->p_reg, &nrfy_buffer);

        if (p_cb->drv_state != NRFX_DRV_STATE_POWERED_ON)
        {
            p_cb->drv_state = NRFX_DRV_STATE_POWERED_ON;
            nrfy_pdm_enable(p_instance->p_reg);
            nrfy_pdm_start(p_instance->p_reg, NULL);
        }
    }

    NRFY_IRQ_ENABLE(nrfx_get_irq_number(p_instance->p_reg));

    return err_code;
}

static int pdm_stop(nrfx_pdm_t * p_instance)
{
    NRFX_ASSERT(p_instance);
    nrfx_pdm_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->drv_state != NRFX_DRV_STATE_UNINITIALIZED);
    int err_code;

    if (p_cb->op_state != NRFX_PDM_STATE_RUNNING)
    {
        if (p_cb->op_state == NRFX_PDM_STATE_IDLE ||
            p_cb->op_state == NRFX_PDM_STATE_STARTING)
        {
            nrfy_pdm_disable(p_instance->p_reg);
            p_cb->op_state = NRFX_PDM_STATE_IDLE;
            err_code = 0;
            NRFX_LOG_INFO("Function: %s, error code: %s.",
                          __func__,
                          NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
        err_code = -EBUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    p_cb->drv_state = NRFX_DRV_STATE_INITIALIZED;
    p_cb->op_state = NRFX_PDM_STATE_STOPPING;

    nrfy_pdm_abort(p_instance->p_reg, NULL);

    return 0;
}

int nrfx_pdm_init(nrfx_pdm_t *              p_instance,
                  nrfx_pdm_config_t const * p_config,
                  nrfx_pdm_event_handler_t  event_handler)
{
    return pdm_init(p_instance, p_config, event_handler);
}

int nrfx_pdm_reconfigure(nrfx_pdm_t *              p_instance,
                         nrfx_pdm_config_t const * p_config)
{
    return pdm_reconfigure(p_instance, p_config);
}

void nrfx_pdm_uninit(nrfx_pdm_t * p_instance)
{
    pdm_uninit(p_instance);
}

bool nrfx_pdm_init_check(nrfx_pdm_t const * p_instance)
{
    return pdm_init_check(p_instance);
}

int nrfx_pdm_start(nrfx_pdm_t * p_instance)
{
    return pdm_start(p_instance);
}

int nrfx_pdm_buffer_set(nrfx_pdm_t * p_instance,
                        int16_t *    buffer,
                        uint16_t     buffer_length)
{
    return pdm_buffer_set(p_instance, buffer, buffer_length);
}

int nrfx_pdm_stop(nrfx_pdm_t * p_instance)
{
    return pdm_stop(p_instance);
}

static bool prescaler_diff_check(uint32_t *                best_freq,
                                 uint32_t *                best_rate,
                                 uint32_t *                best_diff,
                                 uint8_t                   ratio,
                                 uint32_t                  actual_freq,
                                 nrfx_pdm_output_t const * output_config)
{
    if (actual_freq < output_config->output_freq_min ||
        actual_freq > output_config->output_freq_max)
    {
        return false;
    }

    uint32_t actual_rate = actual_freq / ratio;
    uint32_t diff = NRFX_DIFF(actual_rate, output_config->sampling_rate);

    if (diff < *best_diff)
    {
        *best_diff = diff;
		*best_rate = actual_rate;
		*best_freq = actual_freq;
		return true;
    }

    return false;
}

static bool pdm_prescalers_check(uint32_t *                best_freq,
                                 uint32_t *                best_rate,
                                 uint32_t *                best_diff,
                                 uint8_t                   ratio,
                                 nrfx_pdm_output_t const * output_config,
                                 nrfx_pdm_prescalers_t *   prescalers)
{
    bool better_found = false;
#if NRF_PDM_HAS_PRESCALER
	uint32_t req_freq = output_config->sampling_rate * ratio;
	uint32_t prescaler = output_config->base_clock_freq / req_freq;
	uint32_t actual_freq = output_config->base_clock_freq / prescaler;

	if (prescaler_diff_check(best_freq, best_rate, best_diff,
                             ratio, actual_freq, output_config))
    {
		prescalers->prescaler = prescaler;

		better_found = true;
	}

	if (*best_diff == 0)
    {
		return true;
	}

	prescaler += 1;
	actual_freq  = output_config->base_clock_freq / prescaler;

	if (prescaler_diff_check(best_freq, best_rate, best_diff,
                             ratio, actual_freq, output_config))
    {
		prescalers->prescaler = prescaler;

		better_found = true;
	}
#elif NRF_PDM_HAS_PDMCLKCTRL
#if defined(PDM_PDMCLKCTRL_FACTOR)
	uint32_t req_freq = output_config->sampling_rate * ratio;

	uint32_t clk_factor = (uint32_t)((req_freq * PDM_FREQ_FACTOR) /
						 (output_config->base_clock_freq + req_freq / 2));
	uint32_t actual_freq = output_config->base_clock_freq / (PDM_FREQ_FACTOR / clk_factor);

	if (prescaler_diff_check(best_freq, best_rate, best_diff,
                             ratio, actual_freq, output_config))
    {
		prescalers->clock_freq = (nrf_pdm_freq_t)(clk_factor * PDM_PDMCLKCTRL_FACTOR);

		better_found = true;
	}
#else
	static const struct
    {
		uint32_t       freq_val;
		nrf_pdm_freq_t freq_enum;
	} freqs[] =
    {
		{ 1000000, NRF_PDM_FREQ_1000K },
		{ 1032000, NRF_PDM_FREQ_1032K },
		{ 1067000, NRF_PDM_FREQ_1067K },
#if NRF_PDM_HAS_FREQ_1231K
		{ 1231000, NRF_PDM_FREQ_1231K },
#endif
#if NRF_PDM_HAS_FREQ_1280K
		{ 1280000, NRF_PDM_FREQ_1280K },
#endif
#if NRF_PDM_HAS_FREQ_1333K
		{ 1333000, NRF_PDM_FREQ_1333K }
#endif
	};

	for (uint32_t f = 0; f < NRFX_ARRAY_SIZE(freqs); f++)
    {
		uint32_t actual_freq = freqs[f].freq_val;

		if (prescaler_diff_check(best_freq, best_rate, best_diff,
                                 ratio, actual_freq, output_config))
        {
			prescalers->clock_freq = freqs[f].freq_enum;

			if (*best_diff == 0)
            {
				return true;
			}

			better_found = true;
		}

		if ((actual_freq / ratio) > output_config->sampling_rate)
        {
			break;
		}
	}
#endif
#endif // NRF_PDM_HAS_PRESCALER
    return better_found;
}

int nrfx_pdm_prescalers_calc(nrfx_pdm_output_t const * output_config,
                             nrfx_pdm_prescalers_t *   prescalers)
{
    NRFX_ASSERT(output_config);
    NRFX_ASSERT(prescalers);

    uint32_t best_diff = UINT32_MAX;
	uint32_t best_rate = 0;
	uint32_t best_freq = 0;
    uint8_t ratio;

#if NRF_PDM_HAS_RATIO_CONFIG
	static const struct
    {
		uint8_t         ratio_val;
		nrf_pdm_ratio_t ratio_enum;
	} ratios[] =
    {
#if NRF_PDM_HAS_RATIO32
		{ 32, NRF_PDM_RATIO_32X },
#endif
#if NRF_PDM_HAS_RATIO48
		{ 48, NRF_PDM_RATIO_48X },
#endif
#if NRF_PDM_HAS_RATIO50
		{ 50, NRF_PDM_RATIO_50X },
#endif
		{ 64, NRF_PDM_RATIO_64X },
		{ 80, NRF_PDM_RATIO_80X },
#if NRF_PDM_HAS_RATIO96
		{ 96, NRF_PDM_RATIO_96X },
#endif
#if NRF_PDM_HAS_RATIO100
		{ 100, NRF_PDM_RATIO_100X },
#endif
#if NRF_PDM_HAS_RATIO128
		{ 128, NRF_PDM_RATIO_128X },
#endif
#if NRF_PDM_HAS_RATIO150
		{ 150, NRF_PDM_RATIO_150X },
#endif
#if NRF_PDM_HAS_RATIO192
		{ 192, NRF_PDM_RATIO_192X }
#endif
	};

	for (uint32_t r = 0; r < NRFX_ARRAY_SIZE(ratios) && best_diff != 0; r++)
    {
		ratio = ratios[r].ratio_val;

		if (pdm_prescalers_check(&best_freq, &best_rate, &best_diff,
                                ratio, output_config, prescalers))
        {
			prescalers->ratio = ratios[r].ratio_enum;
		}
	}
#else
    // Ratio value hardcoded in hardware.
	ratio = PDM_RATIO_VALUE;

    pdm_prescalers_check(&best_freq, &best_rate, &best_diff,
                         ratio, output_config, prescalers);
#endif

	if (best_diff == UINT32_MAX) {
		return -EINVAL;
	}

	return 0;
}

void nrfx_pdm_irq_handler(nrfx_pdm_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_pdm_control_block_t * p_cb = &p_instance->cb;

    nrfy_pdm_buffer_t buffer =
    {
        .p_buff = p_cb->buff_address[p_cb->active_buffer],
        .length = p_cb->buff_length[p_cb->active_buffer],
    };

    uint32_t evt_mask = nrfy_pdm_events_process(p_instance->p_reg,
                                                NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STARTED) |
                                                NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STOPPED),
                                                &buffer);

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STARTED))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRF_PDM_EVENT_STARTED));

        uint8_t finished_buffer = p_cb->active_buffer;

        // Check if the next buffer was set before.
        uint8_t next_buffer = (~p_cb->active_buffer) & 0x01;
        if (p_cb->buff_address[next_buffer] ||
            p_cb->op_state == NRFX_PDM_STATE_STARTING)
        {
            nrfx_pdm_evt_t evt;
            evt.error = NRFX_PDM_NO_ERROR;
            p_cb->error = 0;

            // Release the full buffer if ready and request the next one.
            if (p_cb->op_state == NRFX_PDM_STATE_STARTING)
            {
                evt.buffer_released = 0;
                p_cb->op_state = NRFX_PDM_STATE_RUNNING;
            }
            else
            {
                evt.buffer_released = p_cb->buff_address[finished_buffer];
                p_cb->buff_address[finished_buffer] = 0;
                p_cb->active_buffer = next_buffer;
            }
            evt.buffer_requested = true;
            p_cb->event_handler(&evt);
        }
        else
        {
            // No next buffer available. Report an error.
            // Do not request the new buffer as it was already done.
            if (p_cb->error == 0)
            {
                nrfx_pdm_evt_t const evt = {
                    .buffer_requested = false,
                    .buffer_released  = NULL,
                    .error = NRFX_PDM_ERROR_OVERFLOW
                };
                p_cb->error = 1;
                p_cb->event_handler(&evt);
            }
        }

        if (p_cb->op_state == NRFX_PDM_STATE_STARTING)
        {
            p_cb->op_state = NRFX_PDM_STATE_RUNNING;
        }
    }
    else if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STOPPED))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRF_PDM_EVENT_STOPPED));
        nrfy_pdm_disable(p_instance->p_reg);
        p_cb->op_state = NRFX_PDM_STATE_IDLE;

        // Release the buffers.
        nrfx_pdm_evt_t evt;
        evt.error = NRFX_PDM_NO_ERROR;
        evt.buffer_requested = false;
        if (p_cb->buff_address[p_cb->active_buffer])
        {
            evt.buffer_released = p_cb->buff_address[p_cb->active_buffer];
            p_cb->buff_address[p_cb->active_buffer] = 0;
            p_cb->event_handler(&evt);
        }

        uint8_t second_buffer = (~p_cb->active_buffer) & 0x01;
        if (p_cb->buff_address[second_buffer])
        {
            evt.buffer_released = p_cb->buff_address[second_buffer];
            p_cb->buff_address[second_buffer] = 0;
            p_cb->event_handler(&evt);
        }
        p_cb->active_buffer = 0;
    }

    if (p_cb->irq_buff_request)
    {
        nrfx_pdm_evt_t const evt =
        {
            .buffer_requested = true,
            .buffer_released  = NULL,
            .error = NRFX_PDM_NO_ERROR,
        };
        p_cb->irq_buff_request = 0;
        p_cb->event_handler(&evt);
    }
}
