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

#if NRFX_CHECK(NRFX_PDM_ENABLED)

#include <nrfx_pdm.h>
#include <haly/nrfy_pdm.h>
#include <haly/nrfy_gpio.h>

#define NRFX_LOG_MODULE PDM
#include <nrfx_log.h>

#if !NRFX_FEATURE_PRESENT(NRFX_PDM, _ENABLED)
#error "No enabled PDM instances. Check <nrfx_config.h>."
#endif

#define EVT_TO_STR(event)                                       \
    (event == NRF_PDM_EVENT_STARTED ? "NRF_PDM_EVENT_STARTED" : \
    (event == NRF_PDM_EVENT_STOPPED ? "NRF_PDM_EVENT_STOPPED" : \
    (event == NRF_PDM_EVENT_END     ? "NRF_PDM_EVENT_END"     : \
                                      "UNKNOWN EVENT")))


/** @brief PDM driver instance status. */
typedef enum
{
    NRFX_PDM_STATE_IDLE,
    NRFX_PDM_STATE_RUNNING,
    NRFX_PDM_STATE_STARTING,
    NRFX_PDM_STATE_STOPPING
} nrfx_pdm_state_t;

/** @brief PDM driver instance control block.*/
typedef struct
{
    nrfx_pdm_event_handler_t  event_handler;    ///< Event handler function pointer.
    int16_t *                 buff_address[2];  ///< Sample buffers.
    uint16_t                  buff_length[2];   ///< Length of the sample buffers.
    nrfx_drv_state_t          drv_state;        ///< Driver state.
    volatile nrfx_pdm_state_t op_state;         ///< PDM peripheral operation state.
    uint8_t                   active_buffer;    ///< Number of currently active buffer.
    uint8_t                   error;            ///< Driver error flag.
    volatile uint8_t          irq_buff_request; ///< Request the next buffer in the ISR.
    bool                      skip_gpio_cfg;    ///< Do not touch GPIO configuration of used pins.
} nrfx_pdm_cb_t;

#define _NRFX_PDM_CB_INITIALIZER(periph_name, prefix, idx, _) \
    [NRFX_CONCAT(NRFX_, periph_name, idx, _INST_IDX)] = {     \
        .drv_state = NRFX_DRV_STATE_UNINITIALIZED,            \
    },

static nrfx_pdm_cb_t m_cb[NRFX_PDM_ENABLED_COUNT] = {
    NRFX_FOREACH_ENABLED(PDM, _NRFX_PDM_CB_INITIALIZER, (), ())
};

/** @brief Function for getting instance control block.
 *
 * Function is optimized for case when there is only one PDM instance.
 *
 * @param[in] idx Instance index.
 *
 * @return Control block.
 */
static nrfx_pdm_cb_t * get_cb(uint32_t idx)
{
    if (NRFX_PDM_ENABLED_COUNT == 1)
    {
        return &m_cb[0];
    }
    else
    {
        return &m_cb[idx];
    }
}

static void pdm_configure(nrfx_pdm_t const *        p_instance,
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
        NRFX_COND_CODE_1(NRF_PDM_HAS_PDMCLKCTRL, (.clock_freq  = p_config->clock_freq,), ())
        NRFX_COND_CODE_1(NRF_PDM_HAS_PRESCALER, (.prescaler  = p_config->prescaler,), ())
        .gain_l      = p_config->gain_l,
        .gain_r      = p_config->gain_r,
        NRFX_COND_CODE_1(NRF_PDM_HAS_RATIO_CONFIG, (.ratio = p_config->ratio,), ())
        NRFX_COND_CODE_1(NRF_PDM_HAS_SELECTABLE_CLOCK, (.mclksrc = p_config->mclksrc,), ())
        .skip_psel_cfg = p_config->skip_psel_cfg
    };

    nrfy_pdm_periph_configure(p_instance->p_reg, &nrfy_config);

    nrfy_pdm_int_init(p_instance->p_reg,
                      NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED,
                      p_config->interrupt_priority,
                      true);
}

static nrfx_err_t pdm_init(nrfx_pdm_t const *        p_instance,
                           nrfx_pdm_config_t const * p_config,
                           nrfx_pdm_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(event_handler);
    nrfx_err_t err_code;

    nrfx_pdm_cb_t * p_cb = get_cb(p_instance->drv_inst_idx);

    if (p_cb->drv_state != NRFX_DRV_STATE_UNINITIALIZED)
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
            err_code = NRFX_ERROR_INVALID_PARAM;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
#if NRF_PDM_HAS_PRESCALER
        if (p_config->prescaler < NRF_PDM_PRESCALER_MIN ||
            p_config->prescaler > NRF_PDM_PRESCALER_MAX)
        {
            err_code = NRFX_ERROR_INVALID_PARAM;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
#endif
        pdm_configure(p_instance, p_config);
    }

    p_cb->drv_state = NRFX_DRV_STATE_INITIALIZED;

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t pdm_reconfigure(nrfx_pdm_t const *        p_instance,
                                  nrfx_pdm_config_t const * p_config)
{
    nrfx_pdm_cb_t * p_cb = get_cb(p_instance->drv_inst_idx);

    NRFX_ASSERT(p_config);
    if (p_cb->drv_state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }

    if (p_config->gain_l > NRF_PDM_GAIN_MAXIMUM ||
        p_config->gain_r > NRF_PDM_GAIN_MAXIMUM)
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

#if NRF_PDM_HAS_PRESCALER
    if (p_config->prescaler < NRF_PDM_PRESCALER_MIN ||
        p_config->prescaler > NRF_PDM_PRESCALER_MAX)
    {
        return NRFX_ERROR_INVALID_PARAM;
    }
#endif

    if (p_cb->op_state != NRFX_PDM_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }
    nrfy_pdm_disable(p_instance->p_reg);
    pdm_configure(p_instance, p_config);
    nrfy_pdm_enable(p_instance->p_reg);
    return NRFX_SUCCESS;
}

static void pdm_uninit(nrfx_pdm_t const * p_instance)
{
    nrfy_pdm_int_uninit(p_instance->p_reg);
    nrfy_pdm_disable(p_instance->p_reg);

    nrfx_pdm_cb_t * p_cb = get_cb(p_instance->drv_inst_idx);

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
    return (get_cb(p_instance->drv_inst_idx)->drv_state != NRFX_DRV_STATE_UNINITIALIZED);
}

static void pdm_buf_request(nrfx_pdm_t const * p_instance)
{
    get_cb(p_instance->drv_inst_idx)->irq_buff_request = 1;
    NRFY_IRQ_PENDING_SET(nrfx_get_irq_number(p_instance->p_reg));
}

static nrfx_err_t pdm_start(nrfx_pdm_t const * p_instance)
{
    nrfx_pdm_cb_t * p_cb = get_cb(p_instance->drv_inst_idx);

    NRFX_ASSERT(p_cb->drv_state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code;

    if (p_cb->op_state != NRFX_PDM_STATE_IDLE)
    {
        if (p_cb->op_state == NRFX_PDM_STATE_RUNNING)
        {
            err_code = NRFX_SUCCESS;
            NRFX_LOG_INFO("Function: %s, error code: %s.",
                          __func__,
                          NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    p_cb->op_state = NRFX_PDM_STATE_STARTING;
    pdm_buf_request(p_instance);

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t pdm_buffer_set(nrfx_pdm_t const * p_instance,
                                 int16_t *          buffer,
                                 uint16_t           buffer_length)
{
    nrfx_pdm_cb_t * p_cb = get_cb(p_instance->drv_inst_idx);

    if (p_cb->drv_state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }
    if (p_cb->op_state == NRFX_PDM_STATE_STOPPING)
    {
        return NRFX_ERROR_BUSY;
    }
    if ((buffer == NULL) || (buffer_length > NRFX_PDM_MAX_BUFFER_SIZE))
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    nrfx_err_t err_code = NRFX_SUCCESS;

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
        err_code = NRFX_ERROR_BUSY;
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

static nrfx_err_t pdm_stop(nrfx_pdm_t const * p_instance)
{
    nrfx_pdm_cb_t * p_cb = get_cb(p_instance->drv_inst_idx);

    NRFX_ASSERT(p_cb->drv_state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code;

    if (p_cb->op_state != NRFX_PDM_STATE_RUNNING)
    {
        if (p_cb->op_state == NRFX_PDM_STATE_IDLE ||
            p_cb->op_state == NRFX_PDM_STATE_STARTING)
        {
            nrfy_pdm_disable(p_instance->p_reg);
            p_cb->op_state = NRFX_PDM_STATE_IDLE;
            err_code = NRFX_SUCCESS;
            NRFX_LOG_INFO("Function: %s, error code: %s.",
                          __func__,
                          NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    p_cb->drv_state = NRFX_DRV_STATE_INITIALIZED;
    p_cb->op_state = NRFX_PDM_STATE_STOPPING;

    nrfy_pdm_abort(p_instance->p_reg, NULL);
    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

#if NRFX_API_VER_AT_LEAST(3, 7, 0)

nrfx_err_t nrfx_pdm_init(nrfx_pdm_t const *        p_instance,
                         nrfx_pdm_config_t const * p_config,
                         nrfx_pdm_event_handler_t  event_handler)
{
    return pdm_init(p_instance, p_config, event_handler);
}

nrfx_err_t nrfx_pdm_reconfigure(nrfx_pdm_t const *        p_instance,
                                nrfx_pdm_config_t const * p_config)
{
    return pdm_reconfigure(p_instance, p_config);
}

void nrfx_pdm_uninit(nrfx_pdm_t const * p_instance)
{
    pdm_uninit(p_instance);
}

bool nrfx_pdm_init_check(nrfx_pdm_t const * p_instance)
{
    return pdm_init_check(p_instance);
}

nrfx_err_t nrfx_pdm_start(nrfx_pdm_t const * p_instance)
{
    return pdm_start(p_instance);
}

nrfx_err_t nrfx_pdm_buffer_set(nrfx_pdm_t const * p_instance,
                               int16_t *          buffer,
                               uint16_t           buffer_length)
{
    return pdm_buffer_set(p_instance, buffer, buffer_length);
}

nrfx_err_t nrfx_pdm_stop(nrfx_pdm_t const * p_instance)
{
    return pdm_stop(p_instance);
}

#else

nrfx_pdm_t const pdm_instance = NRFX_PDM_INSTANCE(NRF_PDM_INDEX);

nrfx_err_t nrfx_pdm_init(nrfx_pdm_config_t const * p_config,
                         nrfx_pdm_event_handler_t  event_handler)
{
    return pdm_init(&pdm_instance, p_config, event_handler);
}

nrfx_err_t nrfx_pdm_reconfigure(nrfx_pdm_config_t const * p_config)
{
    return pdm_reconfigure(&pdm_instance, p_config);
}

void nrfx_pdm_uninit(void)
{
    pdm_uninit(&pdm_instance);
}

bool nrfx_pdm_init_check(void)
{
    return pdm_init_check(&pdm_instance);
}

nrfx_err_t nrfx_pdm_start(void)
{
    return pdm_start(&pdm_instance);
}

nrfx_err_t nrfx_pdm_buffer_set(int16_t * buffer, uint16_t buffer_length)
{
    return pdm_buffer_set(&pdm_instance, buffer, buffer_length);
}

nrfx_err_t nrfx_pdm_stop(void)
{
    return pdm_stop(&pdm_instance);
}

void nrfx_pdm_irq_handler(void)
{
    nrfx_pdm_cb_t * p_cb = get_cb(0);

    nrfy_pdm_buffer_t buffer =
    {
        .p_buff = p_cb->buff_address[p_cb->active_buffer],
        .length = p_cb->buff_length[p_cb->active_buffer],
    };

    uint32_t evt_mask = nrfy_pdm_events_process(NRF_PDM0,
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
        nrfy_pdm_disable(NRF_PDM0);
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

#endif // NRFX_API_VER_AT_LEAST(3, 7, 0)

static void irq_handler(NRF_PDM_Type * p_pdm, nrfx_pdm_cb_t * p_cb)
{
    nrfy_pdm_buffer_t buffer =
    {
        .p_buff = p_cb->buff_address[p_cb->active_buffer],
        .length = p_cb->buff_length[p_cb->active_buffer],
    };

    uint32_t evt_mask = nrfy_pdm_events_process(p_pdm,
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
        nrfy_pdm_disable(p_pdm);
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

NRFX_INSTANCE_IRQ_HANDLERS(PDM, pdm)

#endif // NRFX_CHECK(NRFX_PDM_ENABLED)
