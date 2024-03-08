/*
 * Copyright (c) 2015 - 2024, Nordic Semiconductor ASA
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

#define EVT_TO_STR(event)                                       \
    (event == NRF_PDM_EVENT_STARTED ? "NRF_PDM_EVENT_STARTED" : \
    (event == NRF_PDM_EVENT_STOPPED ? "NRF_PDM_EVENT_STOPPED" : \
    (event == NRF_PDM_EVENT_END     ? "NRF_PDM_EVENT_END"     : \
                                      "UNKNOWN EVENT")))


/** @brief PDM interface status. */
typedef enum
{
    NRFX_PDM_STATE_IDLE,
    NRFX_PDM_STATE_RUNNING,
    NRFX_PDM_STATE_STARTING,
    NRFX_PDM_STATE_STOPPING
} nrfx_pdm_state_t;

/** @brief PDM interface control block.*/
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

static nrfx_pdm_cb_t m_cb;

static void pdm_configure(nrfx_pdm_config_t const * p_config)
{
    if (!p_config->skip_gpio_cfg)
    {
        nrfy_gpio_pin_clear(p_config->clk_pin);
        nrfy_gpio_cfg_output(p_config->clk_pin);
        nrfy_gpio_cfg_input(p_config->din_pin, NRF_GPIO_PIN_NOPULL);
    }
    if (!p_config->skip_psel_cfg)
    {
        nrf_pdm_psel_connect(NRF_PDM0,
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
        .clock_freq  = p_config->clock_freq,
        .gain_l      = p_config->gain_l,
        .gain_r      = p_config->gain_r,
        NRFX_COND_CODE_1(NRF_PDM_HAS_RATIO_CONFIG, (.ratio = p_config->ratio,), ())
        NRFX_COND_CODE_1(NRF_PDM_HAS_MCLKCONFIG, (.mclksrc = p_config->mclksrc,), ())
        .skip_psel_cfg = p_config->skip_psel_cfg
    };

    nrfy_pdm_periph_configure(NRF_PDM0, &nrfy_config);

    nrfy_pdm_int_init(NRF_PDM0,
                      NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED,
                      p_config->interrupt_priority,
                      true);
}

nrfx_err_t nrfx_pdm_init(nrfx_pdm_config_t const * p_config,
                         nrfx_pdm_event_handler_t  event_handler)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(event_handler);
    nrfx_err_t err_code;

    if (m_cb.drv_state != NRFX_DRV_STATE_UNINITIALIZED)
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

    m_cb.buff_address[0] = 0;
    m_cb.buff_address[1] = 0;
    m_cb.active_buffer = 0;
    m_cb.error = 0;
    m_cb.event_handler = event_handler;
    m_cb.op_state = NRFX_PDM_STATE_IDLE;

    if (p_config)
    {
        m_cb.skip_gpio_cfg = p_config->skip_gpio_cfg;

        if (p_config->gain_l > NRF_PDM_GAIN_MAXIMUM ||
            p_config->gain_r > NRF_PDM_GAIN_MAXIMUM)
        {
            err_code = NRFX_ERROR_INVALID_PARAM;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
        pdm_configure(p_config);
    }

    m_cb.drv_state = NRFX_DRV_STATE_INITIALIZED;

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_pdm_reconfigure(nrfx_pdm_config_t const * p_config)
{
    NRFX_ASSERT(p_config);
    if (m_cb.drv_state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }

    if (p_config->gain_l > NRF_PDM_GAIN_MAXIMUM ||
        p_config->gain_r > NRF_PDM_GAIN_MAXIMUM)
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    if (m_cb.op_state != NRFX_PDM_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }
    nrfy_pdm_disable(NRF_PDM0);
    pdm_configure(p_config);
    nrfy_pdm_enable(NRF_PDM0);
    return NRFX_SUCCESS;
}

void nrfx_pdm_uninit(void)
{
    nrfy_pdm_int_uninit(NRF_PDM0);
    nrfy_pdm_disable(NRF_PDM0);

    if (!m_cb.skip_gpio_cfg)
    {
        nrfy_pdm_pins_t pins;
        nrfy_pdm_pins_get(NRF_PDM0, &pins);
        nrfy_gpio_cfg_default(pins.clk_pin);
        nrfy_gpio_cfg_default(pins.din_pin);
    }

    m_cb.drv_state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_pdm_init_check(void)
{
    return (m_cb.drv_state != NRFX_DRV_STATE_UNINITIALIZED);
}

static void pdm_start(void)
{
    m_cb.drv_state = NRFX_DRV_STATE_POWERED_ON;
    nrfy_pdm_enable(NRF_PDM0);
    nrfy_pdm_start(NRF_PDM0, NULL);
}

static void pdm_buf_request(void)
{
    m_cb.irq_buff_request = 1;
    NRFY_IRQ_PENDING_SET(nrfx_get_irq_number(NRF_PDM0));
}

nrfx_err_t nrfx_pdm_start(void)
{
    NRFX_ASSERT(m_cb.drv_state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code;

    if (m_cb.op_state != NRFX_PDM_STATE_IDLE)
    {
        if (m_cb.op_state == NRFX_PDM_STATE_RUNNING)
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

    m_cb.op_state = NRFX_PDM_STATE_STARTING;
    pdm_buf_request();

    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_pdm_buffer_set(int16_t * buffer, uint16_t buffer_length)
{
    if (m_cb.drv_state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }
    if (m_cb.op_state == NRFX_PDM_STATE_STOPPING)
    {
        return NRFX_ERROR_BUSY;
    }
    if ((buffer == NULL) || (buffer_length > NRFX_PDM_MAX_BUFFER_SIZE))
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    nrfx_err_t err_code = NRFX_SUCCESS;

    // Enter the PDM critical section.
    NRFY_IRQ_DISABLE(nrfx_get_irq_number(NRF_PDM0));

    uint8_t next_buffer = (~m_cb.active_buffer) & 0x01;
    if (m_cb.op_state == NRFX_PDM_STATE_STARTING)
    {
        next_buffer = 0;
    }

    if (m_cb.buff_address[next_buffer])
    {
        // Buffer already set.
        err_code = NRFX_ERROR_BUSY;
    }
    else
    {
        m_cb.buff_address[next_buffer] = buffer;
        m_cb.buff_length[next_buffer] = buffer_length;

        nrfy_pdm_buffer_t nrfy_buffer =
        {
            .p_buff = buffer,
            .length = buffer_length,
        };
        nrfy_pdm_buffer_set(NRF_PDM0, &nrfy_buffer);

        if (m_cb.drv_state != NRFX_DRV_STATE_POWERED_ON)
        {
            pdm_start();
        }
    }

    NRFY_IRQ_ENABLE(nrfx_get_irq_number(NRF_PDM0));
    return err_code;
}

nrfx_err_t nrfx_pdm_stop(void)
{
    NRFX_ASSERT(m_cb.drv_state != NRFX_DRV_STATE_UNINITIALIZED);
    nrfx_err_t err_code;

    if (m_cb.op_state != NRFX_PDM_STATE_RUNNING)
    {
        if (m_cb.op_state == NRFX_PDM_STATE_IDLE ||
            m_cb.op_state == NRFX_PDM_STATE_STARTING)
        {
            nrfy_pdm_disable(NRF_PDM0);
            m_cb.op_state = NRFX_PDM_STATE_IDLE;
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
    m_cb.drv_state = NRFX_DRV_STATE_INITIALIZED;
    m_cb.op_state = NRFX_PDM_STATE_STOPPING;

    nrfy_pdm_abort(NRF_PDM0, NULL);
    err_code = NRFX_SUCCESS;
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

void nrfx_pdm_irq_handler(void)
{
    nrfy_pdm_buffer_t buffer =
    {
        .p_buff = m_cb.buff_address[m_cb.active_buffer],
        .length = m_cb.buff_length[m_cb.active_buffer],
    };

    uint32_t evt_mask = nrfy_pdm_events_process(NRF_PDM0,
                                                NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STARTED) |
                                                NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STOPPED),
                                                &buffer);

    if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STARTED))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRF_PDM_EVENT_STARTED));

        uint8_t finished_buffer = m_cb.active_buffer;

        // Check if the next buffer was set before.
        uint8_t next_buffer = (~m_cb.active_buffer) & 0x01;
        if (m_cb.buff_address[next_buffer] ||
            m_cb.op_state == NRFX_PDM_STATE_STARTING)
        {
            nrfx_pdm_evt_t evt;
            evt.error = NRFX_PDM_NO_ERROR;
            m_cb.error = 0;

            // Release the full buffer if ready and request the next one.
            if (m_cb.op_state == NRFX_PDM_STATE_STARTING)
            {
                evt.buffer_released = 0;
                m_cb.op_state = NRFX_PDM_STATE_RUNNING;
            }
            else
            {
                evt.buffer_released = m_cb.buff_address[finished_buffer];
                m_cb.buff_address[finished_buffer] = 0;
                m_cb.active_buffer = next_buffer;
            }
            evt.buffer_requested = true;
            m_cb.event_handler(&evt);
        }
        else
        {
            // No next buffer available. Report an error.
            // Do not request the new buffer as it was already done.
            if (m_cb.error == 0)
            {
                nrfx_pdm_evt_t const evt = {
                    .buffer_requested = false,
                    .buffer_released  = NULL,
                    .error = NRFX_PDM_ERROR_OVERFLOW
                };
                m_cb.error = 1;
                m_cb.event_handler(&evt);
            }
        }

        if (m_cb.op_state == NRFX_PDM_STATE_STARTING)
        {
            m_cb.op_state = NRFX_PDM_STATE_RUNNING;
        }
    }
    else if (evt_mask & NRFY_EVENT_TO_INT_BITMASK(NRF_PDM_EVENT_STOPPED))
    {
        NRFX_LOG_DEBUG("Event: %s.", EVT_TO_STR(NRF_PDM_EVENT_STOPPED));
        nrfy_pdm_disable(NRF_PDM0);
        m_cb.op_state = NRFX_PDM_STATE_IDLE;

        // Release the buffers.
        nrfx_pdm_evt_t evt;
        evt.error = NRFX_PDM_NO_ERROR;
        evt.buffer_requested = false;
        if (m_cb.buff_address[m_cb.active_buffer])
        {
            evt.buffer_released = m_cb.buff_address[m_cb.active_buffer];
            m_cb.buff_address[m_cb.active_buffer] = 0;
            m_cb.event_handler(&evt);
        }

        uint8_t second_buffer = (~m_cb.active_buffer) & 0x01;
        if (m_cb.buff_address[second_buffer])
        {
            evt.buffer_released = m_cb.buff_address[second_buffer];
            m_cb.buff_address[second_buffer] = 0;
            m_cb.event_handler(&evt);
        }
        m_cb.active_buffer = 0;
    }

    if (m_cb.irq_buff_request)
    {
        nrfx_pdm_evt_t const evt =
        {
            .buffer_requested = true,
            .buffer_released  = NULL,
            .error = NRFX_PDM_NO_ERROR,
        };
        m_cb.irq_buff_request = 0;
        m_cb.event_handler(&evt);
    }
}

#endif // NRFX_CHECK(NRFX_PDM_ENABLED)
