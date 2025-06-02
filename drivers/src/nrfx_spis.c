/*
 * Copyright (c) 2013 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_SPIS_ENABLED)

#if !NRFX_FEATURE_PRESENT(NRFX_SPIS, _ENABLED)
#error "No enabled SPIS instances. Check <nrfx_config.h>."
#endif

#include <nrfx_spis.h>
#include "prs/nrfx_prs.h"

#define NRFX_LOG_MODULE SPIS
#include <nrfx_log.h>

#define EVT_TO_STR(event)                                           \
    (event == NRF_SPIS_EVENT_ACQUIRED ? "NRF_SPIS_EVENT_ACQUIRED" : \
    (event == NRF_SPIS_EVENT_END      ? "NRF_SPIS_EVENT_END"      : \
                                        "UNKNOWN ERROR"))

#define SPISX_LENGTH_VALIDATE(periph_name, prefix, i, drv_inst_idx, rx_len, tx_len) \
    (((drv_inst_idx) == NRFX_CONCAT(NRFX_, periph_name, prefix, i, _INST_IDX)) && \
     NRFX_EASYDMA_LENGTH_VALIDATE(NRFX_CONCAT(periph_name, prefix, i), rx_len, tx_len))

#define SPIS_LENGTH_VALIDATE(drv_inst_idx, rx_len, tx_len)    \
        (NRFX_FOREACH_ENABLED(SPIS, SPISX_LENGTH_VALIDATE, (||), (0), drv_inst_idx, rx_len, tx_len))

#if NRFX_CHECK(NRFX_SPIS_NRF52_ANOMALY_109_WORKAROUND_ENABLED)
#include <nrfx_gpiote.h>
#define USE_DMA_ISSUE_WORKAROUND
// This handler is called by the GPIOTE driver when a falling edge is detected
// on the CSN line. There is no need to do anything here. The handling of the
// interrupt itself provides a protection for DMA transfers.
static void csn_event_handler(nrfx_gpiote_pin_t     pin,
                              nrfx_gpiote_trigger_t trigger,
                              void *                p_context)
{
    (void)pin;
    (void)trigger;
    (void)p_context;
}
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
static nrfx_gpiote_t const gpiote = NRFX_GPIOTE_INSTANCE(0);
#endif
#endif


/** @brief States of the SPI transaction state machine. */
typedef enum
{
    SPIS_STATE_INIT,                                 /**< Initialization state. In this state the module waits for a call to @ref spi_slave_buffers_set. */
    SPIS_BUFFER_RESOURCE_REQUESTED,                  /**< State where the configuration of the memory buffers, which are to be used in SPI transaction, has started. */
    SPIS_BUFFER_RESOURCE_CONFIGURED,                 /**< State where the configuration of the memory buffers, which are to be used in SPI transaction, has completed. */
    SPIS_XFER_COMPLETED                              /**< State where SPI transaction has been completed. */
} nrfx_spis_state_t;

/** @brief SPIS control block - driver instance local data. */
typedef struct
{
    volatile uint32_t          tx_buffer_size;  //!< SPI slave TX buffer size in bytes.
    volatile uint32_t          rx_buffer_size;  //!< SPI slave RX buffer size in bytes.
    nrfx_spis_event_handler_t  handler;         //!< SPI event handler.
    volatile const uint8_t *   tx_buffer;       //!< SPI slave TX buffer.
    volatile uint8_t *         rx_buffer;       //!< SPI slave RX buffer.
    nrfx_drv_state_t           state;           //!< driver initialization state.
    volatile nrfx_spis_state_t spi_state;       //!< SPI slave state.
    void *                     p_context;       //!< Context set on initialization.
    bool                       skip_gpio_cfg;
#if defined(USE_DMA_ISSUE_WORKAROUND)
    uint32_t                   csn_pin;
    uint8_t                    gpiote_ch;
#endif
} spis_cb_t;

static spis_cb_t m_cb[NRFX_SPIS_ENABLED_COUNT];

static void pins_configure(nrfx_spis_config_t const * p_config)
{
    nrf_gpio_cfg(p_config->sck_pin,
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_SPIS_CLOCKPIN_SCK_NEEDED)
    nrfy_gpio_pin_clock_set(p_config->sck_pin, true);
#endif

    if (p_config->mosi_pin != NRF_SPIS_PIN_NOT_CONNECTED)
    {
        nrf_gpio_cfg(p_config->mosi_pin,
                     NRF_GPIO_PIN_DIR_INPUT,
                     NRF_GPIO_PIN_INPUT_CONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     NRF_GPIO_PIN_S0S1,
                     NRF_GPIO_PIN_NOSENSE);
    }

    if (p_config->miso_pin != NRF_SPIS_PIN_NOT_CONNECTED)
    {
        nrf_gpio_cfg(p_config->miso_pin,
                     NRF_GPIO_PIN_DIR_INPUT,
                     NRF_GPIO_PIN_INPUT_CONNECT,
                     NRF_GPIO_PIN_NOPULL,
                     p_config->miso_drive,
                     NRF_GPIO_PIN_NOSENSE);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_SPIS_CLOCKPIN_MISO_NEEDED)
        nrfy_gpio_pin_clock_set(p_config->miso_pin, true);
#endif
    }

    nrf_gpio_cfg(p_config->csn_pin,
                 NRF_GPIO_PIN_DIR_INPUT,
                 NRF_GPIO_PIN_INPUT_CONNECT,
                 p_config->csn_pullup,
                 NRF_GPIO_PIN_S0S1,
                 NRF_GPIO_PIN_NOSENSE);
}

static bool spis_configure(nrfx_spis_t const *        p_instance,
                           nrfx_spis_config_t const * p_config)
{
    NRF_SPIS_Type * p_spis = p_instance->p_reg;
    if (p_config->mode > NRF_SPIS_MODE_3)
    {
        return false;
    }

    if (!p_config->skip_gpio_cfg)
    {
        pins_configure(p_config);
    }

    if (!p_config->skip_psel_cfg)
    {
        nrf_spis_pins_set(p_spis,
                          p_config->sck_pin,
                          p_config->mosi_pin,
                          p_config->miso_pin,
                          p_config->csn_pin);
    }

#if defined(USE_DMA_ISSUE_WORKAROUND)
    spis_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    // If the GPIOTE channel was already used with a CSN pin, deinitialize it
    // first as that pin number may be different now.
    if (p_cb->csn_pin != NRF_SPIS_PIN_NOT_CONNECTED)
    {
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
        nrfx_gpiote_pin_uninit(&gpiote, p_cb->csn_pin);
#else
        nrfx_gpiote_pin_uninit(p_cb->csn_pin);
#endif
        p_cb->csn_pin = NRF_SPIS_PIN_NOT_CONNECTED;
    }

    // Get the CSN pin number from the PSEL register in the peripheral
    // as in p_config that pin number may be omitted.
    uint32_t csn_pin = nrf_spis_csn_pin_get(p_spis);

    // Configure a GPIOTE channel to generate interrupts on each falling edge
    // on the CSN line. Handling of these interrupts will make the CPU active
    // and thus will protect the DMA transfers started by SPIS right after it
    // is selected for communication.
    nrfx_gpiote_trigger_config_t trig_config = {
        .trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
        .p_in_channel = &p_cb->gpiote_ch
    };
    nrfx_gpiote_handler_config_t hndl_config = {
        .handler = csn_event_handler
    };

#if NRFX_API_VER_AT_LEAST(3, 2, 0)
    nrfx_gpiote_input_pin_config_t config = {
        .p_pull_config    = NULL,
        .p_trigger_config = &trig_config,
        .p_handler_config = &hndl_config
    };
    nrfx_err_t err_code = nrfx_gpiote_input_configure(&gpiote, csn_pin, &config);
#else
    nrfx_err_t err_code = nrfx_gpiote_input_configure(csn_pin, NULL, &trig_config, &hndl_config);
#endif
    if (err_code != NRFX_SUCCESS)
    {
        NRFX_LOG_ERROR("Function: %s, error code: %s.",
                       __func__,
                       NRFX_LOG_ERROR_STRING_GET(err_code));
        return false;
    }

#if NRFX_API_VER_AT_LEAST(3, 2, 0)
    nrfx_gpiote_trigger_enable(&gpiote, csn_pin, true);
#else
    nrfx_gpiote_trigger_enable(csn_pin, true);
#endif

    p_cb->csn_pin = csn_pin;
#endif

    // Configure SPI mode.
    nrf_spis_configure(p_spis, p_config->mode, p_config->bit_order);

    // Configure DEF and ORC characters.
    nrf_spis_def_set(p_spis, p_config->def);
    nrf_spis_orc_set(p_spis, p_config->orc);

    // Clear possible pending events.
    nrf_spis_event_clear(p_spis, NRF_SPIS_EVENT_END);
    nrf_spis_event_clear(p_spis, NRF_SPIS_EVENT_ACQUIRED);

    // Enable END_ACQUIRE shortcut.
    nrf_spis_shorts_enable(p_spis, NRF_SPIS_SHORT_END_ACQUIRE);

    NRFX_IRQ_PRIORITY_SET(nrfx_get_irq_number(p_instance->p_reg), p_config->irq_priority);
    NRFX_IRQ_ENABLE(nrfx_get_irq_number(p_instance->p_reg));

    return true;
}

nrfx_err_t nrfx_spis_init(nrfx_spis_t const *        p_instance,
                          nrfx_spis_config_t const * p_config,
                          nrfx_spis_event_handler_t  event_handler,
                          void *                     p_context)
{
    NRFX_ASSERT(p_config);
    NRFX_ASSERT(event_handler);

    spis_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code;

    NRF_SPIS_Type * p_spis = p_instance->p_reg;

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

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    static nrfx_irq_handler_t const irq_handlers[NRFX_SPIS_ENABLED_COUNT] = {
        NRFX_INSTANCE_IRQ_HANDLERS_LIST(SPIS, spis)
    };
    if (nrfx_prs_acquire(p_spis,
            irq_handlers[p_instance->drv_inst_idx]) != NRFX_SUCCESS)
    {
        err_code = NRFX_ERROR_BUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED)

    p_cb->handler   = event_handler;
    p_cb->p_context = p_context;

#if defined(USE_DMA_ISSUE_WORKAROUND)
    p_cb->csn_pin = NRF_SPIS_PIN_NOT_CONNECTED;

    // Allocate a GPIOTE channel that will be used to handle the anomaly 109
    // (the GPIOTE driver may be already initialized at this point, by this
    // driver when another SPIS instance is used or by an application code,
    // so just ignore the returned value here).
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
    (void)nrfx_gpiote_init(&gpiote, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    err_code = nrfx_gpiote_channel_alloc(&gpiote, &p_cb->gpiote_ch);
#else
    (void)nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    err_code = nrfx_gpiote_channel_alloc(&p_cb->gpiote_ch);
#endif

    if (err_code != NRFX_SUCCESS)
    {
#if NRFX_CHECK(NRFX_PRS_ENABLED)
        nrfx_prs_release(p_spis);
#endif
        err_code = NRFX_ERROR_INTERNAL;
        NRFX_LOG_ERROR("Function: %s, error code: %s.",
                        __func__,
                        NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif

    if (p_config)
    {
        p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;

        if (!spis_configure(p_instance, p_config))
        {
#if defined(USE_DMA_ISSUE_WORKAROUND)
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
            nrfx_gpiote_channel_free(&gpiote, p_cb->gpiote_ch);
#else
            nrfx_gpiote_channel_free(p_cb->gpiote_ch);
#endif
#endif
            err_code = NRFX_ERROR_INVALID_PARAM;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                            __func__,
                            NRFX_LOG_ERROR_STRING_GET(err_code));
#if NRFX_CHECK(NRFX_PRS_ENABLED)
            nrfx_prs_release(p_spis);
#endif
            return err_code;
        }
    }

    nrf_spis_rx_buffer_set(p_spis, NULL, 0);
    nrf_spis_tx_buffer_set(p_spis, NULL, 0);

    p_cb->spi_state = SPIS_STATE_INIT;
    // Enable IRQ.
    nrf_spis_int_enable(p_spis, NRF_SPIS_INT_ACQUIRED_MASK |
                                NRF_SPIS_INT_END_MASK);

    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    // Enable SPI slave device.
    nrf_spis_enable(p_spis);

    NRFX_LOG_INFO("Initialized.");
    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_spis_reconfigure(nrfx_spis_t const *        p_instance,
                                 nrfx_spis_config_t const * p_config)
{
    NRFX_ASSERT(p_config);

    nrfx_err_t err_code;
    spis_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }
    nrf_spis_disable(p_instance->p_reg);
    if (spis_configure(p_instance, p_config))
    {
        err_code = NRFX_SUCCESS;
    }
    else
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    nrf_spis_enable(p_instance->p_reg);
    return err_code;
}

void nrfx_spis_uninit(nrfx_spis_t const * p_instance)
{
    spis_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    NRF_SPIS_Type * p_spis = p_instance->p_reg;

#if defined(USE_DMA_ISSUE_WORKAROUND)
    if (p_cb->csn_pin != NRF_SPIS_PIN_NOT_CONNECTED)
    {
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
        nrfx_gpiote_pin_uninit(&gpiote, p_cb->csn_pin);
#else
        nrfx_gpiote_pin_uninit(p_cb->csn_pin);
#endif
    }
#if NRFX_API_VER_AT_LEAST(3, 2, 0)
    nrfx_gpiote_channel_free(&gpiote, p_cb->gpiote_ch);
#else
    nrfx_gpiote_channel_free(p_cb->gpiote_ch);
#endif
#endif

    #define DISABLE_ALL 0xFFFFFFFF
    nrf_spis_disable(p_spis);
    NRFX_IRQ_DISABLE(nrfx_get_irq_number(p_instance->p_reg));
    nrf_spis_int_disable(p_spis, DISABLE_ALL);
    #undef  DISABLE_ALL

    if (!p_cb->skip_gpio_cfg)
    {
        nrf_gpio_cfg_default(nrf_spis_sck_pin_get(p_spis));
        nrf_gpio_cfg_default(nrf_spis_csn_pin_get(p_spis));

        uint32_t miso_pin = nrf_spis_miso_pin_get(p_spis);
        if (miso_pin != NRF_SPIS_PIN_NOT_CONNECTED)
        {
            nrf_gpio_cfg_default(miso_pin);
        }

        uint32_t mosi_pin = nrf_spis_mosi_pin_get(p_spis);
        if (mosi_pin != NRF_SPIS_PIN_NOT_CONNECTED)
        {
            nrf_gpio_cfg_default(mosi_pin);
        }
    }

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(p_spis);
#endif

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_spis_init_check(nrfx_spis_t const * p_instance)
{
    spis_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    return (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
}

/** @brief Function for executing the state entry action. */
static void spis_state_entry_action_execute(NRF_SPIS_Type * p_spis,
                                            spis_cb_t     * p_cb)
{
    nrfx_spis_evt_t event;

    switch (p_cb->spi_state)
    {
        case SPIS_BUFFER_RESOURCE_REQUESTED:
            nrf_spis_task_trigger(p_spis, NRF_SPIS_TASK_ACQUIRE);
            break;

        case SPIS_BUFFER_RESOURCE_CONFIGURED:
            event.evt_type  = NRFX_SPIS_BUFFERS_SET_DONE;
            event.rx_amount = 0;
            event.tx_amount = 0;

            NRFX_ASSERT(p_cb->handler != NULL);
            p_cb->handler(&event, p_cb->p_context);
            break;

        case SPIS_XFER_COMPLETED:
            event.evt_type    = NRFX_SPIS_XFER_DONE;
            event.rx_amount   = nrf_spis_rx_amount_get(p_spis);
            event.tx_amount   = nrf_spis_tx_amount_get(p_spis);

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif
            event.p_tx_buf    = nrf_spis_tx_buffer_get(p_spis);
            event.tx_buf_size = nrf_spis_tx_maxcnt_get(p_spis);
            event.p_rx_buf    = nrf_spis_rx_buffer_get(p_spis);
            event.rx_buf_size = nrf_spis_rx_maxcnt_get(p_spis);
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

            NRFX_LOG_INFO("Transfer rx_len:%d.", event.rx_amount);
            NRFX_LOG_DEBUG("Rx data:");
            NRFX_LOG_HEXDUMP_DEBUG((uint8_t const *)event.p_rx_buf, event.rx_buf_size);
            NRFX_ASSERT(p_cb->handler != NULL);
            p_cb->handler(&event, p_cb->p_context);
            break;

        default:
            // No implementation required.
            break;
    }
}

/** @brief Function for changing the state of the SPI state machine.
 *
 * @param[in] p_spis    SPIS instance register.
 * @param[in] p_cb      SPIS instance control block.
 * @param[in] new_state State where the state machine transits to.
 */
static void spis_state_change(NRF_SPIS_Type   * p_spis,
                              spis_cb_t       * p_cb,
                              nrfx_spis_state_t new_state)
{
    p_cb->spi_state = new_state;
    spis_state_entry_action_execute(p_spis, p_cb);
}

nrfx_err_t nrfx_spis_buffers_set(nrfx_spis_t const * p_instance,
                                 uint8_t const *     p_tx_buffer,
                                 size_t              tx_buffer_length,
                                 uint8_t *           p_rx_buffer,
                                 size_t              rx_buffer_length)
{
    spis_cb_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_tx_buffer != NULL || tx_buffer_length == 0);
    NRFX_ASSERT(p_rx_buffer != NULL || rx_buffer_length == 0);

    if (!SPIS_LENGTH_VALIDATE(p_instance->drv_inst_idx,
                              rx_buffer_length,
                              tx_buffer_length))
    {
        return NRFX_ERROR_INVALID_LENGTH;
    }

    // EasyDMA requires that transfer buffers are placed in Data RAM region;
    // signal error if they are not.
    if ((p_tx_buffer != NULL && !nrfx_is_in_ram(p_tx_buffer)) ||
        (p_rx_buffer != NULL && !nrfx_is_in_ram(p_rx_buffer)))
    {
        err_code = NRFX_ERROR_INVALID_ADDR;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    switch (p_cb->spi_state)
    {
        case SPIS_STATE_INIT:
        case SPIS_XFER_COMPLETED:
        case SPIS_BUFFER_RESOURCE_CONFIGURED:
            p_cb->tx_buffer      = p_tx_buffer;
            p_cb->rx_buffer      = p_rx_buffer;
            p_cb->tx_buffer_size = tx_buffer_length;
            p_cb->rx_buffer_size = rx_buffer_length;
            err_code             = NRFX_SUCCESS;

            spis_state_change(p_instance->p_reg, p_cb, SPIS_BUFFER_RESOURCE_REQUESTED);
            break;

        case SPIS_BUFFER_RESOURCE_REQUESTED:
            err_code = NRFX_ERROR_INVALID_STATE;
            break;

        default:
            // @note: execution of this code path would imply internal error in the design.
            err_code = NRFX_ERROR_INTERNAL;
            break;
    }

    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static void irq_handler(NRF_SPIS_Type * p_spis, spis_cb_t * p_cb)
{
    // @note: as multiple events can be pending for processing, the correct event processing order
    // is as follows:
    // - SPI transaction complete event.
    // - SPI semaphore acquired event.

    // Check for SPI transaction complete event.
    if (nrf_spis_event_check(p_spis, NRF_SPIS_EVENT_END))
    {
        volatile nrfx_spis_state_t prev_spi_state = p_cb->spi_state;
        nrf_spis_event_clear(p_spis, NRF_SPIS_EVENT_END);
        NRFX_LOG_DEBUG("SPIS: Event: %s.", EVT_TO_STR(NRF_SPIS_EVENT_END));

        spis_state_change(p_spis, p_cb, SPIS_XFER_COMPLETED);

        if (prev_spi_state == SPIS_BUFFER_RESOURCE_REQUESTED) {
            /* In this case nrfx_spis_buffers_set was called while xfer was running.
             * NRFX_SPIS_XFER_DONE event for most recent xfer was reported above.
             * SPIS state has to be changed back to SPIS_BUFFER_RESOURCE_REQUESTED to keep waiting for resource.
             * It has to be done outside of spis_state_change function because the task NRF_SPIS_TASK_ACQUIRE was already triggered by nrfx_spis_buffers_set.
             */
            p_cb->spi_state = SPIS_BUFFER_RESOURCE_REQUESTED;
        }
    }

    // Check for SPI semaphore acquired event.
    if (nrf_spis_event_check(p_spis, NRF_SPIS_EVENT_ACQUIRED))
    {
        nrf_spis_event_clear(p_spis, NRF_SPIS_EVENT_ACQUIRED);
        NRFX_LOG_DEBUG("SPIS: Event: %s.", EVT_TO_STR(NRF_SPIS_EVENT_ACQUIRED));

        switch (p_cb->spi_state)
        {
            case SPIS_BUFFER_RESOURCE_REQUESTED:

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

                nrf_spis_tx_buffer_set(p_spis, (uint8_t *)p_cb->tx_buffer, p_cb->tx_buffer_size);
                nrf_spis_rx_buffer_set(p_spis, (uint8_t *)p_cb->rx_buffer, p_cb->rx_buffer_size);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

                nrf_spis_task_trigger(p_spis, NRF_SPIS_TASK_RELEASE);

                spis_state_change(p_spis, p_cb, SPIS_BUFFER_RESOURCE_CONFIGURED);
                break;

            default:
                // No implementation required.
                break;
        }
    }
}

NRFX_INSTANCE_IRQ_HANDLERS(SPIS, spis)

#endif // NRFX_CHECK(NRFX_SPIS_ENABLED)
