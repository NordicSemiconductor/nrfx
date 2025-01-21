/*
 * Copyright (c) 2016 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_QSPI_ENABLED)

#include <nrfx_qspi.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_gpio.h>
#include <nrf_erratas.h>

#define NRFX_LOG_MODULE QSPI
#include <nrfx_log.h>

/** @brief Command byte used to read status register. */
#define QSPI_STD_CMD_RDSR 0x05

/** @brief Byte used to mask status register and retrieve the write-in-progess bit. */
#define QSPI_MEM_STATUSREG_WIP_MASK 0x01

/** @brief Default time used in timeout function. */
#define QSPI_DEF_WAIT_TIME_US 10

/**
 * @brief Default number of tries in timeout function.
 *
 * When the flash memory is busy with some operation, waiting for the READY
 * event even when only the ACTIVATE task is triggered may take significant
 * amount of time. The below default number of attempts gives the maximum
 * waiting time of 500 ms what should cover most cases, including erasing
 * of sectors in most flash chips.
 */
#define QSPI_DEF_WAIT_ATTEMPTS 50000

/**
 * @brief Macro for initializing a QSPI pin.
 *
 * QSPI peripheral expects high drive pin strength.
 */
#define QSPI_PIN_INIT(_pin) nrf_gpio_cfg((_pin),                        \
                                         NRF_GPIO_PIN_DIR_INPUT,        \
                                         NRF_GPIO_PIN_INPUT_DISCONNECT, \
                                         NRF_GPIO_PIN_NOPULL,           \
                                         NRF_GPIO_PIN_H0H1,             \
                                         NRF_GPIO_PIN_NOSENSE)

#if !defined(USE_WORKAROUND_FOR_ANOMALY_121) && defined(NRF53_SERIES)
    // ANOMALY 121 - Configuration of QSPI peripheral requires additional steps.
    #define USE_WORKAROUND_FOR_ANOMALY_121 1
#endif

/** @brief QSPI driver states.*/
typedef enum
{
    NRFX_QSPI_STATE_UNINITIALIZED = 0,
    NRFX_QSPI_STATE_IDLE,
    NRFX_QSPI_STATE_WRITE,
    NRFX_QSPI_STATE_READ,
    NRFX_QSPI_STATE_ERASE,
    NRFX_QSPI_STATE_CINSTR,
} nrfx_qspi_state_t;

/** @brief Control block - driver instance local data. */
typedef struct
{
    nrfx_qspi_handler_t handler;            /**< Handler. */
    void *              p_context;          /**< Driver context used in interrupt. */
    void *              p_buffer_primary;   /**< Pointer to the primary buffer. */
    void *              p_buffer_secondary; /**< Pointer to the secondary buffer. */
    uint32_t            size_primary;       /**< Size of the primary buffer. */
    uint32_t            size_secondary;     /**< Size of the secondary buffer. */
    uint32_t            addr_primary;       /**< Address for the primary buffer. */
    uint32_t            addr_secondary;     /**< Address for the secondary buffer. */
    nrfx_qspi_evt_ext_t evt_ext;            /**< Extended event. */
    nrfx_qspi_state_t   state;              /**< Driver state. */
    uint32_t            timeout;            /**< Time in milliseconds used for operation timeout. */
    bool volatile       activated;          /**< Flag indicating whether the QSPI is active. */
    bool volatile       timeout_signal;     /**< Flag indicating a timeout of an operation.
                                             *   The flag is used to trigger premature timeout
                                             *   if @ref nrfx_qspi_timeout_signal is used. */
    bool                skip_gpio_cfg;      /**< Do not touch GPIO configuration of used pins. */
} qspi_control_block_t;

static qspi_control_block_t m_cb;

static nrfx_err_t qspi_activate(bool wait);
static nrfx_err_t qspi_ready_wait(void);
static void       qspi_workaround_215_43_apply(void);
static bool       qspi_errata_159_conditions_check(void);

static nrfx_err_t qspi_xfer(void *            p_buffer,
                            size_t            length,
                            uint32_t          address,
                            nrfx_qspi_state_t desired_state)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_buffer != NULL);

    if (qspi_errata_159_conditions_check())
    {
        return NRFX_ERROR_FORBIDDEN;
    }

    if (!nrfx_is_in_ram(p_buffer) || !nrfx_is_word_aligned(p_buffer))
    {
        return NRFX_ERROR_INVALID_ADDR;
    }

    if (m_cb.state != NRFX_QSPI_STATE_IDLE &&
        (m_cb.state != desired_state || !m_cb.activated))
    {
        return NRFX_ERROR_BUSY;
    }

    nrf_qspi_task_t task;
    if (desired_state == NRFX_QSPI_STATE_WRITE)
    {
        nrf_qspi_write_buffer_set(NRF_QSPI, p_buffer, length, address);
        task = NRF_QSPI_TASK_WRITESTART;
    }
    else
    {
        nrf_qspi_read_buffer_set(NRF_QSPI, p_buffer, length, address);
        task = NRF_QSPI_TASK_READSTART;
    }

    m_cb.timeout_signal = false;

    if (!m_cb.handler)
    {
        if (!m_cb.activated && qspi_activate(true) == NRFX_ERROR_TIMEOUT)
        {
            return NRFX_ERROR_TIMEOUT;
        }

        nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
        nrf_qspi_task_trigger(NRF_QSPI, task);

        return qspi_ready_wait();
    }

    if (m_cb.p_buffer_primary)
    {
        m_cb.p_buffer_secondary = p_buffer;
        m_cb.size_secondary     = length;
        m_cb.addr_secondary     = address;
    }
    else
    {
        m_cb.p_buffer_primary = p_buffer;
        m_cb.size_primary     = length;
        m_cb.addr_primary     = address;

        m_cb.state = desired_state;
        nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
        nrf_qspi_int_enable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
        if (!m_cb.activated)
        {
            (void)qspi_activate(false);
        }
        else
        {
            nrf_qspi_task_trigger(NRF_QSPI, task);
        }
    }

    return NRFX_SUCCESS;
}

static bool qspi_pins_configure(nrfx_qspi_config_t const * p_config)
{
    // If both GPIO configuration and pin selection are to be skipped,
    // the pin numbers may be not specified at all, so even validation
    // of those numbers cannot be performed.
    if (p_config->skip_gpio_cfg && p_config->skip_psel_cfg)
    {
        return true;
    }

    // Check if the user set meaningful values to struct fields. If not, return false.
    if ((p_config->pins.sck_pin == NRF_QSPI_PIN_NOT_CONNECTED) ||
        (p_config->pins.csn_pin == NRF_QSPI_PIN_NOT_CONNECTED) ||
        (p_config->pins.io0_pin == NRF_QSPI_PIN_NOT_CONNECTED) ||
        (p_config->pins.io1_pin == NRF_QSPI_PIN_NOT_CONNECTED))
    {
        return false;
    }

#if defined(NRF5340_XXAA)
    // Check if dedicated QSPI pins are used.
    enum {
        QSPI_IO0_DEDICATED = NRF_GPIO_PIN_MAP(0, 13),
        QSPI_IO1_DEDICATED = NRF_GPIO_PIN_MAP(0, 14),
        QSPI_IO2_DEDICATED = NRF_GPIO_PIN_MAP(0, 15),
        QSPI_IO3_DEDICATED = NRF_GPIO_PIN_MAP(0, 16),
        QSPI_SCK_DEDICATED = NRF_GPIO_PIN_MAP(0, 17),
        QSPI_CSN_DEDICATED = NRF_GPIO_PIN_MAP(0, 18)
    };

    if ((p_config->pins.sck_pin != QSPI_SCK_DEDICATED) ||
        (p_config->pins.csn_pin != QSPI_CSN_DEDICATED) ||
        (p_config->pins.io0_pin != QSPI_IO0_DEDICATED) ||
        (p_config->pins.io1_pin != QSPI_IO1_DEDICATED) ||
        (p_config->pins.io2_pin != NRF_QSPI_PIN_NOT_CONNECTED &&
         p_config->pins.io2_pin != QSPI_IO2_DEDICATED) ||
        (p_config->pins.io3_pin != NRF_QSPI_PIN_NOT_CONNECTED &&
         p_config->pins.io3_pin != QSPI_IO3_DEDICATED))
    {
        return false;
    }
#endif

    if (!p_config->skip_gpio_cfg)
    {
        QSPI_PIN_INIT(p_config->pins.sck_pin);
        QSPI_PIN_INIT(p_config->pins.csn_pin);
        QSPI_PIN_INIT(p_config->pins.io0_pin);
        QSPI_PIN_INIT(p_config->pins.io1_pin);
        if (p_config->pins.io2_pin != NRF_QSPI_PIN_NOT_CONNECTED)
        {
            QSPI_PIN_INIT(p_config->pins.io2_pin);
        }
        if (p_config->pins.io3_pin != NRF_QSPI_PIN_NOT_CONNECTED)
        {
            QSPI_PIN_INIT(p_config->pins.io3_pin);
        }
    }

    if (!p_config->skip_psel_cfg)
    {
        nrf_qspi_pins_set(NRF_QSPI, &p_config->pins);
    }

    return true;
}

static void qspi_pins_deconfigure(void)
{
    nrf_qspi_pins_t pins;
    nrf_qspi_pins_get(NRF_QSPI, &pins);

    nrf_gpio_cfg_default(pins.sck_pin);
    nrf_gpio_cfg_default(pins.csn_pin);
    nrf_gpio_cfg_default(pins.io0_pin);
    nrf_gpio_cfg_default(pins.io1_pin);
    if (pins.io2_pin != NRF_QSPI_PIN_NOT_CONNECTED)
    {
        nrf_gpio_cfg_default(pins.io2_pin);
    }
    if (pins.io3_pin != NRF_QSPI_PIN_NOT_CONNECTED)
    {
        nrf_gpio_cfg_default(pins.io3_pin);
    }
}

static nrfx_err_t qspi_ready_wait(void)
{
    bool result;
    uint32_t attempts = m_cb.timeout > 0 ?
                        (m_cb.timeout * 1000UL) / QSPI_DEF_WAIT_TIME_US : QSPI_DEF_WAIT_ATTEMPTS;

    NRFX_WAIT_FOR(nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY) || m_cb.timeout_signal,
                  attempts,
                  QSPI_DEF_WAIT_TIME_US,
                  result);

    if (!result || m_cb.timeout_signal)
    {
        return NRFX_ERROR_TIMEOUT;
    }

    return NRFX_SUCCESS;
}

static nrfx_err_t qspi_configure(nrfx_qspi_config_t const * p_config)
{
    if (!qspi_pins_configure(p_config))
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    m_cb.timeout = p_config->timeout;
    m_cb.skip_gpio_cfg = p_config->skip_gpio_cfg;

    /* The code below accesses the IFTIMING and IFCONFIG1 registers what
     * may trigger anomaly 215 on nRF52840 or anomaly 43 on nRF5340. Use
     * the proper workaround then.
     */
    if (NRF52_ERRATA_215_ENABLE_WORKAROUND || NRF53_ERRATA_43_ENABLE_WORKAROUND)
    {
        /* The interrupt is disabled because of the anomaly handling.
         * It will be reenabled if needed before the next QSPI operation.
         */
        nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);
        qspi_workaround_215_43_apply();
    }

    nrf_qspi_xip_offset_set(NRF_QSPI, p_config->xip_offset);

    nrf_qspi_ifconfig0_set(NRF_QSPI, &p_config->prot_if);
#if NRFX_CHECK(USE_WORKAROUND_FOR_ANOMALY_121)
    uint32_t regval = nrf_qspi_ifconfig0_raw_get(NRF_QSPI);
    if (p_config->phy_if.sck_freq == NRF_QSPI_FREQ_DIV1)
    {
        regval |= ((1UL << 16) | (1UL << 17));
    }
    else
    {
        regval &= ~(1UL << 17);
        regval |=  (1UL << 16);
    }
    nrf_qspi_ifconfig0_raw_set(NRF_QSPI, regval);
    nrf_qspi_iftiming_set(NRF_QSPI, 6);
#endif
    nrf_qspi_ifconfig1_set(NRF_QSPI, &p_config->phy_if);

    if (m_cb.handler)
    {
        NRFX_IRQ_PRIORITY_SET(QSPI_IRQn, p_config->irq_priority);
        NRFX_IRQ_ENABLE(QSPI_IRQn);
    }

    return NRFX_SUCCESS;
}

static nrfx_err_t qspi_activate(bool wait)
{
    nrf_qspi_enable(NRF_QSPI);

    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);

    if (wait)
    {
        nrfx_err_t ret = qspi_ready_wait();

        if (ret == NRFX_SUCCESS)
        {
            m_cb.activated = true;
        }
        return ret;
    }

    return NRFX_SUCCESS;
}

static void qspi_deactivate(void)
{
    m_cb.activated = false;

    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);

    nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_DEACTIVATE);

    nrf_qspi_disable(NRF_QSPI);

    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
}

static bool qspi_errata_159_conditions_check(void)
{
#if NRF_CLOCK_HAS_HFCLK192M && NRF53_ERRATA_159_ENABLE_WORKAROUND
    if ((nrf_clock_hfclk192m_div_get(NRF_CLOCK) != NRF_CLOCK_HFCLK_DIV_1) ||
        (nrf_clock_hfclk_div_get(NRF_CLOCK) != NRF_CLOCK_HFCLK_DIV_2))
    {
        return true;
    }
    else
#endif
    {
        return false;
    }
}

static void qspi_workaround_215_43_apply(void)
{
    nrf_qspi_pins_t pins;
    nrf_qspi_pins_t disconnected_pins = {
        .sck_pin = NRF_QSPI_PIN_NOT_CONNECTED,
        .csn_pin = NRF_QSPI_PIN_NOT_CONNECTED,
        .io0_pin = NRF_QSPI_PIN_NOT_CONNECTED,
        .io1_pin = NRF_QSPI_PIN_NOT_CONNECTED,
        .io2_pin = NRF_QSPI_PIN_NOT_CONNECTED,
        .io3_pin = NRF_QSPI_PIN_NOT_CONNECTED,
    };

    /* Disconnect pins to not wait for response from external memory. */
    nrf_qspi_pins_get(NRF_QSPI, &pins);
    nrf_qspi_pins_set(NRF_QSPI, &disconnected_pins);

    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ACTIVATE);

    while (!nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY))
    {}

    /* Restore previous pins. */
    nrf_qspi_pins_set(NRF_QSPI, &pins);
}

nrfx_err_t nrfx_qspi_init(nrfx_qspi_config_t const * p_config,
                          nrfx_qspi_handler_t        handler,
                          void *                     p_context)
{
    nrfx_err_t err_code;

    NRFX_ASSERT(p_config);

    if (m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED)
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

    m_cb.handler = handler;
    m_cb.p_context = p_context;

    if (p_config)
    {
        err_code = qspi_configure(p_config);
        if (err_code != NRFX_SUCCESS)
        {
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                            __func__,
                            NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }

    m_cb.p_buffer_primary = NULL;
    m_cb.p_buffer_secondary = NULL;

    m_cb.state = NRFX_QSPI_STATE_IDLE;
    m_cb.activated = false;

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_qspi_reconfigure(nrfx_qspi_config_t const * p_config)
{
    NRFX_ASSERT(p_config);
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (m_cb.state == NRFX_QSPI_STATE_UNINITIALIZED)
    {
        return NRFX_ERROR_INVALID_STATE;
    }

    if (m_cb.state != NRFX_QSPI_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }

    if (!m_cb.activated)
    {
        err_code = qspi_configure(p_config);
    }
    else
    {
        qspi_deactivate();
        err_code = qspi_configure(p_config);
    }

    return err_code;
}

void nrfx_qspi_timeout_signal(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    m_cb.timeout_signal = true;
}

nrfx_err_t nrfx_qspi_cinstr_xfer(nrf_qspi_cinstr_conf_t const * p_config,
                                 void const *                   p_tx_buffer,
                                 void *                         p_rx_buffer)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    if (qspi_errata_159_conditions_check())
    {
        return NRFX_ERROR_FORBIDDEN;
    }

    if (m_cb.state != NRFX_QSPI_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }

    if (!m_cb.activated && qspi_activate(true) == NRFX_ERROR_TIMEOUT)
    {
        return NRFX_ERROR_TIMEOUT;
    }

    /* For custom instruction transfer driver has to switch to blocking mode.
     * If driver was previously configured to non-blocking mode, interrupts
     * will get reenabled before next standard transfer.
     */
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);

    /* The code below accesses the CINSTRCONF register what may trigger
     * anomaly 215 on nRF52840 or anomaly 43 on nRF5340. Use the proper
     * workaround then.
     */
    if (NRF52_ERRATA_215_ENABLE_WORKAROUND || NRF53_ERRATA_43_ENABLE_WORKAROUND)
    {
        qspi_workaround_215_43_apply();
    }

    /* In some cases, only opcode should be sent. To prevent execution, set function code is
     * surrounded by an if.
     */
    if (p_tx_buffer)
    {
        nrf_qspi_cinstrdata_set(NRF_QSPI, p_config->length, p_tx_buffer);
    }

    m_cb.timeout_signal = false;

    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_cinstr_transfer_start(NRF_QSPI, p_config);

    if (qspi_ready_wait() == NRFX_ERROR_TIMEOUT)
    {
        // This timeout should never occur when WIPWAIT is not active, since in this
        // case the QSPI peripheral should send the command immediately, without any
        // waiting for previous write to complete.
        NRFX_ASSERT(p_config->wipwait);

        return NRFX_ERROR_TIMEOUT;
    }
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

    if (p_rx_buffer)
    {
        nrf_qspi_cinstrdata_get(NRF_QSPI, p_config->length, p_rx_buffer);
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_qspi_cinstr_quick_send(uint8_t               opcode,
                                       nrf_qspi_cinstr_len_t length,
                                       void const *          p_tx_buffer)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_tx_buffer);

    nrf_qspi_cinstr_conf_t config = NRFX_QSPI_DEFAULT_CINSTR(opcode, length);
    return nrfx_qspi_cinstr_xfer(&config, p_tx_buffer, NULL);
}

nrfx_err_t nrfx_qspi_lfm_start(nrf_qspi_cinstr_conf_t const * p_config)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_config->length == NRF_QSPI_CINSTR_LEN_1B);

    if (qspi_errata_159_conditions_check())
    {
        return NRFX_ERROR_FORBIDDEN;
    }

    if (m_cb.state != NRFX_QSPI_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }

    if (!m_cb.activated && qspi_activate(true) == NRFX_ERROR_TIMEOUT)
    {
        return NRFX_ERROR_TIMEOUT;
    }

    /* For transferring arbitrary byte length custom instructions driver has to switch to
     * blocking mode. If driver was previously configured to non-blocking mode, interrupts
     * will get reenabled before next standard transfer.
     */
    nrf_qspi_int_disable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);

    /* The code below accesses the CINSTRCONF register what may trigger
     * anomaly 215 on nRF52840 or anomaly 43 on nRF5340. Use the proper
     * workaround then.
     */
    if (NRF52_ERRATA_215_ENABLE_WORKAROUND || NRF53_ERRATA_43_ENABLE_WORKAROUND)
    {
        qspi_workaround_215_43_apply();
    }

    NRFX_ASSERT(!(nrf_qspi_cinstr_long_transfer_is_ongoing(NRF_QSPI)));

    m_cb.timeout_signal = false;

    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_cinstr_long_transfer_start(NRF_QSPI, p_config);

    if (qspi_ready_wait() == NRFX_ERROR_TIMEOUT)
    {
        /* In case of error, abort long frame mode */
        nrf_qspi_cinstr_long_transfer_continue(NRF_QSPI, NRF_QSPI_CINSTR_LEN_1B, true);
        return NRFX_ERROR_TIMEOUT;
    }

    m_cb.state = NRFX_QSPI_STATE_CINSTR;
    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_qspi_lfm_xfer(void const * p_tx_buffer,
                              void *       p_rx_buffer,
                              size_t       transfer_length,
                              bool         finalize)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);
    NRFX_ASSERT(nrf_qspi_cinstr_long_transfer_is_ongoing(NRF_QSPI));

    if (qspi_errata_159_conditions_check())
    {
        return NRFX_ERROR_FORBIDDEN;
    }

    nrfx_err_t status = NRFX_SUCCESS;

    /* Perform transfers in packets of 8 bytes. Last transfer may be shorter. */
    nrf_qspi_cinstr_len_t length = NRF_QSPI_CINSTR_LEN_9B;
    for (uint32_t curr_byte = 0; curr_byte < transfer_length; curr_byte += 8)
    {
        uint32_t remaining_bytes = transfer_length - curr_byte;
        m_cb.timeout_signal = false;

        if (remaining_bytes < 8)
        {
            length = (nrf_qspi_cinstr_len_t)(remaining_bytes + 1);
        }

        if (p_tx_buffer)
        {
            nrf_qspi_cinstrdata_set(NRF_QSPI,
                                    length,
                                    &((uint8_t const *)p_tx_buffer)[curr_byte]);
        }

        nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

        if (remaining_bytes <= 8)
        {
            nrf_qspi_cinstr_long_transfer_continue(NRF_QSPI, length, finalize);
        }
        else
        {
            nrf_qspi_cinstr_long_transfer_continue(NRF_QSPI, length, false);
        }

        if (qspi_ready_wait() == NRFX_ERROR_TIMEOUT)
        {
            /* In case of error, abort long frame mode */
            nrf_qspi_cinstr_long_transfer_continue(NRF_QSPI, NRF_QSPI_CINSTR_LEN_1B, true);
            status = NRFX_ERROR_TIMEOUT;
            break;
        }

        if (p_rx_buffer)
        {
            nrf_qspi_cinstrdata_get(NRF_QSPI,
                                    length,
                                    &((uint8_t *)p_rx_buffer)[curr_byte]);
        }
    }
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

    if ((finalize) || (status == NRFX_ERROR_TIMEOUT))
    {
        m_cb.state = NRFX_QSPI_STATE_IDLE;
    }

    return status;
}

nrfx_err_t nrfx_qspi_mem_busy_check(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    nrfx_err_t ret_code;
    uint8_t status_value = 0;

    nrf_qspi_cinstr_conf_t const config = {
        .opcode = QSPI_STD_CMD_RDSR,
        .length = NRF_QSPI_CINSTR_LEN_2B,
        // Keep the IO3 line high during the transfer. Otherwise, its low level
        // can be interpreted by the memory chip as an active HOLD#/RESET#
        // signal and the status register value may not be output.
        // Such configuration is also consistent with what the QSPI peripheral
        // uses when it sends the Read Status Register command itself.
        .io3_level = true,
    };
    ret_code = nrfx_qspi_cinstr_xfer(&config, &status_value, &status_value);

    if (ret_code != NRFX_SUCCESS)
    {
        return ret_code;
    }

    if ((status_value & QSPI_MEM_STATUSREG_WIP_MASK) != 0x00)
    {
        return NRFX_ERROR_BUSY;
    }

    return NRFX_SUCCESS;
}

void nrfx_qspi_uninit(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    NRFX_IRQ_DISABLE(QSPI_IRQn);

    qspi_deactivate();
    if (!m_cb.skip_gpio_cfg)
    {
        qspi_pins_deconfigure();
    }

    m_cb.state = NRFX_QSPI_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

nrfx_err_t nrfx_qspi_activate(bool wait)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    if (m_cb.activated)
    {
        return NRFX_ERROR_ALREADY;
    }

    return qspi_activate(wait);
}

nrfx_err_t nrfx_qspi_deactivate(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    if (m_cb.state != NRFX_QSPI_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }

    qspi_deactivate();
    return NRFX_SUCCESS;
}

bool nrfx_qspi_init_check(void)
{
    return (m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);
}

nrfx_err_t nrfx_qspi_write(void const * p_tx_buffer,
                           size_t       tx_buffer_length,
                           uint32_t     dst_address)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif
    return qspi_xfer((void *)p_tx_buffer, tx_buffer_length, dst_address, NRFX_QSPI_STATE_WRITE);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

nrfx_err_t nrfx_qspi_read(void *   p_rx_buffer,
                          size_t   rx_buffer_length,
                          uint32_t src_address)
{
    return qspi_xfer((void *)p_rx_buffer, rx_buffer_length, src_address, NRFX_QSPI_STATE_READ);
}

nrfx_err_t nrfx_qspi_erase(nrf_qspi_erase_len_t length,
                           uint32_t             start_address)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    if (qspi_errata_159_conditions_check())
    {
        return NRFX_ERROR_FORBIDDEN;
    }

    if (!nrfx_is_word_aligned((void const *)start_address))
    {
        return NRFX_ERROR_INVALID_ADDR;
    }

    if (m_cb.handler && m_cb.state != NRFX_QSPI_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }

    nrf_qspi_erase_ptr_set(NRF_QSPI, start_address, length);
    m_cb.timeout_signal = false;

    if (!m_cb.handler)
    {
        if (!m_cb.activated && qspi_activate(true) == NRFX_ERROR_TIMEOUT)
        {
            return NRFX_ERROR_TIMEOUT;
        }

        nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
        nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ERASESTART);

        return qspi_ready_wait();
    }

    m_cb.state = NRFX_QSPI_STATE_ERASE;
    nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);
    nrf_qspi_int_enable(NRF_QSPI, NRF_QSPI_INT_READY_MASK);

    if (!m_cb.activated)
    {
        (void)qspi_activate(false);
    }
    else
    {
        nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ERASESTART);
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_qspi_chip_erase(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    return nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_ALL, 0);
}

nrfx_qspi_evt_ext_t const * nrfx_qspi_event_extended_get(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);
    NRFX_ASSERT(m_cb.evt_ext.type != NRFX_QSPI_EVENT_NONE);
    return &m_cb.evt_ext;
}

bool nrfx_qspi_xfer_buffered_check(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    return (bool)m_cb.p_buffer_secondary;
}

#if NRF_QSPI_HAS_XIP_ENC
nrfx_err_t nrfx_qspi_xip_encrypt(nrf_qspi_encryption_t const * p_config)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    if (m_cb.state != NRFX_QSPI_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }

    if (p_config)
    {
        nrf_qspi_xip_encryption_configure(NRF_QSPI, p_config);
        nrf_qspi_xip_encryption_set(NRF_QSPI, true);
    }
    else
    {
        nrf_qspi_xip_encryption_set(NRF_QSPI, false);
    }

    return NRFX_SUCCESS;
}
#endif

#if NRF_QSPI_HAS_DMA_ENC
nrfx_err_t nrfx_qspi_dma_encrypt(nrf_qspi_encryption_t const * p_config)
{
    NRFX_ASSERT(m_cb.state != NRFX_QSPI_STATE_UNINITIALIZED);

    if (m_cb.state != NRFX_QSPI_STATE_IDLE)
    {
        return NRFX_ERROR_BUSY;
    }

    if (p_config)
    {
        nrf_qspi_dma_encryption_configure(NRF_QSPI, p_config);
        nrf_qspi_dma_encryption_set(NRF_QSPI, true);
    }
    else
    {
        nrf_qspi_dma_encryption_set(NRF_QSPI, false);
    }

    return NRFX_SUCCESS;
}
#endif

static void qspi_event_xfer_handle(nrfx_qspi_evt_ext_xfer_t * p_xfer)
{
    p_xfer->p_buffer = (uint8_t *)m_cb.p_buffer_primary;
    p_xfer->size     = m_cb.size_primary;
    p_xfer->addr     = m_cb.addr_primary;
    if (m_cb.p_buffer_secondary)
    {
        m_cb.p_buffer_primary = m_cb.p_buffer_secondary;
        m_cb.size_primary     = m_cb.size_secondary;
        m_cb.addr_primary     = m_cb.addr_secondary;

        m_cb.p_buffer_secondary = NULL;
    }
    else
    {
        m_cb.p_buffer_primary = NULL;
    }
}

static void qspi_event_erase_handle(nrfx_qspi_evt_ext_erase_t * p_erase)
{
    p_erase->addr = nrf_qspi_erase_ptr_get(NRF_QSPI);
    p_erase->len  = nrf_qspi_erase_len_get(NRF_QSPI);
}

static void qspi_extended_event_process(nrfx_qspi_evt_ext_t * p_event)
{
    switch (m_cb.state)
    {
        case NRFX_QSPI_STATE_WRITE:
            p_event->type = NRFX_QSPI_EVENT_WRITE_DONE;
            qspi_event_xfer_handle(&p_event->data.xfer);
            break;

        case NRFX_QSPI_STATE_READ:
            p_event->type = NRFX_QSPI_EVENT_READ_DONE;
            qspi_event_xfer_handle(&p_event->data.xfer);
            break;

        case NRFX_QSPI_STATE_ERASE:
            p_event->type = NRFX_QSPI_EVENT_ERASE_DONE;
            qspi_event_erase_handle(&p_event->data.erase);
            break;

        default:
            break;
    }
}

static void qspi_activate_event_process(void)
{
    switch (m_cb.state)
    {
        case NRFX_QSPI_STATE_WRITE:
            nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_WRITESTART);
            break;

        case NRFX_QSPI_STATE_READ:
            nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_READSTART);
            break;

        case NRFX_QSPI_STATE_ERASE:
            nrf_qspi_task_trigger(NRF_QSPI, NRF_QSPI_TASK_ERASESTART);
            break;

        default:
            break;
    }
}

void nrfx_qspi_irq_handler(void)
{
    // Catch Event ready interrupts
    if (nrf_qspi_event_check(NRF_QSPI, NRF_QSPI_EVENT_READY))
    {
        nrf_qspi_event_clear(NRF_QSPI, NRF_QSPI_EVENT_READY);

        if (!m_cb.activated)
        {
            m_cb.activated = true;
            qspi_activate_event_process();
            return;
        }

        qspi_extended_event_process(&m_cb.evt_ext);
        if (!m_cb.p_buffer_primary)
        {
            m_cb.state = NRFX_QSPI_STATE_IDLE;
        }

        if (!m_cb.timeout_signal)
        {
            m_cb.handler(NRFX_QSPI_EVENT_DONE, m_cb.p_context);
        }

        m_cb.evt_ext.type = NRFX_QSPI_EVENT_NONE;
    }
}

#endif // NRFX_CHECK(NRFX_QSPI_ENABLED)
