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

#include <nrfx_spim.h>
#include "prs/nrfx_prs.h"

#define NRFX_LOG_MODULE SPIM
#include <nrfx_log.h>

#if NRF_ERRATA_STATIC_CHECK(52, 58)
#include <nrfx_gpiote.h>
#endif

// Requested pin can either match dedicated pin or be not connected at all.
#define SPIM_DEDICATED_PIN_VALIDATE(requested_pin, supported_pin) \
    (((requested_pin) == NRF_SPIM_PIN_NOT_CONNECTED) || ((requested_pin) == (supported_pin)))

#if NRF_ERRATA_STATIC_CHECK(53, 135) && defined(NRF_SPIM4)
static void anomaly_135_enable(void)
{
    *((volatile uint32_t *)0x5000ac04) = 1;
}

static void anomaly_135_disable(void)
{
    *((volatile uint32_t *)0x5000ac04) = 0;
}
#endif

#if NRF_ERRATA_STATIC_CHECK(52, 198) && defined(NRF_SPIM3)
static uint32_t m_anomaly_198_preserved_value;
static void anomaly_198_enable(uint8_t const * p_buffer, size_t buf_len)
{
    m_anomaly_198_preserved_value = *((volatile uint32_t *)0x40000E00);

    if (buf_len == 0)
    {
        return;
    }
    uint32_t buffer_end_addr = ((uint32_t)p_buffer) + buf_len;
    uint32_t block_addr      = ((uint32_t)p_buffer) & ~0x1FFFul;
    uint32_t block_flag      = (1UL << ((block_addr >> 13) & 0xFFFF));
    uint32_t occupied_blocks = 0;

    if (block_addr >= 0x20010000)
    {
        occupied_blocks = (1UL << 8);
    }
    else
    {
        do {
            occupied_blocks |= block_flag;
            block_flag <<= 1;
            block_addr  += 0x2000;
        } while ((block_addr < buffer_end_addr) && (block_addr < 0x20012000));
    }

    *((volatile uint32_t *)0x40000E00) = occupied_blocks;
}

static void anomaly_198_disable(void)
{
    *((volatile uint32_t *)0x40000E00) = m_anomaly_198_preserved_value;
}
#endif // NRF_ERRATA_STATIC_CHECK(52, 198) && defined(NRF_SPIM3)

#if NRF_ERRATA_STATIC_CHECK(52, 58)
static int nrf52_errata_58_workaround_enable(nrfx_spim_control_block_t * p_cb,
                                             NRF_SPIM_Type *             p_spim)
{
    int err_code;

    if (!p_cb->p_gpiote_inst)
    {
        NRFX_LOG_WARNING("nRF52 Anomaly 58 enabled but not initiated!");
        return 0;
    }

    if (!nrfx_gpiote_init_check(p_cb->p_gpiote_inst))
    {
        err_code = nrfx_gpiote_init(p_cb->p_gpiote_inst, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
        if (err_code != 0)
        {
            return err_code;
        }
    }

    err_code = nrfx_gpiote_channel_alloc(p_cb->p_gpiote_inst, &p_cb->gpiote_ch);
    if (err_code != 0)
    {
        return err_code;
    }

    nrfx_gpiote_trigger_config_t trigger_config = {
        .trigger = NRFX_GPIOTE_TRIGGER_TOGGLE,
        .p_in_channel = &p_cb->gpiote_ch,
    };
    nrfx_gpiote_input_pin_config_t gpiote_config = {
        .p_pull_config = NULL,
        .p_trigger_config = &trigger_config,
        .p_handler_config = NULL,
    };

    const uint32_t sck_pin = nrf_spim_sck_pin_get(p_spim);

    err_code = nrfx_gpiote_input_configure(p_cb->p_gpiote_inst, sck_pin, &gpiote_config);
    if (err_code != 0)
    {
        return err_code;
    }

    nrfx_gpiote_trigger_enable(p_cb->p_gpiote_inst, sck_pin, false);

    err_code = nrfx_gppi_conn_alloc(nrfx_gpiote_in_event_address_get(p_cb->p_gpiote_inst, sck_pin),
                                    nrfy_spim_task_address_get(p_spim, NRF_SPIM_TASK_STOP),
                                    &p_cb->gppi_handle);
    if (err_code != 0)
    {
        return err_code;
    }

    /* Stop the spim instance when SCK toggles */
    nrfx_gppi_conn_enable(p_cb->gppi_handle);

    p_cb->apply_nrf52_errata_58 = true;

    /* The spim instance cannot be stopped mid-byte, so it will finish
     * transmitting the first byte and then stop. Effectively ensuring
     * that only 1 byte is transmitted.
     */

    return 0;
}

static int nrf52_errata_58_workaround_disable(nrfx_spim_control_block_t * p_cb,
                                              NRF_SPIM_Type *             p_spim)
{
    nrfx_gpiote_trigger_disable(p_cb->p_gpiote_inst, nrf_spim_sck_pin_get(p_spim));
    nrfx_gppi_conn_disable(p_cb->gppi_handle);

    int err_code;

    err_code = nrfx_gpiote_channel_free(p_cb->p_gpiote_inst, p_cb->gpiote_ch);
    if (err_code != 0)
    {
        return err_code;
    }

    /* Workaround is for SoC with PPI so first 2 arguments are not used thus can be 0. */
    nrfx_gppi_conn_free(0, 0, p_cb->gppi_handle);

    p_cb->apply_nrf52_errata_58 = false;

    return 0;
}
#endif // NRF_ERRATA_STATIC_CHECK(52, 58)

static void spim_abort(NRF_SPIM_Type * p_spim, nrfx_spim_control_block_t * p_cb)
{
    if (p_cb->transfer_in_progress)
    {
        nrfy_spim_abort(p_spim, NULL);
        bool stopped;
        uint32_t stopped_mask = NRFY_EVENT_TO_INT_BITMASK(NRF_SPIM_EVENT_STOPPED);
        NRFX_WAIT_FOR(nrfy_spim_events_process(p_spim, stopped_mask, NULL), 100, 1, stopped);
        if (!stopped)
        {
            NRFX_LOG_ERROR("Failed to stop instance with base address: %p.", (void *)p_spim);
        }
        p_cb->transfer_in_progress = false;
    }
#if defined(NRF_SPIM_CHECK_DISABLE_ON_XFER_END)
    if (p_cb->disable_on_xfer_end)
#endif
    {
        nrfy_spim_disable(p_spim);
#if NRF_ERRATA_STATIC_CHECK(53, 135) && defined(NRF_SPIM4)
        if (NRF_ERRATA_DYNAMIC_CHECK(53, 135) && p_spim == NRF_SPIM4)
        {
            anomaly_135_disable();
        }
#endif
#if NRF_ERRATA_STATIC_CHECK(54L, 57) && defined(NRF_SPIM00)
    if (NRF_ERRATA_DYNAMIC_CHECK(54L, 57) && p_spim == NRF_SPIM00)
    {
        *(volatile uint32_t *)(((uint8_t *)p_spim) + 0xC04) = 0x0;
    }
#endif
    }
}

static void pin_init(uint32_t             pin,
                     nrf_gpio_pin_dir_t   dir,
                     nrf_gpio_pin_pull_t  pull,
                     nrf_gpio_pin_drive_t drive,
                     uint32_t             initial_state)
{
    nrf_gpio_pin_input_t input;

    if (pin == NRF_SPIM_PIN_NOT_CONNECTED)
    {
        return;
    }

    if (dir == NRF_GPIO_PIN_DIR_OUTPUT)
    {
        if (initial_state)
        {
            nrfy_gpio_pin_set(pin);
        }
        else
        {
            nrfy_gpio_pin_clear(pin);
        }
        input = NRF_GPIO_PIN_INPUT_DISCONNECT;
    }
    else
    {
        input = NRF_GPIO_PIN_INPUT_CONNECT;
    }

    nrfy_gpio_cfg(pin, dir, input, pull, drive, NRF_GPIO_PIN_NOSENSE);
}

static void configure_pins(nrfx_spim_t *              p_instance,
                           nrfx_spim_config_t const * p_config)
{
    nrfx_spim_control_block_t * p_cb = &p_instance->cb;

    p_cb->ss_active_high = p_config->ss_active_high;

    if (p_config->skip_gpio_cfg)
    {
        return;
    }

    nrf_gpio_pin_drive_t pin_drive;
    // Configure pin drive - high drive for 32 MHz clock frequency.
#if defined(NRF_SPIM_FORCE_H0H1)
    pin_drive = NRF_GPIO_PIN_H0H1;
#elif (NRF_SPIM_HAS_FREQUENCY && NRF_SPIM_HAS_32_MHZ_FREQ) || NRF_SPIM_HAS_PRESCALER
    pin_drive = (p_config->frequency == NRFX_MHZ_TO_HZ(32)) ? NRF_GPIO_PIN_H0H1 : NRF_GPIO_PIN_S0S1;
#else
    pin_drive = NRF_GPIO_PIN_S0S1;
#endif
    // Configure pins used by the peripheral:
    // - SCK - output with initial value corresponding with the SPI mode used:
    //   0 - for modes 0 and 1 (CPOL = 0), 1 - for modes 2 and 3 (CPOL = 1);
    //   according to the reference manual guidelines this pin and its input
    //   buffer must always be connected for the SPI to work.
    uint32_t sck_val = (p_config->mode <= NRF_SPIM_MODE_1) ? 0 : 1;
    pin_init(p_config->sck_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_NOPULL, pin_drive, sck_val);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_SPIM_CLOCKPIN_SCK_NEEDED)
    nrfy_gpio_pin_clock_set(p_config->sck_pin, true);
#endif
    // - MOSI (optional) - output with initial value 0
    pin_init(p_config->mosi_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_NOPULL, pin_drive, 0);
#if NRF_GPIO_HAS_CLOCKPIN && defined(NRF_SPIM_CLOCKPIN_MOSI_NEEDED)
    nrfy_gpio_pin_clock_set(p_config->mosi_pin, true);
#endif
    // - MISO (optional) - input
    pin_init(p_config->miso_pin, NRF_GPIO_PIN_DIR_INPUT, p_config->miso_pull, pin_drive, 0);
    // - Slave Select (optional) - output with initial value 1 (inactive).
    uint32_t ss_val = !p_config->ss_active_high;
    pin_init(p_config->ss_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_NOPULL, pin_drive, ss_val);
#if NRF_SPIM_HAS_DCX
    // - DCX (optional) - output.
    pin_init(p_config->dcx_pin, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_NOPULL, pin_drive, 1);
#endif
}

#if NRF_SPIM_HAS_FREQUENCY
static bool spim_frequency_valid_check(nrfx_spim_t const * p_instance, uint32_t frequency)
{
    (void)p_instance;
    switch (frequency)
    {
        case NRFX_KHZ_TO_HZ(125):
            /* FALLTHROUGH */
        case NRFX_KHZ_TO_HZ(250):
            /* FALLTHROUGH */
        case NRFX_KHZ_TO_HZ(500):
            /* FALLTHROUGH */
        case NRFX_MHZ_TO_HZ(1):
            /* FALLTHROUGH */
        case NRFX_MHZ_TO_HZ(2):
            /* FALLTHROUGH */
        case NRFX_MHZ_TO_HZ(4):
            /* FALLTHROUGH */
        case NRFX_MHZ_TO_HZ(8):
            return true;
#if NRF_SPIM_HAS_16_MHZ_FREQ
        case NRFX_MHZ_TO_HZ(16):
            return true;
#endif
#if NRF_SPIM_HAS_32_MHZ_FREQ
        case NRFX_MHZ_TO_HZ(32):
            return true;
#endif
        default:
            return false;
    }
}

static nrf_spim_frequency_t spim_frequency_bit_decode(uint32_t frequency)
{
    switch (frequency)
    {
        case NRFX_KHZ_TO_HZ(125):
            return NRF_SPIM_FREQ_125K;
        case NRFX_KHZ_TO_HZ(250):
            return NRF_SPIM_FREQ_250K;
        case NRFX_KHZ_TO_HZ(500):
            return NRF_SPIM_FREQ_500K;
        case NRFX_MHZ_TO_HZ(1):
            return NRF_SPIM_FREQ_1M;
        case NRFX_MHZ_TO_HZ(2):
            return NRF_SPIM_FREQ_2M;
        case NRFX_MHZ_TO_HZ(4):
            return NRF_SPIM_FREQ_4M;
        case NRFX_MHZ_TO_HZ(8):
            return NRF_SPIM_FREQ_8M;
#if NRF_SPIM_HAS_16_MHZ_FREQ
        case NRFX_MHZ_TO_HZ(16):
            return NRF_SPIM_FREQ_16M;
#endif
#if NRF_SPIM_HAS_32_MHZ_FREQ
        case NRFX_MHZ_TO_HZ(32):
            return NRF_SPIM_FREQ_32M;
#endif
        default:
            NRFX_ASSERT(false);
            return NRF_SPIM_FREQ_4M;
    }
}
#elif NRF_SPIM_HAS_PRESCALER
static bool spim_frequency_valid_check(nrfx_spim_t const * p_instance, uint32_t frequency)
{
    (void)p_instance;

    uint32_t base_frequency = NRFX_SPIM_BASE_FREQUENCY_GET(p_instance);
    uint32_t prescaler = NRF_SPIM_PRESCALER_CALCULATE(p_instance->p_reg, frequency);

    return ((base_frequency % frequency) < prescaler) &&
            NRFX_IS_EVEN(prescaler) &&
            (prescaler <= NRF_SPIM_PRESCALER_MAX_GET(p_instance->p_reg)) &&
            (prescaler >= NRF_SPIM_PRESCALER_MIN_GET(p_instance->p_reg));
}

static uint32_t spim_prescaler_calculate(nrfx_spim_t const * p_instance, uint32_t frequency)
{
    (void)p_instance;
    return NRF_SPIM_PRESCALER_CALCULATE(p_instance->p_reg, frequency);
}
#else
    #error "Unable to determine frequency division support type."
#endif


static int spim_configuration_verify(nrfx_spim_t const *        p_instance,
                                     nrfx_spim_config_t const * p_config)
{
    int err_code;
    if (!spim_frequency_valid_check(p_instance, p_config->frequency))
    {
        err_code = -EINVAL;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRF_SPIM_HAS_32_MHZ_FREQ && defined(SPIM_SCK_DEDICATED)
    // Check if dedicated SPIM pins are used, unless both GPIO configuration
    // and pin selection are to be skipped (pin numbers may be not specified
    // in such case).
    if (!(p_config->skip_gpio_cfg && p_config->skip_psel_cfg) &&
        (p_instance->p_reg == NRF_SPIM4) && (p_config->frequency == NRFX_MHZ_TO_HZ(32)))
    {
        if (!SPIM_DEDICATED_PIN_VALIDATE(p_config->sck_pin, SPIM_SCK_DEDICATED) ||
            !SPIM_DEDICATED_PIN_VALIDATE(p_config->mosi_pin, SPIM_MOSI_DEDICATED) ||
            !SPIM_DEDICATED_PIN_VALIDATE(p_config->miso_pin, SPIM_MISO_DEDICATED) ||
            (p_config->use_hw_ss &&
             !SPIM_DEDICATED_PIN_VALIDATE(p_config->ss_pin, SPIM_CSN_DEDICATED)) ||
            !SPIM_DEDICATED_PIN_VALIDATE(p_config->dcx_pin, SPIM_DCX_DEDICATED))
        {
            err_code = -EINVAL;
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }
#endif // NRF_SPIM_HAS_32_MHZ_FREQ && defined(SPIM_SCK_DEDICATED)

    return 0;
}

static void spim_configure(nrfx_spim_t *              p_instance,
                           nrfx_spim_config_t const * p_config)
{
    nrfx_spim_control_block_t * p_cb = &p_instance->cb;
#if NRF_SPIM_HAS_HW_CSN
    uint8_t csn_duration = (p_config->use_hw_ss ? p_config->ss_duration : NRF_SPIM_CSNDUR_DEFAULT);
#endif
#if NRF_SPIM_HAS_FREQUENCY
    nrf_spim_frequency_t frequency = spim_frequency_bit_decode(p_config->frequency);
#elif NRF_SPIM_HAS_PRESCALER
    uint32_t prescaler = spim_prescaler_calculate(p_instance, p_config->frequency);
#endif

#if NRF_ERRATA_STATIC_CHECK(54L, 55)
    if (NRF_ERRATA_DYNAMIC_CHECK(54L, 55))
    {
        p_cb->apply_nrf54l_errata_55 = 1;
    }
#endif

#if NRF_ERRATA_STATIC_CHECK(54L, 8) || NRF_ERRATA_STATIC_CHECK(54H, 212)
    if (NRF_ERRATA_DYNAMIC_CHECK(54L, 8) || NRF_ERRATA_DYNAMIC_CHECK(54H, 212))
    {
        /* Workaround must be applied only if PRESCALER is larger than 2 and CPHA=0 */
        if ((prescaler > 2) &&
            ((p_config->mode == NRF_SPIM_MODE_0) || (p_config->mode == NRF_SPIM_MODE_2)))
        {
            uint8_t min_dur = (uint8_t)((prescaler / 2) + 1);
            csn_duration = NRFX_MAX(csn_duration, min_dur);
            p_cb->apply_errata_8_212 = 1;
        }
        else
        {
            p_cb->apply_errata_8_212 = 0;
        }
    }
#endif

    p_cb->skip_gpio_cfg = p_config->skip_gpio_cfg;
    configure_pins(p_instance, p_config);

#if NRF_SPIM_HAS_HW_CSN
    if (p_config->use_hw_ss)
    {
        p_cb->ss_pin = NRF_SPIM_PIN_NOT_CONNECTED;
    }
    else
#endif
    {
        p_cb->ss_pin = p_config->ss_pin;
        p_cb->ss_active_high = p_config->ss_active_high;
    }

    nrfy_spim_config_t nrfy_config =
    {
        .pins =
        {
            .sck_pin  = p_config->sck_pin,
            .mosi_pin = p_config->mosi_pin,
            .miso_pin = p_config->miso_pin
        },
        .orc       = p_config->orc,
#if NRF_SPIM_HAS_FREQUENCY
        .frequency = frequency,
#elif NRF_SPIM_HAS_PRESCALER
        .prescaler = prescaler,
#endif
        .mode      = p_config->mode,
        .bit_order = p_config->bit_order,
#if NRFY_SPIM_HAS_EXTENDED
        .ext_config =
        {
            .pins =
            {
#if NRFY_SPIM_HAS_DCX
                .dcx_pin = p_config->dcx_pin,
#endif
#if NRFY_SPIM_HAS_HW_CSN
                .csn_pin =  (p_config->use_hw_ss ? p_config->ss_pin : NRF_SPIM_CSN_DEFAULT),
#endif
            },
#if NRFY_SPIM_HAS_HW_CSN
            .csn_pol      =  (p_config->use_hw_ss ? (p_config->ss_active_high ?
                                NRF_SPIM_CSN_POL_HIGH : NRF_SPIM_CSN_POL_LOW) :
                                    (nrf_spim_csn_pol_t)NRF_SPIM_CSNPOL_DEFAULT),
            .csn_duration = csn_duration,
#endif
#if NRFY_SPIM_HAS_RXDELAY
            .rx_delay = p_config->rx_delay,
#endif
        },
#endif // NRFY_SPIM_HAS_EXTENDED
        .skip_psel_cfg = p_config->skip_psel_cfg
    };

    nrfy_spim_periph_configure(p_instance->p_reg, &nrfy_config);
    if (p_cb->handler)
    {
        nrfy_spim_int_init(p_instance->p_reg, 0, p_config->irq_priority, false);
    }
}

int nrfx_spim_init(nrfx_spim_t *              p_instance,
                   nrfx_spim_config_t const * p_config,
                   nrfx_spim_event_handler_t  handler,
                   void *                     p_context)
{
    NRFX_ASSERT(p_instance);
    NRFX_ASSERT(p_config);

    nrfx_spim_control_block_t * p_cb = &p_instance->cb;
    int err_code;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    if (p_config)
    {
        err_code = spim_configuration_verify(p_instance, p_config);
        if (err_code != 0)
        {
            return err_code;
        }
    }

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    err_code = nrfx_prs_acquire(p_instance->p_reg,
                                (nrfx_irq_handler_t)nrfx_spim_irq_handler, p_instance);
    if (err_code < 0)
    {
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif // NRFX_CHECK(NRFX_PRS_ENABLED)

    p_cb->handler = handler;
    p_cb->p_context = p_context;

    if (p_config)
    {
        spim_configure(p_instance, p_config);
    }

    p_cb->transfer_in_progress = false;
    p_cb->state = NRFX_DRV_STATE_INITIALIZED;

    err_code = 0;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

int nrfx_spim_reconfigure(nrfx_spim_t *              p_instance,
                          nrfx_spim_config_t const * p_config)
{
    NRFX_ASSERT(p_instance);

    nrfx_spim_control_block_t * p_cb = &p_instance->cb;

    if (p_cb->state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        return -EINPROGRESS;
    }
    if (p_cb->transfer_in_progress)
    {
        return -EBUSY;
    }
    int err_code = spim_configuration_verify(p_instance, p_config);
    if (err_code != 0)
    {
        return err_code;
    }

    spim_configure(p_instance, p_config);

    return 0;
}

static void spim_pin_uninit(uint32_t pin)
{
    if (pin == NRF_SPIM_PIN_NOT_CONNECTED ||
#if defined(SPIM_DCX_DISCONNECTED_READBACK)
        pin == SPIM_DCX_DISCONNECTED_READBACK ||
#endif
        0)
    {
        return;
    }

    nrfy_gpio_cfg_default(pin);
}

void nrfx_spim_uninit(nrfx_spim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_spim_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_spim_int_uninit(p_instance->p_reg);
    if (p_cb->handler)
    {
        nrfy_spim_int_disable(p_instance->p_reg, NRF_SPIM_ALL_INTS_MASK);
        spim_abort(p_instance->p_reg, p_cb);
    }

    nrfy_spim_pins_t pins;
    nrfy_spim_pins_get(p_instance->p_reg, &pins);

    if (!p_cb->skip_gpio_cfg)
    {
        spim_pin_uninit(pins.sck_pin);
        spim_pin_uninit(pins.miso_pin);
        spim_pin_uninit(pins.mosi_pin);
        spim_pin_uninit(p_cb->ss_pin);
#if NRFY_SPIM_HAS_EXTENDED
        nrfy_spim_ext_pins_t ext_pins;
        nrfy_spim_ext_pins_get(p_instance->p_reg, &ext_pins);
#if NRFY_SPIM_HAS_DCX
        spim_pin_uninit(ext_pins.dcx_pin);
#endif
#if NRFY_SPIM_HAS_HW_CSN
        spim_pin_uninit(ext_pins.csn_pin);
#endif
#endif // NRFY_SPIM_HAS_EXTENDED
    }

#if defined(NRF_SPIM3)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 195) && (p_instance->p_reg == NRF_SPIM3))
    {
        *(volatile uint32_t *)0x4002F004 = 1;
    }
#endif

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(p_instance->p_reg);
#endif

    p_cb->state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_spim_init_check(nrfx_spim_t const * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_spim_control_block_t const * p_cb = &p_instance->cb;

    return (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
}

#if NRF_SPIM_HAS_DCX
int nrfx_spim_xfer_dcx(nrfx_spim_t *                 p_instance,
                       nrfx_spim_xfer_desc_t const * p_xfer_desc,
                       uint32_t                      flags,
                       uint8_t                       cmd_length)
{
    NRFX_ASSERT(p_instance);

    (void)flags;

    NRFX_ASSERT(cmd_length <= NRF_SPIM_DCX_CNT_ALL_CMD);

    nrfy_spim_dcx_cnt_set((NRF_SPIM_Type *)p_instance->p_reg, cmd_length);
    return nrfx_spim_xfer(p_instance, p_xfer_desc, 0);
}
#endif


static void set_ss_pin_state(nrfx_spim_control_block_t * p_cb, bool active)
{
    if (p_cb->ss_pin != NRF_SPIM_PIN_NOT_CONNECTED)
    {
        if (p_cb->ss_active_high)
        {
            if (active)
            {
                nrfy_gpio_pin_set(p_cb->ss_pin);
            }
            else
            {
                nrfy_gpio_pin_clear(p_cb->ss_pin);
            }
        }
        else
        {
            if (active)
            {
                nrfy_gpio_pin_clear(p_cb->ss_pin);
            }
            else
            {
                nrfy_gpio_pin_set(p_cb->ss_pin);
            }
        }
    }
}

static void finish_transfer(NRF_SPIM_Type * p_spim, nrfx_spim_control_block_t * p_cb)
{
    // If Slave Select signal is used, this is the time to deactivate it.
    set_ss_pin_state(p_cb, false);

    // By clearing this flag before calling the handler we allow subsequent
    // transfers to be started directly from the handler function.
    if (p_cb->transfer_in_progress)
    {
        spim_abort(p_spim, p_cb);
    }

    p_cb->evt.type = NRFX_SPIM_EVENT_DONE;
    p_cb->handler(&p_cb->evt, p_cb->p_context);
}

static int spim_xfer(NRF_SPIM_Type *               p_spim,
                     nrfx_spim_control_block_t *   p_cb,
                     nrfx_spim_xfer_desc_t const * p_xfer_desc,
                     uint32_t                      flags)
{
    int err_code;
    // EasyDMA requires that transfer buffers are placed in Data RAM region;
    // signal error if they are not.
    if ((p_xfer_desc->p_tx_buffer != NULL &&
         !nrf_dma_accessible_check(p_spim, p_xfer_desc->p_tx_buffer)) ||
        (p_xfer_desc->p_rx_buffer != NULL &&
         !nrf_dma_accessible_check(p_spim, p_xfer_desc->p_rx_buffer)))
    {
        p_cb->transfer_in_progress = false;
        err_code = -EACCES;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

#if NRF_ERRATA_STATIC_CHECK(52, 198) && defined(NRF_SPIM3)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 198) && p_spim == NRF_SPIM3)
    {
        anomaly_198_enable(p_xfer_desc->p_tx_buffer, p_xfer_desc->tx_length);
    }
#endif

#if NRFY_SPIM_HAS_ARRAY_LIST
    nrfy_spim_tx_list_set(p_spim, NRFX_SPIM_FLAG_TX_POSTINC & flags);
    nrfy_spim_rx_list_set(p_spim, NRFX_SPIM_FLAG_RX_POSTINC & flags);
#endif

#if NRF_ERRATA_STATIC_CHECK(52, 58)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 58) &&
        p_xfer_desc->rx_length == 1      &&
        p_xfer_desc->tx_length <= 1)
    {
        err_code = nrf52_errata_58_workaround_enable(p_cb, p_spim);
        if (err_code != 0)
        {
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                            __func__,
                            NRFX_LOG_ERROR_STRING_GET(err_code));
            return err_code;
        }
    }
#endif

    nrfy_spim_xfer_desc_t xfer_desc = *p_xfer_desc;
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 109) && (flags & NRFX_SPIM_FLAG_HOLD_XFER))
    {
        nrfy_spim_event_clear(p_spim, NRF_SPIM_EVENT_STARTED);
        xfer_desc.tx_length = 0;
        xfer_desc.rx_length = 0;
        nrfy_spim_int_enable(p_spim, NRF_SPIM_INT_STARTED_MASK);
    }
    nrfy_spim_buffers_set(p_spim, &xfer_desc);

    nrfy_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);
#if defined(NRF_SPIM_CHECK_DISABLE_ON_XFER_END)
    p_cb->disable_on_xfer_end = (flags & (NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER |
                                          NRFX_SPIM_FLAG_HOLD_XFER |
                                          NRFX_SPIM_FLAG_REPEATED_XFER)) ?
                                true : !nrfy_spim_enable_check(p_spim);
#endif
#if NRF_ERRATA_STATIC_CHECK(53, 135) && defined(NRF_SPIM4)
    if (NRF_ERRATA_DYNAMIC_CHECK(53, 135) && p_spim == NRF_SPIM4)
    {
        anomaly_135_enable();
    }
#endif
#if NRF_ERRATA_STATIC_CHECK(54L, 57) && defined(NRF_SPIM00)
    if (NRF_ERRATA_DYNAMIC_CHECK(54L, 57) && p_spim == NRF_SPIM00)
    {
        *(volatile uint32_t *)(((uint8_t *)p_spim) + 0xC04) = 0x2;
    }
#endif
    nrfy_spim_enable(p_spim);

#if NRF_ERRATA_STATIC_CHECK(54L, 55)
    if (p_cb->apply_nrf54l_errata_55)
    {
        *(volatile uint32_t *)((uint8_t *)p_spim + 0xc80) = 0x82;
    }
#endif

#if NRF_ERRATA_STATIC_CHECK(54L, 8) || NRF_ERRATA_STATIC_CHECK(54H, 212)
    if (p_cb->apply_errata_8_212)
    {
        *(volatile uint32_t *)((uint8_t *)p_spim + 0xc84) = 0x82;
        if (p_cb->handler)
        {
            nrfy_spim_event_clear(p_spim, NRF_SPIM_EVENT_STARTED);
            nrfy_spim_int_enable(p_spim, NRF_SPIM_INT_STARTED_MASK);
        }
    }
#endif

    if (!(flags & NRFX_SPIM_FLAG_HOLD_XFER))
    {
        nrfy_spim_xfer_start(p_spim, p_cb->handler ? NULL : &xfer_desc);
    }

    if (!p_cb->handler)
    {
#if NRF_ERRATA_STATIC_CHECK(54L, 55)
        if (p_cb->apply_nrf54l_errata_55)
        {
            *(volatile uint32_t *)((uint8_t *)p_spim + 0xc80) = 0;
        }
#endif

#if NRF_ERRATA_STATIC_CHECK(54L, 8) || NRF_ERRATA_STATIC_CHECK(54H, 212)
        if (p_cb->apply_errata_8_212)
        {
            *(volatile uint32_t *)((uint8_t *)p_spim + 0xc84) = 0;
        }
#endif

#if NRF_ERRATA_STATIC_CHECK(52, 198) && defined(NRF_SPIM3)
        if (NRF_ERRATA_DYNAMIC_CHECK(52, 198) && (p_spim == NRF_SPIM3))
        {
            anomaly_198_disable();
        }
#endif
        set_ss_pin_state(p_cb, false);
        if (!(flags & NRFX_SPIM_FLAG_HOLD_XFER))
        {
            spim_abort(p_spim, p_cb);
        }
    }
    else
    {
        if (flags & NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER)
        {
            nrfy_spim_int_disable(p_spim, NRF_SPIM_INT_END_MASK);
        }
        else
        {
            nrfy_spim_int_enable(p_spim, NRF_SPIM_INT_END_MASK);
        }
    }
    err_code = 0;
    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

int nrfx_spim_xfer(nrfx_spim_t *                 p_instance,
                   nrfx_spim_xfer_desc_t const * p_xfer_desc,
                   uint32_t                      flags)
{
    NRFX_ASSERT(p_instance);

    nrfx_spim_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);
    NRFX_ASSERT(p_xfer_desc->p_tx_buffer != NULL || p_xfer_desc->tx_length == 0);
    NRFX_ASSERT(p_xfer_desc->p_rx_buffer != NULL || p_xfer_desc->rx_length == 0);
    NRFX_ASSERT(!(flags & NRFX_SPIM_FLAG_HOLD_XFER) ||
                (p_cb->ss_pin == NRF_SPIM_PIN_NOT_CONNECTED));

    int err_code = 0;

#if !NRFY_SPIM_HAS_ARRAY_LIST
    if ((NRFX_SPIM_FLAG_TX_POSTINC | NRFX_SPIM_FLAG_RX_POSTINC) & flags)
    {
        err_code = -ENOTSUP;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
#endif

    if (p_cb->transfer_in_progress)
    {
        err_code = -EBUSY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }
    else
    {
        if (p_cb->handler && !(flags & (NRFX_SPIM_FLAG_REPEATED_XFER |
                                        NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER)))
        {
            p_cb->transfer_in_progress = true;
        }
    }

    p_cb->evt.xfer_desc = *p_xfer_desc;

    set_ss_pin_state(p_cb, true);

    return spim_xfer(p_instance->p_reg, p_cb,  p_xfer_desc, flags);
}

void nrfx_spim_abort(nrfx_spim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    nrfx_spim_control_block_t * p_cb = &p_instance->cb;

    NRFX_ASSERT(p_cb->state != NRFX_DRV_STATE_UNINITIALIZED);

    spim_abort(p_instance->p_reg, p_cb);
}

void nrfx_spim_irq_handler(nrfx_spim_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    NRF_SPIM_Type * p_spim = p_instance->p_reg;
    nrfx_spim_control_block_t * p_cb = &p_instance->cb;

#if NRF_ERRATA_STATIC_CHECK(54L, 55)
    if (p_cb->apply_nrf54l_errata_55 && nrfy_spim_event_check(p_spim, NRF_SPIM_EVENT_END))
    {
        *(volatile uint32_t *)((uint8_t *)p_spim + 0xc80) = 0;
    }
#endif

#if NRF_ERRATA_STATIC_CHECK(54L, 8) || NRF_ERRATA_STATIC_CHECK(54H, 212)
    if (p_cb->apply_errata_8_212)
    {
        if (nrfy_spim_int_enable_check(p_spim, NRF_SPIM_INT_STARTED_MASK) &&
            nrfy_spim_event_check(p_spim, NRF_SPIM_EVENT_STARTED))
        {
            nrfy_spim_event_clear(p_spim, NRF_SPIM_EVENT_STARTED);
            *(volatile uint32_t *)((uint8_t *)p_spim + 0xc84) = 0;
            nrfy_spim_int_disable(p_spim, NRF_SPIM_INT_STARTED_MASK);
        }
    }
#endif

    if (NRF_ERRATA_DYNAMIC_CHECK(52, 109) &&
        nrfy_spim_int_enable_check(p_spim, NRF_SPIM_INT_STARTED_MASK) &&
        nrfy_spim_event_check(p_spim, NRF_SPIM_EVENT_STARTED))
    {
        /* Handle first, zero-length, auxiliary transmission. */
        nrfy_spim_event_clear(p_spim, NRF_SPIM_EVENT_STARTED);
        nrfy_spim_event_clear(p_spim, NRF_SPIM_EVENT_END);

        NRFX_ASSERT(nrfy_spim_tx_maxcnt_get(p_spim) == 0);
        NRFX_ASSERT(nrfy_spim_rx_maxcnt_get(p_spim) == 0);

        /* Disable STARTED interrupt, used only in auxiliary transmission. */
        nrfy_spim_int_disable(p_spim, NRF_SPIM_INT_STARTED_MASK);

        /* Start the actual, glitch-free transmission. */
        nrfy_spim_buffers_set(p_spim, &p_cb->evt.xfer_desc);
        nrfy_spim_xfer_start(p_spim, NULL);
        return;
    }

    if (nrfy_spim_events_process(p_spim,
                                 NRFY_EVENT_TO_INT_BITMASK(NRF_SPIM_EVENT_END),
                                 &p_cb->evt.xfer_desc))
    {
#if NRF_ERRATA_STATIC_CHECK(52, 198) && defined(NRF_SPIM3)
        if (NRF_ERRATA_DYNAMIC_CHECK(52, 198) && p_spim == NRF_SPIM3)
        {
            anomaly_198_disable();
        }
#endif
        NRFX_ASSERT(p_cb->handler);
        NRFX_LOG_DEBUG("Event: NRF_SPIM_EVENT_END.");
        finish_transfer(p_spim, p_cb);
    }

#if NRF_ERRATA_STATIC_CHECK(52, 58)
    if (p_cb->apply_nrf52_errata_58)
    {
        int err_code = nrf52_errata_58_workaround_disable(p_cb, p_spim);
        if (err_code != 0)
        {
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                            __func__,
                            NRFX_LOG_ERROR_STRING_GET(err_code));
        }
    }
#endif
}

#if NRF_ERRATA_STATIC_CHECK(52, 58)
void nrfx_spim_nrf52_anomaly_58_init(nrfx_spim_t * p_instance, nrfx_gpiote_t * p_gpiote_inst)
{
    NRFX_ASSERT(p_instance);

    p_instance->cb.p_gpiote_inst = p_gpiote_inst;
}
#endif
