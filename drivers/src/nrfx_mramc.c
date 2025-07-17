/*
 * Copyright (c) 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_MRAMC_ENABLED)

#include <nrfx_mramc.h>
#include <hal/nrf_ficr.h>

#define NRFX_LOG_MODULE MRAMC
#include <nrfx_log.h>

// Control block - driver instance local data.
typedef struct
{
    nrfx_mramc_evt_handler_t handler;
    nrfx_drv_state_t         state;
} mramc_control_block_t;
static mramc_control_block_t m_cb;

NRFX_STATIC_INLINE uint32_t mram_variant_get(NRF_FICR_Type const * p_reg)
{
    return p_reg->INFO.MRAM;
}

NRFX_STATIC_INLINE uint32_t total_memory_size_get(void)
{
    uint32_t size = mram_variant_get(NRF_FICR);

    if (size == FICR_INFO_MRAM_MRAM_Unspecified)
    {
        return (uint32_t)FICR_INFO_MRAM_MRAM_Unspecified;
    }

    return (size * 1024UL);
}

bool nrfx_mramc_valid_address_check(uint32_t addr, bool uicr_allowed)
{
    if ((addr - NRFY_MRAMC_MRAM_BASE_ADDRESS) < total_memory_size_get())
    {
        return true;
    }
#if !defined(NRF_TRUSTZONE_NONSECURE)
    if (uicr_allowed && (addr - (uint32_t)NRF_UICR) < sizeof(NRF_UICR_Type))
    {
        return true;
    }
#else
    (void)uicr_allowed;
#endif

    return false;
}

bool nrfx_mramc_fits_memory_check(uint32_t addr, bool uicr_allowed, uint32_t bytes)
{
    if ((addr - NRFY_MRAMC_MRAM_BASE_ADDRESS + bytes) < total_memory_size_get())
    {
        return true;
    }
#if !defined(NRF_TRUSTZONE_NONSECURE)
    if (uicr_allowed && (addr - (uint32_t)NRF_UICR + bytes) < sizeof(NRF_UICR_Type))
    {
        return true;
    }
#else
    (void)uicr_allowed;
#endif

    return false;
}

void nrfx_mramc_all_erase(void)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_mramc_erase_all(NRF_MRAMC);
    while (!nrfy_mramc_erase_all_get(NRF_MRAMC))
    {}
}

void nrfx_mramc_word_write(uint32_t address, uint32_t value)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(nrfx_mramc_valid_address_check(address, true));
    NRFX_ASSERT(nrfx_mramc_fits_memory_check(address, true, NRFY_MRAMC_BYTES_IN_WORD));
    NRFX_ASSERT(nrfx_is_word_aligned((void const *)address));

    nrfy_mramc_word_write(NRF_MRAMC, address, value);
}

void nrfx_mramc_words_write(uint32_t address, void const * src, uint32_t num_words)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(src);
    NRFX_ASSERT(nrfx_mramc_valid_address_check(address, true));
    NRFX_ASSERT(nrfx_mramc_fits_memory_check(address, true, (num_words * NRFY_MRAMC_BYTES_IN_WORD)));
    NRFX_ASSERT(nrfx_is_word_aligned((void const *)address));
    NRFX_ASSERT(nrfx_is_word_aligned(src));

    nrfy_mramc_words_write(NRF_MRAMC, address, src, num_words);
}

void nrfx_mramc_buffer_read(void * dst, uint32_t address, uint32_t num_bytes)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(nrfx_mramc_valid_address_check(address, true));
    NRFX_ASSERT(nrfx_mramc_fits_memory_check(address, true, num_bytes));

    nrfy_mramc_buffer_read(dst, address, num_bytes);
}

void nrfx_mramc_config_write_mode_set(nrf_mramc_mode_write_t write_mode)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrf_mramc_config_t config;
    nrfy_mramc_config_get(NRF_MRAMC, &config);
    config.mode_write = write_mode;
    nrfy_mramc_config_set(NRF_MRAMC, &config);
}

void nrfx_mramc_config_erase_mode_set(nrf_mramc_mode_erase_t erase_mode)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrf_mramc_config_t config;
    nrfy_mramc_config_get(NRF_MRAMC, &config);
    config.mode_erase = erase_mode;
    nrfy_mramc_config_set(NRF_MRAMC, &config);
}

static nrfx_err_t mramc_configure(nrfx_mramc_config_t const * p_config)
{
    nrfy_mramc_config_t nrfy_config = {
        .config = {
            .mode_write      = p_config->config.mode_write,
            .mode_erase      = p_config->config.mode_erase,
#if NRF_MRAMC_HAS_CONFIG_DISABLEECC
            .disable_ecc     = p_config->config.disable_ecc,
#endif
        },
        .preload_timeout = {
            .value          = p_config->preload_timeout.value,
            .direct_write   = p_config->preload_timeout.direct_write,
        },
        .powerdown = {
            .enable         = p_config->powerdown.enable,
            .power_down_cfg = p_config->powerdown.power_down_cfg,
            .timeout_value  = p_config->powerdown.timeout_value,
        },
    };
    nrfy_mramc_configure(NRF_MRAMC, &nrfy_config);
    if (m_cb.handler)
    {
        nrfy_mramc_int_init(NRF_MRAMC, NRF_MRAMC_ALL_INTS_MASK, p_config->irq_priority, true);
    }

    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_mramc_init(nrfx_mramc_config_t const * p_config,
                           nrfx_mramc_evt_handler_t    handler)
{
    NRFX_ASSERT(p_config);

    nrfx_err_t err_code = NRFX_SUCCESS;

    if (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = NRFX_ERROR_ALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    m_cb.handler = handler;

    err_code = mramc_configure(p_config);
    if (err_code != NRFX_SUCCESS)
    {
        return err_code;
    }

    m_cb.state = NRFX_DRV_STATE_INITIALIZED;

    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_mramc_reconfigure(nrfx_mramc_config_t const * p_config)
{
    NRFX_ASSERT(p_config);
    nrfx_err_t err_code;

    if (m_cb.state == NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = NRFX_ERROR_INVALID_STATE;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    return mramc_configure(p_config);
}

void nrfx_mramc_uninit(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_mramc_int_uninit(NRF_MRAMC);
    if (m_cb.handler)
    {
        nrfy_mramc_int_disable(NRF_MRAMC, NRF_MRAMC_ALL_INTS_MASK);
    }

    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

uint32_t nrfx_mramc_memory_size_get(void)
{
    return total_memory_size_get();
}

void nrfx_mramc_area_erase(uint32_t address, uint32_t size)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(nrfx_mramc_valid_address_check(address, true));
    NRFX_ASSERT(nrfx_mramc_fits_memory_check(address, true, (size * NRFY_MRAMC_BYTES_IN_WORD)));
    NRFX_ASSERT(nrfx_is_word_aligned((void const *)address));

    nrfy_mramc_erase_area_set(NRF_MRAMC, address, size);
    while (!nrfx_mramc_ready_check())
    {}
}

void nrfx_mramc_irq_handler(void)
{
    NRFX_ASSERT(m_cb.handler);

    uint32_t evts = nrfy_mramc_events_process(NRF_MRAMC, NRF_MRAMC_ALL_INTS_MASK);

    if (evts & NRF_MRAMC_INT_READY_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_MRAMC_EVENT_READY.");
        m_cb.handler(NRF_MRAMC_EVENT_READY);
    }

    if (evts & NRF_MRAMC_INT_READYNEXT_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_MRAMC_EVENT_READYNEXT.");
        m_cb.handler(NRF_MRAMC_EVENT_READYNEXT);
    }

    if (evts & NRF_MRAMC_INT_ECCERROR_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_MRAMC_EVENT_ECCERROR.");
        m_cb.handler(NRF_MRAMC_EVENT_ECCERROR);
    }

    if (evts & NRF_MRAMC_INT_ECCERRORCORR_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_MRAMC_EVENT_ECCERRORCORR.");
        m_cb.handler(NRF_MRAMC_EVENT_ECCERRORCORR);
    }

    if (evts & NRF_MRAMC_INT_TRIMCONFIGREQ_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_MRAMC_EVENT_TRIMCONFIGREQ.");
        m_cb.handler(NRF_MRAMC_EVENT_TRIMCONFIGREQ);
    }

    if (evts & NRF_MRAMC_INT_ACCESSERR_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_MRAMC_EVENT_ACCESSERR.");
        m_cb.handler(NRF_MRAMC_EVENT_ACCESSERR);
    }
}

#endif // NRFX_CHECK(NRFX_MRAMC_ENABLED)
