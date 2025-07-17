/*
 * Copyright (c) 2023 - 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_RRAMC_ENABLED)

#include <nrfx_rramc.h>
#include <hal/nrf_ficr.h>

#define NRFX_LOG_MODULE RRAMC
#include <nrfx_log.h>

// Control block - driver instance local data.
typedef struct
{
    nrfx_rramc_evt_handler_t handler;
    nrfx_drv_state_t         state;
} rramc_control_block_t;
static rramc_control_block_t m_cb;

NRFX_STATIC_INLINE uint32_t rram_variant_get(NRF_FICR_Type const * p_reg)
{
    return p_reg->INFO.RRAM;
}

NRFX_STATIC_INLINE uint32_t total_memory_size_get(void)
{
    uint32_t size = rram_variant_get(NRF_FICR);

    if (size == FICR_INFO_RRAM_RRAM_Unspecified)
    {
        return (uint32_t)FICR_INFO_RRAM_RRAM_Unspecified;
    }

    return (size * 1024UL);
}

NRFX_STATIC_INLINE __UNUSED bool is_valid_address(uint32_t addr, bool uicr_allowed)
{
    if ((addr - NRFY_RRAMC_RRAM_BASE_ADDRESS) < total_memory_size_get())
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

NRFX_STATIC_INLINE __UNUSED bool fit_in_memory(uint32_t addr, bool uicr_allowed, uint32_t bytes)
{
    if ((addr - NRFY_RRAMC_RRAM_BASE_ADDRESS + bytes) < total_memory_size_get())
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

void nrfx_rramc_all_erase(void)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_rramc_erase_all_set(NRF_RRAMC);
    while (!nrfy_rramc_erase_all_check(NRF_RRAMC))
    {}
}

void nrfx_rramc_byte_write(uint32_t address, uint8_t value)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(is_valid_address(address, true));
    NRFX_ASSERT(fit_in_memory(address, true, 1));

    nrfy_rramc_byte_write(NRF_RRAMC, address, value);
}

void nrfx_rramc_bytes_write(uint32_t address, void const * src, uint32_t num_bytes)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(src);
    NRFX_ASSERT(is_valid_address(address, true));
    NRFX_ASSERT(fit_in_memory(address, true, num_bytes));

    nrfy_rramc_bytes_write(NRF_RRAMC, address, src, num_bytes);
}

void nrfx_rramc_word_write(uint32_t address, uint32_t value)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(is_valid_address(address, true));
    NRFX_ASSERT(fit_in_memory(address, true, NRFY_RRAMC_BYTES_IN_WORD));
    NRFX_ASSERT(nrfx_is_word_aligned((void const *)address));

    nrfy_rramc_word_write(NRF_RRAMC, address, value);
}

void nrfx_rramc_words_write(uint32_t address, void const * src, uint32_t num_words)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(src);
    NRFX_ASSERT(is_valid_address(address, true));
    NRFX_ASSERT(fit_in_memory(address, true, (num_words * NRFY_RRAMC_BYTES_IN_WORD)));
    NRFX_ASSERT(nrfx_is_word_aligned((void const *)address));
    NRFX_ASSERT(nrfx_is_word_aligned(src));

    nrfy_rramc_words_write(NRF_RRAMC, address, src, num_words);
}

void nrfx_rramc_write_enable_set(bool enable, uint32_t write_buff_size)
{
    nrf_rramc_config_t rramc_config = {
        .mode_write      = enable,
        .write_buff_size = (uint8_t)write_buff_size,
    };
    nrfy_rramc_config_set(NRF_RRAMC, &rramc_config);
}

bool nrfx_rramc_write_enable_check(void)
{
    nrf_rramc_config_t rramc_config;
    nrfy_rramc_config_get(NRF_RRAMC, &rramc_config);

    return (rramc_config.mode_write == true);
}

static nrfx_err_t rramc_configure(nrfx_rramc_config_t const * p_config)
{
    nrfy_rramc_config_t nrfy_config =
    {
        .config = {
            .mode_write      = p_config->mode_write,
            .write_buff_size = p_config->write_buff_size,
        },
        .preload_timeout = {
            .value           = p_config->preload_timeout,
            .enable          = p_config->preload_timeout_enable,
        },
        .power = {
            .access_timeout  = p_config->access_timeout,
            .abort_on_pof    = p_config->abort_on_pof,
        },
    };
    nrfy_rramc_configure(NRF_RRAMC, &nrfy_config);

    if (m_cb.handler)
    {
        nrfy_rramc_int_init(NRF_RRAMC, NRF_RRAMC_ALL_INTS_MASK, p_config->irq_priority, true);
    }
    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_rramc_init(nrfx_rramc_config_t const * p_config,
                           nrfx_rramc_evt_handler_t    handler)
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

    err_code = rramc_configure(p_config);
    if (err_code != NRFX_SUCCESS)
    {
        return err_code;
    }

    m_cb.state = NRFX_DRV_STATE_INITIALIZED;

    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_rramc_reconfigure(nrfx_rramc_config_t const * p_config)
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

    return rramc_configure(p_config);
}

void nrfx_rramc_uninit(void)
{
    NRFX_ASSERT(m_cb.state != NRFX_DRV_STATE_UNINITIALIZED);

    nrfy_rramc_int_uninit(NRF_RRAMC);
    if (m_cb.handler)
    {
        nrfy_rramc_int_disable(NRF_RRAMC, NRF_RRAMC_ALL_INTS_MASK);
    }

    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

uint32_t nrfx_rramc_memory_size_get(void)
{
    return total_memory_size_get();
}

void nrfx_rramc_write_buffer_commit(void)
{
    nrfy_rramc_task_trigger(NRF_RRAMC, NRF_RRAMC_TASK_COMMIT_WRITEBUF);
}

void nrfx_rramc_wake_up(void)
{
    nrfy_rramc_task_trigger(NRF_RRAMC, NRF_RRAMC_TASK_WAKEUP);
}

bool nrfx_rramc_write_buffer_empty_check(void)
{
    return nrfy_rramc_empty_buffer_check(NRF_RRAMC);
}

void nrfx_rramc_irq_handler(void)
{
    NRFX_ASSERT(m_cb.handler);

    uint32_t evts = nrfy_rramc_events_process(NRF_RRAMC, NRF_RRAMC_ALL_INTS_MASK);

    if (evts & NRF_RRAMC_INT_ERROR_ACCESS_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_RRAMC_EVENT_ERROR_ACCESS.");
        m_cb.handler(NRF_RRAMC_EVENT_ERROR_ACCESS);
    }

    if (evts & NRF_RRAMC_INT_READY_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_RRAMC_EVENT_READY.");
        m_cb.handler(NRF_RRAMC_EVENT_READY);
    }

    if (evts & NRF_RRAMC_INT_READY_NEXT_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_RRAMC_EVENT_READY_NEXT.");
        m_cb.handler(NRF_RRAMC_EVENT_READY_NEXT);
    }

    if (evts & NRF_RRAMC_INT_WOKENUP_MASK)
    {
        NRFX_LOG_DEBUG("Event: NRF_RRAMC_EVENT_WOKENUP.");
        m_cb.handler(NRF_RRAMC_EVENT_WOKENUP);
    }
}

#endif // NRFX_CHECK(NRFX_RRAMC_ENABLED)
