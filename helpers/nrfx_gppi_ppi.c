/*
 * Copyright (c) 2022 - 2025, Nordic Semiconductor ASA
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

#include <helpers/nrfx_gppi.h>

#if NRFX_CHECK(NRFX_PPI_ENABLED)
#include <nrfx_ppi.h>
#endif

#if defined(PPI_PRESENT)

bool nrfx_gppi_channel_check(uint8_t channel)
{
    return (nrf_ppi_channel_enable_get(NRF_PPI, (nrf_ppi_channel_t)channel) ==
            NRF_PPI_CHANNEL_ENABLED);
}

void nrfx_gppi_channels_disable_all(void)
{
    nrf_ppi_channels_disable_all(NRF_PPI);
}

void nrfx_gppi_channels_enable(uint32_t mask)
{
    nrf_ppi_channels_enable(NRF_PPI, mask);
}

void nrfx_gppi_channels_disable(uint32_t mask)
{
    nrf_ppi_channels_disable(NRF_PPI, mask);
}

void nrfx_gppi_event_endpoint_setup(uint8_t channel, uint32_t eep)
{
    nrf_ppi_event_endpoint_setup(NRF_PPI, (nrf_ppi_channel_t)channel, eep);
}

void nrfx_gppi_task_endpoint_setup(uint8_t channel, uint32_t tep)
{
    nrf_ppi_task_endpoint_setup(NRF_PPI, (nrf_ppi_channel_t)channel, tep);
}

void nrfx_gppi_channel_endpoints_setup(uint8_t  channel, uint32_t eep, uint32_t tep)
{
    nrf_ppi_channel_endpoint_setup(NRF_PPI, (nrf_ppi_channel_t)channel, eep, tep);
}

void nrfx_gppi_channel_endpoints_clear(uint8_t channel, uint32_t eep, uint32_t tep)
{
    nrfx_gppi_event_endpoint_clear(channel, eep);
    nrfx_gppi_task_endpoint_clear(channel, tep);
}

void nrfx_gppi_event_endpoint_clear(uint8_t channel, uint32_t eep)
{
    (void)eep;
     nrf_ppi_event_endpoint_setup(NRF_PPI, (nrf_ppi_channel_t)channel, 0);
}

void nrfx_gppi_task_endpoint_clear(uint8_t channel, uint32_t tep)
{
    (void)tep;
    nrf_ppi_task_endpoint_setup(NRF_PPI, (nrf_ppi_channel_t)channel, 0);
}

#if defined(PPI_FEATURE_FORKS_PRESENT)
void nrfx_gppi_fork_endpoint_setup(uint8_t channel, uint32_t fork_tep)
{
    nrf_ppi_fork_endpoint_setup(NRF_PPI, (nrf_ppi_channel_t)channel, fork_tep);
}

void nrfx_gppi_fork_endpoint_clear(uint8_t channel, uint32_t fork_tep)
{
    (void)fork_tep;
    nrf_ppi_fork_endpoint_setup(NRF_PPI, (nrf_ppi_channel_t)channel, 0);
}
#endif

void nrfx_gppi_channels_group_set(uint32_t                  channel_mask,
                                  nrfx_gppi_channel_group_t channel_group)
{
    nrf_ppi_channels_group_set(NRF_PPI,
                               channel_mask,
                               (nrf_ppi_channel_group_t)channel_group);
}

void nrfx_gppi_channels_include_in_group(uint32_t                  channel_mask,
                                         nrfx_gppi_channel_group_t channel_group)
{
    nrf_ppi_channels_include_in_group(NRF_PPI,
                                      channel_mask,
                                      (nrf_ppi_channel_group_t)channel_group);
}

void nrfx_gppi_channels_remove_from_group(uint32_t                  channel_mask,
                                          nrfx_gppi_channel_group_t channel_group)
{
    nrf_ppi_channels_remove_from_group(NRF_PPI,
                                       channel_mask,
                                       (nrf_ppi_channel_group_t)channel_group);
}

void nrfx_gppi_group_clear(nrfx_gppi_channel_group_t channel_group)
{
    nrf_ppi_group_clear(NRF_PPI, (nrf_ppi_channel_group_t)channel_group);
}

void nrfx_gppi_group_enable(nrfx_gppi_channel_group_t channel_group)
{
    nrf_ppi_group_enable(NRF_PPI, (nrf_ppi_channel_group_t)channel_group);
}

void nrfx_gppi_group_disable(nrfx_gppi_channel_group_t channel_group)
{
    nrf_ppi_group_disable(NRF_PPI, (nrf_ppi_channel_group_t)channel_group);
}

void nrfx_gppi_task_trigger(nrfx_gppi_task_t task)
{
    nrf_ppi_task_trigger(NRF_PPI, (nrf_ppi_task_t)task);
}

uint32_t nrfx_gppi_task_address_get(nrfx_gppi_task_t task)
{
    return (uint32_t)nrf_ppi_task_address_get(NRF_PPI, (nrf_ppi_task_t)task);
}

nrfx_gppi_task_t nrfx_gppi_group_disable_task_get(nrfx_gppi_channel_group_t group)
{
    return (nrfx_gppi_task_t)nrf_ppi_group_disable_task_get(NRF_PPI, (uint8_t)group);
}

nrfx_gppi_task_t nrfx_gppi_group_enable_task_get(nrfx_gppi_channel_group_t group)
{
    return (nrfx_gppi_task_t)nrf_ppi_group_enable_task_get(NRF_PPI, (uint8_t)group);
}

nrfx_err_t nrfx_gppi_channel_alloc(uint8_t * p_channel)
{
#if NRFX_CHECK(NRFX_PPI_ENABLED)
    return nrfx_ppi_channel_alloc((nrf_ppi_channel_t *)p_channel);
#else
    (void)p_channel;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}

nrfx_err_t nrfx_gppi_channel_free(uint8_t channel)
{
#if NRFX_CHECK(NRFX_PPI_ENABLED)
    return nrfx_ppi_channel_free((nrf_ppi_channel_t)channel);
#else
    (void)channel;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}

nrfx_err_t nrfx_gppi_group_alloc(nrfx_gppi_channel_group_t * p_group)
{
#if NRFX_CHECK(NRFX_PPI_ENABLED)
    return nrfx_ppi_group_alloc((nrf_ppi_channel_group_t *)p_group);
#else
    (void)p_group;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}

nrfx_err_t nrfx_gppi_group_free(nrfx_gppi_channel_group_t group)
{
#if NRFX_CHECK(NRFX_PPI_ENABLED)
    return nrfx_ppi_group_free((nrf_ppi_channel_group_t)group);
#else
    (void)group;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}
#endif // defined(PPI_PRESENT)
