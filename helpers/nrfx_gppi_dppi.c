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

#if NRFX_CHECK(NRFX_DPPI_ENABLED) && (!defined(DPPIC_COUNT) || (DPPIC_COUNT == 1))
#include <nrfx_dppi.h>

#if NRFX_API_VER_AT_LEAST(3, 8, 0)
static nrfx_dppi_t dppi = NRFX_DPPI_INSTANCE(0);
#endif
#endif

#if defined(DPPI_PRESENT) && (defined(NRF53_SERIES) || defined(NRF91_SERIES))
bool nrfx_gppi_channel_check(uint8_t channel)
{
    return nrfy_dppi_channel_check(NRF_DPPIC, channel);
}

void nrfx_gppi_channels_disable_all(void)
{
    nrfy_dppi_channels_disable_all(NRF_DPPIC);
}

void nrfx_gppi_channels_enable(uint32_t mask)
{
    nrfy_dppi_channels_enable(NRF_DPPIC, mask);
}

void nrfx_gppi_channels_disable(uint32_t mask)
{
    nrfy_dppi_channels_disable(NRF_DPPIC, mask);
}

void nrfx_gppi_task_trigger(nrfx_gppi_task_t task)
{
    nrfy_dppi_task_trigger(NRF_DPPIC, (nrf_dppi_task_t)task);
}

void nrfx_gppi_event_endpoint_setup(uint8_t channel, uint32_t eep)
{
    NRFX_ASSERT(eep);
    NRF_DPPI_ENDPOINT_SETUP(eep, channel);
}

void nrfx_gppi_task_endpoint_setup(uint8_t channel, uint32_t tep)
{
    NRFX_ASSERT(tep);
    NRF_DPPI_ENDPOINT_SETUP(tep, channel);
}

void nrfx_gppi_channel_endpoints_setup(uint8_t  channel, uint32_t eep, uint32_t tep)
{
    nrfx_gppi_event_endpoint_setup(channel, eep);
    nrfx_gppi_task_endpoint_setup(channel, tep);
}

void nrfx_gppi_channel_endpoints_clear(uint8_t channel, uint32_t eep, uint32_t tep)
{
    nrfx_gppi_event_endpoint_clear(channel, eep);
    nrfx_gppi_task_endpoint_clear(channel, tep);
}

void nrfx_gppi_event_endpoint_clear(uint8_t channel, uint32_t eep)
{
    NRFX_ASSERT(eep);
    (void)channel;
    NRF_DPPI_ENDPOINT_CLEAR(eep);
}

void nrfx_gppi_task_endpoint_clear(uint8_t channel, uint32_t tep)
{
    NRFX_ASSERT(tep);
    (void)channel;
    NRF_DPPI_ENDPOINT_CLEAR(tep);
}

void nrfx_gppi_fork_endpoint_setup(uint8_t channel, uint32_t fork_tep)
{
    nrfx_gppi_task_endpoint_setup(channel, fork_tep);
}

void nrfx_gppi_fork_endpoint_clear(uint8_t channel, uint32_t fork_tep)
{
    nrfx_gppi_task_endpoint_clear(channel, fork_tep);
}

void nrfx_gppi_channels_group_set(uint32_t                  channel_mask,
                                  nrfx_gppi_channel_group_t channel_group)
{
    nrfy_dppi_channels_group_set(NRF_DPPIC,
                                 channel_mask,
                                 (nrf_dppi_channel_group_t)channel_group);
}

void nrfx_gppi_channels_include_in_group(uint32_t                  channel_mask,
                                         nrfx_gppi_channel_group_t channel_group)
{
    nrfy_dppi_channels_include_in_group(NRF_DPPIC,
                                        channel_mask,
                                        (nrf_dppi_channel_group_t)channel_group);
}

void nrfx_gppi_channels_remove_from_group(uint32_t                  channel_mask,
                                          nrfx_gppi_channel_group_t channel_group)
{
    nrfy_dppi_channels_remove_from_group(NRF_DPPIC,
                                         channel_mask,
                                         (nrf_dppi_channel_group_t)channel_group);
}

void nrfx_gppi_group_clear(nrfx_gppi_channel_group_t channel_group)
{
    nrfy_dppi_group_clear(NRF_DPPIC, (nrf_dppi_channel_group_t)channel_group);
}

void nrfx_gppi_group_enable(nrfx_gppi_channel_group_t channel_group)
{
    nrfy_dppi_group_enable(NRF_DPPIC, (nrf_dppi_channel_group_t)channel_group);
}

void nrfx_gppi_group_disable(nrfx_gppi_channel_group_t channel_group)
{
    nrfy_dppi_group_disable(NRF_DPPIC, (nrf_dppi_channel_group_t)channel_group);
}

uint32_t nrfx_gppi_task_address_get(nrfx_gppi_task_t gppi_task)
{
    return nrfy_dppi_task_address_get(NRF_DPPIC, (nrf_dppi_task_t)gppi_task);
}

nrfx_gppi_task_t nrfx_gppi_group_disable_task_get(nrfx_gppi_channel_group_t group)
{
    return (nrfx_gppi_task_t) nrfy_dppi_group_disable_task_get((uint8_t)group);
}

nrfx_gppi_task_t nrfx_gppi_group_enable_task_get(nrfx_gppi_channel_group_t group)
{
    return (nrfx_gppi_task_t) nrfy_dppi_group_enable_task_get((uint8_t)group);
}

nrfx_err_t nrfx_gppi_channel_alloc(uint8_t * p_channel)
{
#if NRFX_CHECK(NRFX_DPPI_ENABLED) && (!defined(DPPIC_COUNT) || (DPPIC_COUNT == 1))
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
    return nrfx_dppi_channel_alloc(&dppi, p_channel);
#else
    return nrfx_dppi_channel_alloc(p_channel);
#endif
#else
    (void)p_channel;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}

nrfx_err_t nrfx_gppi_channel_free(uint8_t channel)
{
#if NRFX_CHECK(NRFX_DPPI_ENABLED) && (!defined(DPPIC_COUNT) || (DPPIC_COUNT == 1))
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
    return nrfx_dppi_channel_free(&dppi, channel);
#else
    return nrfx_dppi_channel_free(channel);
#endif
#else
    (void)channel;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}

nrfx_err_t nrfx_gppi_group_alloc(nrfx_gppi_channel_group_t * p_group)
{
#if NRFX_CHECK(NRFX_DPPI_ENABLED) && (!defined(DPPIC_COUNT) || (DPPIC_COUNT == 1))
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
    return nrfx_dppi_group_alloc(&dppi, (nrf_dppi_channel_group_t *)p_group);
#else
    return nrfx_dppi_group_alloc((nrf_dppi_channel_group_t *)p_group);
#endif
#else
    (void)p_group;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}

nrfx_err_t nrfx_gppi_group_free(nrfx_gppi_channel_group_t group)
{
#if NRFX_CHECK(NRFX_DPPI_ENABLED) && (!defined(DPPIC_COUNT) || (DPPIC_COUNT == 1))
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
    return nrfx_dppi_group_free(&dppi, (nrf_dppi_channel_group_t)group);
#else
    return nrfx_dppi_group_free((nrf_dppi_channel_group_t)group);
#endif
#else
    (void)group;
    return NRFX_ERROR_NOT_SUPPORTED;
#endif
}

nrfx_err_t nrfx_gppi_edge_connection_setup(uint8_t             channel,
                                           nrfx_dppi_t const * p_src_dppi,
                                           uint8_t             src_channel,
                                           nrfx_dppi_t const * p_dst_dppi,
                                           uint8_t             dst_channel)
{
    (void)channel;
    (void)p_src_dppi;
    (void)src_channel;
    (void)p_dst_dppi;
    (void)dst_channel;

    return NRFX_SUCCESS; /* No operation as there is only one DPPIC instance. */
}

#endif // defined(DPPI_PRESENT) && (defined(NRF53_SERIES) || defined(NRF91_SERIES))
