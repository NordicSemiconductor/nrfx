/*
 * Copyright (c) 2025 - 2026, Nordic Semiconductor ASA
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
#include "nrfx_gppiv1.h"
#include <helpers/nrfx_gppi.h>

void nrfx_gppi_init(nrfx_gppi_t * p_instance)
{
    (void)p_instance;
    NRFX_ASSERT(0);
}

uint32_t nrfx_gppi_group_domain_id_get(nrfx_gppi_group_handle_t handle)
{
    (void)handle;
    NRFX_ASSERT(0);
    return 0;
}

uint32_t nrfx_gppi_domain_id_get(uint32_t addr)
{
    (void)addr;
    NRFX_ASSERT(0);
    return 0;
}

int nrfx_gppi_domain_channel_get(nrfx_gppi_handle_t handle, uint32_t domain_id)
{
    (void)handle;
    (void)domain_id;
    NRFX_ASSERT(0);
    return -ENOTSUP;
}

int nrfx_gppi_domain_conn_alloc(uint32_t producer, uint32_t consumer,
                                nrfx_gppi_handle_t * p_handle)
{
    (void)producer;
    (void)consumer;
    (void)p_handle;
    NRFX_ASSERT(0);
    return -ENOTSUP;
}

int nrfx_gppi_ep_attach(uint32_t ep, nrfx_gppi_handle_t handle)
{
    nrfx_gppiv1_fork_endpoint_setup((uint8_t)handle, ep);
    return 0;
}

int nrfx_gppi_conn_alloc(uint32_t eep, uint32_t tep, nrfx_gppi_handle_t * p_handle)
{
    int err = nrfx_gppiv1_channel_alloc((uint8_t *)p_handle);
    if (err != 0)
    {
        return err;
    }
    nrfx_gppiv1_channel_endpoints_setup((uint8_t)*p_handle, eep, tep);
    return 0;
}

void nrfx_gppi_domain_conn_free(nrfx_gppi_handle_t handle)
{
    nrfx_gppiv1_channel_free((uint8_t)handle);
}

void nrfx_gppi_ep_clear(uint32_t ep)
{
    NRF_DPPI_ENDPOINT_CLEAR(ep);
}

void nrfx_gppi_ep_enable(uint32_t ep)
{
    (void)ep;
    NRFX_ASSERT(0);
}

void nrfx_gppi_ep_disable(uint32_t ep)
{
    (void)ep;
    NRFX_ASSERT(0);
}

void nrfx_gppi_conn_free(uint32_t eep, uint32_t tep, nrfx_gppi_handle_t handle)
{
    nrfx_gppiv1_channel_endpoints_clear((uint8_t)handle, eep, tep);
    nrfx_gppiv1_channel_free((uint8_t)handle);
}

void nrfx_gppi_conn_enable(nrfx_gppi_handle_t handle)
{
    nrfx_gppiv1_channels_enable(NRFX_BIT(handle));
}

void nrfx_gppi_conn_disable(nrfx_gppi_handle_t handle)
{
    nrfx_gppiv1_channels_disable(NRFX_BIT(handle));
}

void nrfx_gppi_channels_enable(uint32_t domain_id, uint32_t ch_mask)
{
    (void)domain_id;
    (void)ch_mask;
    NRFX_ASSERT(0);
}

bool nrfx_gppi_chan_is_enabled(uint32_t domain_id, uint32_t ch)
{
    (void)domain_id;
    (void)ch;
    NRFX_ASSERT(0);
    return false;
}

void nrfx_gppi_channels_disable(uint32_t domain_id, uint32_t ch_mask)
{
    (void)domain_id;
    (void)ch_mask;
    NRFX_ASSERT(0);
}

int nrfx_gppi_group_alloc(uint32_t domain_id, nrfx_gppi_group_handle_t * p_handle)
{
    (void)p_handle;
    NRFX_ASSERT(0);
    return -ENOTSUP;
}

void nrfx_gppi_group_free(nrfx_gppi_group_handle_t handle)
{
    (void)handle;
    NRFX_ASSERT(0);
}

void nrfx_gppi_group_ch_add(nrfx_gppi_group_handle_t handle, uint32_t channel)
{
    (void)handle;
    (void)channel;
    NRFX_ASSERT(0);
}

void nrfx_gppi_group_ch_remove(nrfx_gppi_group_handle_t handle, uint32_t channel)
{
    (void)handle;
    (void)channel;
    NRFX_ASSERT(0);
}

uint32_t nrfx_gppi_group_channels_get(nrfx_gppi_group_handle_t handle)
{
    (void)handle;
    NRFX_ASSERT(0);
    return 0;
}

void nrfx_gppi_group_enable(nrfx_gppi_group_handle_t handle)
{
	(void)handle;
    NRFX_ASSERT(0);
}

void nrfx_gppi_group_disable(nrfx_gppi_group_handle_t handle)
{
	(void)handle;
    NRFX_ASSERT(0);
}

uint32_t nrfx_gppi_group_task_en_addr(nrfx_gppi_group_handle_t handle)
{
    NRFX_ASSERT(0);
    return 0;
}

uint32_t nrfx_gppi_group_task_dis_addr(nrfx_gppi_group_handle_t handle)
{
    NRFX_ASSERT(0);
    return 0;
}

int nrfx_gppi_channel_alloc(uint32_t node_id)
{
    (void)node_id;
    NRFX_ASSERT(0);
    return -ENOTSUP;
}

void nrfx_gppi_channel_free(uint32_t node_id, uint8_t channel)
{
    (void)node_id;
    (void)channel;
    NRFX_ASSERT(0);
}

int nrfx_gppi_group_channel_alloc(uint32_t node_id)
{
    (void)node_id;
    NRFX_ASSERT(0);
    return -ENOTSUP;
}

void nrfx_gppi_group_channel_free(uint32_t node_id, uint8_t channel)
{
    (void)node_id;
    (void)channel;
    NRFX_ASSERT(0);
}

int nrfx_gppi_ext_conn_alloc(uint32_t producer, uint32_t consumer, nrfx_gppi_handle_t * p_handle,
                             nrfx_gppi_resource_t * p_resource)
{
    (void)producer;
    (void)consumer;
    (void)p_handle;
    (void)p_resource;
    NRFX_ASSERT(0);
    return -ENOTSUP;
}
