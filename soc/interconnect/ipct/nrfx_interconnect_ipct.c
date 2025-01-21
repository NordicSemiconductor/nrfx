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
#include <nrfx.h>

#if defined(HALTIUM_XXAA)

#include <soc/interconnect/ipct/nrfx_interconnect_ipct_haltium_global.h>
#include <soc/interconnect/ipct/nrfx_interconnect_ipct.h>
#include <soc/interconnect/apb/nrfx_interconnect_apb.h>

#if defined(NRF_RADIOCORE)
    #include <soc/interconnect/ipct/nrfx_interconnect_ipct_haltium_radiocore.h>
#elif defined(NRF_APPLICATION)
    #include <soc/interconnect/ipct/nrfx_interconnect_ipct_haltium_application.h>
#elif defined(NRF_PPR)
    #include <soc/interconnect/ipct/nrfx_interconnect_ipct_haltium_ppr.h>
#elif defined(NRF_FLPR)
    #include <soc/interconnect/ipct/nrfx_interconnect_ipct_haltium_flpr.h>
#else
    #include <soc/interconnect/ipct/nrfx_interconnect_ipct_haltium_ext.h>
#endif

NRFX_INTERCONNECT_IPCT_GLOBAL_DEFINE;
NRFX_INTERCONNECT_IPCT_LOCAL_DEFINE;
static const nrfx_interconnect_ipct_t m_local_ipct_interconnect[] =
                                      NRFX_INTERCONNECT_IPCT_LOCAL_IPCT_PROP;
static const nrfx_interconnect_ipct_t m_global_ipct_interconnect[] =
                                      NRFX_INTERCONNECT_IPCT_GLOBAL_IPCT_PROP;

nrf_domain_t nrfx_interconnect_ipct_domain_get(nrfx_interconnect_ipct_t const * p_ipct_prop)
{
    return (nrf_domain_t)nrf_address_domain_get((uint32_t)p_ipct_prop->p_ipct);
}

/* Set of macros to allow calculation of array index where given index is located.
 * We create a structure with fields for each instance which has non-zero
 * CHANNEL_MASK. Then offsetof is used to get the index of the given instance.
 */
#define _IPCT_STRUCT_ELEM(periph_name, prefix, inst_num, _) \
    NRFX_COND_CODE_1(NRFX_IS_EMPTY(inst_num), (), \
            (NRFX_COND_CODE_1(NRFX_IPCT_OWNED_MASK(inst_num), \
                              (uint8_t NRFX_CONCAT(dummy, inst_num);), ())))

#define IPCT_IDX_STRUCT() \
    struct ipct_idx_dummy_struct { \
        NRFX_FOREACH_PRESENT(IPCT, _IPCT_STRUCT_ELEM, (), (), _) \
    }

#define IPCT_IDX(inst_num) \
    NRFX_COND_CODE_1(NRFX_IPCT_OWNED_MASK(inst_num), \
            (offsetof(struct ipct_idx_dummy_struct, NRFX_CONCAT(dummy, inst_num))), \
            (-1))

IPCT_IDX_STRUCT();

static const int ipct_main_idx = IPCT_IDX(NRFX_INTERCONNECT_MAIN_IPCT_INSTANCE);

nrfx_interconnect_ipct_t const * nrfx_interconnect_ipct_main_get(void)
{
    return ipct_main_idx >= 0 ? &m_global_ipct_interconnect[ipct_main_idx] : NULL;
}

nrfx_interconnect_ipct_t const * nrfx_interconnect_ipct_get(nrfx_interconnect_apb_t const * p_apb_prop)
{
    uint8_t bus = nrf_address_bus_get((uint32_t)p_apb_prop->p_dppi, p_apb_prop->size);

    if (nrfx_interconnect_apb_domain_get(p_apb_prop) == NRF_DOMAIN_GLOBAL)
    {
        for (uint8_t i = 0; i < NRFX_ARRAY_SIZE(m_global_ipct_interconnect); i++)
        {
            // Check if some IPCT is on the same bus
            if (nrf_address_bus_get((uint32_t)m_global_ipct_interconnect[i].p_ipct, p_apb_prop->size) == bus)
            {
                return &m_global_ipct_interconnect[i];
            }
        }
        // The power domain connected to `bus` doesn't contain its own IPCT peripheral
        for (uint8_t i = 0; i < nrfx_interconnect_apb_global_num_of_get(); i++)
        {
            // Check whether it is possible to connect via DPPI
            if (nrf_apb_interconnect_by_idx_global_get(i)->p_dppi == p_apb_prop->p_dppi)
            {
                return nrfx_interconnect_ipct_main_get();
            }
        }
    }
    else
    {
        return &m_local_ipct_interconnect[0];
    }
    return NULL;
}

size_t nrfx_interconnect_ipct_global_num_of_get(void)
{
    return NRFX_ARRAY_SIZE(m_global_ipct_interconnect);
}

nrfx_interconnect_ipct_t const * nrfx_interconnect_ipct_global_by_idx_get(uint8_t idx)
{
    return idx < NRFX_ARRAY_SIZE(m_global_ipct_interconnect) ? &m_global_ipct_interconnect[idx] : NULL;
}

#endif // defined(HALTIUM_XXAA)
