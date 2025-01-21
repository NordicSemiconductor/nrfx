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

#if defined(HALTIUM_XXAA)

#include <soc/interconnect/apb/nrfx_interconnect_apb_haltium_global.h>
#include <soc/interconnect/apb/nrfx_interconnect_apb.h>

#if defined(NRF_RADIOCORE)
    #include <soc/interconnect/apb/nrfx_interconnect_apb_haltium_radiocore.h>
#elif defined(NRF_APPLICATION)
    #include <soc/interconnect/apb/nrfx_interconnect_apb_haltium_application.h>
#elif defined(NRF_PPR)
    #include <soc/interconnect/apb/nrfx_interconnect_apb_haltium_ppr.h>
#elif defined(NRF_FLPR)
    #include <soc/interconnect/apb/nrfx_interconnect_apb_haltium_flpr.h>
#else
    #include <soc/interconnect/apb/nrfx_interconnect_apb_haltium_ext.h>
#endif

NRFX_INTERCONNECT_APB_GLOBAL_DPPI_DEFINE;
NRFX_INTERCONNECT_APB_LOCAL_DPPI_DEFINE;
static const nrfx_interconnect_apb_t m_local_apb_interconnect[] =
                                     NRFX_INTERCONNECT_APB_LOCAL_BUSES_PROP;
static const nrfx_interconnect_apb_t m_global_apb_interconnect[] =
                                     NRFX_INTERCONNECT_APB_GLOBAL_BUSES_PROP;

nrf_domain_t nrfx_interconnect_apb_domain_get(nrfx_interconnect_apb_t const * p_apb_prop)
{
    return (nrf_domain_t)nrf_address_domain_get((uint32_t)p_apb_prop->p_dppi);
}

/* Set of macros to allow calculation of array index where given index is located.
 * We create a structure with fields for each instance which has non-zero
 * CHANNEL_MASK. Then offsetof is used to get the index of the given instance.
 */
#define _APB_STRUCT_ELEM(periph_name, prefix, inst_num, _) \
    NRFX_COND_CODE_0(prefix, (), \
            (NRFX_COND_CODE_1(NRFX_DPPI_OWNED_MASK(inst_num), \
                              (uint8_t NRFX_CONCAT(dummy, inst_num);), ())))

#define APB_IDX_STRUCT() \
    struct apb_idx_dummy_struct { \
        NRFX_FOREACH_PRESENT(DPPIC, _APB_STRUCT_ELEM, (), (), _) \
    }

#define APB_IDX(inst_num) \
    NRFX_COND_CODE_1(NRFX_DPPI_OWNED_MASK(inst_num), \
            (offsetof(struct apb_idx_dummy_struct, NRFX_CONCAT(dummy, inst_num))), \
            (-1))

APB_IDX_STRUCT();

static const int apb_main_idx = APB_IDX(NRFX_INTERCONNECT_MAIN_DPPI_INSTANCE);

nrfx_interconnect_apb_t const * nrfx_interconnect_apb_main_get(void)
{
    return apb_main_idx >= 0 ? &m_global_apb_interconnect[apb_main_idx] : NULL;
}

nrfx_interconnect_apb_t const * nrfx_interconnect_apb_get(uint32_t addr)
{
    nrf_domain_t domain = (nrf_domain_t)nrf_address_domain_get(addr);
    uint32_t num_of_entries;
    nrfx_interconnect_apb_t const * apb_interconnect;
    if (domain == NRF_DOMAIN_GLOBAL)
    {
        num_of_entries = NRFX_ARRAY_SIZE(m_global_apb_interconnect);
        apb_interconnect = m_global_apb_interconnect;
    }
    else
    {
        num_of_entries = NRFX_ARRAY_SIZE(m_local_apb_interconnect);
        apb_interconnect = m_local_apb_interconnect;
    }

    for (uint8_t i = 0; i < num_of_entries; i++)
    {
        nrfx_interconnect_apb_t const * p_apb = &apb_interconnect[i];
        uint8_t bus_address_area = nrf_address_bus_get(addr, p_apb->size);

        if (bus_address_area == nrf_address_bus_get((uint32_t)p_apb->p_dppi, p_apb->size))
        {
            return p_apb;
        }
    }
    return NULL;
}

size_t nrfx_interconnect_apb_global_num_of_get(void)
{
    return NRFX_ARRAY_SIZE(m_global_apb_interconnect);
}

nrfx_interconnect_apb_t const * nrf_apb_interconnect_by_idx_global_get(uint8_t idx)
{
    return idx < NRFX_ARRAY_SIZE(m_global_apb_interconnect) ? &m_global_apb_interconnect[idx] : NULL;
}

#endif // defined(HALTIUM_XXAA)
