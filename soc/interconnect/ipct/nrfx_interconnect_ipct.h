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

#ifndef NRFX_INTERCONNECT_IPCT_H__
#define NRFX_INTERCONNECT_IPCT_H__

#include <nrfx.h>
#include <hal/nrf_ipct.h>
#include <soc/interconnect/apb/nrfx_interconnect_apb.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_interconnect_ipct IPCT support
 * @{
 * @ingroup nrfx
 * @brief   Support for IPCT interconnection services.
 */

/** @brief IPCT properties structure. */
typedef struct {
    NRF_IPCT_Type * p_ipct;                 ///< Pointer to IPCT instance
    nrfx_atomic_t * p_ipct_channels;        ///< Pointer to the mask of available IPCT channels.
    uint32_t        ipct_pub_channels_mask; ///< Mask of configurable IPCT publish channels.
    uint32_t        ipct_sub_channels_mask; ///< Mask of configurable IPCT subscribe channels.
} nrfx_interconnect_ipct_t;

/**
 * @brief Function for getting the domain to which the specified IPCT belongs.
 *
 * @param[in] p_ipct_prop Pointer to IPCT properties structure.
 *
 * @return Domain that includes the specified IPCT.
 */
nrf_domain_t nrfx_interconnect_ipct_domain_get(nrfx_interconnect_ipct_t const * p_ipct_prop);

/**
 * @brief Function for getting IPCT properties structure by pointer to APB bus properties
 *        structure to which this particular IPCT belongs.
 *
 * @param[in] p_apb_prop Pointer to APB properties structure.
 *
 * @return Pointer to the IPCT properties structure that belongs to the specified APB bus.
 */
nrfx_interconnect_ipct_t const * nrfx_interconnect_ipct_get(nrfx_interconnect_apb_t const * p_apb_prop);

/**
 * @brief Function for getting number entries for global domain in IPCT peripheral properties
 *        array.
 *
 * @return Number of entries in global IPCT peripheral properties array.
 */
size_t nrfx_interconnect_ipct_global_num_of_get(void);

/**
 * @brief Function for getting IPCT properties structure by index of IPCT peripheral properties
 *        array.
 *
 * @param[in] idx Index of entry in IPCT peripheral properties array.
 *
 * @return Pointer to the properties structure that represents IPCT assigned to given index.
 */
nrfx_interconnect_ipct_t const * nrfx_interconnect_ipct_global_by_idx_get(uint8_t idx);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_INTERCONNECT_IPCT_H__
