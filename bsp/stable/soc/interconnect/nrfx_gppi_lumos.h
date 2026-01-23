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

#ifndef NRFX_GPPI_LUMOS_H__
#define NRFX_GPPI_LUMOS_H__

#include <helpers/nrfx_gppi_routes.h>

/**
 * @defgroup nrfx_gppi_lumos GPPI nRF54L Series and nRF71 Series specific structures
 * @{
 * @ingroup nrfx_gppi
 *
 * @brief Structures for SoC specific GPPI API.
 */

/** @brief DPPI domain ID. */
typedef enum {
	NRFX_GPPI_DOMAIN_MCU  = 0, ///< MCU domain.
	NRFX_GPPI_DOMAIN_RAD  = 1, ///< Radio domain.
	NRFX_GPPI_DOMAIN_PERI = 2, ///< Peripheral domain.
	NRFX_GPPI_DOMAIN_LP   = 3, ///< Low power domain.
} nrfx_gppi_domain_id_t;

/** @brief Identification of a DPPI node in the system. */
typedef enum {
	NRFX_GPPI_NODE_DPPIC00,                               ///< DPPIC00 node
	NRFX_GPPI_NODE_DPPIC10,                               ///< DPPIC10 node
	NRFX_GPPI_NODE_DPPIC20,                               ///< DPPIC20 node
	NRFX_GPPI_NODE_DPPIC30,                               ///< DPPIC30 node
    NRFX_GPPI_NODE_DPPI_COUNT,                            ///< Number of DPPI nodes in the system.
	NRFX_GPPI_NODE_PPIB00_10 = NRFX_GPPI_NODE_DPPI_COUNT, ///< PPIB00-PPIB10 bridge node
	NRFX_GPPI_NODE_PPIB11_21,                             ///< PPIB11-PPIB21 bridge node
	NRFX_GPPI_NODE_PPIB01_20,                             ///< PPIB01-PPIB20 bridge node
	NRFX_GPPI_NODE_PPIB22_30,                             ///< PPIB22-PPIB30 bridge node
    NRFX_GPPI_NODE_COUNT                                  ///< Number of nodes in the system.
} nrfx_gppi_node_id_t;

/** @cond Driver internal data. */
const nrfx_gppi_route_t ***nrfx_gppi_route_map_get(void);
const nrfx_gppi_route_t *nrfx_gppi_routes_get(void);
const nrfx_gppi_node_t *nrfx_gppi_nodes_get(void);
void nrfx_gppi_channel_init(nrfx_gppi_node_id_t node_id, uint32_t ch_mask);
void nrfx_gppi_groups_init(nrfx_gppi_node_id_t node_id, uint32_t group_mask);
/** @endcond */

/** @} */

#endif // NRFX_GPPI_LUMOS_H__
