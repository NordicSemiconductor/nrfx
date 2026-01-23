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

#include <helpers/nrfx_gppi.h>
#include <helpers/nrfx_gppi_routes.h>
#include "nrfx_gppi_lumos.h"

/* Available channels for each node. */
static nrfx_atomic_t channels[NRFX_GPPI_NODE_COUNT];
static nrfx_atomic_t group_channels[NRFX_GPPI_NODE_DPPI_COUNT];

/* All nodes in the system. */
static const nrfx_gppi_node_t nodes[] = {
	NRFX_GPPI_DPPI_NODE_DEFINE(00, NRFX_GPPI_DOMAIN_MCU),
	NRFX_GPPI_DPPI_NODE_DEFINE(10, NRFX_GPPI_DOMAIN_RAD),
	NRFX_GPPI_DPPI_NODE_DEFINE(20, NRFX_GPPI_DOMAIN_PERI),
	NRFX_GPPI_DPPI_NODE_DEFINE(30, NRFX_GPPI_DOMAIN_LP),
	NRFX_GPPI_PPIB_NODE_DEFINE(00,10),
	NRFX_GPPI_PPIB_NODE_DEFINE(11,21),
	NRFX_GPPI_PPIB_NODE_DEFINE(01,20),
	NRFX_GPPI_PPIB_NODE_DEFINE(22,30),
};

/* All routes in the system. */
static const nrfx_gppi_route_t dppi_routes[] = {
	NRFX_GPPI_ROUTE_DEFINE("mcu", (&nodes[NRFX_GPPI_NODE_DPPIC00])),
	NRFX_GPPI_ROUTE_DEFINE("rad", (&nodes[NRFX_GPPI_NODE_DPPIC10])),
	NRFX_GPPI_ROUTE_DEFINE("peri", (&nodes[NRFX_GPPI_NODE_DPPIC20])),
	NRFX_GPPI_ROUTE_DEFINE("lp", (&nodes[NRFX_GPPI_NODE_DPPIC30])),
	NRFX_GPPI_ROUTE_DEFINE("mcu_rad",
			(&nodes[NRFX_GPPI_NODE_DPPIC00],
			 &nodes[NRFX_GPPI_NODE_PPIB00_10],
			 &nodes[NRFX_GPPI_NODE_DPPIC10])),
	NRFX_GPPI_ROUTE_DEFINE("mcu_peri",
			(&nodes[NRFX_GPPI_NODE_DPPIC00],
			 &nodes[NRFX_GPPI_NODE_PPIB01_20],
			 &nodes[NRFX_GPPI_NODE_DPPIC20])),
	NRFX_GPPI_ROUTE_DEFINE("mcu_lp",
			(&nodes[NRFX_GPPI_NODE_DPPIC00],
			 &nodes[NRFX_GPPI_NODE_PPIB01_20],
			 &nodes[NRFX_GPPI_NODE_DPPIC20],
			 &nodes[NRFX_GPPI_NODE_PPIB22_30],
			 &nodes[NRFX_GPPI_NODE_DPPIC30])),
	NRFX_GPPI_ROUTE_DEFINE("rad_peri",
			(&nodes[NRFX_GPPI_NODE_DPPIC10],
			 &nodes[NRFX_GPPI_NODE_PPIB11_21],
			 &nodes[NRFX_GPPI_NODE_DPPIC20])),
	NRFX_GPPI_ROUTE_DEFINE("rad_lp",
			(&nodes[NRFX_GPPI_NODE_DPPIC10],
			 &nodes[NRFX_GPPI_NODE_PPIB11_21],
			 &nodes[NRFX_GPPI_NODE_DPPIC20],
			 &nodes[NRFX_GPPI_NODE_PPIB22_30],
			 &nodes[NRFX_GPPI_NODE_DPPIC30])),
	NRFX_GPPI_ROUTE_DEFINE("peri_lp",
			(&nodes[NRFX_GPPI_NODE_DPPIC20],
			 &nodes[NRFX_GPPI_NODE_PPIB22_30],
			 &nodes[NRFX_GPPI_NODE_DPPIC30])),
};

/* Helper arrays to find route based on source and destination domain ID.
 * Since domain index starts from 1 everything is shifted by 1 to save
 * space in arrays.
 */
static const nrfx_gppi_route_t *mcu_routes[] = {
	&dppi_routes[0], &dppi_routes[4], &dppi_routes[5], &dppi_routes[6]
};

static const nrfx_gppi_route_t *rad_routes[] = {
	&dppi_routes[1], &dppi_routes[7], &dppi_routes[8]
};

static const nrfx_gppi_route_t *peri_routes[] = {
	&dppi_routes[2] , &dppi_routes[9]
};

static const nrfx_gppi_route_t *lp_routes[] = {
	&dppi_routes[3]
};

static const nrfx_gppi_route_t **dppi_route_map[] = {
	mcu_routes, rad_routes, peri_routes, lp_routes
};

__WEAK uint32_t nrfx_gppi_domain_id_get(uint32_t addr)
{
	return ((addr >> 18) & 0x7) - 1;
}

const nrfx_gppi_route_t ***nrfx_gppi_route_map_get(void)
{
    return dppi_route_map;
}

const nrfx_gppi_route_t *nrfx_gppi_routes_get(void)
{
    return dppi_routes;
}

const nrfx_gppi_node_t *nrfx_gppi_nodes_get(void)
{
    return nodes;
}

void nrfx_gppi_channel_init(nrfx_gppi_node_id_t node_id, uint32_t ch_mask)
{
    NRFX_ASSERT(node_id < NRFX_GPPI_NODE_COUNT);

    *nodes[node_id].generic.p_channels = ch_mask;
}

void nrfx_gppi_groups_init(nrfx_gppi_node_id_t node_id, uint32_t group_mask)
{
    NRFX_ASSERT(node_id < NRFX_GPPI_NODE_DPPI_COUNT);

    *nodes[node_id].dppi.p_group_channels = group_mask;
}
