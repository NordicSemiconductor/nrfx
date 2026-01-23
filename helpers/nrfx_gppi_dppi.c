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

#include <nrfx.h>

#ifdef DPPIC_PRESENT
#include <hal/nrf_dppi.h>
#include <helpers/nrfx_gppi.h>
#include <helpers/nrfx_flag32_allocator.h>

#define NRFX_LOG_MODULE GPPI
#include <nrfx_log.h>

#if NRFX_CHECK(NRFX_GPPI_FIXED_CONNECTIONS)

#define NRFX_DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

/* Due to fixed connections between DPPIC instances only one channel is used per connection. */
#define DPPI_CH_MAX_CNT 1

#define DPPI_EXT_OFF 0
#define DPPI_EXT_BITS 1

#define DPPI_INST_OFF (DPPI_EXT_OFF + DPPI_EXT_BITS)
#define DPPI_SINGLE_INST_BITS 5
#define DPPI_INST_MAX_CNT 3
#define DPPI_INST_BITS (DPPI_SINGLE_INST_BITS * DPPI_INST_MAX_CNT)

#define DPPI_INST_CNT_OFF (DPPI_INST_OFF + DPPI_INST_BITS)
#define DPPI_INST_CNT_BITS 3

#define DPPI_CH_OFF (DPPI_INST_CNT_OFF + DPPI_INST_CNT_BITS)
#define DPPI_CH_BITS 5
#define DPPI_CH_RESERVED NRFX_BIT_MASK(DPPI_CH_BITS)

#define DPPI_REV_OFF (DPPI_CH_OFF + DPPI_CH_BITS)
#define DPPI_REV_BITS 1

#define DPPI_ROUTE_OFF (DPPI_REV_OFF + DPPI_REV_BITS)
#define DPPI_ROUTE_BITS 6

#define DPPI_RESERVED_BITS 1

#define DPPI_TOTAL_BITS (DPPI_RESERVED_BITS + DPPI_ROUTE_BITS + DPPI_REV_BITS + \
                         DPPI_CH_BITS + DPPI_INST_CNT_BITS + DPPI_INST_BITS + DPPI_EXT_BITS)
NRFX_STATIC_ASSERT(DPPI_TOTAL_BITS == 32);

/* Not used. */
#define HANDLE_CHAN(_i, _chan) 0

/* Extract channel from handle. */
#define HANDLE_GET_CHAN(_handle, _i) (((_handle) >> DPPI_CH_OFF) & NRFX_BIT_MASK(DPPI_CH_BITS))

/* Extract number of DPPI instances in the connection. */
#define HANDLE_GET_DPPI_CNT(_handle) \
    (((_handle) >> DPPI_INST_CNT_OFF) & NRFX_BIT_MASK(DPPI_INST_CNT_BITS))

/* Extract nth DPPIC instance ID. */
#define HANDLE_GET_DPPI_ID(_handle, _i)                        \
    (((_handle) >> (DPPI_SINGLE_INST_BITS * (_i) + DPPI_INST_OFF)) &    \
     NRFX_BIT_MASK(DPPI_SINGLE_INST_BITS))

#define HANDLE_INST(_i, _id) ((_id) << (DPPI_SINGLE_INST_BITS * (_i) + DPPI_INST_OFF))

/* Extract route ID from handle. */
#define HANDLE_GET_ROUTE_ID(handle) (((handle) >> DPPI_ROUTE_OFF) & NRFX_BIT_MASK(DPPI_ROUTE_BITS))

/* Determine if handle has reversed route. */
#define HANDLE_IS_REVERSED(handle) (handle & NRFX_BIT(DPPI_REV_OFF))

#define HANDLE_INIT(route_id, rev, dppi_cnt, fixed_ch)                \
    ((route_id) << DPPI_ROUTE_OFF) |                    \
    ((rev) ? NRFX_BIT(DPPI_REV_OFF) : 0) |                        \
    ((dppi_cnt) << DPPI_INST_CNT_OFF) |                    \
    ((fixed_ch) << DPPI_CH_OFF) | NRFX_BIT(DPPI_EXT_OFF)

#else /* NRFX_GPPI_FIXED_CONNECTIONS */

#define DPPI_CH_MAX_CNT 5
#define DPPI_CH_OFF 0
#define DPPI_CH_BITS 5
#define DPPI_CH_RESERVED NRFX_BIT_MASK(DPPI_CH_BITS)

#define DPPI_REV_OFF (DPPI_CH_OFF + DPPI_CH_MAX_CNT * DPPI_CH_BITS)
#define DPPI_REV_BITS 1

#define DPPI_ROUTE_OFF (DPPI_REV_OFF + DPPI_REV_BITS)
#define DPPI_ROUTE_BITS 6

#define DPPI_TOTAL_BITS (DPPI_CH_MAX_CNT * DPPI_CH_BITS + DPPI_REV_BITS + DPPI_ROUTE_BITS)
NRFX_STATIC_ASSERT(DPPI_TOTAL_BITS == 32);

/* Extract channel from handle. */
#define HANDLE_GET_CHAN(_handle, _i) (((_handle) >> ((_i) * DPPI_CH_BITS)) & NRFX_BIT_MASK(DPPI_CH_BITS))

/* Extract route ID from handle. */
#define HANDLE_GET_ROUTE_ID(handle) (((handle) >> DPPI_ROUTE_OFF) & NRFX_BIT_MASK(DPPI_ROUTE_BITS))

/* Determine if handle has reversed route. */
#define HANDLE_IS_REVERSED(handle) ((handle) & NRFX_BIT(DPPI_REV_OFF))

#define HANDLE_CHAN(i, ch) ((ch) << (DPPI_CH_BITS * (i) + DPPI_CH_OFF))
/* Not used. */
#define HANDLE_INST(i, id) 0

#define HANDLE_INIT(route_id, rev, dppi_cnt, fixed_ch)                \
    ((route_id) << DPPI_ROUTE_OFF) | (rev ? NRFX_BIT(DPPI_REV_OFF) : 0)

#endif /* NRFX_GPPI_FIXED_CONNECTIONS */

#define GHANDLE_CHAN_OFF 0
#define GHANDLE_CHAN_BITS 8
#define GHANDLE_DOMAIN_OFF 8
#define GHANDLE_DOMAIN_BITS 8
#define GHANDLE_GET_CHAN(handle) ((handle >> GHANDLE_CHAN_OFF) & NRFX_BIT_MASK(GHANDLE_CHAN_BITS))
#define GHANDLE_GET_DOMAIN(handle) ((handle >> GHANDLE_DOMAIN_OFF) & NRFX_BIT_MASK(GHANDLE_DOMAIN_BITS))

/* Handle has different layout for Lumos and Haltium. Lumos has different channels
 * on each node (DPPIC/PPIB) and Haltium has allocation through SDFW.
 *
 * Lumos:
 *
 * --------------------------------------------------------------------------
 * | Route ID 6b | Reversed 1b | ch4 5b | ch3 5b | ch2 5b | ch1 5b | ch0 5b |
 * --------------------------------------------------------------------------
 *
 *  Haltium:
 *  There is fixed channel but local domain has no access to routes so need to
 *  know which DPPIC instances belong to route (max 3).
 *
 * ---------------------------------------------------------------------------------------------
 * | Route ID 6b | Reversed 1b | ch 6b | dppi_cnt 3b | dppi2 5b | dppi1 5b | ddpi0 5b | ext 1b |
 * ---------------------------------------------------------------------------------------------
 *
 * sec bit indicates whether handle is managed by SDFW
 */

static nrfx_gppi_t * p_gppi;

void nrfx_gppi_init(nrfx_gppi_t * p_instance)
{
    p_gppi = p_instance;
}

static void flag_free(nrfx_atomic_t *p_mask, uint8_t flag)
{
    int err;

    err = nrfx_flag32_free(p_mask, flag);
    (void)err;
    NRFX_ASSERT(err == 0);
}

#if defined(NRFX_GPPI_MULTI_DOMAIN)

static uint32_t alloc_bit_locked(nrfx_atomic_t *mask)
{
    uint32_t rv = 31 - NRFX_CLZ(*mask);

    *mask &= ~NRFX_BIT(rv);

    return rv;
}

uint32_t nrfx_gppi_group_domain_id_get(nrfx_gppi_group_handle_t handle)
{
    return GHANDLE_GET_DOMAIN(handle);
}

static int alloc_channels(uint8_t * p_channels, const nrfx_gppi_route_t * p_route,
                          nrfx_gppi_resource_t * p_resource)
{
    int rv = 0;

    NRFX_CRITICAL_SECTION_ENTER();

    if (NRFX_IS_ENABLED(NRFX_GPPI_FIXED_CONNECTIONS))
    {
        /* In Haltium connections between channels in DPPI and PPIB are fixed which
         * means that to correctly allocate a route same channel must be available
         * in all nodes.
         */
        uint32_t mask = UINT32_MAX;
        uint32_t ch;

        for (size_t i = 0; i < p_route->len; i++)
        {
            mask &= *p_route->p_nodes[i]->generic.p_channels;
        }

        if (!mask)
        {
            rv = -ENOMEM;
            goto unlock;
        }

        ch = 31 - NRFX_CLZ(mask);
        for (size_t i = 0; i < p_route->len; i++)
        {
            *p_route->p_nodes[i]->generic.p_channels &= ~NRFX_BIT(ch);
        }

        p_channels[0] = (uint8_t)ch;
    }
    else
    {
        /* Lumus support flexible setup so any channel can be allocated in each node. */
        for (size_t i = 0; i < p_route->len; i++)
        {
            const nrfx_gppi_node_t * p_node = p_route->p_nodes[i];
            bool check = (p_resource == NULL) || (p_resource->domain_id != p_node->domain_id);

            if (check && (*p_node->generic.p_channels == 0))
            {
                rv = -ENOMEM;
                goto unlock;
            }
        }
        for (size_t i = 0; i < p_route->len; i++)
        {
            const nrfx_gppi_node_t * p_node = p_route->p_nodes[i];
            bool do_alloc = (p_resource == NULL) || (p_resource->domain_id != p_node->domain_id);

            if (do_alloc)
            {
                p_channels[i] = (uint8_t)alloc_bit_locked(p_node->generic.p_channels);
            }
            else
            {
                p_channels[i] = p_resource->channel;
            }
        }
    }
unlock:

    NRFX_CRITICAL_SECTION_EXIT();

    return rv;
}

static inline uint32_t get_ppi_ch(bool pub, uint8_t * p_channels, size_t i, bool rev)
{
    if (NRFX_IS_ENABLED(NRFX_GPPI_FIXED_CONNECTIONS)) {
        return p_channels[0];
    }

    if (pub) {
        return rev ? p_channels[i - 1] : p_channels[i + 1];
    }
    return rev ? p_channels[i + 1] : p_channels[i - 1];
}

static inline uint32_t get_ppib_ch(bool pub, uint32_t channel,
                                   const nrfx_gppi_node_t * p_node, bool rev)
{
#if NRFX_CHECK(NRFX_GPPI_FIXED_CONNECTIONS)
    return channel + p_node->ch_off[(pub ^ rev) ? 1 : 0];
#else
    (void)pub;
    (void)p_node;
    (void)rev;
    return channel;
#endif
}

static int nrf_ppib_write(volatile uint32_t * p_addr, uint32_t val)
{
#if NRFX_CHECK(NRFX_GPPI_CONFIG_DPPI_PPIB_EXT_FUNC)
	return nrfx_gppi_ext_ppib_write(p_addr, val);
#else
    *p_addr = val;
    return 0;
#endif
}

#define EP_ENABLE(ch) ((ch) | NRF_SUBSCRIBE_PUBLISH_ENABLE)

static int ppib_configure(const nrfx_gppi_node_t * p_node, uint32_t ppib_ch,
                          uint32_t sub_val, uint32_t pub_val, bool rev)
{
    NRF_PPIB_Type * p_pub_reg = p_node->ppib.p_reg[rev ? 0 : 1];
    NRF_PPIB_Type * p_sub_reg = p_node->ppib.p_reg[rev ? 1 : 0];
    uint32_t sub_ppib_ch = get_ppib_ch(false, ppib_ch, p_node, rev);
    uint32_t pub_ppib_ch = get_ppib_ch(true, ppib_ch, p_node, rev);
    int rv;

    rv = nrf_ppib_write(&p_sub_reg->SUBSCRIBE_SEND[sub_ppib_ch], sub_val);
    if (rv != 0) {
        return rv;
    }

    rv = nrf_ppib_write(&p_pub_reg->PUBLISH_RECEIVE[pub_ppib_ch], pub_val);
    if (rv != 0) {
        return rv;
    }

    NRFX_LOG_INFO("Setup connection subscribe PPIB(%p) ch %d to DPPI ch:%d, "
            "publish PPIB(%p) ch:%d to DPPI ch:%d",
            p_sub_reg, sub_ppib_ch, sub_val, p_pub_reg, pub_ppib_ch, pub_val);

    return 0;
}

int nrfx_gppi_channel_alloc(uint32_t node_id)
{
    NRFX_ASSERT(p_gppi != NULL);
    const nrfx_gppi_node_t * p_node = &p_gppi->nodes[node_id];

    return nrfx_flag32_alloc(p_node->generic.p_channels);
}

void nrfx_gppi_channel_free(uint32_t node_id, uint8_t channel)
{
    const nrfx_gppi_node_t * p_node = &p_gppi->nodes[node_id];

    flag_free(p_node->generic.p_channels, (uint8_t)channel);
}

int nrfx_gppi_group_channel_alloc(uint32_t node_id)
{
    NRFX_ASSERT(p_gppi != NULL);
    const nrfx_gppi_node_t * p_node = &p_gppi->nodes[node_id];

    return nrfx_flag32_alloc(p_node->dppi.p_group_channels);
}

void nrfx_gppi_group_channel_free(uint32_t node_id, uint8_t channel)
{
    const nrfx_gppi_node_t * p_node = &p_gppi->nodes[node_id];

    flag_free(p_node->dppi.p_group_channels, (uint8_t)channel);
}

int nrfx_gppi_ext_conn_alloc(uint32_t producer, uint32_t consumer, nrfx_gppi_handle_t * p_handle,
                             nrfx_gppi_resource_t * p_resource)
{
    NRFX_ASSERT(p_gppi != NULL);
    uint8_t channels[DPPI_CH_MAX_CNT];
    const nrfx_gppi_route_t * p_route;
    uint32_t h;
    int rv = 0;
    uint8_t route_idx;
    bool rev_conn;

    if (producer > consumer) {
        rev_conn = true;
        p_route = p_gppi->route_map[consumer][producer - consumer];
    } else {
        rev_conn = false;
        p_route = p_gppi->route_map[producer][consumer - producer];
    }

    /* Return is allocation failed. */
    rv = alloc_channels(channels, p_route, p_resource);
    if (rv < 0) {
        return rv;
    }

    route_idx = (uint8_t)(((uintptr_t)p_route - (uintptr_t)p_gppi->routes) /
                sizeof(nrfx_gppi_route_t));
    NRFX_LOG_INFO("alloc, source domain:%d destination domain:%d, (idx: %d len:%d) %s",
        producer, consumer, route_idx, p_route->len, rev_conn ? "reversed" : "");

    h = HANDLE_INIT(route_idx, rev_conn, (uint32_t)NRFX_DIV_ROUND_UP(p_route->len, 2), channels[0]);
    for (size_t i = 0; i < p_route->len; i++) {
        const nrfx_gppi_node_t * p_node = p_route->p_nodes[i];

        if (p_node->type == NRFX_GPPI_NODE_PPIB) {
            uint32_t ch = channels[NRFX_IS_ENABLED(NRFX_GPPI_FIXED_CONNECTIONS) ? 0 : i];
            uint32_t sub_ch;
            uint32_t pub_ch;
            bool rev;

            if (NRFX_IS_ENABLED(NRFX_GPPI_FIXED_CONNECTIONS))
            {
                rev = (p_route->p_nodes[i - 1]->domain_id < p_route->p_nodes[i + 1]->domain_id) ?
                        rev_conn : !rev_conn;
            }
            else
            {
                rev = rev_conn;
            }

            sub_ch = get_ppi_ch(false, channels, i, rev);
            pub_ch = get_ppi_ch(true, channels, i, rev);
            rv = ppib_configure(p_node, ch, EP_ENABLE(sub_ch), EP_ENABLE(pub_ch), rev);
            if (rv != 0) {
                return rv;
            }
        } else if (NRFX_IS_ENABLED(NRFX_GPPI_FIXED_CONNECTIONS)) {
            h |= HANDLE_INST(i / 2, p_node->domain_id);
        }

        if (!NRFX_IS_ENABLED(NRFX_GPPI_FIXED_CONNECTIONS)) {
            if ((p_resource == NULL) || (p_node->domain_id != p_resource->domain_id))
            {
                h |= HANDLE_CHAN(i, channels[i]);
            }
            else if (p_node->domain_id == p_resource->domain_id)
            {
                h |= HANDLE_CHAN(i, DPPI_CH_RESERVED);
            }
        }
    }

    *p_handle = h;
    NRFX_LOG_INFO("Alloc done, handle:%08x route:%d", h, route_idx);

    return 0;
}

int nrfx_gppi_domain_conn_alloc(uint32_t producer, uint32_t consumer,
                                nrfx_gppi_handle_t * p_handle)
{
    return nrfx_gppi_ext_conn_alloc(producer, consumer, p_handle, NULL);
}

void nrfx_gppi_domain_conn_free(nrfx_gppi_handle_t handle)
{
    uint32_t route_id = HANDLE_GET_ROUTE_ID(handle);
    const nrfx_gppi_route_t * p_route = &p_gppi->routes[route_id];
    bool rev_conn = HANDLE_IS_REVERSED(handle);

    NRFX_LOG_INFO("Freeing connection handle:%08x (route %d)", handle, route_id);
    for (size_t i = 0; i < p_route->len; i++) {
        uint32_t chan = HANDLE_GET_CHAN(handle, i);
        const nrfx_gppi_node_t *p_node = p_route->p_nodes[i];
        int rv;

        if (p_node->type == NRFX_GPPI_NODE_PPIB) {
            bool rev;

            if (NRFX_IS_ENABLED(NRFX_GPPI_FIXED_CONNECTIONS))
            {
                rev = (p_route->p_nodes[i - 1]->domain_id < p_route->p_nodes[i + 1]->domain_id) ?
                        rev_conn : !rev_conn;
            }
            else
            {
                rev = rev_conn;
            }

            rv = ppib_configure(p_node, chan, 0, 0, rev);
            (void)rv;
            NRFX_ASSERT(rv == 0);
        }
        if (chan != DPPI_CH_RESERVED)
        {
            flag_free(p_node->generic.p_channels, (uint8_t)chan);
        }
    }
}
#else
int nrfx_gppi_domain_conn_alloc(uint32_t producer, uint32_t consumer, nrfx_gppi_handle_t *p_handle)
{
    NRFX_ASSERT(p_gppi != NULL);
    (void)producer;
    (void)consumer;
    int chan;

    chan = nrfx_flag32_alloc(&p_gppi->ch_mask);
    if (chan < 0) {
        return chan;
    }

    *p_handle = (nrfx_gppi_handle_t)chan;
    return 0;
}

void nrfx_gppi_domain_conn_free(nrfx_gppi_handle_t handle)
{
    flag_free(&p_gppi->ch_mask, (uint8_t)handle);
}

int nrfx_gppi_channel_alloc(uint32_t node_id)
{
    NRFX_ASSERT(p_gppi != NULL);
    (void)node_id;
    return nrfx_flag32_alloc(&p_gppi->ch_mask);
}

void nrfx_gppi_channel_free(uint32_t node_id, uint8_t channel)
{
    (void)node_id;
    flag_free(&p_gppi->ch_mask, (uint8_t)channel);
}

int nrfx_gppi_group_channel_alloc(uint32_t node_id)
{
    NRFX_ASSERT(p_gppi != NULL);
    (void)node_id;
    return nrfx_flag32_alloc(&p_gppi->group_mask);
}

void nrfx_gppi_group_channel_free(uint32_t node_id, uint8_t channel)
{
    (void)node_id;
    flag_free(&p_gppi->group_mask, (uint8_t)channel);
}

#endif /* NRFX_GPPI_MULTI_DOMAIN  */

int nrfx_gppi_conn_alloc(uint32_t eep, uint32_t tep, nrfx_gppi_handle_t * p_handle)
{
    int rv;

    if ((nrfx_gppi_ep_channel_get(eep) >= 0) || (nrfx_gppi_ep_channel_get(tep) >= 0))
    {
        return -EINVAL;
    }

    rv = nrfx_gppi_domain_conn_alloc(nrfx_gppi_domain_id_get(eep),
                                     nrfx_gppi_domain_id_get(tep), p_handle);
    if (rv != 0)
    {
        return rv;
    }

    nrfx_gppi_ep_attach(eep, *p_handle);
    nrfx_gppi_ep_attach(tep, *p_handle);

    return 0;
}

void nrfx_gppi_conn_free(uint32_t eep, uint32_t tep, nrfx_gppi_handle_t handle)
{
    nrfx_gppi_ep_clear(eep);
    nrfx_gppi_ep_clear(tep);
    nrfx_gppi_domain_conn_free(handle);
}

static NRF_DPPIC_Type *dppi_reg_get(uint32_t domain_id)
{
#if defined(NRFX_GPPI_MULTI_DOMAIN)
    return p_gppi->routes[domain_id].p_nodes[0]->dppi.p_reg;
#else
    (void)domain_id;
    return NRF_DPPIC;
#endif
}

static nrfx_atomic_t *get_group_chan_mask(uint32_t domain_id)
{
#if defined(NRFX_GPPI_MULTI_DOMAIN)
    return p_gppi->routes[domain_id].p_nodes[0]->dppi.p_group_channels;
#else
    (void)domain_id;
    return &p_gppi->group_mask;
#endif
}

#if NRFX_CHECK(NRFX_GPPI_FIXED_CONNECTIONS)
#define FOR_EACH_DPPI(_gppi, _handle, _ch, _reg, _d_id)                                     \
    uint32_t cnt = HANDLE_GET_DPPI_CNT(_handle);                                            \
    size_t i;                                                                               \
    _ch = HANDLE_GET_CHAN(_handle, 0);                                                      \
    for (i = 0, _d_id = HANDLE_GET_DPPI_ID(_handle, 0), _reg = dppi_reg_get(d_id);          \
         i < cnt;                                                                           \
         i++, _d_id = HANDLE_GET_DPPI_ID(_handle, i), _reg = dppi_reg_get(_d_id))
#elif defined(NRFX_GPPI_MULTI_DOMAIN)
#define FOR_EACH_DPPI(_gppi, _handle, _ch, _reg, _d_id)                                     \
    const nrfx_gppi_route_t *_route = &_gppi->routes[HANDLE_GET_ROUTE_ID(_handle)];         \
    size_t i;                                                                               \
	for (i = 0, _ch = HANDLE_GET_CHAN(_handle, i), _d_id = _route->p_nodes[i]->domain_id,	\
			_reg = _route->p_nodes[i]->dppi.p_reg;					                        \
	     i < _route->len;									                                \
	     i += 2, _ch = HANDLE_GET_CHAN(_handle, i), _d_id = _route->p_nodes[i]->domain_id,	\
	     _reg = _route->p_nodes[i]->dppi.p_reg) if (_ch != DPPI_CH_RESERVED)
#else
#define FOR_EACH_DPPI(_gppi, _handle, _ch, _reg, _d_id)                                     \
    _reg = NRF_DPPIC;                                                                       \
    _d_id = 0;                                                                              \
    (void)_d_id;                                                                            \
    (void)_reg;                                                                             \
    (void)_gppi;                                                                            \
    _ch = _handle;
#endif

void nrfx_gppi_conn_enable(nrfx_gppi_handle_t handle)
{
    NRF_DPPIC_Type * p_reg;
    uint32_t ch;
    uint32_t d_id;

    FOR_EACH_DPPI(p_gppi, handle, ch, p_reg, d_id)
    {
        (void)d_id;
        nrf_dppi_channels_enable(p_reg, NRFX_BIT(ch));
    }
}

void nrfx_gppi_conn_disable(nrfx_gppi_handle_t handle)
{
    NRF_DPPIC_Type * p_reg;
    uint32_t ch;
    uint32_t d_id;

    FOR_EACH_DPPI(p_gppi, handle, ch, p_reg, d_id)
    {
        (void)d_id;
        nrf_dppi_channels_disable(p_reg, NRFX_BIT(ch));
    }
}

bool nrfx_gppi_chan_is_enabled(uint32_t domain_id, uint32_t ch)
{
    return nrf_dppi_channel_check(dppi_reg_get(domain_id), (uint8_t)ch);
}

void nrfx_gppi_channels_enable(uint32_t domain_id, uint32_t ch_mask)
{
    nrf_dppi_channels_enable(dppi_reg_get(domain_id), ch_mask);
}

void nrfx_gppi_channels_disable(uint32_t domain_id, uint32_t ch_mask)
{
    nrf_dppi_channels_disable(dppi_reg_get(domain_id), ch_mask);
}

int nrfx_gppi_ep_to_ch_attach(uint32_t ep, uint8_t channel)
{
    NRF_DPPI_ENDPOINT_SETUP(ep, channel);
    return 0;
}

void nrfx_gppi_ep_clear(uint32_t ep)
{
    NRF_DPPI_ENDPOINT_CLEAR(ep);
}

void nrfx_gppi_ep_ch_clear(uint32_t ep, uint8_t channel)
{
    (void)channel;
    nrfx_gppi_ep_clear(ep);
}

void nrfx_gppi_ep_enable(uint32_t ep)
{
    NRF_DPPI_ENDPOINT_ENABLE(ep);
}

void nrfx_gppi_ep_disable(uint32_t ep)
{
    NRF_DPPI_ENDPOINT_DISABLE(ep);
}

int nrfx_gppi_domain_channel_get(nrfx_gppi_handle_t handle, uint32_t node_id)
{
#if NRFX_CHECK(NRFX_GPPI_FIXED_CONNECTIONS)
    (void)node_id;
    return HANDLE_GET_CHAN(handle, 0);
#elif !defined(NRFX_GPPI_MULTI_DOMAIN)
    (void)node_id;
    return handle;
#else
    const nrfx_gppi_route_t * p_route = &p_gppi->routes[HANDLE_GET_ROUTE_ID(handle)];

    for (size_t i = 0; i < p_route->len; i++)
    {
        if (p_route->p_nodes[i]->domain_id == node_id)
        {
            return HANDLE_GET_CHAN(handle, i);
        }
    }

    return -EINVAL;
#endif
}


int nrfx_gppi_ep_attach(uint32_t ep, nrfx_gppi_handle_t handle)
{
    int ch = nrfx_gppi_domain_channel_get(handle, nrfx_gppi_domain_id_get(ep));

    if (ch < 0) {
        return ch;
    }

    return nrfx_gppi_ep_to_ch_attach(ep, (uint8_t)ch);
}

int nrfx_gppi_group_alloc(uint32_t domain_id, nrfx_gppi_group_handle_t *handle)
{
    nrfx_atomic_t *group_mask = get_group_chan_mask(domain_id);
    int gch;

    gch = nrfx_flag32_alloc(group_mask);
    if (gch < 0) {
        return gch;
    }

    *handle = NRFX_IS_ENABLED(NRFX_GPPI_MULTI_DOMAIN) ?
        ((domain_id << 8) | (uint32_t)gch) : (uint8_t)gch;

    return 0;
}

void nrfx_gppi_group_free(nrfx_gppi_group_handle_t handle)
{
    uint32_t domain_id = GHANDLE_GET_DOMAIN(handle);
    uint8_t ch = GHANDLE_GET_CHAN(handle);
    nrfx_atomic_t *group_mask = get_group_chan_mask(domain_id);
    NRF_DPPIC_Type * p_reg = dppi_reg_get(domain_id);

    nrf_dppi_group_clear(p_reg, (nrf_dppi_channel_group_t)ch);
    flag_free(group_mask, ch);
}

void nrfx_gppi_group_ch_add(nrfx_gppi_group_handle_t handle, uint32_t ch)
{
    NRF_DPPIC_Type * p_reg = dppi_reg_get(GHANDLE_GET_DOMAIN(handle));
    nrf_dppi_channel_group_t group = (nrf_dppi_channel_group_t)GHANDLE_GET_CHAN(handle);
    nrf_dppi_task_t en_task = nrf_dppi_group_enable_task_get((uint8_t)group);
    nrf_dppi_task_t dis_task = nrf_dppi_group_disable_task_get((uint8_t)group);
    uint32_t en_reg = nrf_dppi_subscribe_get(p_reg, en_task);
    uint32_t dis_reg = nrf_dppi_subscribe_get(p_reg, dis_task);

    /* Writes to CHG are ignored if SUBSCRIBE registers for a given group are enabled. */
    if (en_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_clear(p_reg, en_task);
    }
    if (dis_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_clear(p_reg, dis_task);
    }

    nrf_dppi_channels_include_in_group(p_reg, NRFX_BIT(ch), group);

    if (en_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_set(p_reg, en_task, (uint8_t)en_reg);
    }
    if (dis_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_set(p_reg, dis_task, (uint8_t)dis_reg);
    }
}

void nrfx_gppi_group_ch_remove(nrfx_gppi_group_handle_t handle, uint32_t ch)
{
    NRF_DPPIC_Type * p_reg = dppi_reg_get(GHANDLE_GET_DOMAIN(handle));
    nrf_dppi_channel_group_t group = (nrf_dppi_channel_group_t)GHANDLE_GET_CHAN(handle);
    nrf_dppi_task_t en_task = nrf_dppi_group_enable_task_get((uint8_t)group);
    nrf_dppi_task_t dis_task = nrf_dppi_group_disable_task_get((uint8_t)group);
    uint32_t en_reg = nrf_dppi_subscribe_get(p_reg, en_task);
    uint32_t dis_reg = nrf_dppi_subscribe_get(p_reg, dis_task);

    /* Writes to CHG are ignored if SUBSCRIBE registers for a given group are enabled. */
    if (en_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_clear(p_reg, en_task);
    }
    if (dis_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_clear(p_reg, dis_task);
    }

    nrf_dppi_channels_remove_from_group(p_reg, NRFX_BIT(ch), group);

    if (en_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_set(p_reg, en_task, (uint8_t)en_reg);
    }
    if (dis_reg & NRF_SUBSCRIBE_PUBLISH_ENABLE)
    {
        nrf_dppi_subscribe_set(p_reg, dis_task, (uint8_t)dis_reg);
    }
}

uint32_t nrfx_gppi_group_channels_get(nrfx_gppi_group_handle_t handle)
{
    NRF_DPPIC_Type * p_reg = dppi_reg_get(GHANDLE_GET_DOMAIN(handle));
    uint32_t group = GHANDLE_GET_CHAN(handle);

    return p_reg->CHG[group];
}

void nrfx_gppi_group_enable(nrfx_gppi_group_handle_t handle)
{
    NRF_DPPIC_Type * p_reg = dppi_reg_get(GHANDLE_GET_DOMAIN(handle));
    nrf_dppi_channel_group_t group = (nrf_dppi_channel_group_t)GHANDLE_GET_CHAN(handle);

    nrf_dppi_group_enable(p_reg, group);
}

void nrfx_gppi_group_disable(nrfx_gppi_group_handle_t handle)
{
    NRF_DPPIC_Type * p_reg = dppi_reg_get(GHANDLE_GET_DOMAIN(handle));
    nrf_dppi_channel_group_t group = (nrf_dppi_channel_group_t)GHANDLE_GET_CHAN(handle);

    nrf_dppi_group_disable(p_reg, group);
}

uint32_t nrfx_gppi_group_task_en_addr(nrfx_gppi_group_handle_t handle)
{
    NRF_DPPIC_Type * p_reg = dppi_reg_get(GHANDLE_GET_DOMAIN(handle));
    nrf_dppi_channel_group_t group = (nrf_dppi_channel_group_t)GHANDLE_GET_CHAN(handle);

    return nrf_dppi_task_address_get(p_reg, nrf_dppi_group_enable_task_get(group));
}

uint32_t nrfx_gppi_group_task_dis_addr(nrfx_gppi_group_handle_t handle)
{
    NRF_DPPIC_Type * p_reg = dppi_reg_get(GHANDLE_GET_DOMAIN(handle));
    nrf_dppi_channel_group_t group = (nrf_dppi_channel_group_t)GHANDLE_GET_CHAN(handle);

    return nrf_dppi_task_address_get(p_reg, nrf_dppi_group_disable_task_get(group));
}
#endif
