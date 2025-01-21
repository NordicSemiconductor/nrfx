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

#if NRFX_CHECK(NRFX_DPPI_ENABLED) && defined(HALTIUM_XXAA)

#include <string.h>
#include <soc/interconnect/apb/nrfx_interconnect_apb.h>
#include <soc/interconnect/ipct/nrfx_interconnect_ipct.h>
#include <hal/nrf_ppib.h>
#include <helpers/nrfx_flag32_allocator.h>

#define CHANNEL_INVALID UINT8_MAX

#define NUMBER_OF_VIRTUAL_CHANNELS NRFX_GPPI_PROG_APP_CHANNELS_NUM
#define VIRTUAL_CHANNELS_MASK      NRFX_GPPI_PROG_APP_CHANNELS_MASK

typedef struct
{
    nrfx_interconnect_apb_t const * p_src_apb;
    nrfx_interconnect_apb_t const * p_dst_apb;
    uint8_t                         dppi_channel;
    uint8_t                         local_dppi_channel;
    uint8_t                         ipct_channel;
    uint8_t                         local_ipct_channel;
} nrfx_gppi_channels_path_t;

static nrfx_gppi_channels_path_t channels_path[NUMBER_OF_VIRTUAL_CHANNELS];
static nrfx_atomic_t m_virtual_channels = VIRTUAL_CHANNELS_MASK;

static void path_cleanup(nrfx_gppi_channels_path_t * p_path)
{
    NRFX_ASSERT(p_path);
    memset(p_path, 0, sizeof(nrfx_gppi_channels_path_t));
    p_path->dppi_channel = CHANNEL_INVALID;
    p_path->local_dppi_channel = CHANNEL_INVALID;
    p_path->ipct_channel = CHANNEL_INVALID;
}

static nrfx_err_t channel_free(nrfx_atomic_t * p_allocated_channels, uint8_t channel)
{
    NRFX_ASSERT(p_allocated_channels);
    if (channel == CHANNEL_INVALID)
    {
        return NRFX_ERROR_INVALID_PARAM;
    }
    return nrfx_flag32_free(p_allocated_channels, channel);
}

static nrfx_err_t channel_allocate(nrfx_atomic_t * p_channels_available,
                                   uint8_t       * p_channel,
                                   uint32_t        mask)
{
    NRFX_ASSERT(p_channel);
    uint32_t chan_avail;
    uint32_t chan_avail_masked;
    uint8_t chan_to_alloc;
    uint32_t prev_mask;

    do {
        chan_avail = *p_channels_available;
        chan_avail_masked = chan_avail & mask;
        if (chan_avail_masked == 0)
        {
            return NRFX_ERROR_NO_MEM;
        }
        else
        {
            chan_to_alloc = (uint8_t)(31UL - NRF_CLZ(chan_avail_masked));
        }

        prev_mask = NRFX_ATOMIC_FETCH_AND(p_channels_available,
                                          ~NRFX_BIT(chan_to_alloc));
    } while (prev_mask == *p_channels_available);
    *p_channel = chan_to_alloc;
    return NRFX_SUCCESS;
}

/* The main connection is needed when connecting two APBs from Global Domain.
   In addition no of them is main APB. In case of Haltium microcontrollers the main APB
   is APB32. */
static bool is_main_connection_needed(nrfx_interconnect_apb_t const * p_src_apb,
                                      nrfx_interconnect_apb_t const * p_dst_apb)
{
    if (nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN_GLOBAL ||
        nrfx_interconnect_apb_domain_get(p_dst_apb) == NRF_DOMAIN_GLOBAL)
    {
        return (p_src_apb != nrfx_interconnect_apb_main_get() &&
                p_dst_apb != nrfx_interconnect_apb_main_get() &&
                p_src_apb != p_dst_apb);
    }
    return false;
}

/* This function removes the direct connection between two APBs via PPIB bridge. */
static void apb_connection_remove(nrfx_interconnect_apb_t const * p_src_apb,
                                  nrfx_interconnect_apb_t const * p_dst_apb,
                                  uint8_t                         ppib_channel)
{
    NRFX_ASSERT(p_src_apb != p_dst_apb);
    NRFX_ASSERT(p_src_apb);
    NRFX_ASSERT(p_dst_apb);
    NRFX_ASSERT(nrfx_interconnect_apb_domain_get(p_src_apb) ==
                nrfx_interconnect_apb_domain_get(p_dst_apb));
    NRFX_ASSERT(nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN);
    // Inside local domain PPIB connection should be cleared here.
    nrf_ppib_subscribe_clear(p_src_apb->p_ppib,
                             nrf_ppib_send_task_get(ppib_channel));
    nrf_ppib_publish_clear(p_dst_apb->p_ppib,
                           nrf_ppib_receive_event_get(ppib_channel));
}

/* This function connects directly two APBs via PPIB bridge. */
static void apb_connection_create(nrfx_interconnect_apb_t const * p_src_apb,
                                  nrfx_interconnect_apb_t const * p_dst_apb,
                                  uint8_t                         ppib_channel)
{
    NRFX_ASSERT(p_src_apb != p_dst_apb);
    NRFX_ASSERT(p_src_apb);
    NRFX_ASSERT(p_dst_apb);
    NRFX_ASSERT(nrfx_interconnect_apb_domain_get(p_src_apb) ==
                nrfx_interconnect_apb_domain_get(p_dst_apb));
    NRFX_ASSERT(nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN);
    // Inside local domain PPIB connection should be set here.
    nrf_ppib_subscribe_set(p_src_apb->p_ppib,
                           nrf_ppib_send_task_get(ppib_channel),
                           ppib_channel);
    nrf_ppib_publish_set(p_dst_apb->p_ppib,
                         nrf_ppib_receive_event_get(ppib_channel),
                         ppib_channel);
}

/* This function removes direct connection between two domains via IPCT.
   It must be called before all local connections are removed
   (Before all `local_connection_remove()` function calls). */
static nrfx_err_t ipct_connection_remove(nrfx_interconnect_ipct_t const * p_src_ipct,
                                         nrfx_interconnect_ipct_t const * p_dst_ipct,
                                         nrfx_gppi_channels_path_t      * p_path)
{
    NRFX_ASSERT(p_src_ipct);
    NRFX_ASSERT(p_dst_ipct);
    NRFX_ASSERT(p_dst_ipct != p_src_ipct);
    NRFX_ASSERT(p_path);
    NRFX_ASSERT(nrfx_interconnect_ipct_domain_get(p_src_ipct) == NRF_DOMAIN_GLOBAL ||
                nrfx_interconnect_ipct_domain_get(p_dst_ipct) == NRF_DOMAIN_GLOBAL);

    nrfx_err_t err_code;
    uint8_t src_ipct_chan = nrfx_interconnect_ipct_domain_get(p_src_ipct) == NRF_DOMAIN_GLOBAL ?
                            p_path->ipct_channel :
                            p_path->local_ipct_channel;
    uint8_t dst_ipct_chan = nrfx_interconnect_ipct_domain_get(p_dst_ipct) == NRF_DOMAIN_GLOBAL ?
                            p_path->ipct_channel :
                            p_path->local_ipct_channel;

    err_code = channel_free(p_src_ipct->p_ipct_channels, src_ipct_chan);
    if (err_code == NRFX_SUCCESS)
    {
        /* Source channel is either already freed or it is not permitted by the configuration. */
        err_code = channel_free(p_dst_ipct->p_ipct_channels, dst_ipct_chan);
        if (err_code == NRFX_SUCCESS)
        {
            /* Clearing both IPCT configurations. */
            nrf_ipct_shorts_disable(p_src_ipct->p_ipct, NRFX_BIT(src_ipct_chan));
            nrf_ipct_shorts_disable(p_dst_ipct->p_ipct, NRFX_BIT(dst_ipct_chan));
            nrf_ipct_subscribe_clear(p_src_ipct->p_ipct, nrf_ipct_send_task_get(src_ipct_chan));
            nrf_ipct_publish_clear(p_dst_ipct->p_ipct, nrf_ipct_receive_event_get(dst_ipct_chan));
        }
    }
    return err_code;
}

/* Enable or disable all channels for all involved DPPIC peripherals. */
static void dppic_channel_set(uint8_t chan, bool enable)
{
    nrfx_gppi_channels_path_t * p_path = &channels_path[chan];
    nrfx_interconnect_apb_t const * p_src_apb = p_path->p_src_apb;
    nrfx_interconnect_apb_t const * p_dst_apb = p_path->p_dst_apb;
    uint8_t src_dppi_chan = nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN_GLOBAL ?
                            p_path->dppi_channel :
                            p_path->local_dppi_channel;
    uint8_t dst_dppi_chan = nrfx_interconnect_apb_domain_get(p_dst_apb) == NRF_DOMAIN_GLOBAL ?
                            p_path->dppi_channel :
                            p_path->local_dppi_channel;

    NRFX_ASSERT(nrfx_flag32_is_allocated(m_virtual_channels, chan));
    NRFX_ASSERT(p_src_apb);
    NRFX_ASSERT(p_dst_apb);
    nrfy_dppi_channels_set(p_src_apb->p_dppi, NRFX_BIT(src_dppi_chan), enable);
    nrfy_dppi_channels_set(p_dst_apb->p_dppi, NRFX_BIT(dst_dppi_chan), enable);
    if (is_main_connection_needed(p_src_apb, p_dst_apb))
    {
        nrfy_dppi_channels_set(nrfx_interconnect_apb_main_get()->p_dppi,
                               NRFX_BIT(p_path->dppi_channel),
                               enable);
    }
    if (nrfx_interconnect_apb_domain_get(p_src_apb) != nrfx_interconnect_apb_domain_get(p_dst_apb))
    {
        // Remove also DPPICn channels related with IPCT peripherals if needed.
        nrfx_interconnect_ipct_t const * p_src_ipct = nrfx_interconnect_ipct_get(p_src_apb);
        nrfx_interconnect_ipct_t const * p_dst_ipct = nrfx_interconnect_ipct_get(p_dst_apb);
        nrfx_interconnect_apb_t const * p_src_ipct_apb =
                                        (nrfx_interconnect_apb_get((uint32_t)p_src_ipct->p_ipct));
        nrfx_interconnect_apb_t const * p_dst_ipct_apb =
                                        (nrfx_interconnect_apb_get((uint32_t)p_dst_ipct->p_ipct));

        NRFX_ASSERT(p_src_ipct && p_src_ipct_apb);
        NRFX_ASSERT(p_dst_ipct && p_dst_ipct_apb);
        nrfy_dppi_channels_set(p_src_ipct_apb->p_dppi,
                               NRFX_BIT(src_dppi_chan),
                               enable);
        nrfy_dppi_channels_set(p_dst_ipct_apb->p_dppi,
                               NRFX_BIT(dst_dppi_chan),
                               enable);
    }
}

/* This function creates direct connection between two domains via IPCT.
   It must be called after all local connections are created
   (After all `local_connection_create()` function calls). */
static nrfx_err_t ipct_connection_create(nrfx_interconnect_ipct_t const * p_src_ipct,
                                         nrfx_interconnect_ipct_t const * p_dst_ipct,
                                         nrfx_gppi_channels_path_t      * p_path)
{
    NRFX_ASSERT(p_src_ipct);
    NRFX_ASSERT(p_dst_ipct);
    NRFX_ASSERT(p_dst_ipct != p_src_ipct);
    NRFX_ASSERT(p_path);
    NRFX_ASSERT(nrfx_interconnect_ipct_domain_get(p_src_ipct) == NRF_DOMAIN_GLOBAL ||
                nrfx_interconnect_ipct_domain_get(p_dst_ipct) == NRF_DOMAIN_GLOBAL);

    nrfx_err_t err_code;
    uint8_t src_dppi_channel;
    uint8_t dst_dppi_channel;
    uint8_t * src_ipct_channel;
    uint8_t * dst_ipct_channel;
    uint32_t src_chan_mask;
    uint32_t dst_chan_mask;
    nrfx_interconnect_apb_t const * p_src_apb =
                                    nrfx_interconnect_apb_get((uint32_t)p_src_ipct->p_ipct);
    nrfx_interconnect_apb_t const * p_dst_apb =
                                    nrfx_interconnect_apb_get((uint32_t)p_dst_ipct->p_ipct);

    if (nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN_GLOBAL)
    {
        /* Connetion from Global Domain (src) to Local Domain (dst). */
        src_dppi_channel = p_path->dppi_channel;
        src_ipct_channel = &p_path->ipct_channel;
        dst_dppi_channel = p_path->local_dppi_channel;
        dst_ipct_channel = &p_path->local_ipct_channel;
    }
    else if (nrfx_interconnect_apb_domain_get(p_dst_apb) == NRF_DOMAIN_GLOBAL)
    {
        /* Connetion from Local Domain (src) to Global Domain (dst). */
        src_dppi_channel = p_path->local_dppi_channel;
        src_ipct_channel = &p_path->local_ipct_channel;
        dst_dppi_channel = p_path->dppi_channel;
        dst_ipct_channel = &p_path->ipct_channel;
    }
    else
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    src_chan_mask = *p_src_ipct->p_ipct_channels & p_src_ipct->ipct_pub_channels_mask;
    dst_chan_mask = *p_dst_ipct->p_ipct_channels & p_dst_ipct->ipct_sub_channels_mask;
    err_code = channel_allocate(p_src_ipct->p_ipct_channels, src_ipct_channel, src_chan_mask);
    if (err_code == NRFX_SUCCESS)
    {
        /* No more IPCT channels available for source domain. */
        err_code = channel_allocate(p_dst_ipct->p_ipct_channels, dst_ipct_channel, dst_chan_mask);
        if (err_code == NRFX_SUCCESS)
        {
            /* Setting up both IPCT configurations. */
            nrf_ipct_shorts_enable(p_src_ipct->p_ipct, NRFX_BIT(*src_ipct_channel));
            nrf_ipct_shorts_enable(p_dst_ipct->p_ipct, NRFX_BIT(*dst_ipct_channel));
            nrf_ipct_subscribe_set(p_src_ipct->p_ipct,
                                   nrf_ipct_send_task_get(*src_ipct_channel),
                                   src_dppi_channel);
            nrf_ipct_publish_set(p_dst_ipct->p_ipct,
                                 nrf_ipct_receive_event_get(*dst_ipct_channel),
                                 dst_dppi_channel);
        }
        else
        {
            /* No more IPCT channels available for destination domain.
               Then We can free previously allocated channel for source domain. */
            (void)channel_free(p_src_ipct->p_ipct_channels, *src_ipct_channel);
        }
    }
    return err_code;
}

/* This function creates connection between two APBs inside one domain via DPPIC
   (and PPIB if needed).
   It must be called before IPCT connection is created
   (Before `ipct_connection_create()` function call). */
static nrfx_err_t local_connection_create(nrfx_interconnect_apb_t const * p_src_apb,
                                          nrfx_interconnect_apb_t const * p_dst_apb,
                                          uint8_t *                       dppi_channel)
{
    nrfx_err_t err_code;
    uint8_t reserved_src_channel = CHANNEL_INVALID;
    uint8_t reserved_dst_channel = CHANNEL_INVALID;
    bool use_main_apb_interconnect = false;
    uint32_t chan_mask;
    uint32_t chan_mask_to_exclude = UINT32_MAX;

    NRFX_ASSERT(p_src_apb);
    NRFX_ASSERT(p_dst_apb);

    NRFX_CRITICAL_SECTION_ENTER();
    if (p_src_apb == p_dst_apb)
    {
        chan_mask_to_exclude = p_src_apb->dppi_pub_channels_mask |
                               p_src_apb->dppi_sub_channels_mask;
        /* Creating connection whithin one APB. No need to have publish/subscribe mask then.*/
        chan_mask = *p_src_apb->p_dppi_channels;
        if (chan_mask & ~chan_mask_to_exclude)
        {
            /* Try not to utilize channels that can be connected to other APBs. */
            chan_mask &= ~chan_mask_to_exclude;
        }
        err_code = channel_allocate(p_src_apb->p_dppi_channels, dppi_channel, chan_mask);
    }
    else
    {
        /* Creating connection between two different APBs. */
        chan_mask = (*p_src_apb->p_dppi_channels & p_src_apb->dppi_pub_channels_mask) &
                    (*p_dst_apb->p_dppi_channels & p_dst_apb->dppi_sub_channels_mask);
        if (is_main_connection_needed(p_src_apb, p_dst_apb))
        {
            /* The path requires to go through the main APB. */
            use_main_apb_interconnect = true;
            chan_mask &= (nrfx_interconnect_apb_main_get()->dppi_pub_channels_mask &
                          nrfx_interconnect_apb_main_get()->dppi_sub_channels_mask &
                          (uint32_t)(*nrfx_interconnect_apb_main_get()->p_dppi_channels));
        }
        else if (p_src_apb == nrfx_interconnect_apb_main_get())
        {
            /* The path does not go through the main APB,
               however the source endpoint belongs to it. */
            chan_mask_to_exclude = p_src_apb->dppi_sub_channels_mask;

        }
        else if (p_dst_apb == nrfx_interconnect_apb_main_get())
        {
            /* The path does not go through the main APB,
               however the destination endpoint belongs to it. */
            chan_mask_to_exclude = p_dst_apb->dppi_pub_channels_mask;
        }
        if (chan_mask & ~chan_mask_to_exclude)
        {
            /* Try not to utilize channels that can be used for broader connections. */
            chan_mask &= ~chan_mask_to_exclude;
        }

        /* Allocating same channel for all involved DPPICs. */
        err_code = channel_allocate(p_src_apb->p_dppi_channels, dppi_channel, chan_mask);
        if (err_code == NRFX_SUCCESS)
        {
            reserved_src_channel = *dppi_channel;
            err_code = channel_allocate(p_dst_apb->p_dppi_channels, dppi_channel, chan_mask);
            if (err_code == NRFX_SUCCESS)
            {
                reserved_dst_channel = *dppi_channel;
                if (use_main_apb_interconnect)
                {
                    *dppi_channel = CHANNEL_INVALID;
                    err_code = channel_allocate(
                                    nrfx_interconnect_apb_main_get()->p_dppi_channels,
                                    dppi_channel,
                                    chan_mask);
                }
            }
        }

        if (err_code != NRFX_SUCCESS)
        {
            /* For at least one of involved DPPICs there was no channels available. */
            (void)channel_free(p_src_apb->p_dppi_channels, reserved_src_channel);
            (void)channel_free(p_dst_apb->p_dppi_channels, reserved_dst_channel);
            (void)channel_free(nrfx_interconnect_apb_main_get()->p_dppi_channels, *dppi_channel);
        }
        else if (nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN)
        {
            /* Inside our domain we are allowed to configure APB connection by ourself.
               (For Global Domain it is done by Secure Deomain). */
            apb_connection_create(p_src_apb, p_dst_apb, *dppi_channel);
        }
    }
    NRFX_CRITICAL_SECTION_EXIT();
    return err_code;
}

/* This function removes connection between two APBs inside one domain.
   It must be called after IPCT connection is removed
   (After `ipct_connection_remove()` function call). */
static nrfx_err_t local_connection_remove(nrfx_interconnect_apb_t const * p_src_apb,
                                          nrfx_interconnect_apb_t const * p_dst_apb,
                                          uint8_t                         dppi_channel)
{
    nrfx_err_t err_code;

    NRFX_ASSERT(dppi_channel != CHANNEL_INVALID);
    NRFX_ASSERT(p_src_apb);
    NRFX_ASSERT(p_dst_apb);
    NRFX_ASSERT(nrfx_interconnect_apb_domain_get(p_src_apb) ==
                nrfx_interconnect_apb_domain_get(p_dst_apb));
    if (p_src_apb == p_dst_apb)
    {
        /* Removing connection within one APB. */
        err_code = channel_free(p_src_apb->p_dppi_channels, dppi_channel);
        if (err_code != NRFX_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        /* Removing connection between two APBs. */
        NRFX_CRITICAL_SECTION_ENTER();
        err_code = channel_free(p_src_apb->p_dppi_channels, dppi_channel);
        if (err_code == NRFX_SUCCESS)
        {
            err_code = channel_free(p_dst_apb->p_dppi_channels, dppi_channel);
            if (err_code == NRFX_SUCCESS)
            {
                if (is_main_connection_needed(p_src_apb, p_dst_apb))
                {
                    /* The path required to go through the main APB*/
                    err_code = channel_free(nrfx_interconnect_apb_main_get()->p_dppi_channels,
                                            dppi_channel);
                }
            }
        }
        NRFX_CRITICAL_SECTION_EXIT();
    }

    if (err_code != NRFX_SUCCESS)
    {
        return err_code;
    }
    if (nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN && p_src_apb != p_dst_apb)
    {
        /* Inside our domain we are allowed to configure APB connection by ourself. */
        apb_connection_remove(p_src_apb, p_dst_apb, dppi_channel);
    }
    return err_code;
}

nrfx_err_t nrfx_gppi_channel_alloc(uint8_t * p_channel)
{
    nrfx_err_t err;
    nrfx_gppi_channels_path_t * chan;

    err = nrfx_flag32_alloc(&m_virtual_channels, p_channel);
    if (err == NRFX_SUCCESS)
    {
        chan = &channels_path[*p_channel];
        path_cleanup(chan);
    }
    return err;
}

void nrfx_gppi_event_endpoint_setup(uint8_t channel, uint32_t eep)
{
    (void)channel;
    (void)eep;
    // `tep` is also needed to decide whether `main_apb` is to be used.
    NRFX_ASSERT(false);
}

void nrfx_gppi_task_endpoint_setup(uint8_t channel, uint32_t tep)
{
    (void)channel;
    (void)tep;
    // `eep` is also needed to decide whether `main_apb` is to be used.
    NRFX_ASSERT(false);
}

void nrfx_gppi_event_endpoint_clear(uint8_t channel, uint32_t eep)
{
    (void)channel;
    (void)eep;
    // `tep` is also needed to decide whether `main_apb` is to be used.
    NRFX_ASSERT(false);
}

void nrfx_gppi_task_endpoint_clear(uint8_t channel, uint32_t tep)
{
    (void)channel;
    (void)tep;
    // `eep` is also needed to decide whether `main_apb` is to be used.
    NRFX_ASSERT(false);
}

void nrfx_gppi_fork_endpoint_setup(uint8_t channel, uint32_t fork_tep)
{
    NRFX_ASSERT(fork_tep);
    NRFX_ASSERT(nrfx_flag32_is_allocated(m_virtual_channels, channel));
    nrfx_interconnect_apb_t const * p_apb = nrfx_interconnect_apb_get(fork_tep);
    nrfx_gppi_channels_path_t * p_path = &channels_path[channel];
    uint8_t dppi_chan = nrfx_interconnect_apb_domain_get(p_apb) == NRF_DOMAIN_GLOBAL ?
                        p_path->dppi_channel :
                        p_path->local_dppi_channel;
    NRFX_ASSERT(p_apb);
    // The endpoint must belong to one of used APB in existing connection.
    if ((p_path->p_dst_apb != p_apb) && (p_path->p_src_apb != p_apb))
    {
        if (!is_main_connection_needed(p_path->p_src_apb, p_path->p_dst_apb) ||
            p_apb != nrfx_interconnect_apb_main_get())
            {
                NRFX_ASSERT(false);
            }
    }
    NRF_DPPI_ENDPOINT_SETUP(fork_tep, dppi_chan);
}

void nrfx_gppi_fork_endpoint_clear(uint8_t channel, uint32_t fork_tep)
{
    NRFX_ASSERT(fork_tep);
    NRFX_ASSERT(nrfx_flag32_is_allocated(m_virtual_channels, channel));
    nrfx_interconnect_apb_t const * p_apb = nrfx_interconnect_apb_get(fork_tep);
    nrfx_gppi_channels_path_t * p_path = &channels_path[channel];

    NRFX_ASSERT(p_apb);
    // The endpoint must belong to one of used APB in existing connection.
    if ((p_path->p_dst_apb != p_apb) && (p_path->p_src_apb != p_apb))
    {
        if (!is_main_connection_needed(p_path->p_src_apb, p_path->p_dst_apb) ||
            p_apb != nrfx_interconnect_apb_main_get())
            {
                NRFX_ASSERT(false);
            }
    }
    NRF_DPPI_ENDPOINT_CLEAR(fork_tep);
}

void nrfx_gppi_channel_endpoints_setup(uint8_t channel, uint32_t eep, uint32_t tep)
{
    NRFX_ASSERT(tep);
    NRFX_ASSERT(eep);

    nrfx_err_t err_code;
    nrfx_interconnect_apb_t const * p_src_apb = (nrfx_interconnect_apb_get(eep));
    nrfx_interconnect_apb_t const * p_dst_apb = (nrfx_interconnect_apb_get(tep));
    nrfx_gppi_channels_path_t * p_path = &channels_path[channel];

    NRFX_ASSERT(p_src_apb);
    NRFX_ASSERT(p_dst_apb);

    uint8_t * src_dppi_chan = nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN_GLOBAL ?
                              &p_path->dppi_channel :
                              &p_path->local_dppi_channel;
    uint8_t * dst_dppi_chan = nrfx_interconnect_apb_domain_get(p_dst_apb) == NRF_DOMAIN_GLOBAL ?
                              &p_path->dppi_channel :
                              &p_path->local_dppi_channel;

    if (nrfx_interconnect_apb_domain_get(p_src_apb) == nrfx_interconnect_apb_domain_get(p_dst_apb))
    {
        /* Endpoints belongs to the same domain - one local connection needed. */
        NRFX_ASSERT(src_dppi_chan == dst_dppi_chan);
        err_code = local_connection_create(p_src_apb, p_dst_apb, src_dppi_chan);
        if (err_code != NRFX_SUCCESS)
        {
            NRFX_ASSERT(false);
            (void)local_connection_remove(p_src_apb, p_dst_apb, *src_dppi_chan);
        }
    }
    else
    {
        /* Endpoints in different domains - two local connections and one IPCT connection needed. */
        nrfx_interconnect_ipct_t const * p_src_ipct = nrfx_interconnect_ipct_get(p_src_apb);
        nrfx_interconnect_ipct_t const * p_dst_ipct = nrfx_interconnect_ipct_get(p_dst_apb);
        nrfx_interconnect_apb_t const * p_src_ipct_apb =
                                        (nrfx_interconnect_apb_get((uint32_t)p_src_ipct->p_ipct));
        nrfx_interconnect_apb_t const * p_dst_ipct_apb =
                                        (nrfx_interconnect_apb_get((uint32_t)p_dst_ipct->p_ipct));
        NRFX_ASSERT(src_dppi_chan != dst_dppi_chan);

        /* Creating local connection from source to IPCT peripheral inside the first domain. */
        err_code = local_connection_create(p_src_apb, p_src_ipct_apb, src_dppi_chan);
        NRFX_ASSERT(err_code == NRFX_SUCCESS);
        if (err_code == NRFX_SUCCESS)
        {
            /* Creating local connection from IPCT peripheral to destination inside the second domain. */
            err_code = local_connection_create(p_dst_ipct_apb, p_dst_apb, dst_dppi_chan);
            NRFX_ASSERT(err_code == NRFX_SUCCESS);
            if (err_code == NRFX_SUCCESS)
            {
                /* Creating IPCT connection between the first and the second domain. */
                err_code = ipct_connection_create(p_src_ipct, p_dst_ipct, p_path);
                NRFX_ASSERT(err_code == NRFX_SUCCESS);
            }
        }
        if (err_code != NRFX_SUCCESS)
        {
            (void)local_connection_remove(p_src_apb, p_src_ipct_apb, *src_dppi_chan);
            (void)local_connection_remove(p_dst_ipct_apb, p_dst_apb, *dst_dppi_chan);
            (void)ipct_connection_remove(p_src_ipct, p_dst_ipct, p_path);
        }
    }

    if (err_code == NRFX_SUCCESS)
    {
        p_path->p_src_apb = p_src_apb;
        p_path->p_dst_apb = p_dst_apb;
        NRF_DPPI_ENDPOINT_SETUP(eep, *src_dppi_chan);
        NRF_DPPI_ENDPOINT_SETUP(tep, *dst_dppi_chan);
    }
    else
    {
        path_cleanup(p_path);
    }
}

void nrfx_gppi_channel_endpoints_clear(uint8_t channel, uint32_t eep, uint32_t tep)
{
    NRFX_ASSERT(tep);
    NRFX_ASSERT(eep);

    nrfx_err_t err_code;
    nrfx_interconnect_apb_t const * p_src_apb = (nrfx_interconnect_apb_get(eep));
    nrfx_interconnect_apb_t const * p_dst_apb = (nrfx_interconnect_apb_get(tep));
    nrfx_gppi_channels_path_t * p_path = &channels_path[channel];
    uint8_t * src_dppi_chan = nrfx_interconnect_apb_domain_get(p_src_apb) == NRF_DOMAIN_GLOBAL ?
                              &p_path->dppi_channel :
                              &p_path->local_dppi_channel;
    uint8_t * dst_dppi_chan = nrfx_interconnect_apb_domain_get(p_dst_apb) == NRF_DOMAIN_GLOBAL ?
                              &p_path->dppi_channel :
                              &p_path->local_dppi_channel;

    NRFX_ASSERT(p_src_apb);
    NRFX_ASSERT(p_dst_apb);
    NRFX_ASSERT(p_path->p_src_apb == p_src_apb);
    NRFX_ASSERT(p_path->p_dst_apb == p_dst_apb);

    if (nrfx_interconnect_apb_domain_get(p_src_apb) == nrfx_interconnect_apb_domain_get(p_dst_apb))
    {
        /* Endpoints belongs to the same domain - need to remove one local connection. */
        NRFX_ASSERT(src_dppi_chan == dst_dppi_chan);
        err_code = local_connection_remove(p_src_apb, p_dst_apb, *src_dppi_chan);
        NRFX_ASSERT(err_code == NRFX_SUCCESS);
    }
    else
    {
        /* Endpoints in different domains - need to remove two local connections and one IPCT connection. */
        NRFX_ASSERT(src_dppi_chan != dst_dppi_chan);
        nrfx_interconnect_ipct_t const * p_src_ipct = nrfx_interconnect_ipct_get(p_src_apb);
        nrfx_interconnect_ipct_t const * p_dst_ipct = nrfx_interconnect_ipct_get(p_dst_apb);
        nrfx_interconnect_apb_t const * p_src_ipct_apb =
                                        nrfx_interconnect_apb_get((uint32_t)p_src_ipct->p_ipct);
        nrfx_interconnect_apb_t const * p_dst_ipct_apb =
                                        nrfx_interconnect_apb_get((uint32_t)p_dst_ipct->p_ipct);

        /* Removing IPCT connection between the first and the second domain. */
        err_code = ipct_connection_remove(p_src_ipct, p_dst_ipct, p_path);
        NRFX_ASSERT(err_code == NRFX_SUCCESS);
        if (err_code == NRFX_SUCCESS)
        {
            /* Removing local connection from IPCT peripheral to destination inside the first domain. */
            err_code = local_connection_remove(p_src_apb, p_src_ipct_apb, *src_dppi_chan);
            NRFX_ASSERT(err_code == NRFX_SUCCESS);
            if (err_code == NRFX_SUCCESS)
            {
                /* Removing local connection from IPCT peripheral to destination inside the second domain. */
                err_code = local_connection_remove(p_dst_ipct_apb, p_dst_apb, *dst_dppi_chan);
                NRFX_ASSERT(err_code == NRFX_SUCCESS);
            }
        }
    }
    if (err_code == NRFX_SUCCESS)
    {
        path_cleanup(p_path);
        NRF_DPPI_ENDPOINT_CLEAR(eep);
        NRF_DPPI_ENDPOINT_CLEAR(tep);
    }
    NRFX_ASSERT(err_code == NRFX_SUCCESS);
}

nrfx_err_t nrfx_gppi_channel_free(uint8_t channel)
{
    nrfx_err_t err;
    nrfx_gppi_channels_path_t * p_path;

    err = nrfx_flag32_free(&m_virtual_channels, channel);
    if (err != NRFX_SUCCESS)
    {
        return err;
    }
    p_path = &channels_path[channel];
    path_cleanup(p_path);
    return err;
}

bool nrfx_gppi_channel_check(uint8_t channel)
{
    nrfx_interconnect_apb_t const * p_src_apb = channels_path[channel].p_src_apb;
    nrfx_interconnect_apb_t const * p_dst_apb = channels_path[channel].p_dst_apb;
    uint8_t dppi_channel = channels_path[channel].dppi_channel;

    NRFX_ASSERT(nrfx_flag32_is_allocated(m_virtual_channels, channel));
    if (dppi_channel != CHANNEL_INVALID)
    {
        NRFX_ASSERT(p_src_apb);
        NRFX_ASSERT(p_dst_apb);
        if (!nrf_dppi_channel_check(p_src_apb->p_dppi, dppi_channel) ||
            !nrf_dppi_channel_check(p_dst_apb->p_dppi, dppi_channel))
        {
            /* At least one of DPPIC channels is not valid. */
            return false;
        }
        if (is_main_connection_needed(p_src_apb, p_dst_apb) &&
            !nrf_dppi_channel_check(nrfx_interconnect_apb_main_get()->p_dppi, dppi_channel))
        {
            /* DPPIC channel for main APB is not valid. */
            return false;
        }
        return true;
    }
    return false;
}

void nrfx_gppi_channels_disable_all(void)
{
    uint32_t mask = ~(uint32_t)m_virtual_channels;
    while (mask)
    {
        uint8_t chan = (uint8_t)NRF_CTZ(mask);
        dppic_channel_set(chan, false);
        mask &= ~NRFX_BIT(chan);
    }
}

void nrfx_gppi_channels_enable(uint32_t mask)
{
    while (mask)
    {
        uint8_t chan = (uint8_t)NRF_CTZ(mask);
        dppic_channel_set(chan, true);
        mask &= ~NRFX_BIT(chan);
    }
}

void nrfx_gppi_channels_disable(uint32_t mask)
{
    // Remove all connections for all channels determined by mask.
    while (mask)
    {
        // Remove assigned channels for all involved DPPICn peripherals.
        uint8_t chan = (uint8_t)NRF_CTZ(mask);
        dppic_channel_set(chan, false);
        mask &= ~NRFX_BIT(chan);
    }
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

    /* Not supported because PPIB connections are configured through UICR. */
    return NRFX_ERROR_NOT_SUPPORTED;
}

#endif // NRFX_DPPI_ENABLED && (DPPIC_COUNT > 1)
