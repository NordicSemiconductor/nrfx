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

#include <helpers/nrfx_gppi.h>

#if defined(LUMOS_XXAA)

#include <helpers/nrfx_flag32_allocator.h>
#include <hal/nrf_ppib.h>
#include <hal/nrf_dppi.h>

#include <soc/interconnect/dppic_ppib/nrfx_interconnect_dppic_ppib.h>

#include <soc/interconnect/dppic_ppib/nrfx_interconnect_dppic_ppib_lumos.h>

/** @brief Invalid channel number. */
#define NRFX_GPPI_CHANNEL_INVALID UINT8_MAX

#define NRFX_GPPI_PPIB_HAS_DYNAMIC_CONFIG 1

#define INVALID_DPPI_CHANNEL 0xFFUL

static nrfx_atomic_t m_virtual_channels = NRFX_GPPI_PROG_APP_CHANNELS_MASK;

static nrfx_err_t dppic_channel_get(nrfx_interconnect_dppic_t * p_dppic,
                                    uint8_t                     virtual_channel,
                                    uint8_t *                   p_dppic_channel)
{
    for (uint8_t i = 0; i < NRF_DPPI_CH_NUM_MAX; i++)
    {
        if (p_dppic->allocate_flag[i] == virtual_channel)
        {
            *p_dppic_channel = i;
            return NRFX_SUCCESS;
        }
    }

    return NRFX_ERROR_INVALID_PARAM;
}

static nrfx_err_t dppic_virtual_channel_set(nrfx_interconnect_dppic_t * p_dppic,
                                            uint32_t                    dppi_channel,
                                            uint8_t                     virtual_channel)
{
    p_dppic->allocate_flag[dppi_channel] = virtual_channel;

    return NRFX_SUCCESS;
}

static nrfx_err_t dppic_channel_alloc(nrfx_interconnect_dppic_t * p_dppic, uint8_t * p_channel)
{
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
    return nrfx_dppi_channel_alloc(&p_dppic->dppic, p_channel);
#else
    return nrfx_flag32_alloc(&p_dppic->channels_mask, p_channel);
#endif
}

static nrfx_err_t dppic_channel_free(nrfx_interconnect_dppic_t * p_dppic, uint8_t channel)
{
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
    return nrfx_dppi_channel_free(&p_dppic->dppic, channel);
#else
    return nrfx_flag32_free(&p_dppic->channels_mask, channel);
#endif
}

static nrfx_err_t ppib_channel_get(nrfx_interconnect_ppib_t * p_ppib,
                                   uint8_t                    virtual_channel,
                                   uint8_t *                  p_ppib_channel)
{
    for (uint8_t i = 0; i < PPIB_CHANNEL_MAX_COUNT; i++)
    {
        if (p_ppib->allocate_flag[i] == virtual_channel)
        {
            *p_ppib_channel = i;
            return NRFX_SUCCESS;
        }
    }

    return NRFX_ERROR_INVALID_PARAM;
}

#if NRFX_GPPI_PPIB_HAS_DYNAMIC_CONFIG
static nrfx_err_t ppib_channel_alloc(nrfx_interconnect_ppib_t * p_ppib, uint8_t * p_channel)
{
    return nrfx_ppib_channel_alloc(&p_ppib->ppib, p_channel);
}
#endif

static nrfx_err_t ppib_channel_free(nrfx_interconnect_ppib_t * p_ppib, uint8_t channel)
{
    return nrfx_ppib_channel_free(&p_ppib->ppib, channel);
}

static nrfx_err_t ppib_virtual_channel_set(nrfx_interconnect_ppib_t * p_ppib,
                                           uint32_t                   ppib_channel,
                                           uint8_t                    virtual_channel)
{
    p_ppib->allocate_flag[ppib_channel] = virtual_channel;

    return NRFX_SUCCESS;
}

/* Enable or disable all channels for all involved DPPIC peripherals. */
static void virtual_channel_enable_set(uint8_t virtual_channel, bool enable)
{
    for (uint8_t i = 0; i < NRFX_INTERCONNECT_DPPIC_COUNT; i++)
    {
        nrfx_interconnect_dppic_t * dppic = nrfx_interconnect_dppic_at_index_get(i);
        uint8_t dppi_channel;
        nrfx_err_t err = dppic_channel_get(dppic, virtual_channel, &dppi_channel);
        if (err == NRFX_SUCCESS)
        {
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
            if (enable)
            {
                nrfx_dppi_channel_enable(&dppic->dppic, dppi_channel);
            }
            else
            {
                nrfx_dppi_channel_disable(&dppic->dppic, dppi_channel);
            }
#else
            nrfy_dppi_channels_set(dppic->dppic, NRFX_BIT((uint32_t)dppi_channel), enable);
#endif
        }
    }
}

static void init(void)
{
    static bool initialized = false;

    if (initialized)
    {
        return;
    }

    for (uint8_t i = 0; i < NRFX_INTERCONNECT_DPPIC_COUNT; i++)
    {
        nrfx_interconnect_dppic_t * dppic = nrfx_interconnect_dppic_at_index_get(i);
        for (uint8_t j = 0; j < NRF_DPPI_CH_NUM_MAX; j++)
        {
            dppic_virtual_channel_set(dppic, j, NRFX_GPPI_CHANNEL_INVALID);
        }
    }

    for (uint8_t i = 0; i < NRFX_INTERCONNECT_PPIB_COUNT; i++)
    {
        nrfx_interconnect_ppib_t * ppib = nrfx_interconnect_ppib_at_index_get(i);
        for (uint8_t j = 0; j < PPIB_CHANNEL_MAX_COUNT; j++)
        {
            ppib_virtual_channel_set(ppib, j, NRFX_GPPI_CHANNEL_INVALID);
        }
    }

    initialized = true;
}

static nrfx_err_t create_ppib_connection(uint8_t                                   virtual_channel,
                                         nrfx_interconnect_dppic_to_dppic_path_t * p_path,
                                         uint8_t                                   src_dppi_channel,
                                         uint8_t                                   dst_dppi_channel)
{
    nrfx_interconnect_ppib_t * p_ppib = p_path->ppib;
    uint8_t    ppib_channel;
#if NRFX_GPPI_PPIB_HAS_DYNAMIC_CONFIG
    nrfx_err_t err = ppib_channel_alloc(p_ppib, &ppib_channel);

    if (err != NRFX_SUCCESS)
    {
        return err;
    }
#else
    NRFX_ASSERT(src_dppi_channel == dst_dppi_channel);
    ppib_channel = src_dppi_channel;
#endif

    ppib_virtual_channel_set(p_ppib, ppib_channel, virtual_channel);
    nrf_ppib_task_t  task  = nrfx_ppib_send_task_get(&p_ppib->ppib.left, ppib_channel);
    nrf_ppib_event_t event = nrfx_ppib_receive_event_get(&p_ppib->ppib.right, ppib_channel);
    if (p_path->ppib_inverted == false)
    {
        nrfx_ppib_subscribe_set(&p_ppib->ppib.left, task, src_dppi_channel);
        nrfx_ppib_publish_set(&p_ppib->ppib.right, event, dst_dppi_channel);
    }
    else
    {
        nrfx_ppib_subscribe_set(&p_ppib->ppib.right, task, src_dppi_channel);
        nrfx_ppib_publish_set(&p_ppib->ppib.left, event, dst_dppi_channel);
    }
    return NRFX_SUCCESS;
}

static nrfx_err_t clear_virtual_channel_path(uint8_t virtual_channel)
{
    // Clear all DPPI channel masks.
    for (uint8_t i = 0; i < NRFX_INTERCONNECT_DPPIC_COUNT; i++)
    {
        nrfx_interconnect_dppic_t * dppic = nrfx_interconnect_dppic_at_index_get(i);
        uint8_t dppi_channel;
        nrfx_err_t err = dppic_channel_get(dppic, virtual_channel, &dppi_channel);
        if (err == NRFX_SUCCESS)
        {
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
            nrfx_dppi_channel_disable(&dppic->dppic, dppi_channel);
#else
            nrfy_dppi_channels_set(dppic->dppic, NRFX_BIT((uint32_t)dppi_channel), false);
#endif

            err = dppic_channel_free(dppic, dppi_channel);
            if (err != NRFX_SUCCESS)
            {
                return err;
            }

            err = dppic_virtual_channel_set(dppic,
                                            (uint32_t)dppi_channel,
                                            NRFX_GPPI_CHANNEL_INVALID);
            if (err != NRFX_SUCCESS)
            {
                return err;
            }
        }
    }

    // Clear all PPIB channel masks.
    for (uint8_t i = 0; i < NRFX_INTERCONNECT_PPIB_COUNT; i++)
    {
        nrfx_interconnect_ppib_t * p_ppib = nrfx_interconnect_ppib_at_index_get(i);
        uint8_t ppib_channel;
        nrfx_err_t err = ppib_channel_get(p_ppib, virtual_channel, &ppib_channel);
        if (err == NRFX_SUCCESS)
        {
            nrf_ppib_task_t task   = nrfx_ppib_send_task_get(&p_ppib->ppib.left, ppib_channel);
            nrf_ppib_event_t event = nrfx_ppib_receive_event_get(&p_ppib->ppib.right,
                                                                 ppib_channel);

            nrfx_ppib_subscribe_clear(&p_ppib->ppib.left, task);
            nrfx_ppib_subscribe_clear(&p_ppib->ppib.right, task);

            nrfx_ppib_publish_clear(&p_ppib->ppib.left, event);
            nrfx_ppib_publish_clear(&p_ppib->ppib.right, event);

            err = ppib_channel_free(p_ppib, ppib_channel);
            if (err != NRFX_SUCCESS)
            {
                return err;
            }

            err = ppib_virtual_channel_set(p_ppib, (uint32_t)ppib_channel, NRFX_GPPI_CHANNEL_INVALID);
            if (err != NRFX_SUCCESS)
            {
                return err;
            }
        }
    }

    return NRFX_SUCCESS;
}

static nrfx_err_t gppi_dppi_connection_setup(uint8_t         virtual_channel,
                                             nrf_apb_index_t src_domain,
                                             uint8_t       * p_src_dppi_channel,
                                             nrf_apb_index_t dst_domain,
                                             uint8_t       * p_dst_dppi_channel)
{
    nrfx_err_t err = NRFX_SUCCESS;
    NRFX_ASSERT(src_domain);
    NRFX_ASSERT(dst_domain);
    NRFX_ASSERT(p_src_dppi_channel);
    NRFX_ASSERT(p_dst_dppi_channel);

    uint8_t src_dppi_channel = *p_src_dppi_channel;
    uint8_t dst_dppi_channel = *p_dst_dppi_channel;

    nrfx_interconnect_dppic_t * p_src_dppic = nrfx_interconnect_dppic_get(src_domain);
    nrfx_interconnect_dppic_t * p_dst_dppic = nrfx_interconnect_dppic_get(dst_domain);

    nrfx_interconnect_dppic_to_dppic_path_t path =
    {
        .src_dppic = p_src_dppic,
        .dst_dppic = p_dst_dppic,
    };

    /* Both channels must be either valid or not. */
    if ((src_dppi_channel == INVALID_DPPI_CHANNEL && dst_dppi_channel != INVALID_DPPI_CHANNEL)
     || (src_dppi_channel != INVALID_DPPI_CHANNEL && dst_dppi_channel == INVALID_DPPI_CHANNEL))
    {
        clear_virtual_channel_path(virtual_channel);
        NRFX_ASSERT(false);
        return NRFX_ERROR_INVALID_PARAM;
    }

#if !NRFX_GPPI_PPIB_HAS_DYNAMIC_CONFIG
    /* Without dynamic PPIB configs both source and destination must have the same number. */
    if (src_dppi_channel != dst_dppi_channel)
    {
        clear_virtual_channel_path(virtual_channel);
        NRFX_ASSERT(false);
        return NRFX_ERROR_INVALID_PARAM;
    }
#endif

    if (nrfx_interconnect_direct_connection_check(&path))
    {
        if (src_dppi_channel == INVALID_DPPI_CHANNEL)
        {
#if !NRFX_GPPI_PPIB_HAS_DYNAMIC_CONFIG
            nrfx_atomic_t possible_mask = path.src_dppic->channels_mask;
            possible_mask &= path.dst_dppic->channels_mask;
            possible_mask &=
                NRFX_BIT_MASK(nrf_ppib_channel_number_get(path.ppib->ppib.left.p_reg));

            uint8_t common_channel;
            nrfx_flag32_alloc(&possible_mask, &common_channel);
            if (err != NRFX_SUCCESS)
            {
                clear_virtual_channel_path(virtual_channel);
                NRFX_ASSERT(false);
                return err;
            }

            path.src_dppic->channels_mask &= ~NRFX_BIT(common_channel);
            path.dst_dppic->channels_mask &= ~NRFX_BIT(common_channel);

            src_dppi_channel = common_channel;
            dst_dppi_channel = common_channel;
#else
            err = dppic_channel_alloc(p_src_dppic, &src_dppi_channel);
            if (err != NRFX_SUCCESS)
            {
                clear_virtual_channel_path(virtual_channel);
                NRFX_ASSERT(false);
                return err;
            }

            err = dppic_channel_alloc(p_dst_dppic, &dst_dppi_channel);
            if (err != NRFX_SUCCESS)
            {
                clear_virtual_channel_path(virtual_channel);
                NRFX_ASSERT(false);
                return err;
            }
#endif
            dppic_virtual_channel_set(p_src_dppic, src_dppi_channel, virtual_channel);
            dppic_virtual_channel_set(p_dst_dppic, dst_dppi_channel, virtual_channel);
        }

        err = create_ppib_connection(virtual_channel,
                                     &path,
                                     src_dppi_channel,
                                     dst_dppi_channel);
        if (err != NRFX_SUCCESS)
        {
            clear_virtual_channel_path(virtual_channel);
            NRFX_ASSERT(false);
            return err;
        }
    }
    else
    {
        nrfx_interconnect_dppic_t * p_main_dppic = nrfx_interconnect_dppic_main_get();
        p_src_dppic = nrfx_interconnect_dppic_get(src_domain);
        p_dst_dppic = nrfx_interconnect_dppic_get(dst_domain);

        nrfx_interconnect_dppic_to_dppic_path_t path_src_to_main =
        {
            .src_dppic = p_src_dppic,
            .dst_dppic = p_main_dppic,
        };

        nrfx_interconnect_dppic_to_dppic_path_t path_main_to_dst =
        {
            .src_dppic = p_main_dppic,
            .dst_dppic = p_dst_dppic,
        };

        if (nrfx_interconnect_direct_connection_check(&path_src_to_main) &&
            nrfx_interconnect_direct_connection_check(&path_main_to_dst))
        {
            uint8_t main_dppi_channel;

            if (src_dppi_channel == INVALID_DPPI_CHANNEL)
            {
#if !NRFX_GPPI_PPIB_HAS_DYNAMIC_CONFIG
                nrfx_atomic_t possible_mask = p_src_dppic->channels_mask;
                possible_mask &= p_main_dppic->channels_mask;
                possible_mask &= p_dst_dppic->channels_mask;
                possible_mask &= NRFX_BIT_MASK(
                    nrf_ppib_channel_number_get(path_src_to_main.ppib->ppib.left.p_reg));
                possible_mask &= NRFX_BIT_MASK(
                    nrf_ppib_channel_number_get(path_main_to_dst.ppib->ppib.left.p_reg));

                uint8_t common_channel;
                nrfx_flag32_alloc(&possible_mask, &common_channel);
                if (err != NRFX_SUCCESS)
                {
                    clear_virtual_channel_path(virtual_channel);
                    NRFX_ASSERT(false);
                    return err;
                }

                p_src_dppic->channels_mask &= ~NRFX_BIT(common_channel);
                p_main_dppic->channels_mask &= ~NRFX_BIT(common_channel);
                p_dst_dppic->channels_mask &= ~NRFX_BIT(common_channel);

                src_dppi_channel = common_channel;
                dst_dppi_channel = common_channel;
                main_dppi_channel = common_channel;
#else
                err = dppic_channel_alloc(p_src_dppic, &src_dppi_channel);
                if (err != NRFX_SUCCESS)
                {
                    clear_virtual_channel_path(virtual_channel);
                    NRFX_ASSERT(false);
                    return err;
                }

                err = dppic_channel_alloc(p_main_dppic, &main_dppi_channel);
                if (err != NRFX_SUCCESS)
                {
                    clear_virtual_channel_path(virtual_channel);
                    NRFX_ASSERT(false);
                    return err;
                }

                err = dppic_channel_alloc(p_dst_dppic, &dst_dppi_channel);
                if (err != NRFX_SUCCESS)
                {
                    clear_virtual_channel_path(virtual_channel);
                    NRFX_ASSERT(false);
                    return err;
                }
#endif
                dppic_virtual_channel_set(p_src_dppic, src_dppi_channel, virtual_channel);
                dppic_virtual_channel_set(p_dst_dppic, dst_dppi_channel, virtual_channel);
            }
            else
            {
#if !NRFX_GPPI_PPIB_HAS_DYNAMIC_CONFIG
                nrfx_atomic_t possible_mask = NRFX_BIT(src_dppi_channel);
                possible_mask &= p_main_dppic->channels_mask;
                possible_mask &= NRFX_BIT(dst_dppi_channel);
                possible_mask &= NRFX_BIT_MASK(
                    nrf_ppib_channel_number_get(path_src_to_main.ppib->ppib.left.p_reg));
                possible_mask &= NRFX_BIT_MASK(
                    nrf_ppib_channel_number_get(path_main_to_dst.ppib->ppib.left.p_reg));

                uint8_t common_channel;
                nrfx_flag32_alloc(&possible_mask, &common_channel);
                if (err != NRFX_SUCCESS)
                {
                    clear_virtual_channel_path(virtual_channel);
                    NRFX_ASSERT(false);
                    return err;
                }

                p_src_dppic->channels_mask &= ~NRFX_BIT(common_channel);
                p_main_dppic->channels_mask &= ~NRFX_BIT(common_channel);
                p_dst_dppic->channels_mask &= ~NRFX_BIT(common_channel);

                main_dppi_channel = common_channel;
#else
                err = dppic_channel_alloc(p_main_dppic, &main_dppi_channel);
                if (err != NRFX_SUCCESS)
                {
                    clear_virtual_channel_path(virtual_channel);
                    NRFX_ASSERT(false);
                    return err;
                }
#endif
            }

            dppic_virtual_channel_set(p_main_dppic, main_dppi_channel, virtual_channel);

            err = create_ppib_connection(virtual_channel,
                                         &path_src_to_main ,
                                         src_dppi_channel,
                                         main_dppi_channel);
            if (err != NRFX_SUCCESS)
            {
                clear_virtual_channel_path(virtual_channel);
                NRFX_ASSERT(false);
                return err;
            }

            err = create_ppib_connection(virtual_channel,
                                         &path_main_to_dst,
                                         main_dppi_channel,
                                         dst_dppi_channel);
            if (err != NRFX_SUCCESS)
            {
                clear_virtual_channel_path(virtual_channel);
                NRFX_ASSERT(false);
                return err;
            }
        }
        else
        {
            clear_virtual_channel_path(virtual_channel);
            NRFX_ASSERT(false);
            return err;
        }
    }

    *p_src_dppi_channel = src_dppi_channel;
    *p_dst_dppi_channel = dst_dppi_channel;

    return err;
}

nrfx_err_t nrfx_gppi_channel_alloc(uint8_t * p_channel)
{
    init();
    return nrfx_flag32_alloc(&m_virtual_channels, p_channel);
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
    for (uint8_t i = 0; i < NRFX_INTERCONNECT_DPPIC_COUNT; i++)
    {
        nrfx_interconnect_dppic_t * dppic = nrfx_interconnect_dppic_at_index_get(i);
        uint8_t dppi_channel;
        nrfx_err_t err = dppic_channel_get(dppic, channel, &dppi_channel);
        if (err == NRFX_SUCCESS)
        {
            NRF_DPPI_ENDPOINT_SETUP(fork_tep, (uint32_t)dppi_channel);
            return;
        }
    }
    NRFX_ASSERT(false);
}

void nrfx_gppi_fork_endpoint_clear(uint8_t channel, uint32_t fork_tep)
{
    for (uint8_t i = 0; i < NRFX_INTERCONNECT_DPPIC_COUNT; i++)
    {
        nrfx_interconnect_dppic_t * dppic = nrfx_interconnect_dppic_at_index_get(i);
        uint8_t dppi_channel;
        nrfx_err_t err = dppic_channel_get(dppic, channel, &dppi_channel);
        if (err == NRFX_SUCCESS)
        {
            NRF_DPPI_ENDPOINT_CLEAR(fork_tep);
            return;
        }
    }
    NRFX_ASSERT(false);
}

void nrfx_gppi_channel_endpoints_setup(uint8_t channel, uint32_t eep, uint32_t tep)
{
    nrfx_err_t err = NRFX_SUCCESS;
    nrf_apb_index_t src_domain = nrfx_interconnect_apb_index_get(eep);
    nrf_apb_index_t dst_domain = nrfx_interconnect_apb_index_get(tep);

    NRFX_ASSERT(src_domain);
    NRFX_ASSERT(dst_domain);

    uint8_t src_dppi_channel = INVALID_DPPI_CHANNEL;
    uint8_t dst_dppi_channel = INVALID_DPPI_CHANNEL;

    if (src_domain == dst_domain)
    {
        nrfx_interconnect_dppic_t * dppic = nrfx_interconnect_dppic_get(src_domain);
        if (dppic_channel_alloc(dppic, &src_dppi_channel) == NRFX_SUCCESS)
        {
            dst_dppi_channel = src_dppi_channel;
            dppic_virtual_channel_set(dppic, src_dppi_channel, channel);
        }
        else
        {
            clear_virtual_channel_path(channel);
            NRFX_ASSERT(false);
            return;
        }
    }
    else
    {
        err = gppi_dppi_connection_setup(channel,
                                         src_domain,
                                         &src_dppi_channel,
                                         dst_domain,
                                         &dst_dppi_channel);
    }

    if (err != NRFX_SUCCESS)
    {
        NRFX_ASSERT(false);
        return;
    }

    NRF_DPPI_ENDPOINT_SETUP(eep, src_dppi_channel);
    NRF_DPPI_ENDPOINT_SETUP(tep, dst_dppi_channel);
}

void nrfx_gppi_channel_endpoints_clear(uint8_t channel, uint32_t eep, uint32_t tep)
{
    clear_virtual_channel_path(channel);

    NRF_DPPI_ENDPOINT_CLEAR(eep);
    NRF_DPPI_ENDPOINT_CLEAR(tep);
}

nrfx_err_t nrfx_gppi_channel_free(uint8_t channel)
{
    nrfx_err_t err = nrfx_flag32_free(&m_virtual_channels, channel);

    if(err != NRFX_SUCCESS)
    {
        return err;
    }

    return clear_virtual_channel_path(channel);
}

bool nrfx_gppi_channel_check(uint8_t channel)
{
    NRFX_ASSERT(nrfx_flag32_is_allocated(m_virtual_channels, channel));

    for (uint8_t i = 0; i < NRFX_INTERCONNECT_DPPIC_COUNT; i++)
    {
        nrfx_interconnect_dppic_t * dppic = nrfx_interconnect_dppic_at_index_get(i);
        uint8_t dppi_channel;
        nrfx_err_t err = dppic_channel_get(dppic, channel, &dppi_channel);
        if (err == NRFX_SUCCESS)
        {
            NRF_DPPIC_Type *p_reg;
#if NRFX_API_VER_AT_LEAST(3, 8, 0)
            p_reg = dppic->dppic.p_reg;
#else
            p_reg = dppic->dppic;
#endif
            if (nrf_dppi_channel_check(p_reg, (uint32_t)dppi_channel) == false)
            {
                return false;
            }
            break;
        }
    }

    return true;
}

void nrfx_gppi_channels_disable_all(void)
{
    uint32_t mask = ~(uint32_t)m_virtual_channels;

    while (mask)
    {
        uint8_t channel = (uint8_t)NRF_CTZ(mask);
        virtual_channel_enable_set(channel, false);
        mask &= ~NRFX_BIT(channel);
    }
}

void nrfx_gppi_channels_enable(uint32_t mask)
{
    while (mask)
    {
        uint8_t channel = (uint8_t)NRF_CTZ(mask);
        virtual_channel_enable_set(channel, true);
        mask &= ~NRFX_BIT(channel);
    }
}

void nrfx_gppi_channels_disable(uint32_t mask)
{
    // Remove all connections for all channels determined by mask.
    while (mask)
    {
        // Remove assigned channels for all involved DPPICn peripherals.
        uint8_t channel = (uint8_t)NRF_CTZ(mask);
        virtual_channel_enable_set(channel, false);
        mask &= ~NRFX_BIT(channel);
    }
}

nrfx_err_t nrfx_gppi_edge_connection_setup(uint8_t             channel,
                                           nrfx_dppi_t const * p_src_dppi,
                                           uint8_t             src_channel,
                                           nrfx_dppi_t const * p_dst_dppi,
                                           uint8_t             dst_channel)
{
    nrf_apb_index_t src_domain = nrfx_interconnect_apb_index_get((uint32_t)p_src_dppi->p_reg);
    nrf_apb_index_t dst_domain = nrfx_interconnect_apb_index_get((uint32_t)p_dst_dppi->p_reg);

    uint8_t src_dppi_channel = src_channel;
    uint8_t dst_dppi_channel = dst_channel;

    if (p_src_dppi == p_dst_dppi)
    {
        return NRFX_SUCCESS; /* No OP */
    }

    return gppi_dppi_connection_setup(channel, src_domain, &src_dppi_channel, dst_domain, &dst_dppi_channel);
}

#endif // defined(LUMOS_XXAA)
