/*
 * Copyright (c) 2018 - 2025, Nordic Semiconductor ASA
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

// Driver for single instance DPPI
#if NRFX_CHECK(NRFX_DPPI_ENABLED)

#include <nrfx_dppi.h>
#include <helpers/nrfx_flag32_allocator.h>

#define NRFX_LOG_MODULE DPPI
#include <nrfx_log.h>

#if !defined(NRFX_DPPI0_CHANNELS_USED) && defined(NRFX_DPPI_CHANNELS_USED)
#define NRFX_DPPI0_CHANNELS_USED NRFX_DPPI_CHANNELS_USED
#endif

#if !defined(NRFX_DPPI0_GROUPS_USED) && defined(NRFX_DPPI_GROUPS_USED)
#define NRFX_DPPI0_GROUPS_USED NRFX_DPPI_GROUPS_USED
#endif

#if defined(NRF_DPPIC0)

#if !defined(NRFX_DPPI0_CHANNELS_USED)
/* Bitmask that defines DPPI0 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI0_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI0_GROUPS_USED)
/* Bitmask that defines DPPI0 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI0_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC1)

#if !defined(NRFX_DPPI1_CHANNELS_USED)
/* Bitmask that defines DPPI1 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI1_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI1_GROUPS_USED)
/* Bitmask that defines DPPI1 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI1_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC00)

#if !defined(NRFX_DPPI00_CHANNELS_USED)
/* Bitmask that defines DPPI00 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI00_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI00_GROUPS_USED)
/* Bitmask that defines DPPI00 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI00_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC01)

#if !defined(NRFX_DPPI01_CHANNELS_USED)
/* Bitmask that defines DPPI01 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI01_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI01_GROUPS_USED)
/* Bitmask that defines DPPI01 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI01_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC10)

#if !defined(NRFX_DPPI10_CHANNELS_USED)
/* Bitmask that defines DPPI10 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI10_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI10_GROUPS_USED)
/* Bitmask that defines DPPI10 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI10_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC20)

#if !defined(NRFX_DPPI20_CHANNELS_USED)
/* Bitmask that defines DPPI20 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI20_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI20_GROUPS_USED)
/* Bitmask that defines DPPI20 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI20_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC30)

#if !defined(NRFX_DPPI30_CHANNELS_USED)
/* Bitmask that defines DPPI30 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI30_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI30_GROUPS_USED)
/* Bitmask that defines DPPI30 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI30_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC020)

#if !defined(NRFX_DPPI020_CHANNELS_USED)
/* Bitmask that defines DPPI020 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI020_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI020_GROUPS_USED)
/* Bitmask that defines DPPI020 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI020_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC030)

#if !defined(NRFX_DPPI030_CHANNELS_USED)
/* Bitmask that defines DPPI030 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI030_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI030_GROUPS_USED)
/* Bitmask that defines DPPI030 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI030_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC120)

#if !defined(NRFX_DPPI120_CHANNELS_USED)
/* Bitmask that defines DPPI120 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI120_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI120_GROUPS_USED)
/* Bitmask that defines DPPI120 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI120_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC130)

#if !defined(NRFX_DPPI130_CHANNELS_USED)
/* Bitmask that defines DPPI130 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI130_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI130_GROUPS_USED)
/* Bitmask that defines DPPI130 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI130_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC131)

#if !defined(NRFX_DPPI131_CHANNELS_USED)
/* Bitmask that defines DPPI131 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI131_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI131_GROUPS_USED)
/* Bitmask that defines DPPI131 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI131_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC132)

#if !defined(NRFX_DPPI132_CHANNELS_USED)
/* Bitmask that defines DPPI132 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI132_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI132_GROUPS_USED)
/* Bitmask that defines DPPI132 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI132_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC133)

#if !defined(NRFX_DPPI133_CHANNELS_USED)
/* Bitmask that defines DPPI133 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI133_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI133_GROUPS_USED)
/* Bitmask that defines DPPI133 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI133_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC134)

#if !defined(NRFX_DPPI134_CHANNELS_USED)
/* Bitmask that defines DPPI134 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI134_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI134_GROUPS_USED)
/* Bitmask that defines DPPI134 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI134_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC135)

#if !defined(NRFX_DPPI135_CHANNELS_USED)
/* Bitmask that defines DPPI135 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI135_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI135_GROUPS_USED)
/* Bitmask that defines DPPI135 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI135_GROUPS_USED 0UL
#endif

#endif

#if defined(NRF_DPPIC136)

#if !defined(NRFX_DPPI136_CHANNELS_USED)
/* Bitmask that defines DPPI136 channels that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI136_CHANNELS_USED 0UL
#endif

#if !defined(NRFX_DPPI136_GROUPS_USED)
/* Bitmask that defines DPPI136 groups that are reserved for use outside of the nrfx library. */
#define NRFX_DPPI136_GROUPS_USED 0UL
#endif

#endif

#define DPPI_CHANNELS_NUM(idx)  NRFX_BIT_MASK(NRFX_CONCAT(DPPIC, idx, _CH_NUM)
#define DPPI_CHANNELS_USED(idx) NRFX_CONCAT(NRFX_DPPI, idx, _CHANNELS_USED)
#define DPPI_AVAILABLE_CHANNELS_MASK(idx) \
    ((uint32_t)(DPPI_CHANNELS_NUM(idx)) & ~(DPPI_CHANNELS_USED(idx))))

#define DPPI_GROUPS_NUM(idx)  NRFX_BIT_MASK(NRFX_CONCAT(DPPIC, idx, _GROUP_NUM)
#define DPPI_GROUPS_USED(idx) NRFX_CONCAT(NRFX_DPPI, idx, _GROUPS_USED)
#define DPPI_AVAILABLE_GROUPS_MASK(idx) \
    ((uint32_t)(DPPI_GROUPS_NUM(idx)) & ~(DPPI_GROUPS_USED(idx))))

/* Structure holding state of the pins */
typedef struct
{
    /**< Bitmap representing channels availability. */
    nrfx_atomic_t allocated_channels;
    /**< Bitmap representing groups availability. */
    nrfx_atomic_t allocated_groups;
    /**< Bitmap representing available channels. */
    const uint32_t available_channels;
    /**< Bitmap representing available groups. */
    const uint32_t available_groups;
} dppic_control_block_t;

#define _NRFX_DPPIC_CB_INITIALIZER(periph_name, prefix, idx, _)                         \
    [NRFX_CONCAT(NRFX_, periph_name, prefix##idx, _INST_IDX)] = {                       \
        .allocated_channels = (nrfx_atomic_t)DPPI_AVAILABLE_CHANNELS_MASK(prefix##idx), \
        .allocated_groups = (nrfx_atomic_t)DPPI_AVAILABLE_GROUPS_MASK(prefix##idx),     \
        .available_channels = DPPI_AVAILABLE_CHANNELS_MASK(prefix##idx),                \
        .available_groups = DPPI_AVAILABLE_GROUPS_MASK(prefix##idx),                    \
    },

static dppic_control_block_t m_cb[NRFX_DPPI_ENABLED_COUNT] = {
    NRFX_FOREACH_ENABLED(DPPI, _NRFX_DPPIC_CB_INITIALIZER, (), ())
};

#define _NRFX_DPPIC_LIST_INSTANCES(periph_name, prefix, idx, _) NRFX_DPPI_INSTANCE(prefix##idx),

static void dppi_free(nrfx_dppi_t const * p_instance)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    uint32_t mask = p_cb->available_groups & ~p_cb->allocated_groups;
    uint8_t group_idx = NRF_DPPI_CHANNEL_GROUP0;

    // Disable all channels
    nrfy_dppi_channels_disable(p_instance->p_reg, p_cb->available_channels & ~p_cb->allocated_channels);

    // Clear all groups configurations
    while (mask)
    {
        nrf_dppi_channel_group_t group = (nrf_dppi_channel_group_t)group_idx;
        if (mask & NRFX_BIT(group))
        {
            nrfy_dppi_group_clear(p_instance->p_reg, group);
            mask &= ~NRFX_BIT(group);
        }
        group_idx++;
    }

    // Clear all allocated channels.
    p_cb->allocated_channels = p_cb->available_channels;

    // Clear all allocated groups.
    p_cb->allocated_groups = p_cb->available_groups;
}

static nrfx_err_t dppi_channel_alloc(nrfx_dppi_t const * p_instance, uint8_t * p_channel)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    return nrfx_flag32_alloc(&p_cb->allocated_channels, p_channel);
}

static nrfx_err_t dppi_channel_free(nrfx_dppi_t const * p_instance, uint8_t channel)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    nrfy_dppi_channels_disable(p_instance->p_reg, NRFX_BIT(channel));
    return nrfx_flag32_free(&p_cb->allocated_channels, channel);
}

static nrfx_err_t dppi_channel_enable(nrfx_dppi_t const * p_instance, uint8_t channel)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_flag32_is_allocated(p_cb->allocated_channels, channel))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    else
    {
        nrfy_dppi_channels_enable(p_instance->p_reg, NRFX_BIT(channel));
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t dppi_channel_disable(nrfx_dppi_t const * p_instance, uint8_t channel)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_flag32_is_allocated(p_cb->allocated_channels, channel))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    else
    {
        nrfy_dppi_channels_disable(p_instance->p_reg, NRFX_BIT(channel));
        err_code = NRFX_SUCCESS;
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t dppi_group_alloc(nrfx_dppi_t const *        p_instance,
                                   nrf_dppi_channel_group_t * p_group)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    return nrfx_flag32_alloc(&p_cb->allocated_groups, (uint8_t *)p_group);
}

static nrfx_err_t dppi_group_free(nrfx_dppi_t const * p_instance, nrf_dppi_channel_group_t group)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];

    nrfy_dppi_group_disable(p_instance->p_reg, group);
    return nrfx_flag32_free(&p_cb->allocated_groups, group);
}

static nrfx_err_t dppi_channel_include_in_group(nrfx_dppi_t const *      p_instance,
                                                uint8_t                  channel,
                                                nrf_dppi_channel_group_t group)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_flag32_is_allocated(p_cb->allocated_groups, group) ||
        !nrfx_flag32_is_allocated(p_cb->allocated_channels, channel))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    else
    {
        NRFY_CRITICAL_SECTION_ENTER();
        nrfy_dppi_channels_include_in_group(p_instance->p_reg, NRFX_BIT(channel), group);
        NRFY_CRITICAL_SECTION_EXIT();
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t dppi_channel_remove_from_group(nrfx_dppi_t const *      p_instance,
                                                 uint8_t                  channel,
                                                 nrf_dppi_channel_group_t group)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_flag32_is_allocated(p_cb->allocated_groups, group) ||
        !nrfx_flag32_is_allocated(p_cb->allocated_channels, channel))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    else
    {
        NRFY_CRITICAL_SECTION_ENTER();
        nrfy_dppi_channels_remove_from_group(p_instance->p_reg, NRFX_BIT(channel), group);
        NRFY_CRITICAL_SECTION_EXIT();
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t dppi_group_clear(nrfx_dppi_t const *      p_instance,
                                   nrf_dppi_channel_group_t group)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_flag32_is_allocated(p_cb->allocated_groups, group))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    else
    {
        nrfy_dppi_channels_remove_from_group(p_instance->p_reg, p_cb->available_channels, group);
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t dppi_group_enable(nrfx_dppi_t const *      p_instance,
                                    nrf_dppi_channel_group_t group)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_flag32_is_allocated(p_cb->allocated_groups, group))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    else
    {
        nrfy_dppi_group_enable(p_instance->p_reg, group);
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static nrfx_err_t dppi_group_disable(nrfx_dppi_t const *      p_instance,
                                     nrf_dppi_channel_group_t group)
{
    dppic_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    nrfx_err_t err_code = NRFX_SUCCESS;

    if (!nrfx_flag32_is_allocated(p_cb->allocated_groups, group))
    {
        err_code = NRFX_ERROR_INVALID_PARAM;
    }
    else
    {
        nrfy_dppi_group_disable(p_instance->p_reg, group);
    }
    NRFX_LOG_INFO("Function: %s, error code: %s.", __func__, NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

nrfx_err_t nrfx_dppi_periph_get(uint32_t peripheral_addr, nrfx_dppi_t * p_instance)
{
    static const nrfx_dppi_t dppi_list[NRFX_DPPI_ENABLED_COUNT] =
    {
        NRFX_FOREACH_ENABLED(DPPI, _NRFX_DPPIC_LIST_INSTANCES, (), ())
    };

#if NRF_DPPI_HAS_APB_MAPPING
    for (uint32_t i = 0; i < NRFX_DPPI_ENABLED_COUNT; i++)
    {
        if((nrf_address_bus_get(peripheral_addr, NRF_PERIPH_APB_MASK)) ==
           (nrf_address_bus_get((uint32_t)dppi_list[i].p_reg, NRF_DPPI_APB_MASK)))
        {
            *p_instance = dppi_list[i];
            return NRFX_SUCCESS;
        }
    }
    return NRFX_ERROR_INVALID_PARAM;
#else
    (void)peripheral_addr;
    *p_instance = dppi_list[0];
    return NRFX_SUCCESS;
#endif
}

#if NRFX_API_VER_AT_LEAST(3, 8, 0)

void nrfx_dppi_free(nrfx_dppi_t const * p_instance)
{
    dppi_free(p_instance);
}

nrfx_err_t nrfx_dppi_channel_alloc(nrfx_dppi_t const * p_instance, uint8_t * p_channel)
{
    return dppi_channel_alloc(p_instance, p_channel);
}

nrfx_err_t nrfx_dppi_channel_free(nrfx_dppi_t const * p_instance, uint8_t channel)
{
    return dppi_channel_free(p_instance, channel);
}

nrfx_err_t nrfx_dppi_channel_enable(nrfx_dppi_t const * p_instance, uint8_t channel)
{
    return dppi_channel_enable(p_instance, channel);
}

nrfx_err_t nrfx_dppi_channel_disable(nrfx_dppi_t const * p_instance, uint8_t channel)
{
    return dppi_channel_disable(p_instance, channel);
}

nrfx_err_t nrfx_dppi_group_alloc(nrfx_dppi_t const *        p_instance,
                                 nrf_dppi_channel_group_t * p_group)
{
    return dppi_group_alloc(p_instance, p_group);
}

nrfx_err_t nrfx_dppi_group_free(nrfx_dppi_t const * p_instance, nrf_dppi_channel_group_t group)
{
    return dppi_group_free(p_instance, group);
}

nrfx_err_t nrfx_dppi_channel_include_in_group(nrfx_dppi_t const *      p_instance,
                                              uint8_t                  channel,
                                              nrf_dppi_channel_group_t group)
{
    return dppi_channel_include_in_group(p_instance, channel, group);
}

nrfx_err_t nrfx_dppi_channel_remove_from_group(nrfx_dppi_t const *      p_instance,
                                               uint8_t                  channel,
                                               nrf_dppi_channel_group_t group)
{
    return dppi_channel_remove_from_group(p_instance, channel, group);
}

nrfx_err_t nrfx_dppi_group_clear(nrfx_dppi_t const * p_instance, nrf_dppi_channel_group_t group)
{
    return dppi_group_clear(p_instance, group);
}

nrfx_err_t nrfx_dppi_group_enable(nrfx_dppi_t const * p_instance, nrf_dppi_channel_group_t group)
{
    return dppi_group_enable(p_instance, group);
}

nrfx_err_t nrfx_dppi_group_disable(nrfx_dppi_t const *      p_instance,
                                   nrf_dppi_channel_group_t group)
{
    return dppi_group_disable(p_instance, group);
}

#else

nrfx_dppi_t const dppi_instance = NRFX_DPPI_INSTANCE(NRF_DPPIC_INDEX);

void nrfx_dppi_free(void)
{
    dppi_free(&dppi_instance);
}

nrfx_err_t nrfx_dppi_channel_alloc(uint8_t * p_channel)
{
    return dppi_channel_alloc(&dppi_instance, p_channel);
}

nrfx_err_t nrfx_dppi_channel_free(uint8_t channel)
{
    return dppi_channel_free(&dppi_instance, channel);
}

nrfx_err_t nrfx_dppi_channel_enable(uint8_t channel)
{
    return dppi_channel_enable(&dppi_instance, channel);
}

nrfx_err_t nrfx_dppi_channel_disable(uint8_t channel)
{
    return dppi_channel_disable(&dppi_instance, channel);
}

nrfx_err_t nrfx_dppi_group_alloc(nrf_dppi_channel_group_t * p_group)
{
    return dppi_group_alloc(&dppi_instance, p_group);
}

nrfx_err_t nrfx_dppi_group_free(nrf_dppi_channel_group_t group)
{
    return dppi_group_free(&dppi_instance, group);
}

nrfx_err_t nrfx_dppi_channel_include_in_group(uint8_t                  channel,
                                              nrf_dppi_channel_group_t group)
{
    return dppi_channel_include_in_group(&dppi_instance, channel, group);
}

nrfx_err_t nrfx_dppi_channel_remove_from_group(uint8_t                  channel,
                                               nrf_dppi_channel_group_t group)
{
    return dppi_channel_remove_from_group(&dppi_instance, channel, group);
}

nrfx_err_t nrfx_dppi_group_clear(nrf_dppi_channel_group_t group)
{
    return dppi_group_clear(&dppi_instance, group);
}

nrfx_err_t nrfx_dppi_group_enable(nrf_dppi_channel_group_t group)
{
    return dppi_group_enable(&dppi_instance, group);
}

nrfx_err_t nrfx_dppi_group_disable(nrf_dppi_channel_group_t group)
{
    return dppi_group_disable(&dppi_instance, group);
}

#endif

#endif // NRFX_CHECK(NRFX_DPPI_ENABLED)
