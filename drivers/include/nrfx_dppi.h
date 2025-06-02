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

#ifndef NRFX_DPPI_H__
#define NRFX_DPPI_H__

#include <nrfx.h>
#include <haly/nrfy_dppi.h>

/* On devices with single instance (with no ID) use instance 0. */
#if defined(NRF_DPPIC) && defined(NRFX_DPPI_ENABLED) && !defined(NRFX_DPPI0_ENABLED)
#define NRFX_DPPI0_ENABLED 1
#endif

/**
 * @defgroup nrfx_dppi DPPI allocator
 * @{
 * @ingroup nrf_dppi
 * @brief   Distributed Programmable Peripheral Interconnect (DPPI) allocator.
 */

/** @brief Data structure of the Distributed programmable peripheral interconnect (DPPI) driver instance. */
typedef struct
{
    NRF_DPPIC_Type * p_reg;        ///< Pointer to a structure containing DPPIC registers.
    uint8_t          drv_inst_idx; ///< Index of the driver instance. For internal use only.
} nrfx_dppi_t;

#ifndef __NRFX_DOXYGEN__
enum {
    /* List all enabled driver instances (in the format NRFX_\<instance_name\>_INST_IDX). */
    NRFX_INSTANCE_ENUM_LIST(DPPI)
    NRFX_DPPI_ENABLED_COUNT
};
#endif

/** @brief Macro for creating an instance of the DPPIC driver. */
#define NRFX_DPPI_INSTANCE(id)                            \
{                                                          \
    .p_reg        = NRFX_CONCAT(NRF_, DPPIC, id),          \
    .drv_inst_idx = NRFX_CONCAT(NRFX_DPPI, id, _INST_IDX), \
}

#ifdef __cplusplus
extern "C" {
#endif

#if NRFX_API_VER_AT_LEAST(3, 8, 0) || defined(__NRFX_DOXYGEN__)

/**
 * @brief Function for freeing all allocated channels and groups.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 */
void nrfx_dppi_free(nrfx_dppi_t const * p_instance);

/**
 * @brief Function for allocating a DPPI channel.
 * @details This function allocates the first unused DPPI channel.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_alloc.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 * @param[out] p_channel  Pointer to the DPPI channel number that has been allocated.
 *
 * @retval NRFX_SUCCESS      The channel was successfully allocated.
 * @retval NRFX_ERROR_NO_MEM There is no available channel to be used.
 */
nrfx_err_t nrfx_dppi_channel_alloc(nrfx_dppi_t const * p_instance, uint8_t * p_channel);

/**
 * @brief Function for freeing a DPPI channel.
 * @details This function also disables the chosen channel. Configuration in
 *          PUBLISH/SUBSCRIBE registers used for the channel is not cleared.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_free.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    DPPI channel to be freed.
 *
 * @retval NRFX_SUCCESS             The channel was successfully freed.
 * @retval NRFX_ERROR_INVALID_PARAM The specified channel is not allocated.
 */
nrfx_err_t nrfx_dppi_channel_free(nrfx_dppi_t const * p_instance, uint8_t channel);

/**
 * @brief Function for enabling a DPPI channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    DPPI channel to be enabled.
 *
 * @retval NRFX_SUCCESS             The channel was successfully enabled.
 * @retval NRFX_ERROR_INVALID_PARAM The specified channel is not allocated.
 */
nrfx_err_t nrfx_dppi_channel_enable(nrfx_dppi_t const * p_instance, uint8_t channel);

/**
 * @brief Function for disabling a DPPI channel.
 *
 * @note Disabling channel does not modify PUBLISH/SUBSCRIBE registers configured to use
 *       that channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    DPPI channel to be disabled.
 *
 * @retval NRFX_SUCCESS             The channel was successfully disabled.
 * @retval NRFX_ERROR_INVALID_PARAM The specified channel is not allocated.
 */
nrfx_err_t nrfx_dppi_channel_disable(nrfx_dppi_t const * p_instance, uint8_t channel);

/**
 * @brief Function for allocating a DPPI channel group.
 * @details This function allocates the first unused DPPI group.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_alloc.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 * @param[out] p_group    Pointer to the DPPI channel group that has been allocated.
 *
 * @retval NRFX_SUCCESS      The channel group was successfully allocated.
 * @retval NRFX_ERROR_NO_MEM There is no available channel group to be used.
 */
nrfx_err_t nrfx_dppi_group_alloc(nrfx_dppi_t const *        p_instance,
                                 nrf_dppi_channel_group_t * p_group);

/**
 * @brief Function for freeing a DPPI channel group.
 * @details This function also disables the chosen group.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_free.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] group      DPPI channel group to be freed.
 *
 * @retval NRFX_SUCCESS             The channel group was successfully freed.
 * @retval NRFX_ERROR_INVALID_PARAM The specified group is not allocated.
 */
nrfx_err_t nrfx_dppi_group_free(nrfx_dppi_t const * p_instance, nrf_dppi_channel_group_t group);

/**
 * @brief Function for including a DPPI channel in a channel group.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    DPPI channel to be added.
 * @param[in] group      Channel group in which to include the channel.
 *
 * @warning Channel group configuration can be modified only if subscriptions for tasks
 *          associated with this group are disabled.
 *
 * @retval NRFX_SUCCESS             The channel was successfully included.
 * @retval NRFX_ERROR_INVALID_PARAM The specified group or channel is not allocated.
 */
nrfx_err_t nrfx_dppi_channel_include_in_group(nrfx_dppi_t const *      p_instance,
                                              uint8_t                  channel,
                                              nrf_dppi_channel_group_t group);

/**
 * @brief Function for removing a DPPI channel from a channel group.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    DPPI channel to be removed.
 * @param[in] group      Channel group from which to remove the channel.
 *
 * @warning Channel group configuration can be modified only if subscriptions for tasks
 *          associated with this group are disabled.
 *
 * @retval NRFX_SUCCESS             The channel was successfully removed.
 * @retval NRFX_ERROR_INVALID_PARAM The specified group or channel is not allocated.
 */
nrfx_err_t nrfx_dppi_channel_remove_from_group(nrfx_dppi_t const *      p_instance,
                                               uint8_t                  channel,
                                               nrf_dppi_channel_group_t group);

/**
 * @brief Function for clearing a DPPI channel group.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] group      Channel group to be cleared.
 *
 * @warning Channel group configuration can be modified only if subscriptions for tasks
 *          associated with this group are disabled.
 *
 * @retval NRFX_SUCCESS             The group was successfully cleared.
 * @retval NRFX_ERROR_INVALID_PARAM The specified group is not allocated.
 */
nrfx_err_t nrfx_dppi_group_clear(nrfx_dppi_t const * p_instance, nrf_dppi_channel_group_t group);

/**
 * @brief Function for enabling a DPPI channel group.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] group      Channel group to be enabled.
 *
 * @retval NRFX_SUCCESS             The group was successfully enabled.
 * @retval NRFX_ERROR_INVALID_PARAM The specified group is not allocated.
 */
nrfx_err_t nrfx_dppi_group_enable(nrfx_dppi_t const *      p_instance,
                                  nrf_dppi_channel_group_t group);

/**
 * @brief Function for disabling a DPPI channel group.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] group      Channel group to be disabled.
 *
 * @retval NRFX_SUCCESS             The group was successfully disabled.
 * @retval NRFX_ERROR_INVALID_PARAM The specified group is not allocated.
 */
nrfx_err_t nrfx_dppi_group_disable(nrfx_dppi_t const *      p_instance,
                                   nrf_dppi_channel_group_t group);

/**
 * @brief Function for getting a driver instance for DPPI located on the same bus
 *        as the specified peripheral.
 *
 * @param[in]  peripheral_addr Address of a peripheral.
 * @param[out] p_instance      Pointer to the DPPI driver instance structure
 *                             to be filled.
 *
 * @retval NRFX_SUCCESS             DPPI driver instance was found.
 * @retval NRFX_ERROR_INVALID_PARAM No DPPI driver instance was found
 *                                  for a given peripheral address.
 */
nrfx_err_t nrfx_dppi_periph_get(uint32_t peripheral_addr, nrfx_dppi_t * p_instance);

#else

#if !defined(NRF_DPPIC_INDEX)
/* Choose the instance to use in case of using deprecated single-instance driver variant. */
#if defined(HALTIUM_XXAA)
#define NRF_DPPIC_INDEX 130
#elif defined(LUMOS_XXAA)
#define NRF_DPPIC_INDEX 20
#else
#define NRF_DPPIC_INDEX 0
#endif
#endif

void nrfx_dppi_free(void);

nrfx_err_t nrfx_dppi_channel_alloc(uint8_t * p_channel);

nrfx_err_t nrfx_dppi_channel_free(uint8_t channel);

nrfx_err_t nrfx_dppi_channel_enable(uint8_t channel);

nrfx_err_t nrfx_dppi_channel_disable(uint8_t channel);

nrfx_err_t nrfx_dppi_group_alloc(nrf_dppi_channel_group_t * p_group);

nrfx_err_t nrfx_dppi_group_free(nrf_dppi_channel_group_t group);

nrfx_err_t nrfx_dppi_channel_include_in_group(uint8_t                  channel,
                                              nrf_dppi_channel_group_t group);

nrfx_err_t nrfx_dppi_channel_remove_from_group(uint8_t                  channel,
                                               nrf_dppi_channel_group_t group);

nrfx_err_t nrfx_dppi_group_clear(nrf_dppi_channel_group_t group);

nrfx_err_t nrfx_dppi_group_enable(nrf_dppi_channel_group_t group);

nrfx_err_t nrfx_dppi_group_disable(nrf_dppi_channel_group_t group);

#endif

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_DPPI_H__
