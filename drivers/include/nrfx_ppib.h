/*
 * Copyright (c) 2024 - 2025, Nordic Semiconductor ASA
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

#ifndef NRFX_PPIB_H__
#define NRFX_PPIB_H__

#include <nrfx.h>
#include <hal/nrf_ppib.h>

/**
 * @defgroup nrfx_ppib PPIB allocator
 * @{
 * @ingroup nrf_ppib
 * @brief   Programmable Peripheral Interconnect Bridge (PPIB) allocator.
 */

/** @brief Data structure of the Programmable Peripheral Interconnect Bridge (PPIB) driver instance. */
typedef struct
{
    NRF_PPIB_Type * p_reg; ///< Pointer to a structure containing PPIBC registers.
} nrfx_ppib_t;

/** @brief Data structure of the pair of PPIB driver instances. */
typedef struct
{
    nrfx_ppib_t left;         ///< First driver instance.
    nrfx_ppib_t right;        ///< Second driver instance.
    uint8_t     drv_inst_idx; ///< Index of the driver instance. For internal use only.
} nrfx_ppib_interconnect_t;

#ifndef __NRFX_DOXYGEN__
enum {
    /* List all enabled driver instances (in the format NRFX_\<instance_name\>_INST_IDX). */
    NRFX_INSTANCE_ENUM_LIST(PPIB)
    NRFX_PPIB_ENABLED_COUNT
};

enum {
    /* List all enabled interconnects. Smaller PPIB idx are always on the left. */
#if defined(NRF54L_SERIES) || defined(NRF7120_ENGA_XXAA)
#if NRFX_CHECK(NRFX_PPIB00_ENABLED) && NRFX_CHECK(NRFX_PPIB10_ENABLED)
    NRFX_PPIB_INTERCONNECT_00_10_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_PPIB01_ENABLED) && NRFX_CHECK(NRFX_PPIB20_ENABLED)
    NRFX_PPIB_INTERCONNECT_01_20_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_PPIB11_ENABLED) && NRFX_CHECK(NRFX_PPIB21_ENABLED)
    NRFX_PPIB_INTERCONNECT_11_21_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_PPIB22_ENABLED) && NRFX_CHECK(NRFX_PPIB30_ENABLED)
    NRFX_PPIB_INTERCONNECT_22_30_INST_IDX,
#endif
#endif
#if defined(NRF54LM20A_ENGA_XXAA)
#if NRFX_CHECK(NRFX_PPIB02_ENABLED) && NRFX_CHECK(NRFX_PPIB03_ENABLED)
    NRFX_PPIB_INTERCONNECT_02_03_INST_IDX,
#endif
#if NRFX_CHECK(NRFX_PPIB04_ENABLED) && NRFX_CHECK(NRFX_PPIB12_ENABLED)
    NRFX_PPIB_INTERCONNECT_04_12_INST_IDX,
#endif
#endif
#if defined(HALTIUM_XXAA)
#if NRFX_CHECK(NRFX_PPIB020_ENABLED) && NRFX_CHECK(NRFX_PPIB030_ENABLED)
    NRFX_PPIB_INTERCONNECT_020_030_INST_IDX,
#endif
#endif
    NRFX_PPIB_INTERCONNECT_COUNT
};
#endif

/** @brief Macro for creating an instance of the PPIB driver. */
#define NRFX_PPIB_INSTANCE(id)            \
{                                         \
    .p_reg = NRFX_CONCAT(NRF_, PPIB, id), \
}

/** @brief Macro for creating an instance of the PPIB interconnect driver. */
#define NRFX_PPIB_INTERCONNECT_INSTANCE(id_left, id_right)                                 \
{                                                                                          \
    .left         = NRFX_PPIB_INSTANCE(id_left),                                           \
    .right        = NRFX_PPIB_INSTANCE(id_right),                                          \
    .drv_inst_idx = NRFX_CONCAT(NRFX_PPIB_INTERCONNECT_, id_left, _, id_right, _INST_IDX), \
}

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Function for freeing all allocated channels for a given PPIB interconnection.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_ppib_free(nrfx_ppib_interconnect_t const * p_instance);

/**
 * @brief Function for allocating a PPIB channel.
 * @details This function allocates the highest available PPIB channel.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_alloc.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 * @param[out] p_channel  Pointer to the PPIB channel number that has been allocated.
 *
 * @retval NRFX_SUCCESS      The channel was successfully allocated.
 * @retval NRFX_ERROR_NO_MEM There is no available channel to be used.
 */
nrfx_err_t nrfx_ppib_channel_alloc(nrfx_ppib_interconnect_t const * p_instance, uint8_t * p_channel);

/**
 * @brief Function for freeing a PPIB channel.
 * @details This function also clears the PUBLISH/SUBSCRIBE configuration.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_free.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    PPIB channel to be freed.
 *
 * @retval NRFX_SUCCESS             The channel was successfully freed.
 * @retval NRFX_ERROR_INVALID_PARAM The specified channel is not allocated.
 */
nrfx_err_t nrfx_ppib_channel_free(nrfx_ppib_interconnect_t const * p_instance, uint8_t channel);

/**
 * @brief Function for getting the PPIB SEND task for the specified channel.
 *
 * @details The returned task identifier can be used within @ref nrf_ppib_hal,
 *          for example, to configure a DPPI channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    PPIB channel.
 *
 * @return SEND task associated with the specified channel.
 */
NRFX_STATIC_INLINE nrf_ppib_task_t nrfx_ppib_send_task_get(nrfx_ppib_t const * p_instance,
                                                           uint8_t             channel);

/**
 * @brief Function for getting the address of the PPIB SEND task for the specified channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    PPIB channel.
 *
 * @return Address of the specified SEND task.
 */
NRFX_STATIC_INLINE uint32_t nrfx_ppib_send_task_address_get(nrfx_ppib_t const * p_instance,
                                                            uint8_t             channel);

/**
 * @brief Function for getting the PPIB RECEIVE event for the specified channel.
 *
 * @details The returned event identifier can be used within @ref nrf_ppib_hal,
 *          for example, to configure a DPPI channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    PPIB channel.
 *
 * @return RECEIVE event associated with the specified channel.
 */
NRFX_STATIC_INLINE nrf_ppib_event_t nrfx_ppib_receive_event_get(nrfx_ppib_t const * p_instance,
                                                                uint8_t             channel);

/**
 * @brief Function for getting the address of a PPIB RECEIVE event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    PPIB channel.
 *
 * @return Address of the specified RECEIVE event.
 */
NRFX_STATIC_INLINE uint32_t nrfx_ppib_receive_event_address_get(nrfx_ppib_t const * p_instance,
                                                                uint8_t             channel);

/**
 * @brief Function for setting the subscribe configuration for a given
 *        PPIB task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] task       Task for which to set the configuration.
 * @param[in] channel    Channel through which to subscribe events.
 */
NRFX_STATIC_INLINE void nrfx_ppib_subscribe_set(nrfx_ppib_t const * p_instance,
                                                nrf_ppib_task_t     task,
                                                uint8_t             channel);

/**
 * @brief Function for clearing the subscribe configuration for a given
 *        PPIB task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] task       Task for which to clear the configuration.
 */
NRFX_STATIC_INLINE void nrfx_ppib_subscribe_clear(nrfx_ppib_t const * p_instance,
                                                  nrf_ppib_task_t     task);

/**
 * @brief Function for setting the publish configuration for a given event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] event      Event for which to set the configuration.
 * @param[in] channel    PPIB channel through which to publish the event.
 */
NRFX_STATIC_INLINE void nrfx_ppib_publish_set(nrfx_ppib_t const * p_instance,
                                              nrf_ppib_event_t    event,
                                              uint8_t             channel);
/**
 * @brief Function for clearing the publish configuration for a given event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] event      Event for which to clear the configuration.
 */
NRFX_STATIC_INLINE void nrfx_ppib_publish_clear(nrfx_ppib_t const * p_instance,
                                                nrf_ppib_event_t    event);
#ifndef NRFX_DECLARE_ONLY

NRFX_STATIC_INLINE nrf_ppib_task_t nrfx_ppib_send_task_get(nrfx_ppib_t const * p_instance,
                                                           uint8_t             channel)
{
    (void) p_instance;

    return nrf_ppib_send_task_get(channel);
}

NRFX_STATIC_INLINE uint32_t nrfx_ppib_send_task_address_get(nrfx_ppib_t const * p_instance,
                                                            uint8_t             channel)
{
    return nrf_ppib_task_address_get(p_instance->p_reg, nrf_ppib_send_task_get(channel));
}

NRFX_STATIC_INLINE nrf_ppib_event_t nrfx_ppib_receive_event_get(nrfx_ppib_t const * p_instance,
                                                                uint8_t             channel)
{
    (void) p_instance;

    return nrf_ppib_receive_event_get(channel);
}

NRFX_STATIC_INLINE uint32_t nrfx_ppib_receive_event_address_get(nrfx_ppib_t const * p_instance,
                                                                uint8_t             channel)
{
    return nrf_ppib_event_address_get(p_instance->p_reg, nrf_ppib_receive_event_get(channel));
}

NRFX_STATIC_INLINE void nrfx_ppib_subscribe_set(nrfx_ppib_t const * p_instance,
                                                nrf_ppib_task_t     task,
                                                uint8_t             channel)
{
    nrf_ppib_subscribe_set(p_instance->p_reg, task, channel);
}

NRFX_STATIC_INLINE void nrfx_ppib_subscribe_clear(nrfx_ppib_t const * p_instance,
                                                  nrf_ppib_task_t     task)
{
    nrf_ppib_subscribe_clear(p_instance->p_reg, task);
}

NRFX_STATIC_INLINE void nrfx_ppib_publish_set(nrfx_ppib_t const * p_instance,
                                              nrf_ppib_event_t    event,
                                              uint8_t             channel)
{
    nrf_ppib_publish_set(p_instance->p_reg, event, channel);
}

NRFX_STATIC_INLINE void nrfx_ppib_publish_clear(nrfx_ppib_t const * p_instance,
                                                nrf_ppib_event_t    event)
{
    nrf_ppib_publish_clear(p_instance->p_reg, event);
}

#endif

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_PPIB_H__
