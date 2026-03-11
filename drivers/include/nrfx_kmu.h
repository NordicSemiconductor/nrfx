/*
 * Copyright (c) 2026, Nordic Semiconductor ASA
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

#ifndef NRFX_KMU_H__
#define NRFX_KMU_H__

#include <nrfx.h>
#include <hal/nrf_kmu.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_kmu KMU driver
 * @{
 * @ingroup nrf_kmu
 * @brief   Key Management Unit (KMU) peripheral driver.
 */

/** @brief Macro for calculating how many 32-bit words can be placed in one key slot. */
#define KEY_SLOT_WORDS_COUNT ((KMU_KEYSLOTBITS) / 32)

#if NRF_KMU_HAS_REVOKE_POLICY || defined(__NRFX_DOXYGEN__)
/** @brief KMU revoke policy type. */
typedef enum
{
    NRFX_KMU_RPOLICY_LOCKED   = KMU_TASKS_REVOKE_POLICY_Locked,   ///< Locked revoke policy.
    NRFX_KMU_RPOLICY_ROTATING = KMU_TASKS_REVOKE_POLICY_Rotating, ///< Rotating revoke policy.
    NRFX_KMU_RPOLICY_REVOKED  = KMU_TASKS_REVOKE_POLICY_Revoked,  ///< Revoked revoke policy.
} nrfx_kmu_rpolicy_t;
#endif

#if NRF_KMU_HAS_METADATA || defined(__NRFX_DOXYGEN__)
/** @brief KMU metadata struct type. */
typedef struct {
    uint32_t metadata;     ///< Metadata register.
#if NRF_KMU_HAS_METADATAEXT || defined(__NRFX_DOXYGEN__)
    uint32_t metadata_ext; ///< Metadata extended register.
#endif
} nrfx_kmu_key_slot_metadata_t;
#endif

/** @brief Key slot data structure. */
typedef struct __PACKED
{
    uint32_t                     keyslot_value[KEY_SLOT_WORDS_COUNT]; ///< Key data to be provisioned.
#if NRF_KMU_HAS_REVOKE_POLICY || defined(__NRFX_DOXYGEN__)
    uint32_t                     revoke_policy;                       /**< Key revoke policy.
                                                                       *   @ref nrfx_kmu_rpolicy_t
                                                                       *   holds possible values. */
#endif
    uint32_t                     keyslot_dest;                        /**< Key slot destination when
                                                                       *   performing key push. */
#if NRF_KMU_HAS_METADATA || defined(__NRFX_DOXYGEN__)
    nrfx_kmu_key_slot_metadata_t metadata;                            ///< Metadata to write to keyslot.
#endif
} nrfx_kmu_key_slot_data_t;

/**
 * @brief KMU driver event handler type.
 *
 * @param[in] event_type KMU event.
*/
typedef void (* nrfx_kmu_event_handler_t)(nrf_kmu_event_t event_type);

/**
 * @brief Function for initializing KMU driver.
 *
 * @param[in] handler Event handler. Used only if KMU interrupts are present on a device (can be
 *                    checked with @ref NRF_KMU_HAS_INTEN)
 *
 * @retval 0         Initialization successful.
 * @retval -EALREADY The driver is already initialized.
 */
int nrfx_kmu_init(nrfx_kmu_event_handler_t * handler);

/** @brief Function for uninitializing KMU driver. */
void nrfx_kmu_uninit(void);

/**
 * @brief Function for provisioning data to given key slot in KMU.
 *
 * @note Requires enabled writing to non-volatile memory.
 *
 * @param[in] p_key_slot_data Pointer to the key slot data structure.
 * @param[in] slot_id         ID of a slot to use.
 *
 * @retval 0          Provisioning was successful.
 * @retval -EAGAIN    KMU is currently busy with the previous operation.
 * @retval -EFAULT    An error occured when provisioning the key.
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slot_provision(nrfx_kmu_key_slot_data_t const * p_key_slot_data, uint32_t slot_id);

/**
 * @brief Function for pushing @p num_slots consecutive key slots from KMU to address specified in
 *        @p keyslot_dest field in @ref nrfx_kmu_key_slot_data_t at provisioning.
 *
 * @param[in] slot_id   ID of a slot to use.
 * @param[in] num_slots Number of consecutive keys slots to push.
 *
 * @retval 0          Pushing was successful.
 * @retval -EACCES    The key slot is revoked.
 * @retval -EAGAIN    KMU is currently busy with the previous operation. Can also happen after
 *                    pushing only a part of the given keys.
 * @retval -EFAULT    An error occured when pushing the key slot (slot is bloked or empty).
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slots_push(uint32_t slot_id, uint32_t num_slots);

#if NRF_KMU_HAS_PUSH_BLOCK || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for blocking @p num_slots consecutive key slots from being pushed.
 *
 * @param[in] slot_id   ID of a slot to use.
 * @param[in] num_slots Number of consecutive keys slots to block from being pushed.
 *
 * @retval 0          Blocking was successful.
 * @retval -EAGAIN    KMU is currently busy with the previous operation. Can also happen after
 *                    blocking only a part of the given keys.
 * @retval -EFAULT    An error occured when push blocking the key slot.
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slots_push_block(uint32_t slot_id, uint32_t num_slots);
#endif

#if NRF_KMU_HAS_BLOCK || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for blocking @p num_slots consecutive key slots from being pushed or revoked.
 *
 * @param[in] slot_id   ID of a slot to use.
 * @param[in] num_slots Number of consecutive keys slots to block.
 *
 * @retval 0          Blocking was successful.
 * @retval -EAGAIN    KMU is currently busy with the previous operation. Can also happen after
 *                    blocking only a part of the given keys.
 * @retval -EFAULT    An error occured when blocking the key slot.
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slots_block(uint32_t slot_id, uint32_t num_slots);
#endif

/**
 * @brief Function for revoking @p num_slots consecutive key slots.
 *
 * @param[in] slot_id   ID of a slot to use.
 * @param[in] num_slots Number of consecutive keys slots to revoke.
 *
 * @retval 0          Revoking was successful.
 * @retval -EAGAIN    KMU is currently busy with the previous operation. Can also happen after
 *                    revoking only a part of the given keys.
 * @retval -EFAULT    An error occured when revoking the key slot.
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slots_revoke(uint32_t slot_id, uint32_t num_slots);

#if NRF_KMU_HAS_METADATA || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for reading metadata from a key slot.
 *
 * @param[in]  slot_id    ID of a slot to use.
 * @param[out] p_metadata Pointer to a metadata structure to be filled.
 *
 * @retval 0          Metadata was read.
 * @retval -EACCES    The key slot is revoked.
 * @retval -EFAULT    The key slot is empty.
 * @retval -EAGAIN    KMU is currently busy with the previous operation.
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slot_metadata_read(uint32_t slot_id, nrfx_kmu_key_slot_metadata_t * p_metadata);
#endif

/**
 * @brief Function for checking if given @p num_slots consecutive key slots can be written to.
 *
 * @param[in]  slot_id    ID of a slot to check.
 * @param[in]  num_slots  Number of consecutive keys slots to check.
 * @param[out] p_is_empty Pointer to a variable determining if @p num_slots consecutive key slots
 *                        are empty. If at least one of the @p num_slots key slots is occupied
 *                        or revoked (non-rotating policy), @p p_is_empty is equal to false.
 *
 * @retval 0          Metadata was read.
 * @retval -EAGAIN    KMU is currently busy with the previous operation.
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slots_empty_check(uint32_t slot_id, uint32_t num_slots, bool * p_is_empty);

/**
 * @brief Function for checking if given @p num_slots consecutive key slots are revoked.
 *
 * @param[in]  slot_id      ID of a slot to check.
 * @param[in]  num_slots    Number of consecutive keys slots to check.
 * @param[out] p_is_revoked Pointer to a variable determining if @p num_slots consecutive key slots
 *                          are revoked. If at least one of the @p num_slots key slots is empty
 *                          or occupied, @p p_is_empty is equal to false.
 *
 * @retval 0          Metadata was read.
 * @retval -EAGAIN    KMU is currently busy with the previous operation.
 * @retval -ETIMEDOUT Timeout occured when waiting for result event.
 */
int nrfx_kmu_key_slots_revoked_check(uint32_t slot_id, uint32_t num_slots, bool * p_is_revoked);

/**
 * @brief Function for finding first @p num_slots consecutive available key slots.
 *
 * @param[in] num_slots Number of consecutive key slots to find.
 *
 * @retval non-negative Slot ID of the first available key slot.
 * @retval -ENOMEM      num_slots consecutive empty key slots were not found, too many occupied
 *                      and/or revoked key slots.
 */
int nrfx_kmu_next_available_key_slots_find(uint32_t num_slots);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_KMU_H__
