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

#include <nrfx.h>
#include <nrfx_kmu.h>

#define NRFX_LOG_MODULE KMU
#include <nrfx_log.h>

#if defined(NRF53_SERIES) || defined(NRF91_SERIES)
#error "Current implementation of KMU driver does not support nRF53 and nRF91 devices"
#endif

/** @cond Driver internal data. */
typedef struct
{
    nrfx_drv_state_t state;
} nrfx_kmu_control_block_t;

static nrfx_kmu_control_block_t m_cb;

static void clear_all_events(void)
{
#if NRF_KMU_HAS_PROVISION
    nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_EVENTS_PROVISIONED);
#endif
    nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_PUSHED);
    nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_REVOKED);
    nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_ERROR);
#if NRF_KMU_HAS_METADATA
    nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_EVENTS_EVENTS_METADATA_READ);
#endif
#if NRF_KMU_HAS_PUSH_BLOCK
    nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_EVENTS_EVENTS_PUSHBLOCKED);
#endif
#if NRF_KMU_HAS_BLOCK
    nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_EVENTS_EVENTS_BLOCKED);
#endif
}

static int wait_for_task_result(nrf_kmu_event_t event)
{
    bool err;

    NRFX_WAIT_FOR(nrf_kmu_event_check(NRF_KMU, event) ||
                  nrf_kmu_event_check(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_REVOKED) ||
                  nrf_kmu_event_check(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_ERROR),
                  500, 10, err);

    if (!err)
    {
        return -ETIMEDOUT;
    }

    if (nrf_kmu_event_check(NRF_KMU, event))
    {
        clear_all_events();
        return 0;
    }
    else if (nrf_kmu_event_check(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_REVOKED))
    {
        clear_all_events();
        return -EACCES;
    }

    clear_all_events();

    return -EFAULT;
}

static int key_slot_empty_check(uint32_t slot_id, bool * is_empty)
{
    bool is_ready = false;

    NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

    if (!is_ready)
    {
        return -EAGAIN;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_READ_METADATA);

    int err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_METADATA_READ);

    if (err == -ETIMEDOUT)
    {
        return err;
    }
    else if (err == -EFAULT)
    {
        *is_empty = true;
        return 0;
    }

    *is_empty = false;
    return 0;
}

static bool key_slot_revoked_check(uint32_t slot_id, bool * is_revoked)
{
    bool is_ready = false;

    NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

    if (!is_ready)
    {
        return -EAGAIN;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_READ_METADATA);

    int err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_METADATA_READ);

    if (err == -ETIMEDOUT)
    {
        return err;
    }
    else if (err == -EACCES)
    {
        *is_revoked = true;
        return 0;
    }

    *is_revoked = false;
    return 0;
}

int nrfx_kmu_init(nrfx_kmu_event_handler_t* event_handler)
{
    (void)event_handler;

    if (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        int err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    clear_all_events();

    m_cb.state = NRFX_DRV_STATE_INITIALIZED;

    return 0;
}

void nrfx_kmu_uninit(void)
{
    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
}

int nrfx_kmu_key_slot_provision(nrfx_kmu_key_slot_data_t const * p_key_slot_data, uint32_t slot_id)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (p_key_slot_data) &&
                (slot_id < KMU_KEYSLOTNUM));

    bool is_ready = false;

    NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

    if (!is_ready)
    {
        return -EAGAIN;
    }

    nrf_kmu_src_set(NRF_KMU, (uint32_t)p_key_slot_data);
    nrf_kmu_keyslot_set(NRF_KMU, slot_id);
    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PROVISION_KEYSLOT);

    return wait_for_task_result(NRF_KMU_EVENT_EVENTS_PROVISIONED);
}

int nrfx_kmu_key_slots_push(uint32_t slot_id, uint32_t num_slots)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (slot_id < KMU_KEYSLOTNUM) &&
                (slot_id + num_slots - 1 < KMU_KEYSLOTNUM) &&
                (num_slots > 0));

    bool is_ready = false;

    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

        if (!is_ready)
        {
            return -EAGAIN;
        }

        nrf_kmu_keyslot_set(NRF_KMU, i);
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PUSH_KEYSLOT);

        int err = wait_for_task_result(NRF_KMU_EVENT_KEYSLOT_PUSHED);

        if (err < 0)
        {
            return err;
        }
    }

    return 0;
}

#if NRF_KMU_HAS_PUSH_BLOCK
int nrfx_kmu_key_slots_push_block(uint32_t slot_id, uint32_t num_slots)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (slot_id < KMU_KEYSLOTNUM) &&
                (slot_id + num_slots - 1 < KMU_KEYSLOTNUM) &&
                (num_slots > 0));

    bool is_ready = false;

    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

        if (!is_ready)
        {
            return -EAGAIN;
        }

        nrf_kmu_keyslot_set(NRF_KMU, i);
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PUSH_BLOCK);

        int err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_PUSHBLOCKED);

        if (err < 0)
        {
            return err;
        }
    }

    return 0;
}
#endif

#if NRF_KMU_HAS_BLOCK
int nrfx_kmu_key_slots_block(uint32_t slot_id, uint32_t num_slots)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (slot_id < KMU_KEYSLOTNUM) &&
                (slot_id + num_slots - 1 < KMU_KEYSLOTNUM) &&
                (num_slots > 0));

    bool is_ready = false;

    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

        if (!is_ready)
        {
            return -EAGAIN;
        }

        nrf_kmu_keyslot_set(NRF_KMU, i);
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_BLOCK);

        int err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_BLOCKED);

        if (err < 0)
        {
            return err;
        }
    }

    return 0;
}
#endif

int nrfx_kmu_key_slots_revoke(uint32_t slot_id, uint32_t num_slots)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (slot_id < KMU_KEYSLOTNUM) &&
                (slot_id + num_slots - 1 < KMU_KEYSLOTNUM) &&
                (num_slots > 0));

    bool is_ready = false;

    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

        if (!is_ready)
        {
            return -EAGAIN;
        }

        nrf_kmu_keyslot_set(NRF_KMU, i);
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_REVOKE_KEYSLOT);

        int err = wait_for_task_result(NRF_KMU_EVENT_KEYSLOT_REVOKED);

        if (err < 0)
        {
            return err;
        }
    }

    return 0;
}

#if NRF_KMU_HAS_METADATA
int nrfx_kmu_key_slot_metadata_read(uint32_t slot_id, nrfx_kmu_key_slot_metadata_t* p_metadata)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (slot_id < KMU_KEYSLOTNUM) &&
                (p_metadata));

    bool is_ready = false;

    NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

    if (!is_ready)
    {
        return -EAGAIN;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_READ_METADATA);

    int err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_METADATA_READ);

    if (err < 0)
    {
        return err;
    }

    p_metadata->metadata = nrf_kmu_metadata_get(NRF_KMU);
#if NRF_KMU_HAS_METADATAEXT
    p_metadata->metadata_ext = nrf_kmu_metadataext_get(NRF_KMU);
#endif

    return 0;
}
#endif

int nrfx_kmu_key_slots_empty_check(uint32_t slot_id, uint32_t num_slots, bool * p_is_empty)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (slot_id < KMU_KEYSLOTNUM) &&
                (slot_id + num_slots - 1 < KMU_KEYSLOTNUM) &&
                (num_slots > 0) &&
                p_is_empty);

    int err;
    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        err = key_slot_empty_check(i, p_is_empty);
        if (err != 0)
        {
            return err;
        }
        if (!*p_is_empty)
        {
            return 0;
        }

    }

    return 0;
}

int nrfx_kmu_key_slots_revoked_check(uint32_t slot_id, uint32_t num_slots, bool * p_is_revoked)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (slot_id < KMU_KEYSLOTNUM) &&
                (slot_id + num_slots - 1 < KMU_KEYSLOTNUM) &&
                (num_slots > 0) &&
                p_is_revoked);

    int err;
    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        err = key_slot_revoked_check(i, p_is_revoked);
        if (err != 0)
        {
            return err;
        }
        if (!*p_is_revoked)
        {
            return 0;
        }
    }

    return 0;
}

int nrfx_kmu_next_available_key_slots_find(uint32_t num_slots)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (num_slots > 0));

    uint32_t first_key_slot = 0;
    bool potential_key_slot = false;
    int err;
    bool slot_empty;

    for (uint32_t i = 0; i < KMU_KEYSLOTNUM; i++)
    {
        err = key_slot_empty_check(i, &slot_empty);
        if (err != 0)
        {
            return err;
        }
        if (slot_empty && !potential_key_slot)
        {
            potential_key_slot = true;
            first_key_slot = i;
        }
        if (potential_key_slot)
        {
            if (slot_empty && (i - first_key_slot + 1 == num_slots))
            {
                return (int)first_key_slot;
            }
            else if (!slot_empty)
            {
                potential_key_slot = false;
            }
        }
    }

    return -ENOMEM;
}
