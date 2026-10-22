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
#if NRFX_CHECK(KMU_USES_UICR)
#include <hal/nrf_uicr.h>
#include <hal/nrf_nvmc.h>
#endif

#define NRFX_LOG_MODULE KMU
#include <nrfx_log.h>

#if (defined(NRF53_SERIES) || defined(NRF91_SERIES)) && !NRFX_API_VER_AT_LEAST(4, 7, 0)
#error "KMU driver does not support nRF53 and nRF91 devices in API versions lower than 4.7.0"
#endif

/** @cond Driver internal data. */
typedef struct
{
    nrfx_drv_state_t         state;
#if NRF_KMU_HAS_INTEN
    nrfx_kmu_event_handler_t handler;
    uint32_t                 next_slot_to_push;
    uint32_t                 last_slot_to_push;
#endif
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

static int kmu_status_check(void)
{
#if NRF_KMU_HAS_STATUS_BLOCKED
    if (nrf_kmu_status_get(NRF_KMU) & NRF_KMU_STATUS_BLOCKED_MASK)
    {
        return -ECANCELED;
    }
#endif
#if NRF_KMU_HAS_STATUS_SELECTED
    if (nrf_kmu_status_get(NRF_KMU) & NRF_KMU_STATUS_SELECTED_MASK)
    {
        return -EAGAIN;
    }
#endif
#if NRF_KMU_HAS_STATUS_STATUS
    bool is_ready = false;

    NRFX_WAIT_FOR(nrf_kmu_status_get(NRF_KMU) == 0, 500, 10, is_ready);

    if (!is_ready)
    {
        return -EAGAIN;
    }
#endif

    return 0;
}

#if NRFX_CHECK(KMU_USES_UICR)
#if NRF_ERRATA_STATIC_CHECK(91, 7)
static bool uicr_errata_7_workaround_begin(void)
{
    bool irq_disabled = __get_PRIMASK() == 1;
    if (NRF_ERRATA_DYNAMIC_CHECK(91, 7) && !irq_disabled)
    {
        __disable_irq();
    }
    return irq_disabled;
}

static void uicr_errata_7_workaround_end(bool irq_disabled)
{
    if (NRF_ERRATA_DYNAMIC_CHECK(91, 7))
    {
        __DSB();
        if (!irq_disabled)
        {
            __enable_irq();
        }
    }
}
#endif

static void uicr_keyslot_config_dest_set(uint32_t slot_id, uint32_t dest)
{
    while (!nrf_nvmc_ready_check(NRF_NVMC));

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif
    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    nrf_uicr_keyslot_config_dest_set(NRF_UICR, slot_id - 1, dest);
#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif
}

static uint32_t uicr_keyslot_config_dest_get(uint32_t slot_id)
{
#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif
    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    uint32_t dest = nrf_uicr_keyslot_config_dest_get(NRF_UICR, slot_id - 1);

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif

    return dest;
}

static
void uicr_keyslot_config_perm_permissions_raw_set(uint32_t slot_id,
                                                  uint32_t perm)
{
    while (!nrf_nvmc_ready_check(NRF_NVMC));

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif

    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    nrf_uicr_keyslot_config_perm_permissions_set(NRF_UICR, slot_id - 1, perm);

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif
}

static uint32_t uicr_keyslot_config_perm_permissions_raw_get(uint32_t slot_id)
{
#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif

    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    uint32_t perm = nrf_uicr_keyslot_config_perm_permissions_get(NRF_UICR, slot_id - 1);

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif

    return perm;
}

static void uicr_keyslot_config_perm_state_set(uint32_t slot_id,
                                               nrf_uicr_keyslot_perm_state_t state)
{
    while (!nrf_nvmc_ready_check(NRF_NVMC));

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif

    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    nrf_uicr_keyslot_config_perm_state_set(NRF_UICR, slot_id - 1, state);

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif
}

static nrf_uicr_keyslot_perm_state_t uicr_keyslot_config_perm_state_get(uint32_t slot_id)
{
    nrf_uicr_keyslot_perm_state_t state;

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif

    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    state = nrf_uicr_keyslot_config_perm_state_get(NRF_UICR, slot_id - 1);

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif

    return state;
}

static void uicr_keyslot_key_value_set(uint32_t slot_id,
                                       uint32_t key_word_value,
                                       uint32_t key_word_indx)
{
    while (!nrf_nvmc_ready_check(NRF_NVMC));

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif
    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    nrf_uicr_keyslot_key_value_set(NRF_UICR, slot_id - 1, key_word_value, key_word_indx);

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif
}

static uint32_t uicr_keyslot_key_value_get(uint32_t slot_id, uint32_t key_word_indx)
{
    uint32_t key_word_val;
#if NRF_ERRATA_STATIC_CHECK(91, 7)
   bool irq_disabled = uicr_errata_7_workaround_begin();
#endif
    /* On older platforms KMU IDs are in range 1-128,
       but in UICR they are referenced by their index (ID-1). */
    key_word_val = nrf_uicr_keyslot_key_value_get(NRF_UICR, slot_id - 1, key_word_indx);

#if NRF_ERRATA_STATIC_CHECK(91, 7)
   uicr_errata_7_workaround_end(irq_disabled);
#endif
    return key_word_val;
}
#endif

static int key_slot_empty_check(uint32_t slot_id, bool * is_empty)
{
    int err = kmu_status_check();
    if (err != 0)
    {
        return err;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

#if NRF_KMU_HAS_METADATA
    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_READ_METADATA);

    err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_METADATA_READ);

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
#else
    uint32_t dest = uicr_keyslot_config_dest_get(slot_id);
    uint32_t perm = uicr_keyslot_config_perm_permissions_raw_get(slot_id);
    nrf_uicr_keyslot_perm_state_t state = uicr_keyslot_config_perm_state_get(slot_id);

    *is_empty = ((dest == UICR_KEYSLOT_DEST_DEFAULT_VAL) &&
                 (perm == UICR_PERM_HALFWORD_DEFAULT_VAL) &&
                 state == NRF_UICR_KEYSLOT_PERM_STATE_ACTIVE);

    nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);

    return 0;
#endif
}

static int key_slot_revoked_check(uint32_t slot_id, bool * is_revoked)
{
    int err = kmu_status_check();
    if (err != 0)
    {
        return err;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

#if NRF_KMU_HAS_METADATA
    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_READ_METADATA);

    err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_METADATA_READ);

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
#else
    nrf_uicr_keyslot_perm_state_t state = uicr_keyslot_config_perm_state_get(slot_id);

    nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);

    *is_revoked = (state == NRF_UICR_KEYSLOT_PERM_STATE_REVOKED);
    return 0;
#endif
}

#if NRFX_API_VER_AT_LEAST(4, 7, 0)
int nrfx_kmu_init(nrfx_kmu_event_handler_t event_handler, uint8_t irq_priority)
#else
int nrfx_kmu_init(nrfx_kmu_event_handler_t event_handler)
#endif
{
#if !NRF_KMU_HAS_INTEN
    (void)event_handler;
#if NRFX_API_VER_AT_LEAST(4, 7, 0)
    (void)irq_priority;
#endif
#endif

    if (m_cb.state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        int err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    clear_all_events();

#if NRF_KMU_HAS_INTEN && NRFX_API_VER_AT_LEAST(4, 7, 0)
    if (event_handler != NULL)
    {
        m_cb.handler = event_handler;
        nrf_kmu_int_enable(NRF_KMU, NRF_KMU_INT_ALL_MASK);
        NRFX_IRQ_PRIORITY_SET(KMU_IRQn, irq_priority);
        NRFX_IRQ_ENABLE(KMU_IRQn);
    }
#endif

    m_cb.state = NRFX_DRV_STATE_INITIALIZED;

    return 0;
}

void nrfx_kmu_uninit(void)
{
#if NRF_KMU_HAS_INTEN
    nrf_kmu_int_disable(NRF_KMU, NRF_KMU_INT_ALL_MASK);
#endif
    m_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
}

int nrfx_kmu_key_slot_provision(nrfx_kmu_key_slot_data_t const * p_key_slot_data, uint32_t slot_id)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
                (p_key_slot_data) &&
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX));

    int err = kmu_status_check();
    if (err != 0)
    {
        return err;
    }

    bool is_empty;
    err = key_slot_empty_check(slot_id, &is_empty);

    if (err != 0)
    {
        return err;
    }
    else if (!is_empty)
    {
        return -EALREADY;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

#if NRF_KMU_HAS_PROVISION
    nrf_kmu_src_set(NRF_KMU, (uint32_t)p_key_slot_data);
    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PROVISION_KEYSLOT);

    return wait_for_task_result(NRF_KMU_EVENT_EVENTS_PROVISIONED);
#else

    /* On some devices STATUS register is not updated after selecting a key slot.
       Trigger its update and wait until it is set to SELECTED. */
    uint32_t key_word_read = uicr_keyslot_key_value_get(slot_id, 0);
    NRFX_WAIT_FOR(kmu_status_check() == -EAGAIN, 500, 10, err);

    if (!err)
    {
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
        return -ETIMEDOUT;
    }

    /* On some devices writes to UICR may fail. Read back to assure the correct value was written. */
    uicr_keyslot_config_dest_set(slot_id, p_key_slot_data->keyslot_dest);
    uint32_t dest_read = uicr_keyslot_config_dest_get(slot_id);

    if (dest_read != p_key_slot_data->keyslot_dest)
    {
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
        return -EFAULT;
    }

    for (uint8_t i = 0; i < KEY_SLOT_WORDS_COUNT; i++)
    {
        uicr_keyslot_key_value_set(slot_id, p_key_slot_data->keyslot_value[i], i);
        key_word_read = uicr_keyslot_key_value_get(slot_id, i);

        if (key_word_read != p_key_slot_data->keyslot_value[i])
        {
            nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
            return -EFAULT;
        }
    }

    uint32_t perm = ((p_key_slot_data->permission.write_enable ? NRF_UICR_KEYSLOT_WRITE_ENABLE : 0) |
                     (p_key_slot_data->permission.read_enable  ? NRF_UICR_KEYSLOT_READ_ENABLE  : 0) |
                     (p_key_slot_data->permission.push_enable  ? NRF_UICR_KEYSLOT_PUSH_ENABLE  : 0));

    uicr_keyslot_config_perm_permissions_raw_set(slot_id, perm);
    uint32_t perm_read = uicr_keyslot_config_perm_permissions_raw_get(slot_id);

    if (perm != perm_read)
    {
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
        return -EFAULT;
    }

    nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);

    return 0;
#endif
}

#if NRF_KMU_HAS_READ_KEY_SLOT_KEY
int nrfx_kmu_key_slot_key_read(uint32_t slot_id, uint32_t * p_key_value)
{
    NRFX_ASSERT(m_cb.state == NRFX_DRV_STATE_INITIALIZED &&
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                p_key_value);

    int err = kmu_status_check();

    if (err != 0)
    {
        return err;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

    uint32_t perm = uicr_keyslot_config_perm_permissions_raw_get(slot_id);
    nrf_uicr_keyslot_perm_state_t state = uicr_keyslot_config_perm_state_get(slot_id);

    if (state == NRF_UICR_KEYSLOT_PERM_STATE_REVOKED)
    {
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
        return -ENODATA;
    }

    if ((perm & NRF_UICR_KEYSLOT_READ_ENABLE) == 0)
    {
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
        return -EACCES;
    }

    for (uint8_t i = 0; i < KEY_SLOT_WORDS_COUNT; i++)
    {
        p_key_value[i] = uicr_keyslot_key_value_get(slot_id, i);
    }

    nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);

    return 0;
}
#endif

int nrfx_kmu_key_slots_push(uint32_t slot_id, uint32_t num_slots)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                (slot_id + num_slots - 1 <= KMU_KEYSLOT_ID_MAX) &&
                (num_slots > 0));

#if NRF_KMU_HAS_INTEN
    if (m_cb.handler)
    {
        nrf_kmu_keyslot_set(NRF_KMU, slot_id);
        m_cb.last_slot_to_push = slot_id + num_slots - 1;
        m_cb.next_slot_to_push = slot_id + 1;
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PUSH_KEYSLOT);
    }
    else
#endif
    {
        for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
        {
            int err = kmu_status_check();
            if (err != 0)
            {
                NRFX_LOG_WARNING("Function: %s, error code: %s.",
                                __func__,
                                NRFX_LOG_ERROR_STRING_GET(err));
                return err;
            }

            nrf_kmu_keyslot_set(NRF_KMU, i);

            NRFX_LOG_INFO("Pushing key with ID: %u.\n", i);

            nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PUSH_KEYSLOT);

            err = wait_for_task_result(NRF_KMU_EVENT_KEYSLOT_PUSHED);

#if NRFX_CHECK(KMU_USES_UICR)
            nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
#endif
            if (err < 0)
            {
                NRFX_LOG_WARNING("Function: %s, error code: %s.",
                                __func__,
                                NRFX_LOG_ERROR_STRING_GET(err));
                return err;
            }
        }
    }

    return 0;
}

#if NRF_KMU_HAS_PUSH_BLOCK
int nrfx_kmu_key_slots_push_block(uint32_t slot_id, uint32_t num_slots)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                (slot_id + num_slots - 1 <= KMU_KEYSLOT_ID_MAX) &&
                (num_slots > 0));

    int err;

    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        err = kmu_status_check();
        if (err != 0)
        {
            return err;
        }


        nrf_kmu_keyslot_set(NRF_KMU, i);
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PUSH_BLOCK);

        err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_PUSHBLOCKED);

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
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                (slot_id + num_slots - 1 <= KMU_KEYSLOT_ID_MAX) &&
                (num_slots > 0));

    int err;

    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        err = kmu_status_check();
        if (err != 0)
        {
            return err;
        }


        nrf_kmu_keyslot_set(NRF_KMU, i);
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_BLOCK);

        err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_BLOCKED);

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
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                (slot_id + num_slots - 1 <= KMU_KEYSLOT_ID_MAX) &&
                (num_slots > 0));

    for (uint32_t i = slot_id; i < slot_id + num_slots; i++)
    {
        int err = kmu_status_check();
        if (err != 0)
        {
            return err;
        }

        nrf_kmu_keyslot_set(NRF_KMU, i);

#if NRF_KMU_HAS_REVOKE_TASK
        nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_REVOKE_KEYSLOT);

        err = wait_for_task_result(NRF_KMU_EVENT_KEYSLOT_REVOKED);

        if (err < 0)
        {
            return err;
        }
#else
        uicr_keyslot_config_perm_state_set(i,
                                           NRF_UICR_KEYSLOT_PERM_STATE_REVOKED);
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
#endif
    }
    return 0;
}

#if NRF_KMU_HAS_METADATA
int nrfx_kmu_key_slot_metadata_read(uint32_t slot_id, nrfx_kmu_key_slot_metadata_t* p_metadata)
{
    NRFX_ASSERT((m_cb.state == NRFX_DRV_STATE_INITIALIZED) &&
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                (p_metadata));

    int err = kmu_status_check();
    if (err != 0)
    {
        return err;
    }

    nrf_kmu_keyslot_set(NRF_KMU, slot_id);

    nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_READ_METADATA);

    err = wait_for_task_result(NRF_KMU_EVENT_EVENTS_EVENTS_METADATA_READ);

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
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                (slot_id + num_slots - 1 <= KMU_KEYSLOT_ID_MAX) &&
                (num_slots > 0) &&
                p_is_empty);

    int err;
    *p_is_empty = false;
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
#if KMU_KEYSLOT_ID_MIN > 0
                (slot_id >= KMU_KEYSLOT_ID_MIN) &&
#endif
                (slot_id <= KMU_KEYSLOT_ID_MAX) &&
                (slot_id + num_slots - 1 <= KMU_KEYSLOT_ID_MAX) &&
                (num_slots > 0) &&
                p_is_revoked);

    int err;
    *p_is_revoked = false;
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

    uint32_t first_key_slot = KMU_KEYSLOT_ID_MIN;
    bool potential_key_slot = false;
    int err;
    bool slot_empty;

    for (uint32_t i = KMU_KEYSLOT_ID_MIN; i <= KMU_KEYSLOT_ID_MAX; i++)
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

#if NRF_KMU_HAS_INTEN
void nrfx_kmu_irq_handler(void)
{
    NRFX_ASSERT(m_cb.handler);

    if (nrf_kmu_event_check(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_PUSHED))
    {
        nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_PUSHED);

        if (m_cb.next_slot_to_push <= m_cb.last_slot_to_push)
        {
            nrf_kmu_keyslot_set(NRF_KMU, m_cb.next_slot_to_push);
            nrf_kmu_task_trigger(NRF_KMU, NRF_KMU_TASK_PUSH_KEYSLOT);
            m_cb.next_slot_to_push++;
        }
        else
        {
            nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
        }

        m_cb.handler(NRF_KMU_EVENT_KEYSLOT_PUSHED);
    }
    else if (nrf_kmu_event_check(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_REVOKED))
    {
        nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_REVOKED);
        m_cb.handler(NRF_KMU_EVENT_KEYSLOT_REVOKED);
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
    }
    else if (nrf_kmu_event_check(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_ERROR))
    {
        nrf_kmu_event_clear(NRF_KMU, NRF_KMU_EVENT_KEYSLOT_ERROR);
        m_cb.handler(NRF_KMU_EVENT_KEYSLOT_ERROR);
        nrf_kmu_keyslot_set(NRF_KMU, KMU_SELECTKEYSLOT_IDLE_ID);
    }
}
#endif
