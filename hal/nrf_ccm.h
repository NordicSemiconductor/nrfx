/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
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

#ifndef NRF_CCM_H__
#define NRF_CCM_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrf_ccm_hal AES CCM encryption HAL
 * @{
 * @ingroup nrf_ccm
 * @brief   Hardware access layer for managing the AES CCM peripheral.
 */

/**
 * @brief CCM tasks.
 */
typedef enum
{
    /*lint -save -e30 -esym(628,__INTADDR__)*/
    NRF_CCM_TASK_KSGEN  = offsetof(NRF_CCM_Type, TASKS_KSGEN), /**< Task for starting generation of key-stream. */
    NRF_CCM_TASKS_CRYPT = offsetof(NRF_CCM_Type, TASKS_CRYPT), /**< Task for starting encryption/decrypt. */
    NRF_CCM_TASKS_STOP  = offsetof(NRF_CCM_Type, TASKS_STOP),  /**< Task for stopping encryption/decrypt. */
    /*lint -restore*/
} nrf_ccm_task_t;

/**
 * @brief CCM events.
 */
typedef enum
{
    /*lint -save -e30*/
    NRF_CCM_EVENT_ENDKSGEN = offsetof(NRF_CCM_Type, EVENTS_ENDKSGEN), /**< Keystream generation completed. */
    NRF_CCM_EVENT_ENDCRYPT = offsetof(NRF_CCM_Type, EVENTS_ENDCRYPT), /**< Encrypt/decrypt completed.*/
    NRF_CCM_EVENT_ERROR    = offsetof(NRF_CCM_Type, EVENTS_ERROR),    /**< CCM error event  */
    /*lint -restore*/
} nrf_ecb_event_t;

/**
 * @brief Function for activating a specific CCM task.
 *
 * @param[in] task  Task to activate.
 */
__STATIC_INLINE void nrf_ccm_task_trigger(nrf_ccm_task_t task);

/**
 * @brief Function for enabling specified interrupts.
 *
 * @param[in] mask  Interrupts to enable.
 */
__STATIC_INLINE void nrf_ccm_int_enable(uint32_t mask);

/**
 * @brief Function for disabling specified interrupts.
 *
 * @param[in] mask  Interrupts to disable.
 */
__STATIC_INLINE void nrf_ccm_int_disable(uint32_t mask);

#ifndef SUPPRESS_INLINE_IMPLEMENTATION

__STATIC_INLINE void nrf_ccm_task_trigger(nrf_ccm_task_t task)
{
    *((volatile uint32_t *)((uint8_t *)NRF_CCM + (uint32_t)task)) = 0x1UL;
}

__STATIC_INLINE void nrf_ccm_int_enable(uint32_t mask)
{
    NRF_CCM->INTENSET = mask;
}

__STATIC_INLINE void nrf_ccm_int_disable(uint32_t mask)
{
    NRF_CCM->INTENCLR = mask;
}

#endif // SUPPRESS_INLINE_IMPLEMENTATION

/** @} */

#ifdef __cplusplus
}
#endif

#endif  // NRF_CCM_H__
