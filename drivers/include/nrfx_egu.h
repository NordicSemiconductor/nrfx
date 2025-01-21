/*
 * Copyright (c) 2019 - 2025, Nordic Semiconductor ASA
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

#ifndef NRFX_EGU_H__
#define NRFX_EGU_H__

#include <nrfx.h>
#include <hal/nrf_egu.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_egu EGU driver
 * @{
 * @ingroup  nrf_egu
 *
 * @brief    Event Generator Unit (EGU) peripheral driver.
 */

/** @brief Structure for the EGU driver instance. */
typedef struct
{
    NRF_EGU_Type * p_reg;        ///< Pointer to a structure with EGU registers.
    uint8_t        drv_inst_idx; ///< Index of the driver instance. For internal use only.
} nrfx_egu_t;

#ifndef __NRFX_DOXYGEN__
enum {
    /* List all enabled driver instances (in the format NRFX_\<instance_name\>_INST_IDX). */
    NRFX_INSTANCE_ENUM_LIST(EGU)
    NRFX_EGU_ENABLED_COUNT
};
#endif

/** @brief Macro for creating an EGU driver instance. */
#define NRFX_EGU_INSTANCE(id)                                 \
{                                                             \
    .p_reg        = NRFX_CONCAT(NRF_, EGU, id),               \
    .drv_inst_idx = NRFX_CONCAT(NRFX_EGU, id, _INST_IDX),     \
}

/**
 * @brief EGU driver event handler.
 *
 * @param[in] event_idx Index of the event that generated the interrupt.
 * @param[in] p_context Context passed to the event handler. Set on initialization.
 */
typedef void (*nrfx_egu_event_handler_t)(uint8_t event_idx, void * p_context);

/**
 * @brief Function for initializing the EGU driver instance.
 *
 * @param[in] p_instance         Pointer to the driver instance structure.
 * @param[in] interrupt_priority Interrupt priority.
 * @param[in] event_handler      Event handler provided by the user. In case of providing NULL,
 *                               event notifications are not done and EGU interrupts are disabled.
 * @param[in] p_context          Context passed to the event handler.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_ALREADY       The driver is already initialized.
 * @retval NRFX_ERROR_INVALID_STATE The driver is already initialized.
 *                                  Deprecated - use @ref NRFX_ERROR_ALREADY instead.
 */
nrfx_err_t nrfx_egu_init(nrfx_egu_t const *       p_instance,
                         uint8_t                  interrupt_priority,
                         nrfx_egu_event_handler_t event_handler,
                         void *                   p_context);

/**
 * @brief Function for enabling interrupts on specified events of a given EGU driver instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] mask       Mask of events with interrupts to be enabled.
 */
void nrfx_egu_int_enable(nrfx_egu_t const * p_instance, uint32_t mask);

/**
 * @brief Function for getting the address of the specified EGU task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] task       EGU task.
 *
 * @return Task address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_egu_task_address_get(nrfx_egu_t const * p_instance,
                                                      nrf_egu_task_t     task);

/**
 * @brief Function for getting the address of the specified EGU event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] event      EGU event.
 *
 * @return Event address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_egu_event_address_get(nrfx_egu_t const * p_instance,
                                                       nrf_egu_event_t    event);

/**
 * @brief Function for disabling interrupts on specified events of a given EGU driver instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] mask       Mask of events with interrupts to be disabled.
 */
void nrfx_egu_int_disable(nrfx_egu_t const * p_instance, uint32_t mask);

/**
 * @brief Function for triggering an event specified by @c event_idx of a given EGU driver instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] event_idx  Index of the event to be triggered.
 */
void nrfx_egu_trigger(nrfx_egu_t const * p_instance, uint8_t event_idx);

/**
 * @brief Function for uninitializing the EGU driver instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_egu_uninit(nrfx_egu_t const * p_instance);

/**
 * @brief Function for checking if the EGU driver instance is initialized.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  Instance is already initialized.
 * @retval false Instance is not initialized.
 */
bool nrfx_egu_init_check(nrfx_egu_t const * p_instance);

/**
 * @brief Macro returning EGU interrupt handler.
 *
 * param[in] idx EGU index.
 *
 * @return Interrupt handler.
 */
#define NRFX_EGU_INST_HANDLER_GET(idx) NRFX_CONCAT_3(nrfx_egu_, idx, _irq_handler)

#ifndef NRFX_DECLARE_ONLY
NRFX_STATIC_INLINE uint32_t nrfx_egu_task_address_get(nrfx_egu_t const * p_instance,
                                                      nrf_egu_task_t     task)
{
    return nrf_egu_task_address_get(p_instance->p_reg, task);
}

NRFX_STATIC_INLINE uint32_t nrfx_egu_event_address_get(nrfx_egu_t const * p_instance,
                                                       nrf_egu_event_t    event)
{
    return nrf_egu_event_address_get(p_instance->p_reg, event);
}
#endif // NRFX_DECLARE_ONLY

/** @} */

/*
 * Declare interrupt handlers for all enabled driver instances in the following format:
 * nrfx_\<periph_name\>_\<idx\>_irq_handler (for example, nrfx_egu_0_irq_handler).
 *
 * A specific interrupt handler for the driver instance can be retrieved by using
 * the NRFX_EGU_INST_HANDLER_GET macro.
 *
 * Here is a sample of using the NRFX_EGU_INST_HANDLER_GET macro to map an interrupt handler
 * in a Zephyr application:
 *
 * IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_EGU_INST_GET(\<instance_index\>)), \<priority\>,
 *             NRFX_EGU_INST_HANDLER_GET(\<instance_index\>), 0, 0);
 */
NRFX_INSTANCE_IRQ_HANDLERS_DECLARE(EGU, egu)

#ifdef __cplusplus
}
#endif

#endif // NRFX_EGU_H__
