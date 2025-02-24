/*
 * Copyright (c) 2015 - 2025, Nordic Semiconductor ASA
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

#ifndef NRFX_PDM_H__
#define NRFX_PDM_H__

#include <nrfx.h>
#include <haly/nrfy_pdm.h>

#ifdef __cplusplus
extern "C" {
#endif

/* On devices with single instance (with no ID) use instance 0. */
#if defined(NRF_PDM) && defined(NRFX_PDM_ENABLED) && !defined(NRFX_PDM0_ENABLED)
#define NRFX_PDM0_ENABLED 1
#endif

/**
 * @defgroup nrfx_pdm PDM driver
 * @{
 * @ingroup nrf_pdm
 * @brief   Pulse Density Modulation (PDM) peripheral driver.
 */

/** @brief Data structure of the Pulse Density Modulation (PDM) driver instance. */
typedef struct
{
    NRF_PDM_Type * p_reg;        ///< Pointer to a structure containing PDM registers.
    uint8_t        drv_inst_idx; ///< Index of the driver instance. For internal use only.
} nrfx_pdm_t;

#ifndef __NRFX_DOXYGEN__
enum {
    /* List all enabled driver instances (in the format NRFX_\<instance_name\>_INST_IDX). */
    NRFX_INSTANCE_ENUM_LIST(PDM)
    NRFX_PDM_ENABLED_COUNT
};
#endif

/** @brief Macro for creating an instance of the PDM driver. */
#define NRFX_PDM_INSTANCE(id)                             \
{                                                         \
    .p_reg        = NRFX_CONCAT(NRF_, PDM, id),           \
    .drv_inst_idx = NRFX_CONCAT(NRFX_PDM, id, _INST_IDX), \
}

/** @brief Maximum supported PDM buffer size. */
#define NRFX_PDM_MAX_BUFFER_SIZE 32767

/** @brief PDM error type. */
typedef enum
{
    NRFX_PDM_NO_ERROR = 0,      ///< No error.
    NRFX_PDM_ERROR_OVERFLOW = 1 ///< Overflow error.
} nrfx_pdm_error_t;

/** @brief PDM event structure. */
typedef struct
{
    bool             buffer_requested; ///< Buffer request flag.
    int16_t *        buffer_released;  ///< Pointer to the released buffer. Can be NULL.
    nrfx_pdm_error_t error;            ///< Error type.
} nrfx_pdm_evt_t;

/** @brief PDM interface driver configuration structure. */
typedef struct
{
    nrf_pdm_mode_t    mode;               ///< Interface operation mode.
    nrf_pdm_edge_t    edge;               ///< Sampling mode.
    uint32_t          clk_pin;            ///< CLK pin number.
    uint32_t          din_pin;            ///< DIN pin number.
#if NRF_PDM_HAS_PDMCLKCTRL
    nrf_pdm_freq_t    clock_freq;         ///< Clock frequency.
#elif NRF_PDM_HAS_PRESCALER
    uint32_t          prescaler;          ///< Prescaler divisor.
#endif
    nrf_pdm_gain_t    gain_l;             ///< Left channel gain.
    nrf_pdm_gain_t    gain_r;             ///< Right channel gain.
    uint8_t           interrupt_priority; ///< Interrupt priority.
#if NRF_PDM_HAS_RATIO_CONFIG
    nrf_pdm_ratio_t   ratio;              ///< Ratio between PDM_CLK and output sample rate.
#endif
#if NRF_PDM_HAS_SELECTABLE_CLOCK
    nrf_pdm_mclksrc_t mclksrc;            ///< Clock source selection.
#endif
    bool              skip_gpio_cfg;      ///< Skip GPIO configuration of pins.
                                          /**< When set to true, the driver does not modify
                                           *   any GPIO parameters of the used pins. Those
                                           *   parameters are supposed to be configured
                                           *   externally before the driver is initialized. */
    bool              skip_psel_cfg;      ///< Skip pin selection configuration.
                                          /**< When set to true, the driver does not modify
                                           *   pin select registers in the peripheral.
                                           *   Those registers are supposed to be set up
                                           *   externally before the driver is initialized.
                                           *   @note When both GPIO configuration and pin
                                           *   selection are to be skipped, the structure
                                           *   fields that specify pins can be omitted,
                                           *   as they are ignored anyway. */
} nrfx_pdm_config_t;

/**
 * @brief PDM driver default configuration.
 *
 * This configuration sets up PDM with the following options:
 * - mono mode
 * - data sampled on the clock falling edge
 * - frequency: 1.032 MHz
 * - standard gain
 *
 * @param[in] _pin_clk CLK output pin.
 * @param[in] _pin_din DIN input pin.
 */
#define NRFX_PDM_DEFAULT_CONFIG(_pin_clk, _pin_din)             \
{                                                               \
    .mode               = NRF_PDM_MODE_MONO,                    \
    .edge               = NRF_PDM_EDGE_LEFTFALLING,             \
    .clk_pin            = _pin_clk,                             \
    .din_pin            = _pin_din,                             \
    NRFX_COND_CODE_1(NRF_PDM_HAS_PDMCLKCTRL,                    \
                     (.clock_freq = NRF_PDM_FREQ_1032K,), ())   \
    NRFX_COND_CODE_1(NRF_PDM_HAS_PRESCALER,                     \
                     (.prescaler = 4,), ())                     \
    .gain_l             = NRF_PDM_GAIN_DEFAULT,                 \
    .gain_r             = NRF_PDM_GAIN_DEFAULT,                 \
    .interrupt_priority = NRFX_PDM_DEFAULT_CONFIG_IRQ_PRIORITY, \
    NRFX_COND_CODE_1(NRF_PDM_HAS_RATIO_CONFIG,                  \
                     (.ratio = NRF_PDM_RATIO_64X,), ())         \
    NRFX_COND_CODE_1(NRF_PDM_HAS_SELECTABLE_CLOCK,              \
                     (.mclksrc = NRF_PDM_MCLKSRC_PCLK32M,), ()) \
}

/**
 * @brief Macro returning PDM interrupt handler.
 *
 * @param[in] idx PDM index.
 *
 * @return Interrupt handler.
 */
#define NRFX_PDM_INST_HANDLER_GET(idx) NRFX_CONCAT_3(nrfx_pdm_, idx, _irq_handler)

/**
 * @brief Handler for the PDM interface ready events.
 *
 * This event handler is called on a buffer request, an error or when a buffer
 * is full and ready to be processed.
 *
 * @param[in] p_evt Pointer to the PDM event structure.
 */
typedef void (*nrfx_pdm_event_handler_t)(nrfx_pdm_evt_t const * p_evt);

#if NRFX_API_VER_AT_LEAST(3, 7, 0) || defined(__NRFX_DOXYGEN__)

/**
 * @brief Function for initializing the PDM driver instance.
 *
 * @param[in] p_instance    Pointer to the driver instance structure.
 * @param[in] p_config      Pointer to the structure with the initial configuration.
 * @param[in] event_handler Event handler provided by the user. Cannot be NULL.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_ALREADY       The driver is already initialized.
 * @retval NRFX_ERROR_INVALID_STATE The driver is already initialized.
 *                                  Deprecated - use @ref NRFX_ERROR_ALREADY instead.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration was specified.
 */
nrfx_err_t nrfx_pdm_init(nrfx_pdm_t const *        p_instance,
                         nrfx_pdm_config_t const * p_config,
                         nrfx_pdm_event_handler_t  event_handler);

/**
 * @brief Function for reconfiguring the PDM driver instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_config   Pointer to the structure with the configuration.
 *
 * @retval NRFX_SUCCESS             Reconfiguration was successful.
 * @retval NRFX_ERROR_BUSY          There is ongoing sampling and driver cannot be reconfigured.
 * @retval NRFX_ERROR_INVALID_STATE The driver is not initialized.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration was specified.
 */
nrfx_err_t nrfx_pdm_reconfigure(nrfx_pdm_t const *        p_instance,
                                nrfx_pdm_config_t const * p_config);

/**
 * @brief Function for uninitializing the PDM driver instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * This function stops PDM sampling, if it is in progress.
 */
void nrfx_pdm_uninit(nrfx_pdm_t const * p_instance);

/**
 * @brief Function for checking if the PDM driver instance is initialized.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  Interface is already initialized.
 * @retval false Interface is not initialized.
 */
bool nrfx_pdm_init_check(nrfx_pdm_t const * p_instance);

/**
 * @brief Function for getting the address of a PDM driver instance task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] task Task.
 *
 * @return Task address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_pdm_task_address_get(nrfx_pdm_t const * p_instance,
                                                      nrf_pdm_task_t     task);

/**
 * @brief Function for getting the state of the PDM drivers instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  The PDM interface is enabled.
 * @retval false The PDM interface is disabled.
 */
NRFX_STATIC_INLINE bool nrfx_pdm_enable_check(nrfx_pdm_t const * p_instance);

/**
 * @brief Function for starting the PDM sampling.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval NRFX_SUCCESS    Sampling was started successfully or was already in progress.
 * @retval NRFX_ERROR_BUSY Previous start/stop operation is in progress.
 */
nrfx_err_t nrfx_pdm_start(nrfx_pdm_t const * p_instance);

/**
 * @brief Function for stopping the PDM sampling.
 *
 * When this function is called, the PDM driver instance is stopped after finishing
 * the current frame.
 * The event handler function might be called once more after calling this function.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval NRFX_SUCCESS    Sampling was stopped successfully or was already stopped before.
 * @retval NRFX_ERROR_BUSY Previous start/stop operation is in progress.
 */
nrfx_err_t nrfx_pdm_stop(nrfx_pdm_t const * p_instance);

/**
 * @brief Function for supplying the sample buffer.
 *
 * Call this function after every buffer request event.
 *
 * @param[in] p_instance    Pointer to the driver instance structure.
 * @param[in] buffer        Pointer to the receive buffer. Cannot be NULL.
 * @param[in] buffer_length Length of the receive buffer in 16-bit words.
 *
 * @retval NRFX_SUCCESS             The buffer was applied successfully.
 * @retval NRFX_ERROR_BUSY          The buffer was already supplied or the peripheral is currently being stopped.
 * @retval NRFX_ERROR_INVALID_STATE The driver was not initialized.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid parameters were provided.
 */
nrfx_err_t nrfx_pdm_buffer_set(nrfx_pdm_t const * p_instance,
                               int16_t *          buffer,
                               uint16_t           buffer_length);

#ifndef NRFX_DECLARE_ONLY
NRFX_STATIC_INLINE uint32_t nrfx_pdm_task_address_get(nrfx_pdm_t const * p_instance,
                                                      nrf_pdm_task_t     task)
{
    return nrf_pdm_task_address_get(p_instance->p_reg, task);
}

NRFX_STATIC_INLINE bool nrfx_pdm_enable_check(nrfx_pdm_t const * p_instance)
{
    return nrf_pdm_enable_check(p_instance->p_reg);
}
#endif // NRFX_DECLARE_ONLY

#else

#if !defined(NRF_PDM_INDEX)
/* Choose the instance to use in case of using deprecated single-instance driver variant. */
#if defined(NRF_PDM130)
#define NRF_PDM_INDEX 130
#elif defined(NRF_PDM21)
#define NRF_PDM_INDEX 21
#elif defined(NRF_PDM20)
#define NRF_PDM_INDEX 20
#else
#define NRF_PDM_INDEX 0
#endif
#endif

/**
 * @brief Function for initializing the PDM interface.
 *
 * @param[in] p_config      Pointer to the structure with the initial configuration.
 * @param[in] event_handler Event handler provided by the user. Cannot be NULL.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_ALREADY       The driver is already initialized.
 * @retval NRFX_ERROR_INVALID_STATE The driver is already initialized.
 *                                  Deprecated - use @ref NRFX_ERROR_ALREADY instead.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration was specified.
 */
nrfx_err_t nrfx_pdm_init(nrfx_pdm_config_t const * p_config,
                         nrfx_pdm_event_handler_t  event_handler);

/**
 * @brief Function for reconfiguring the PDM interface.
 *
 * @param[in] p_config Pointer to the structure with the configuration.
 *
 * @retval NRFX_SUCCESS             Reconfiguration was successful.
 * @retval NRFX_ERROR_BUSY          There is ongoing sampling and driver cannot be reconfigured.
 * @retval NRFX_ERROR_INVALID_STATE The driver is not initialized.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration was specified.
 */
nrfx_err_t nrfx_pdm_reconfigure(nrfx_pdm_config_t const * p_config);

/**
 * @brief Function for uninitializing the PDM interface.
 *
 * This function stops PDM sampling, if it is in progress.
 */
void nrfx_pdm_uninit(void);

/**
 * @brief Function for checking if the PDM interface is initialized.
 *
 * @retval true  Interface is already initialized.
 * @retval false Interface is not initialized.
 */
bool nrfx_pdm_init_check(void);

/**
 * @brief Function for getting the address of a PDM interface task.
 *
 * @param[in] task Task.
 *
 * @return Task address.
 */
NRFX_STATIC_INLINE uint32_t nrfx_pdm_task_address_get(nrf_pdm_task_t task);

/**
 * @brief Function for getting the state of the PDM interface.
 *
 * @retval true  The PDM interface is enabled.
 * @retval false The PDM interface is disabled.
 */
NRFX_STATIC_INLINE bool nrfx_pdm_enable_check(void);

/**
 * @brief Function for starting the PDM sampling.
 *
 * @retval NRFX_SUCCESS    Sampling was started successfully or was already in progress.
 * @retval NRFX_ERROR_BUSY Previous start/stop operation is in progress.
 */
nrfx_err_t nrfx_pdm_start(void);

/**
 * @brief Function for stopping the PDM sampling.
 *
 * When this function is called, the PDM interface is stopped after finishing
 * the current frame.
 * The event handler function might be called once more after calling this function.
 *
 * @retval NRFX_SUCCESS    Sampling was stopped successfully or was already stopped before.
 * @retval NRFX_ERROR_BUSY Previous start/stop operation is in progress.
 */
nrfx_err_t nrfx_pdm_stop(void);

/**
 * @brief Function for supplying the sample buffer.
 *
 * Call this function after every buffer request event.
 *
 * @param[in] buffer        Pointer to the receive buffer. Cannot be NULL.
 * @param[in] buffer_length Length of the receive buffer in 16-bit words.
 *
 * @retval NRFX_SUCCESS             The buffer was applied successfully.
 * @retval NRFX_ERROR_BUSY          The buffer was already supplied or the peripheral is currently being stopped.
 * @retval NRFX_ERROR_INVALID_STATE The driver was not initialized.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid parameters were provided.
 */
nrfx_err_t nrfx_pdm_buffer_set(int16_t * buffer, uint16_t buffer_length);

#ifndef NRFX_DECLARE_ONLY
NRFX_STATIC_INLINE uint32_t nrfx_pdm_task_address_get(nrf_pdm_task_t task)
{
    return nrf_pdm_task_address_get(NRFX_CONCAT(NRF_PDM, NRF_PDM_INDEX), task);
}

NRFX_STATIC_INLINE bool nrfx_pdm_enable_check(void)
{
    return nrf_pdm_enable_check(NRFX_CONCAT(NRF_PDM, NRF_PDM_INDEX));
}
#endif // NRFX_DECLARE_ONLY

void nrfx_pdm_irq_handler(void);

#endif // NRFX_API_VER_AT_LEAST(3, 7, 0) || defined(__NRFX_DOXYGEN__)

/** @} */

/*
 * Declare interrupt handlers for all enabled driver instances in the following format:
 * nrfx_\<periph_name\>_\<idx\>_irq_handler (for example, nrfx_pdm_0_irq_handler).
 *
 * A specific interrupt handler for the driver instance can be retrieved by using
 * the NRFX_PDM_INST_HANDLER_GET macro.
 *
 * Here is a sample of using the NRFX_PDM_INST_HANDLER_GET macro to map an interrupt handler
 * in a Zephyr application:
 *
 * IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PDM_INST_GET(\<instance_index\>)), \<priority\>,
 *             NRFX_PDM_INST_HANDLER_GET(\<instance_index\>), 0, 0);
 */
NRFX_INSTANCE_IRQ_HANDLERS_DECLARE(PDM, pdm)

#ifdef __cplusplus
}
#endif

#endif // NRFX_PDM_H__
