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

#ifndef NRFX_GPIOTE_H__
#define NRFX_GPIOTE_H__

#include <nrfx.h>
#include <haly/nrfy_gpiote.h>
#include <haly/nrfy_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* On devices with single instance (with no ID) use instance 0. */
#if defined(NRF_GPIOTE) && defined(NRFX_GPIOTE_ENABLED) && !defined(NRFX_GPIOTE0_ENABLED)
#define NRFX_GPIOTE0_ENABLED 1
#endif

/**
 * @defgroup nrfx_gpiote GPIOTE driver
 * @{
 * @ingroup nrf_gpiote
 * @brief   GPIO Task Event (GPIOTE) peripheral driver.
 */

/** @brief Data structure of the GPIO tasks and events (GPIOTE) driver instance. */
typedef struct
{
    NRF_GPIOTE_Type * p_reg;        ///< Pointer to a structure containing GPIOTE registers.
    uint8_t           drv_inst_idx; ///< Index of the driver instance. For internal use only.
} nrfx_gpiote_t;

#ifndef __NRFX_DOXYGEN__
enum {
    /* List all enabled driver instances (in the format NRFX_\<instance_name\>_INST_IDX). */
    NRFX_INSTANCE_ENUM_LIST(GPIOTE)
    NRFX_GPIOTE_ENABLED_COUNT
};
#endif

/** @brief Macro for creating an instance of the GPIOTE driver. */
#define NRFX_GPIOTE_INSTANCE(id)                             \
{                                                            \
    .p_reg        = NRFX_CONCAT(NRF_, GPIOTE, id),           \
    .drv_inst_idx = NRFX_CONCAT(NRFX_GPIOTE, id, _INST_IDX), \
}

/** @brief Pin. */
typedef uint32_t nrfx_gpiote_pin_t;

/** @brief Triggering options. */
typedef enum
{
    NRFX_GPIOTE_TRIGGER_NONE,                                   ///< No trigger on a pin.
    NRFX_GPIOTE_TRIGGER_LOTOHI = GPIOTE_CONFIG_POLARITY_LoToHi, ///< Low to high edge trigger.
    NRFX_GPIOTE_TRIGGER_HITOLO,                                 ///< High to low edge trigger.
    NRFX_GPIOTE_TRIGGER_TOGGLE,                                 ///< Edge toggle trigger.
    NRFX_GPIOTE_TRIGGER_LOW,                                    ///< Level low trigger.
    NRFX_GPIOTE_TRIGGER_HIGH,                                   ///< Level high trigger.
    NRFX_GPIOTE_TRIGGER_MAX,                                    ///< Triggering options count.
} nrfx_gpiote_trigger_t;

/**
 * @brief Pin interrupt handler prototype.
 *
 * @param[in] pin       Pin that triggered this event.
 * @param[in] trigger   Trigger that led to this event.
 * @param[in] p_context User context.
 */
typedef void (*nrfx_gpiote_interrupt_handler_t)(nrfx_gpiote_pin_t     pin,
                                                nrfx_gpiote_trigger_t trigger,
                                                void *                p_context);

/** @brief Structure for configuring a GPIOTE task. */
typedef struct
{
    uint8_t               task_ch;  ///< GPIOTE channel to be used.
                                    /**< Set to value allocated using
                                     *   @ref nrfx_gpiote_channel_alloc. It is a user
                                     *   responsibility to free the channel. */
    nrf_gpiote_polarity_t polarity; ///< Task polarity configuration.
                                    /**< @ref NRF_GPIOTE_POLARITY_NONE is used
                                     *   to disable previously configured task. */
    nrf_gpiote_outinit_t  init_val; ///< Initial pin state.
} nrfx_gpiote_task_config_t;

/** @brief Structure for configuring an output pin. */
typedef struct
{
    nrf_gpio_pin_drive_t drive;         ///< Drive configuration.
    nrf_gpio_pin_input_t input_connect; ///< Input buffer connection.
    nrf_gpio_pin_pull_t  pull;          ///< Pull configuration.
                                        /**< Pull setting is used together with
                                         *   drive configurations D0 and D1. */
} nrfx_gpiote_output_config_t;

/** @brief Structure for configuring pin interrupt/event. */
typedef struct
{
    nrfx_gpiote_trigger_t trigger;      ///< Specify trigger.
    uint8_t const *       p_in_channel; ///< Pointer to GPIOTE channel for IN event.
                                        /**< If NULL, the sensing mechanism is used
                                         *   instead. Note that when channel is provided
                                         *   only edge triggering can be used. */
} nrfx_gpiote_trigger_config_t;

/** @brief Structure for configuring a pin interrupt handler. */
typedef struct
{
    nrfx_gpiote_interrupt_handler_t handler;   ///< User handler.
    void *                          p_context; ///< Context passed to the event handler.
} nrfx_gpiote_handler_config_t;

/** @brief @deprecated Structure for configuring an input pin. */
typedef struct
{
    nrf_gpio_pin_pull_t pull; ///< Pull configuration.
} nrfx_gpiote_input_config_t;

/** @brief Structure for configuring an input pin. */
typedef struct
{
    nrf_gpio_pin_pull_t          const * p_pull_config;    ///< Pull configuration. If NULL, the current configuration is untouched.
    nrfx_gpiote_trigger_config_t const * p_trigger_config; ///< Interrupt/event configuration. If NULL, the current configuration is untouched.
    nrfx_gpiote_handler_config_t const * p_handler_config; ///< Handler configuration. If NULL it is untouched.
} nrfx_gpiote_input_pin_config_t;

/** @brief Output pin default configuration. */
#define NRFX_GPIOTE_DEFAULT_OUTPUT_CONFIG           \
{                                                   \
    .drive = NRF_GPIO_PIN_S0S1,                     \
    .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT, \
    .pull = NRF_GPIO_PIN_NOPULL                     \
}

/** @brief Bitfield representing all GPIOTE channels available to the application
 *         for the specified GPIOTE instance.
 *
 * @param[in] idx GPIOTE instance index.
 */
#define NRFX_GPIOTE_APP_CHANNELS_MASK(idx) \
            (NRFX_BIT_MASK(NRFX_CONCAT_3(GPIOTE, idx, _CH_NUM)) & \
             ~(NRFX_CONCAT_3(NRFX_GPIOTE, idx, _CHANNELS_USED)))

/**
 * @brief Macro returning GPIOTE interrupt handler.
 *
 * @param[in] idx GPIOTE index.
 *
 * @return Interrupt handler.
 */
#define NRFX_GPIOTE_INST_HANDLER_GET(idx) NRFX_CONCAT_3(nrfx_gpiote_, idx, _irq_handler)

/**
 * @brief Function for checking if a GPIOTE input pin is set.
 *
 * @param[in] pin Pin.
 *
 * @retval true  The input pin is set.
 * @retval false The input pin is not set.
 */
bool nrfx_gpiote_in_is_set(nrfx_gpiote_pin_t pin);

#if NRFX_API_VER_AT_LEAST(3, 2, 0) || defined(__NRFX_DOXYGEN__)

/** @brief Input pin pull default configuration. */
#define NRFX_GPIOTE_DEFAULT_PULL_CONFIG NRF_GPIO_PIN_NOPULL

/**
 * @brief Function for initializing the GPIOTE driver instance.
 *
 * @param[in] p_instance         Pointer to the driver instance structure.
 * @param[in] interrupt_priority Interrupt priority.
 *
 * @retval NRFX_SUCCESS             Initialization was successful.
 * @retval NRFX_ERROR_ALREADY       The driver is already initialized.
 * @retval NRFX_ERROR_INVALID_STATE The driver is already initialized.
 *                                  Deprecated - use @ref NRFX_ERROR_ALREADY instead.
 */
nrfx_err_t nrfx_gpiote_init(nrfx_gpiote_t const * p_instance, uint8_t interrupt_priority);

/**
 * @brief Function for checking if the GPIOTE driver instance is initialized.
 *
 * @details The GPIOTE driver instance is a shared module. Therefore, check if
 * the module is already initialized and skip initialization if it is.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  The module is already initialized.
 * @retval false The module is not initialized.
 */
bool nrfx_gpiote_init_check(nrfx_gpiote_t const * p_instance);

/**
 * @brief Function for uninitializing the GPIOTE driver instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_gpiote_uninit(nrfx_gpiote_t const * p_instance);

/**
 * @brief Function for allocating a GPIOTE channel.
 * @details This function allocates the first unused GPIOTE channel from
 *          pool defined in @ref NRFX_GPIOTE_APP_CHANNELS_MASK.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_alloc.
 * @note Routines that allocate and free the GPIOTE channels are independent
 *       from the rest of the driver. In particular, the driver does not need
 *       to be initialized when this function is called.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 * @param[out] p_channel  Pointer to the GPIOTE channel that has been allocated.
 *
 * @retval NRFX_SUCCESS      The channel was successfully allocated.
 * @retval NRFX_ERROR_NO_MEM There is no available channel to be used.
 */
nrfx_err_t nrfx_gpiote_channel_alloc(nrfx_gpiote_t const * p_instance, uint8_t * p_channel);

/**
 * @brief Function for freeing a GPIOTE channel.
 * @details This function frees a GPIOTE channel that was allocated using
 *          @ref nrfx_gpiote_channel_alloc.
 *
 * @note Function is thread safe as it uses @ref nrfx_flag32_free.
 * @note Routines that allocate and free the GPIOTE channels are independent
 *       from the rest of the driver. In particular, the driver does not need
 *       to be initialized when this function is called.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] channel    GPIOTE channel to be freed.
 *
 * @retval NRFX_SUCCESS             The channel was successfully freed.
 * @retval NRFX_ERROR_INVALID_PARAM The channel is not user-configurable.
 */
nrfx_err_t nrfx_gpiote_channel_free(nrfx_gpiote_t const * p_instance, uint8_t channel);

/**
 * @brief Function for configuring the specified input pin and input event/interrupt.
 *
 * Prior to calling this function pin can be uninitialized or configured as input or
 * output. However, following transitions and configurations are invalid and result
 * in error returned by the function:
 *
 * - Setting level trigger (e.g. @ref NRFX_GPIOTE_TRIGGER_HIGH) and using GPIOTE
 *   channel for the same pin.
 * - Reconfiguring pin to input (@p p_config->p_pull_config not NULL) when pin was configured
 *   to use GPIOTE task. Prior to that, task must be disabled by configuring it with
 *   polarity set to @ref NRF_GPIOTE_POLARITY_NONE.
 * - Configuring trigger using GPIOTE channel for pin previously configured as output
 *   pin. Only sensing can be used for an output pin.
 *
 * Function can be used to configure trigger and handler for sensing input changes
 * on an output pin. In that case, prior to that output pin must be configured with
 * input buffer connected. In that case @p p_config->p_pull_config is NULL to avoid
 * reconfiguration of the pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin number.
 * @param[in] p_config   Pointer to the structure with input pin configuration.
 *
 * @retval NRFX_SUCCESS             Configuration was successful.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration.
 */
nrfx_err_t nrfx_gpiote_input_configure(nrfx_gpiote_t const *                  p_instance,
                                       nrfx_gpiote_pin_t                      pin,
                                       nrfx_gpiote_input_pin_config_t const * p_config);

/**
 * @brief Function for configuring the specified output pin to be used by the driver.
 *
 * Prior to calling this function pin can be uninitialized or configured as input or
 * output. However, following transitions and configurations are invalid and result
 * in error returned by the function:
 *
 * - Reconfiguring pin to output when pin was configured as input with trigger using
 *   GPIOTE channel. Prior to that, trigger must be disabled by configuring it as
 *   @ref NRFX_GPIOTE_TRIGGER_NONE.
 * - Configuring pin as output without input buffer connected when prior to that
 *   trigger was configured. In that case input buffer must be connected.
 * - Configuring GPIOTE task for pin which was previously configured as input. Before
 *   using GPIOTE task pin must be configured as output by providing @p p_config.
 *
 * @param[in] p_instance    Pointer to the driver instance structure.
 * @param[in] pin           Absolute pin number.
 * @param[in] p_config      Pin configuration. If NULL pin configuration is not applied.
 * @param[in] p_task_config GPIOTE task configuration. If NULL task is not used.
 *
 * @retval NRFX_SUCCESS             Configuration was successful.
 * @retval NRFX_ERROR_INVALID_PARAM Invalid configuration.
 */
nrfx_err_t nrfx_gpiote_output_configure(nrfx_gpiote_t const *               p_instance,
                                        nrfx_gpiote_pin_t                   pin,
                                        nrfx_gpiote_output_config_t const * p_config,
                                        nrfx_gpiote_task_config_t const *   p_task_config);

/**
 * @brief Function for deinitializing the specified pin.
 *
 * Specified pin and associated GPIOTE channel are restored to the default configuration.
 *
 * @warning GPIOTE channel used by the pin is not freed.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin number.
 *
 * @retval NRFX_SUCCESS             Uninitialization was successful.
 * @retval NRFX_ERROR_INVALID_PARAM Pin not used by the driver.
 */
nrfx_err_t nrfx_gpiote_pin_uninit(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for enabling trigger for the given pin.
 *
 * When GPIOTE event is used trigger can be enabled without enabling interrupt,
 * e.g. for PPI.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin number.
 * @param[in] int_enable True to enable the interrupt. Must be true when sensing is used.
 */
void nrfx_gpiote_trigger_enable(nrfx_gpiote_t const * p_instance,
                                nrfx_gpiote_pin_t     pin,
                                bool                  int_enable);

/**
 * @brief Function for disabling trigger for the given pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin number.
 */
void nrfx_gpiote_trigger_disable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Set global callback called for each event.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] handler    Global handler. Can be NULL.
 * @param[in] p_context  Context passed to the handler.
 */
void nrfx_gpiote_global_callback_set(nrfx_gpiote_t const *           p_instance,
                                     nrfx_gpiote_interrupt_handler_t handler,
                                     void *                          p_context);

/**
 * @brief Function for retrieving Task/Event channel index associated with the given pin.
 *
 * @param[in]  p_instance Pointer to the driver instance structure.
 * @param[in]  pin        Absolute pin number.
 * @param[out] p_channel  Location to write the channel index.
 *
 * @retval NRFX_SUCCESS             Channel successfully written.
 * @retval NRFX_ERROR_INVALID_PARAM Pin is not configured or not using Task or Event.
 */
nrfx_err_t nrfx_gpiote_channel_get(nrfx_gpiote_t const * p_instance,
                                   nrfx_gpiote_pin_t     pin,
                                   uint8_t *             p_channel);

/**
 * @brief Function for retrieving number of channels for specified GPIOTE instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @return Number of channels for specified GPIOTE instance.
 */
uint32_t nrfx_gpiote_channels_number_get(nrfx_gpiote_t const * p_instance);

/**
 * @brief Function for setting a GPIOTE output pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_out_set(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for clearing a GPIOTE output pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_out_clear(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for toggling a GPIOTE output pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_out_toggle(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for enabling a GPIOTE output pin task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_out_task_enable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for disabling a GPIOTE output pin task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_out_task_disable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for getting the OUT task for the specified output pin.
 *
 * @details The returned task identifier can be used within @ref nrf_gpiote_hal,
 *          for example, to configure a DPPI channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return OUT task associated with the specified output pin.
 */
nrf_gpiote_task_t nrfx_gpiote_out_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for getting the address of the OUT task for the specified output pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return Address of OUT task.
 */
uint32_t nrfx_gpiote_out_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

#if defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for getting the SET task for the specified output pin.
 *
 * @details The returned task identifier can be used within @ref nrf_gpiote_hal,
 *          for example, to configure a DPPI channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return SET task associated with the specified output pin.
 */
nrf_gpiote_task_t nrfx_gpiote_set_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for getting the address of the SET task for the specified output pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return Address of SET task.
 */
uint32_t nrfx_gpiote_set_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);
#endif // defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__NRFX_DOXYGEN__)

#if defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for getting the CLR task for the specified output pin.
 *
 * @details The returned task identifier can be used within @ref nrf_gpiote_hal,
 *          for example, to configure a DPPI channel.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return CLR task associated with the specified output pin.
 */
nrf_gpiote_task_t nrfx_gpiote_clr_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for getting the address of the SET task for the specified output pin.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return Address of CLR task.
 */
uint32_t nrfx_gpiote_clr_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__NRFX_DOXYGEN__)

/**
 * @brief Function for getting the GPIOTE event for the specified input pin.
 *
 * @details The returned event identifier can be used within @ref nrf_gpiote_hal,
 *          for example, to configure a DPPI channel.
 *          If the pin is configured to use low-accuracy mode, the PORT event
 *          is returned.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return Event associated with the specified input pin.
 */
nrf_gpiote_event_t nrfx_gpiote_in_event_get(nrfx_gpiote_t const * p_instance,
                                            nrfx_gpiote_pin_t     pin);

/**
 * @brief Function for getting the address of a GPIOTE input pin event.
 * @details If the pin is configured to use low-accuracy mode, the address of the PORT event is returned.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 *
 * @return Address of the specified input pin event.
 */
uint32_t nrfx_gpiote_in_event_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

/**
 * @brief Function for forcing a specific state on the pin configured as task.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 * @param[in] state      Pin state.
 */
void nrfx_gpiote_out_task_force(nrfx_gpiote_t const * p_instance,
                                nrfx_gpiote_pin_t     pin,
                                uint8_t               state);

/**
 * @brief Function for triggering the task OUT manually.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_out_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);

#if defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for triggering the task SET manually.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_set_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);
#endif // defined(GPIOTE_FEATURE_SET_PRESENT) || defined(__NRFX_DOXYGEN__)

#if defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for triggering the task CLR manually.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Pin number.
 */
void nrfx_gpiote_clr_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin);
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT) || defined(__NRFX_DOXYGEN__)

#if NRF_GPIOTE_HAS_LATENCY || defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for setting the latency mode for a given GPIOTE instance.
 *
 * @note Available for event mode with rising or falling edge detection on the pin.
 *       Toggle task mode can only be used with low latency mode setting.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] latency    Latency mode to be set.
 */
NRFX_STATIC_INLINE void nrfx_gpiote_latency_set(nrfx_gpiote_t const * p_instance,
                                                nrf_gpiote_latency_t  latency);

/**
 * @brief Function for getting the latency mode for a given GPIOTE instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @return Latency mode.
 */
NRFX_STATIC_INLINE nrf_gpiote_latency_t nrfx_gpiote_latency_get(nrfx_gpiote_t const * p_instance);
#endif

#ifndef NRFX_DECLARE_ONLY

#if NRF_GPIOTE_HAS_LATENCY
NRFX_STATIC_INLINE void nrfx_gpiote_latency_set(nrfx_gpiote_t const * p_instance,
                                                nrf_gpiote_latency_t  latency)
{
    nrfy_gpiote_latency_set(p_instance->p_reg, latency);
}

NRFX_STATIC_INLINE nrf_gpiote_latency_t nrfx_gpiote_latency_get(nrfx_gpiote_t const * p_instance)
{
    return nrfy_gpiote_latency_get(p_instance->p_reg);
}
#endif // NRF_GPIOTE_HAS_LATENCY

#endif // NRFX_DECLARE_ONLY

#else

#if !defined(NRF_GPIOTE_INDEX)
/* Choose the instance to use in case of using deprecated single-instance driver variant. */
#if defined(HALTIUM_XXAA)
#define NRF_GPIOTE_INDEX 130
#elif defined(LUMOS_XXAA)
#define NRF_GPIOTE_INDEX 20
#elif (defined(NRF5340_XXAA_APPLICATION) || defined(NRF91_SERIES)) && \
      defined(NRF_TRUSTZONE_NONSECURE)
#define NRF_GPIOTE_INDEX 1
#else
#define NRF_GPIOTE_INDEX 0
#endif
#endif

#if !defined(nrfx_gpiote_irq_handler)
#define nrfx_gpiote_irq_handler NRFX_CONCAT(nrfx_gpiote_, NRF_GPIOTE_INDEX, _irq_handler)
#endif

#define NRFX_GPIOTE_DEFAULT_INPUT_CONFIG \
{                                        \
    .pull = NRF_GPIO_PIN_NOPULL          \
}

nrfx_err_t nrfx_gpiote_init(uint8_t interrupt_priority);

bool nrfx_gpiote_is_init(void);

void nrfx_gpiote_uninit(void);

nrfx_err_t nrfx_gpiote_channel_alloc(uint8_t * p_channel);

nrfx_err_t nrfx_gpiote_channel_free(uint8_t channel);

nrfx_err_t nrfx_gpiote_input_configure(nrfx_gpiote_pin_t                    pin,
                                       nrfx_gpiote_input_config_t const *   p_input_config,
                                       nrfx_gpiote_trigger_config_t const * p_trigger_config,
                                       nrfx_gpiote_handler_config_t const * p_handler_config);

nrfx_err_t nrfx_gpiote_output_configure(nrfx_gpiote_pin_t                   pin,
                                        nrfx_gpiote_output_config_t const * p_config,
                                        nrfx_gpiote_task_config_t const *   p_task_config);

nrfx_err_t nrfx_gpiote_pin_uninit(nrfx_gpiote_pin_t pin);

void nrfx_gpiote_trigger_enable(nrfx_gpiote_pin_t pin, bool int_enable);

void nrfx_gpiote_trigger_disable(nrfx_gpiote_pin_t pin);

void nrfx_gpiote_global_callback_set(nrfx_gpiote_interrupt_handler_t handler,
                                     void *                          p_context);

nrfx_err_t nrfx_gpiote_channel_get(nrfx_gpiote_pin_t pin, uint8_t *p_channel);

void nrfx_gpiote_out_set(nrfx_gpiote_pin_t pin);

void nrfx_gpiote_out_clear(nrfx_gpiote_pin_t pin);

void nrfx_gpiote_out_toggle(nrfx_gpiote_pin_t pin);

void nrfx_gpiote_out_task_enable(nrfx_gpiote_pin_t pin);

void nrfx_gpiote_out_task_disable(nrfx_gpiote_pin_t pin);

nrf_gpiote_task_t nrfx_gpiote_out_task_get(nrfx_gpiote_pin_t pin);

uint32_t nrfx_gpiote_out_task_address_get(nrfx_gpiote_pin_t pin);

#if defined(GPIOTE_FEATURE_SET_PRESENT)
nrf_gpiote_task_t nrfx_gpiote_set_task_get(nrfx_gpiote_pin_t pin);

uint32_t nrfx_gpiote_set_task_address_get(nrfx_gpiote_pin_t pin);
#endif

#if defined(GPIOTE_FEATURE_CLR_PRESENT)
nrf_gpiote_task_t nrfx_gpiote_clr_task_get(nrfx_gpiote_pin_t pin);

uint32_t nrfx_gpiote_clr_task_address_get(nrfx_gpiote_pin_t pin);
#endif

nrf_gpiote_event_t nrfx_gpiote_in_event_get(nrfx_gpiote_pin_t pin);

uint32_t nrfx_gpiote_in_event_address_get(nrfx_gpiote_pin_t pin);

void nrfx_gpiote_out_task_force(nrfx_gpiote_pin_t pin, uint8_t state);

void nrfx_gpiote_out_task_trigger(nrfx_gpiote_pin_t pin);

#if defined(GPIOTE_FEATURE_SET_PRESENT)
void nrfx_gpiote_set_task_trigger(nrfx_gpiote_pin_t pin);
#endif

#if defined(GPIOTE_FEATURE_CLR_PRESENT)
void nrfx_gpiote_clr_task_trigger(nrfx_gpiote_pin_t pin);
#endif

#if NRF_GPIOTE_HAS_LATENCY
NRFX_STATIC_INLINE void nrfx_gpiote_latency_set(nrf_gpiote_latency_t latency);

NRFX_STATIC_INLINE nrf_gpiote_latency_t nrfx_gpiote_latency_get(void);
#endif

#ifndef NRFX_DECLARE_ONLY
#if NRF_GPIOTE_HAS_LATENCY
NRFX_STATIC_INLINE void nrfx_gpiote_latency_set(nrf_gpiote_latency_t latency)
{
    nrfy_gpiote_latency_set(NRFX_CONCAT(NRF_, GPIOTE, NRF_GPIOTE_INDEX), latency);
}

NRFX_STATIC_INLINE nrf_gpiote_latency_t nrfx_gpiote_latency_get(void)
{
    return nrfy_gpiote_latency_get(NRFX_CONCAT(NRF_, GPIOTE, NRF_GPIOTE_INDEX));
}
#endif // NRF_GPIOTE_HAS_LATENCY
#endif // NRFX_DECLARE_ONLY
#endif // NRFX_API_VER_AT_LEAST(3, 2, 0) || defined(__NRFX_DOXYGEN__)

/** @} */

/*
 * Declare interrupt handlers for all enabled driver instances in the following format:
 * nrfx_\<periph_name\>_\<idx\>_irq_handler (for example, nrfx_gpiote_0_irq_handler).
 *
 * A specific interrupt handler for the driver instance can be retrieved by using
 * the NRFX_GPIOTE_INST_HANDLER_GET macro.
 *
 * Here is a sample of using the NRFX_GPIOTE_INST_HANDLER_GET macro to map an interrupt handler
 * in a Zephyr application:
 *
 * IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_GPIOTE_INST_GET(\<instance_index\>)), \<priority\>,
 *             NRFX_GPIOTE_INST_HANDLER_GET(\<instance_index\>), 0, 0);
 */
NRFX_INSTANCE_IRQ_HANDLERS_DECLARE(GPIOTE, gpiote)

#ifdef __cplusplus
}
#endif

#endif // NRFX_GPIOTE_H__
