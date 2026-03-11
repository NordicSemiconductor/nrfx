/*
 * Copyright (c) 2022 - 2026, Nordic Semiconductor ASA
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

#ifndef NRF_VREGUSB_H__
#define NRF_VREGUSB_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrf_vregusb_hal USB Voltage Regulator HAL
 * @{
 * @ingroup nrf_power
 * @brief   Hardware access layer for managing the USB Voltage Regulator.
 */

#if defined(VREGUSB_EVENTS_VBUSDETECTEDRAW_EVENTS_VBUSDETECTEDRAW_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of the VBUSDETECTEDRAW event. */
#define NRF_VREGUSB_HAS_EVENT_VBUSDETECTEDRAW 1
#else
#define NRF_VREGUSB_HAS_EVENT_VBUSDETECTEDRAW 0
#endif

#if defined(VREGUSB_STATUS_READY_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of STATUS register. */
#define NRF_VREGUSB_HAS_STATUS 1
#else
#define NRF_VREGUSB_HAS_STATUS 0
#endif

#if defined(VREGUSB_CONFIG_RDYTIM_VAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of CONFIG register. */
#define NRF_VREGUSB_HAS_CONFIG 1
#else
#define NRF_VREGUSB_HAS_CONFIG 0
#endif

#if defined(VREGUSB_MIRROR_LOCK_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of MIRROR register. */
#define NRF_VREGUSB_HAS_MIRROR 1
#else
#define NRF_VREGUSB_HAS_MIRROR 0
#endif

#if defined(VREGUSB_OVERRIDE_VBUSDETRAW_VAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of OVERRIDE register. */
#define NRF_VREGUSB_HAS_OVERRIDE 1
#else
#define NRF_VREGUSB_HAS_OVERRIDE 0
#endif

#if defined(VREGUSB_OVERRIDE_SETTLED0V8_VAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of voltage settling feature. */
#define NRF_VREGUSB_HAS_SETTLED 1
#else
#define NRF_VREGUSB_HAS_SETTLED 0
#endif

#if defined(VREGUSB_PWRUP_MANUAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of power up feature. */
#define NRF_VREGUSB_HAS_PWRUP 1
#else
#define NRF_VREGUSB_HAS_PWRUP 0
#endif

#if defined(VREGUSB_TRIM_TRIMVDD_VAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of VDD trimming. */
#define NRF_VREGUSB_HAS_TRIM_VDD 1
#else
#define NRF_VREGUSB_HAS_TRIM_VDD 0
#endif

#if defined(VREGUSB_TRIM_TRIMIBP_VAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of IBP trimming. */
#define NRF_VREGUSB_HAS_TRIM_IBP 1
#else
#define NRF_VREGUSB_HAS_TRIM_IBP 0
#endif

#if defined(VREGUSB_TRIM_TRIM0V8_VAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of 0V8 trimming. */
#define NRF_VREGUSB_HAS_TRIM_0V8 1
#else
#define NRF_VREGUSB_HAS_TRIM_0V8 0
#endif

#if defined(VREGUSB_TRIM_TRIM3V3_VAL_Msk) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol indicating presence of 0V8 trimming. */
#define NRF_VREGUSB_HAS_TRIM_3V3 1
#else
#define NRF_VREGUSB_HAS_TRIM_3V3 0
#endif


/** @brief VREGUSB tasks. */
typedef enum
{
    NRF_VREGUSB_TASK_START = offsetof(NRF_VREGUSB_Type, TASKS_START), /**< Request power up of USB PM. */
    NRF_VREGUSB_TASK_STOP  = offsetof(NRF_VREGUSB_Type, TASKS_STOP),  /**< Stop requesting power up for USB PM. */
} nrf_vregusb_task_t;

/** @brief VREGUSB events. */
typedef enum
{
#if NRF_VREGUSB_HAS_EVENT_VBUSDETECTEDRAW
    NRF_VREGUSB_EVENT_VBUS_DETECTED_RAW = offsetof(NRF_VREGUSB_Type, EVENTS_VBUSDETECTEDRAW), /**< VBUS detected. */
#endif
    NRF_VREGUSB_EVENT_VBUS_DETECTED     = offsetof(NRF_VREGUSB_Type, EVENTS_VBUSDETECTED),    /**< VBUS detected. */
#if NRF_VREGUSB_HAS_SETTLED
    NRF_VREGUSB_EVENT_SETTLED_0V8       = offsetof(NRF_VREGUSB_Type, EVENTS_SETTLED0V8),      /**< 0v8 settled. */
    NRF_VREGUSB_EVENT_SETTLED_3V3       = offsetof(NRF_VREGUSB_Type, EVENTS_SETTLED3V3),      /**< 3v3 settled. */
#endif
    NRF_VREGUSB_EVENT_VBUS_REMOVED      = offsetof(NRF_VREGUSB_Type, EVENTS_VBUSREMOVED),     /**< VBUS removed. */
} nrf_vregusb_event_t;

/** @brief VREGUSB interrupts. */
typedef enum
{
#if NRF_VREGUSB_HAS_EVENT_VBUSDETECTEDRAW
    NRF_VREGUSB_INT_VBUS_DETECTED_RAW_MASK = VREGUSB_INTEN_VBUSDETECTEDRAW_Msk, /**< Interrupt on VBUS detected. */
#endif
    NRF_VREGUSB_INT_VBUS_DETECTED_MASK     = VREGUSB_INTEN_VBUSDETECTED_Msk,    /**< Interrupt on VBUS detected. */
#if NRF_VREGUSB_HAS_SETTLED
    NRF_VREGUSB_INT_SETTLED_0V8_MASK       = VREGUSB_INTEN_SETTLED0V8_Msk,      /**< Interrupt on 0v8 settled. */
    NRF_VREGUSB_INT_SETTLED_3V3_MASK       = VREGUSB_INTEN_SETTLED3V3_Msk,      /**< Interrupt on 3v3 settled. */
#endif
    NRF_VREGUSB_INT_VBUS_REMOVED_MASK      = VREGUSB_INTEN_VBUSREMOVED_Msk,     /**< Interrupt on VBUS removed. */
} nrf_vregusb_int_mask_t;

#if NRF_VREGUSB_HAS_STATUS
/** @brief STATUS register states. */
typedef enum
{
    NRF_VREGUSB_STATUS_READY_MASK        = VREGUSB_STATUS_READY_Msk,      /**< USB PM powered up. */
#if NRF_VREGUSB_HAS_EVENT_VBUSDETECTEDRAW
    NRF_VREGUSB_STATUS_VBUS_DETRAW_MASK  = VREGUSB_STATUS_VBUSDETRAW_Msk, /**< VBUS detected, raw signal. */
#endif
    NRF_VREGUSB_STATUS_VBUS_DET_MASK     = VREGUSB_STATUS_VBUSDET_Msk,    /**< VBUS detected. */
#if NRF_VREGUSB_HAS_SETTLED
    NRF_VREGUSB_STATUS_READY_0V8_MASK    = VREGUSB_STATUS_READY0V8_Msk,   /**< Power 0v8 ready. */
    NRF_VREGUSB_STATUS_SETTLED_0V8_MASK  = VREGUSB_STATUS_SETTLED0V8_Msk, /**< Power 0v8 settled. */
    NRF_VREGUSB_STATUS_READY_3V3_MASK    = VREGUSB_STATUS_READY3V3_Msk,   /**< Power 3v3 ready. */
    NRF_VREGUSB_STATUS_SETTLED_3V3_MASK  = VREGUSB_STATUS_SETTLED3V3_Msk, /**< Power 3v3 settled. */
#endif
} nrf_vregusb_status_mask_t;
#endif

#if NRF_VREGUSB_HAS_PWRUP
/** @brief Power up modes. */
typedef enum
{
    NRF_VREGUSB_PWRUP_MODE_AUTO       = VREGUSB_PWRUP_AUTOMODE_Auto << VREGUSB_PWRUP_AUTOMODE_Pos,   /**< USB PM automatically powered. */
    NRF_VREGUSB_PWRUP_MODE_MANUAL_ON  = (VREGUSB_PWRUP_MANUAL_On << VREGUSB_PWRUP_MANUAL_Pos)  |
                                      (VREGUSB_PWRUP_AUTOMODE_Manual << VREGUSB_PWRUP_AUTOMODE_Pos), /**< USB PM manually powered on. */
    NRF_VREGUSB_PWRUP_MODE_MANUAL_OFF = (VREGUSB_PWRUP_MANUAL_Off << VREGUSB_PWRUP_MANUAL_Pos) |
                                      (VREGUSB_PWRUP_AUTOMODE_Manual << VREGUSB_PWRUP_AUTOMODE_Pos), /**< USB PM manually powered off. */
} nrf_vregusb_pwrup_mode_t;

/** @brief Power up target. */
typedef enum
{
    NRF_VREGUSB_PWRUP_TARGET_USB_PM, /**< Power up USB PM. */
    NRF_VREGUSB_PWRUP_TARGET_0V8,    /**< Power up 0v8. */
    NRF_VREGUSB_PWRUP_TARGET_3V3,    /**< Power up 3v3. */
} nrf_vregusb_pwrup_target_t;
#endif // NRF_VREGUSB_HAS_PWRUP

/** @brief Analog signal to be overridden. */
typedef enum
{
#if NRF_VREGUSB_HAS_EVENT_VBUSDETECTEDRAW
    NRF_VREGUSB_ANALOG_SIGNAL_VBUS_DETRAW, /**< Override VBUSDETRAW signal. */
#endif
    NRF_VREGUSB_ANALOG_SIGNAL_VBUS_DET,    /**< Override VBUSDET signal. */
#if NRF_VREGUSB_HAS_SETTLED
    NRF_VREGUSB_ANALOG_SIGNAL_SETTLED_0V8, /**< Override SETTLED0V8 signal. */
    NRF_VREGUSB_ANALOG_SIGNAL_SETTLED_3V3, /**< Override SETTLED3V3 signal. */
#endif
} nrf_vregusb_analog_signal_t;

/**
 * @brief Function for activating the specified VREGUSB task.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] task  Task to be activated.
 */
NRF_STATIC_INLINE void nrf_vregusb_task_trigger(NRF_VREGUSB_Type * p_reg,
                                                nrf_vregusb_task_t task);

/**
 * @brief Function for getting the address of the specified VREGUSB task register.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] task  The specified task.
 *
 * @return Address of the specified task register.
 */
NRF_STATIC_INLINE uint32_t nrf_vregusb_task_address_get(NRF_VREGUSB_Type const * p_reg,
                                                        nrf_vregusb_task_t       task);

/**
 * @brief Function for clearing the specified VREGUSB event.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] event Event to be cleared.
 */
NRF_STATIC_INLINE void nrf_vregusb_event_clear(NRF_VREGUSB_Type *  p_reg,
                                               nrf_vregusb_event_t event);

/**
 * @brief Function for retrieving the state of the VREGUSB event.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] event Event to be checked.
 *
 * @retval true  The event has been generated.
 * @retval false The event has not been generated.
 */
NRF_STATIC_INLINE bool nrf_vregusb_event_check(NRF_VREGUSB_Type const * p_reg,
                                               nrf_vregusb_event_t      event);

/**
 * @brief Function for getting the address of the specified event register.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] event The specified event.
 *
 * @return Address of the specified event register.
 */
NRF_STATIC_INLINE uint32_t nrf_vregusb_event_address_get(NRF_VREGUSB_Type const * p_reg,
                                                         nrf_vregusb_event_t      event);
/**
 * @brief Function for enabling specified interrupts.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] mask  Mask of interrupts to be enabled.
 *                  Use @ref nrf_vregusb_int_mask_t values for bit masking.
 */
NRF_STATIC_INLINE void nrf_vregusb_int_enable(NRF_VREGUSB_Type * p_reg, uint32_t mask);

/**
 * @brief Function for disabling specified interrupts.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] mask  Mask of interrupts to be disabled.
 *                  Use @ref nrf_vregusb_int_mask_t values for bit masking.
 */
NRF_STATIC_INLINE void nrf_vregusb_int_disable(NRF_VREGUSB_Type * p_reg, uint32_t mask);

/**
 * @brief Function for checking if the specified interrupts are enabled.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] mask  Mask of interrupts to be checked.
 *                  Use @ref nrf_vregusb_int_mask_t values for bit masking.
 *
 * @return Mask of enabled interrupts.
 */
NRF_STATIC_INLINE uint32_t nrf_vregusb_int_enable_check(NRF_VREGUSB_Type const * p_reg,
                                                        uint32_t                 mask);

/**
 * @brief Function for retrieving the state of pending interrupts.
 *
 * @note States of pending interrupt are saved as a bitmask.
 *       One set at particular position means that interrupt for event is pending.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Bitmask with information about pending interrupts.
 *         Use @ref nrf_vregusb_int_mask_t values for bit masking.
 */
NRF_STATIC_INLINE uint32_t nrf_vregusb_int_pending_get(NRF_VREGUSB_Type const * p_reg);

#if NRF_VREGUSB_HAS_STATUS
/**
 * @brief Function for getting VREGUSB status register.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return The STATUS register value.
 *         Use @ref nrf_vregusb_status_mask_t values for bit masking.
 */
NRF_STATIC_INLINE uint32_t nrf_vregusb_status_get(NRF_VREGUSB_Type const * p_reg);
#endif

#if NRF_VREGUSB_HAS_CONFIG
/**
 * @brief Function for setting time from PM power up to ready signal.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   From 0 (0us) to 48 (3us).
 */
NRF_STATIC_INLINE void nrf_vregusb_config_rdy_tim_set(NRF_VREGUSB_Type * p_reg, uint8_t val);

/**
 * @brief Function for getting time from PM power up to ready signal.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Time value.
 */
NRF_STATIC_INLINE uint8_t nrf_vregusb_config_rdy_tim_get(NRF_VREGUSB_Type const * p_reg);

/**
 * @brief Function for setting time from VBUSDETRAW until filtered VBUSDET.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   From 0 (0us) to 8000 (500us).
 */
NRF_STATIC_INLINE void nrf_vregusb_config_vbrft_set(NRF_VREGUSB_Type * p_reg, uint16_t val);

/**
 * @brief Function for getting time from VBUSDETRAW until filtered VBUSDET.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Time value.
 */
NRF_STATIC_INLINE uint16_t nrf_vregusb_config_vbrft_get(NRF_VREGUSB_Type const * p_reg);

/**
 * @brief Function for setting time from power up LDOs until ready.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   From 0 (0us) to 160 (10us).
 */
NRF_STATIC_INLINE void nrf_vregusb_config_rdy_ldo_stim_set(NRF_VREGUSB_Type * p_reg, uint8_t val);

/**
 * @brief Function for getting time from power up LDOs until ready.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Time value.
 */
NRF_STATIC_INLINE uint8_t nrf_vregusb_config_rdy_ldo_stim_get(NRF_VREGUSB_Type const * p_reg);

#if NRF_VREGUSB_HAS_SETTLED
/**
 * @brief Function for setting time from LDOs ready to SETTLED0v8.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   From 0 (0us) to 16000 (1000us).
 */
NRF_STATIC_INLINE void nrf_vregusb_config_setl_0v8_tim_set(NRF_VREGUSB_Type * p_reg, uint16_t val);

/**
 * @brief Function for getting time from LDOs ready to SETTLED0v8.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Time value.
 */
NRF_STATIC_INLINE uint16_t nrf_vregusb_config_setl_0v8_tim_get(NRF_VREGUSB_Type const * p_reg);

/**
 * @brief Function for setting time from SETTLE0v8 to SETTLED3v3.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   From 0 (0us) to 16000 (1000us).
 */
NRF_STATIC_INLINE void nrf_vregusb_config_setl_3v3_tim_set(NRF_VREGUSB_Type * p_reg, uint16_t val);

/**
 * @brief Function for getting time from SETTLE0v8 to SETTLED3v3.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Time value.
 */
NRF_STATIC_INLINE uint16_t nrf_vregusb_config_setl_3v3_tim_get(NRF_VREGUSB_Type const * p_reg);
#endif // NRF_VREGUSB_HAS_SETTLED
#endif // NRF_VREGUSB_HAS_CONFIG

#if NRF_VREGUSB_HAS_TRIM_VDD
/**
 * @brief Function for setting USB PM internal 1.8V trim.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   Current reference trimming value. The value is 35.2mV - 4.4mV steps.
 */
NRF_STATIC_INLINE void nrf_vregusb_trim_vdd_set(NRF_VREGUSB_Type * p_reg, uint8_t val);

/**
 * @brief Function for getting USB PM internal 1.8V trim.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Trimming value.
 */
NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_vdd_get(NRF_VREGUSB_Type const * p_reg);
#endif

#if NRF_VREGUSB_HAS_TRIM_IBP
/**
 * @brief Function for setting 10uA IBP bias current trim.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   Trimming value. The value is -2.28uA + 0.285uA steps.
 */
NRF_STATIC_INLINE void nrf_vregusb_trim_ibp_set(NRF_VREGUSB_Type * p_reg, uint8_t val);

/**
 * @brief Function for getting 10uA IBP bias current trim.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Trimming value.
 */
NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_ibp_get(NRF_VREGUSB_Type const * p_reg);
#endif

#if NRF_VREGUSB_HAS_TRIM_0V8
/**
 * @brief Function for setting USB PHY 0.8V DVDD trimming value.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   Voltage offset trimming value. The value is -160mV + 20mV steps.
 */
NRF_STATIC_INLINE void nrf_vregusb_trim_0v8_set(NRF_VREGUSB_Type * p_reg, uint8_t val);

/**
 * @brief Function for getting USB PHY 0.8V DVDD trimming value.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Voltage offset trimming value.
 */
NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_0v8_get(NRF_VREGUSB_Type const * p_reg);
#endif

#if NRF_VREGUSB_HAS_TRIM_3V3
/**
 * @brief Function for setting USB PHY 3.3V VDD trimming value.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] val   Voltage offset trimming value. The value is -528mV + 66mV steps.
 */
NRF_STATIC_INLINE void nrf_vregusb_trim_3v3_set(NRF_VREGUSB_Type * p_reg, uint8_t val);

/**
 * @brief Function for getting USB PHY 3.3V VDD trimming value.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Voltage offset trimming value.
 */
NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_3v3_get(NRF_VREGUSB_Type const * p_reg);
#endif

#if NRF_VREGUSB_HAS_MIRROR
/**
 * @brief Function for locking mirrored registers.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] lock  When true lock is enabled. When false lock is disabled and mirrored
 *                  registers can be updated at any time.
 */
NRF_STATIC_INLINE void nrf_vregusb_mirror_lock_set(NRF_VREGUSB_Type * p_reg, bool lock);
#endif

#if NRF_VREGUSB_HAS_PWRUP
/**
 * @brief Function for setting power up target configuration.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] target Power up target.
 * @param[in] mode   Mode configuration.
 */
NRF_STATIC_INLINE void nrf_vregusb_pwrup_set(NRF_VREGUSB_Type *         p_reg,
                                             nrf_vregusb_pwrup_target_t target,
                                             nrf_vregusb_pwrup_mode_t   mode);
#endif

#if NRF_VREGUSB_HAS_OVERRIDE
/**
 * @brief Function for override signal to the analog module.
 *
 * @param[in] p_reg  Pointer to the structure of registers of the peripheral.
 * @param[in] signal Signal to be overridden.
 * @param[in] enable True to enable override and false to disable.
 * @param[in] val    Override value.
 */
NRF_STATIC_INLINE void nrf_vregusb_override_set(NRF_VREGUSB_Type *          p_reg,
                                                nrf_vregusb_analog_signal_t signal,
                                                bool                        enable,
                                                uint8_t                     val);
#endif

#ifndef NRF_DECLARE_ONLY

NRF_STATIC_INLINE void nrf_vregusb_task_trigger(NRF_VREGUSB_Type * p_reg,
                                                nrf_vregusb_task_t task)
{
    *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)task)) = 0x1UL;
}

NRF_STATIC_INLINE uint32_t nrf_vregusb_task_address_get(NRF_VREGUSB_Type const * p_reg,
                                                        nrf_vregusb_task_t       task)
{
    return nrf_task_event_address_get(p_reg, task);
}

NRF_STATIC_INLINE void nrf_vregusb_event_clear(NRF_VREGUSB_Type *  p_reg,
                                               nrf_vregusb_event_t event)
{
    *((volatile uint32_t *)((uint8_t *)p_reg + (uint32_t)event)) = 0x0UL;
    nrf_event_readback((uint8_t *)p_reg + (uint32_t)event);
}

NRF_STATIC_INLINE bool nrf_vregusb_event_check(NRF_VREGUSB_Type const * p_reg,
                                               nrf_vregusb_event_t      event)
{
    return nrf_event_check(p_reg, event);
}

NRF_STATIC_INLINE uint32_t nrf_vregusb_event_address_get(NRF_VREGUSB_Type const * p_reg,
                                                         nrf_vregusb_event_t      event)
{
    return nrf_task_event_address_get(p_reg, event);
}

NRF_STATIC_INLINE void nrf_vregusb_int_enable(NRF_VREGUSB_Type * p_reg, uint32_t mask)
{
    p_reg->INTENSET = mask;
}

NRF_STATIC_INLINE void nrf_vregusb_int_disable(NRF_VREGUSB_Type * p_reg, uint32_t mask)
{
    p_reg->INTENCLR = mask;
}

NRF_STATIC_INLINE uint32_t nrf_vregusb_int_enable_check(NRF_VREGUSB_Type const * p_reg,
                                                          uint32_t                mask)
{
    return p_reg->INTENSET & mask;
}

NRF_STATIC_INLINE uint32_t nrf_vregusb_int_pending_get(NRF_VREGUSB_Type const * p_reg)
{
        return p_reg->INTPEND;
}

#if NRF_VREGUSB_HAS_STATUS
NRF_STATIC_INLINE uint32_t nrf_vregusb_status_get(NRF_VREGUSB_Type const * p_reg)
{
    return p_reg->STATUS;
}
#endif

#if NRF_VREGUSB_HAS_CONFIG
NRF_STATIC_INLINE void nrf_vregusb_config_rdy_tim_set(NRF_VREGUSB_Type * p_reg, uint8_t val)
{
    p_reg->CONFIG.RDYTIM = (uint32_t)val;
}

NRF_STATIC_INLINE uint8_t nrf_vregusb_config_rdy_tim_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint8_t)p_reg->CONFIG.RDYTIM;
}

NRF_STATIC_INLINE void nrf_vregusb_config_vbrft_set(NRF_VREGUSB_Type * p_reg, uint16_t val)
{
    p_reg->CONFIG.VBRFT = (uint32_t)val;
}

NRF_STATIC_INLINE uint16_t nrf_vregusb_config_vbrft_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint16_t)p_reg->CONFIG.VBRFT;
}

NRF_STATIC_INLINE void nrf_vregusb_config_rdy_ldo_stim_set(NRF_VREGUSB_Type * p_reg, uint8_t val)
{
    p_reg->CONFIG.RDYLDOSTIM = (uint32_t)val;
}

NRF_STATIC_INLINE uint8_t nrf_vregusb_config_rdy_ldo_stim_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint8_t)p_reg->CONFIG.RDYLDOSTIM;
}

#if NRF_VREGUSB_HAS_SETTLED
NRF_STATIC_INLINE void nrf_vregusb_config_setl_0v8_tim_set(NRF_VREGUSB_Type * p_reg, uint16_t val)
{
    p_reg->CONFIG.SETL0V8TIM = (uint32_t)val;
}

NRF_STATIC_INLINE uint16_t nrf_vregusb_config_setl_0v8_tim_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint16_t)p_reg->CONFIG.SETL0V8TIM;
}

NRF_STATIC_INLINE void nrf_vregusb_config_setl_3v3_tim_set(NRF_VREGUSB_Type * p_reg, uint16_t val)
{
    p_reg->CONFIG.SETL3V3TIM = (uint32_t)val;
}

NRF_STATIC_INLINE uint16_t nrf_vregusb_config_setl_3v3_tim_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint16_t)p_reg->CONFIG.SETL3V3TIM;
}
#endif // NRF_VREGUSB_HAS_SETTLED
#endif // NRF_VREGUSB_HAS_CONFIG

#if NRF_VREGUSB_HAS_TRIM_VDD
NRF_STATIC_INLINE void nrf_vregusb_trim_vdd_set(NRF_VREGUSB_Type * p_reg, uint8_t val)
{
    p_reg->TRIM.TRIMVDD = (uint32_t)val;
}

NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_vdd_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint8_t)p_reg->TRIM.TRIMVDD;
}
#endif

#if NRF_VREGUSB_HAS_TRIM_IBP
NRF_STATIC_INLINE void nrf_vregusb_trim_ibp_set(NRF_VREGUSB_Type * p_reg, uint8_t val)
{
    p_reg->TRIM.TRIMIBP = (uint32_t)val;
}

NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_ibp_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint8_t)p_reg->TRIM.TRIMIBP;
}
#endif

#if NRF_VREGUSB_HAS_TRIM_0V8
NRF_STATIC_INLINE void nrf_vregusb_trim_0v8_set(NRF_VREGUSB_Type * p_reg, uint8_t val)
{
    p_reg->TRIM.TRIM0V8 = (uint32_t)val;
}

NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_0v8_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint8_t)p_reg->TRIM.TRIM0V8;
}
#endif

#if NRF_VREGUSB_HAS_TRIM_3V3
NRF_STATIC_INLINE void nrf_vregusb_trim_3v3_set(NRF_VREGUSB_Type * p_reg, uint8_t val)
{
    p_reg->TRIM.TRIM3V3 = (uint32_t)val;
}

NRF_STATIC_INLINE uint8_t nrf_vregusb_trim_3v3_get(NRF_VREGUSB_Type const * p_reg)
{
    return (uint8_t)p_reg->TRIM.TRIM3V3;
}
#endif

#if NRF_VREGUSB_HAS_MIRROR
NRF_STATIC_INLINE void nrf_vregusb_mirror_lock_set(NRF_VREGUSB_Type * p_reg, bool lock)
{
    p_reg->MIRROR = (lock ? VREGUSB_MIRROR_LOCK_Enabled : VREGUSB_MIRROR_LOCK_Disabled) <<
                    VREGUSB_MIRROR_LOCK_Pos;
}
#endif

#if NRF_VREGUSB_HAS_PWRUP
NRF_STATIC_INLINE void nrf_vregusb_pwrup_set(NRF_VREGUSB_Type *         p_reg,
                                             nrf_vregusb_pwrup_target_t target,
                                             nrf_vregusb_pwrup_mode_t   mode)
{
    switch (target)
    {
        case NRF_VREGUSB_PWRUP_TARGET_USB_PM:
            p_reg->PWRUP = mode;
            break;
        case NRF_VREGUSB_PWRUP_TARGET_0V8:
            p_reg->PWRUP0V8 = mode;
            break;
        case NRF_VREGUSB_PWRUP_TARGET_3V3:
            p_reg->PWRUP3V3 = mode;
            break;
        default:
            NRFX_ASSERT(0);
            break;
    }
}
#endif

#if NRF_VREGUSB_HAS_OVERRIDE
NRF_STATIC_INLINE void nrf_vregusb_override_set(NRF_VREGUSB_Type *          p_reg,
                                                nrf_vregusb_analog_signal_t signal,
                                                bool                        enable,
                                                uint8_t                     val)
{
    switch (signal)
    {
#if NRF_VREGUSB_HAS_EVENT_VBUSDETECTEDRAW
        case NRF_VREGUSB_ANALOG_SIGNAL_VBUS_DETRAW:
              p_reg->OVERRIDE.VBUSDETRAW = ((enable ?
                                             VREGUSB_OVERRIDE_VBUSDETRAW_EN_Enabled :
                                             VREGUSB_OVERRIDE_VBUSDETRAW_EN_Disabled) <<
                                            VREGUSB_OVERRIDE_VBUSDETRAW_EN_Pos) |
                                           val << VREGUSB_OVERRIDE_VBUSDETRAW_VAL_Pos;
              break;
#endif
        case NRF_VREGUSB_ANALOG_SIGNAL_VBUS_DET:
              p_reg->OVERRIDE.VBUSDET = ((enable ?
                                          VREGUSB_OVERRIDE_VBUSDET_EN_Enabled :
                                          VREGUSB_OVERRIDE_VBUSDET_EN_Disabled) <<
                                         VREGUSB_OVERRIDE_VBUSDET_EN_Pos) |
                                        val << VREGUSB_OVERRIDE_VBUSDET_VAL_Pos;
              break;
#if NRF_VREGUSB_HAS_SETTLED
        case NRF_VREGUSB_ANALOG_SIGNAL_SETTLED_0V8:
              p_reg->OVERRIDE.SETTLED0V8 = ((enable ?
                                             VREGUSB_OVERRIDE_SETTLED0V8_EN_Enabled :
                                             VREGUSB_OVERRIDE_SETTLED0V8_EN_Disabled) <<
                                            VREGUSB_OVERRIDE_SETTLED0V8_EN_Pos) |
                                           val << VREGUSB_OVERRIDE_SETTLED0V8_VAL_Pos;
              break;
        case NRF_VREGUSB_ANALOG_SIGNAL_SETTLED_3V3:
              p_reg->OVERRIDE.SETTLED3V3 = ((enable ?
                                             VREGUSB_OVERRIDE_SETTLED3V3_EN_Enabled :
                                             VREGUSB_OVERRIDE_SETTLED3V3_EN_Disabled) <<
                                            VREGUSB_OVERRIDE_SETTLED3V3_EN_Pos) |
                                           val << VREGUSB_OVERRIDE_SETTLED3V3_VAL_Pos;
              break;
#endif
        default:
              NRFX_ASSERT(0);
              break;
    }
}
#endif // NRF_VREGUSB_HAS_OVERRIDE

#endif // NRF_DECLARE_ONLY

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRF_VREGUSB_H__
