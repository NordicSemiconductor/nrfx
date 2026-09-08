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

#ifndef NRFX_CLOCK_LFRC_H__
#define NRFX_CLOCK_LFRC_H__

#include <nrfx.h>
#include <hal/nrf_clock.h>

#if NRFX_CHECK(LFRC_PRESENT) || defined(__NRFX_DOXYGEN__)

#include <hal/nrf_lfrc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_clock_lfrc LFRC driver
 * @{
 * @ingroup nrf_clock
 * @brief   LFRC clock driver.
 */

 /** @brief Symbol specifying driver event offset for LFRC hardware events. */
#define NRFX_CLOCK_LFRC_EVT_OFFSET 32

/** @brief Clock events. */
typedef enum
{
#if NRF_LFRC_HAS_CALIBRATION
    NRFX_CLOCK_LFRC_EVT_CAL_DONE = (NRFX_BITMASK_TO_BITPOS(NRF_LFRC_INT_CALDONE_MASK) +
                                    NRFX_CLOCK_LFRC_EVT_OFFSET),                        ///< Calibration has been done.
#endif
} nrfx_clock_lfrc_evt_type_t;

/**
 * @brief LFRC event handler.
 *
 * @param[in] event Event.
 */
typedef void (*nrfx_clock_lfrc_event_handler_t)(nrfx_clock_lfrc_evt_type_t event);

/**
 * @brief Function for initializing internal structures in the nrfx_clock_lfrc driver.
 *
 * After initialization, the driver is in power off state (clocks are not started).
 *
 * @param[in] event_handler Event handler provided by the user.
 *                          If not provided, driver works in blocking mode.
 *
 * @retval 0         The procedure is successful.
 * @retval -EALREADY The driver is already initialized.
 */
int nrfx_clock_lfrc_init(nrfx_clock_lfrc_event_handler_t event_handler);

/** @brief Function for uninitializing the LFRC driver. */
void nrfx_clock_lfrc_uninit(void);

/**
 * @brief Function for checking if the LFRC driver is initialized.
 *
 * @retval true  Driver is already initialized.
 * @retval false Driver is not initialized.
 */
bool nrfx_clock_lfrc_init_check(void);

#if (NRF_LFRC_HAS_CALIBRATION && NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)) || \
    defined(__NRFX_DOXYGEN__)
/**
 * @brief Function for starting the calibration of internal LFRC.
 *
 * This function starts the calibration process. The process cannot be aborted.
 * LFRC and high-frequency clock must be running before this function is called.
 *
 * @retval 0            The procedure is successful.
 * @retval -EINPROGRESS The low-frequency or high-frequency clock is off.
 * @retval -EBUSY       Clock is in the calibration phase.
 */
int nrfx_clock_lfrc_calibration_start(void);

/**
 * @brief Function for checking if calibration is in progress.
 *
 * This function indicates that the system is in calibration phase.
 *
 * @retval 0      The procedure is successful.
 * @retval -EBUSY Clock is in the calibration phase.
 */
int nrfx_clock_lfrc_calibrating_check(void);
#endif /* (NRF_LFRC_HAS_CALIBRATION && NRFX_CHECK(NRFX_CLOCK_CONFIG_LF_CAL_ENABLED)) || \
          defined(__NRFX_DOXYGEN__) */

/** @brief LFRC interrupt handler. */
void nrfx_clock_lfrc_irq_handler(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* NRFX_CHECK(LFRC_PRESENT) */

#endif // NRFX_CLOCK_LFRC_H__
