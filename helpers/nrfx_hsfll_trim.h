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

#ifndef NRFX_HSFLL_TRIM_H__
#define NRFX_HSFLL_TRIM_H__

#include <nrfx.h>

#include <hal/nrf_hsfll.h>
#include <hal/nrf_ficr.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_hsfll_trim Helper for trimming HSFLL
 * @{
 * @ingroup nrfx
 *
 * @brief Helper layer that provides trimming for HSFLL clock.
 */

/**
 * @brief Function for trimming HSFLL clock based on calibration data stored in FICR.
 *
 * The function reads trim values from FICR and applies them to the HSFLL peripheral. It also
 * triggers frequency change task to update the HSFLL clock.
 *
 * @param[in] p_reg            Pointer to the structure of registers of the HSFLL peripheral.
 * @param[in] trim_index       Index of the fine and coarse trim values.
 * @param[in] hsfll_multiplier Multiplier for the HSFLL clock.
 */
NRFX_STATIC_INLINE void nrfx_hsfll_trim_apply(NRF_HSFLL_Type * p_reg,
                                              uint8_t          trim_index,
                                              uint32_t         hsfll_multiplier);

#ifndef NRFX_DECLARE_ONLY

NRFX_STATIC_INLINE void nrfx_hsfll_trim_apply(NRF_HSFLL_Type * p_reg,
                                              uint8_t          trim_index,
                                              uint32_t         hsfll_multiplier)
{
    nrf_hsfll_trim_t trim = {
        .coarse = nrf_ficr_hsfll_trim_coarse_get(NRF_FICR, trim_index),
        .fine   = nrf_ficr_hsfll_trim_fine_get(NRF_FICR, trim_index),
        .vsup   = nrf_ficr_hsfll_trim_vsup_get(NRF_FICR),
#if NRF_HSFLL_HAS_TCOEF_TRIM
        .tcoef  = nrf_ficr_hsfll_trim_tcoef_get(NRF_FICR),
#endif // NRF_HSFLL_HAS_TCOEF_TRIM
    };

    nrf_hsfll_clkctrl_mult_set(p_reg, hsfll_multiplier);
    nrf_hsfll_trim_set(p_reg, &trim);

    nrf_hsfll_task_trigger(p_reg, NRF_HSFLL_TASK_FREQ_CHANGE);
    /* HSFLL task frequency change needs to be triggered twice to take effect.*/
    nrf_hsfll_task_trigger(p_reg, NRF_HSFLL_TASK_FREQ_CHANGE);
}

#endif // NRFX_DECLARE_ONLY

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* NRFX_HSFLL_TRIM_H__ */
