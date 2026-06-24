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

#ifndef NRF_GPIOHSPADCTRL_H__
#define NRF_GPIOHSPADCTRL_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrf_gpiohspadctrl_hal GPIOHSPADCTRL HAL
 * @{
 * @ingroup nrf_gpio
 * @brief   Hardware access layer for managing the GPIOHSPADCTRL peripheral.
 */

/**
 * @brief Function for configuring the slew rate of the GPIO pins.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 * @param[in] bias  Slew setting for high-speed pad.
 */
NRF_STATIC_INLINE void nrf_gpiohspadctrl_hs_bias_set(NRF_GPIOHSPADCTRL_Type * p_reg, uint32_t bias);

/**
 * @brief Function for getting the slew rate setting of the GPIO pins.
 *
 * @param[in] p_reg Pointer to the structure of registers of the peripheral.
 *
 * @return Slew setting for high-speed pad.
 */
NRF_STATIC_INLINE uint32_t nrf_gpiohspadctrl_hs_bias_get(NRF_GPIOHSPADCTRL_Type const * p_reg);

#ifndef NRF_DECLARE_ONLY

NRF_STATIC_INLINE void nrf_gpiohspadctrl_hs_bias_set(NRF_GPIOHSPADCTRL_Type * p_reg, uint32_t bias)
{
    NRFX_ASSERT(p_reg && (bias <= GPIOHSPADCTRL_BIAS_HSBIAS_Max));

    p_reg->BIAS = (p_reg->BIAS & ~GPIOHSPADCTRL_BIAS_HSBIAS_Msk) |
                  ((bias << GPIOHSPADCTRL_BIAS_HSBIAS_Pos) & GPIOHSPADCTRL_BIAS_HSBIAS_Msk);
}

NRF_STATIC_INLINE uint32_t nrf_gpiohspadctrl_hs_bias_get(NRF_GPIOHSPADCTRL_Type const * p_reg)
{
    NRFX_ASSERT(p_reg);

    return (uint32_t)(p_reg->BIAS & GPIOHSPADCTRL_BIAS_HSBIAS_Msk) >> GPIOHSPADCTRL_BIAS_HSBIAS_Pos;
}

#endif // NRF_DECLARE_ONLY

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRF_GPIOHSPADCTRL_H__
