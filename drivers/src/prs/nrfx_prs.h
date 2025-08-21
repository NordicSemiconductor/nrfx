/*
 * Copyright (c) 2017 - 2025, Nordic Semiconductor ASA
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

#ifndef NRFX_PRS_H__
#define NRFX_PRS_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_prs Peripheral Resource Sharing (PRS)
 * @{
 * @ingroup nrfx
 *
 * @brief Peripheral Resource Sharing interface (PRS).
 */

#if defined(NRF51)
    // SPI0, TWI0
    #define NRFX_PRS_BOX_0_ADDR     NRF_SPI0
    // SPI1, SPIS1, TWI1
    #define NRFX_PRS_BOX_1_ADDR     NRF_SPI1
#elif defined(NRF52805_XXAA) || defined(NRF52810_XXAA)
    // TWIM0, TWIS0, TWI0
    #define NRFX_PRS_BOX_0_ADDR     NRF_TWIM0
    // SPIM0, SPIS0, SPI0
    #define NRFX_PRS_BOX_1_ADDR     NRF_SPIM0
    // UARTE0, UART0
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE0
#elif defined(NRF52811_XXAA)
    // TWIM0, TWIS0, TWI0, SPIM1, SPIS1, SPI1
    #define NRFX_PRS_BOX_0_ADDR     NRF_TWIM0
    // SPIM0, SPIS0, SPI0
    #define NRFX_PRS_BOX_1_ADDR     NRF_SPIM0
    // UART0, UARTE0
    #define NRFX_PRS_BOX_2_ADDR     NRF_UART0
#elif defined(NRF52820_XXAA)
    // SPIM0, SPIS0, TWIM0, TWIS0, SPI0, TWI0
    #define NRFX_PRS_BOX_0_ADDR     NRF_SPIM0
    // SPIM1, SPIS1, TWIM1, TWIS1, SPI1, TWI1
    #define NRFX_PRS_BOX_1_ADDR     NRF_SPIM1
    // UARTE0, UART0
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE0
#elif defined(NRF52832_XXAA) || defined(NRF52832_XXAB) || \
      defined(NRF52833_XXAA) || defined(NRF52840_XXAA)
    // SPIM0, SPIS0, TWIM0, TWIS0, SPI0, TWI0
    #define NRFX_PRS_BOX_0_ADDR     NRF_SPIM0
    // SPIM1, SPIS1, TWIM1, TWIS1, SPI1, TWI1
    #define NRFX_PRS_BOX_1_ADDR     NRF_SPIM1
    // SPIM2, SPIS2, SPI2
    #define NRFX_PRS_BOX_2_ADDR     NRF_SPIM2
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_3_ADDR     NRF_COMP
    // UARTE0, UART0
    #define NRFX_PRS_BOX_4_ADDR     NRF_UARTE0
#elif defined(NRF5340_XXAA_APPLICATION)
    // SPIM0, SPIS0, TWIM0, TWIS0, UARTE0
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE0
    // SPIM1, SPIS1, TWIM1, TWIS1, UARTE1
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE1
    // SPIM2, SPIS2, TWIM2, TWIS2, UARTE2
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE2
    // SPIM3, SPIS3, TWIM3, TWIS3, UARTE3
    #define NRFX_PRS_BOX_3_ADDR     NRF_UARTE3
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_4_ADDR     NRF_COMP
#elif defined(NRF5340_XXAA_NETWORK)
    // SPIM0, SPIS0, TWIM0, TWIS0, UARTE0
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE0
#elif defined(NRF54H20_XXAA)
    // SPIM130, SPIS130, TWIM130, TWIS130, UARTE130
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE130
    // SPIM131, SPIS131, TWIM131, TWIS131, UARTE131
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE131
    // SPIM132, SPIS132, TWIM132, TWIS132, UARTE132
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE132
    // SPIM133, SPIS133, TWIM133, TWIS133, UARTE133
    #define NRFX_PRS_BOX_3_ADDR     NRF_UARTE133
    // SPIM134, SPIS134, TWIM134, TWIS134, UARTE134
    #define NRFX_PRS_BOX_4_ADDR     NRF_UARTE134
    // SPIM135, SPIS135, TWIM135, TWIS135, UARTE135
    #define NRFX_PRS_BOX_5_ADDR     NRF_UARTE135
    // SPIM136, SPIS136, TWIM136, TWIS136, UARTE136
    #define NRFX_PRS_BOX_6_ADDR     NRF_UARTE136
    // SPIM137, SPIS137, TWIM137, TWIS137, UARTE137
    #define NRFX_PRS_BOX_7_ADDR     NRF_UARTE137
    // SPIM120, UARTE120
    #define NRFX_PRS_BOX_8_ADDR     NRF_UARTE120
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_9_ADDR     NRF_COMP
#elif defined(NRF54L05_XXAA) || defined(NRF54L10_XXAA) || defined(NRF54L15_XXAA)
    // SPIM00, SPIS00, UARTE00
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE00
    // SPIM20, SPIS20, TWIM20, TWIS20, UARTE20
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE20
    // SPIM21, SPIS21, TWIM21, TWIS21, UARTE21
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE21
    // SPIM22, SPIS22, TWIM22, TWIS22, UARTE22
    #define NRFX_PRS_BOX_3_ADDR     NRF_UARTE22
    // SPIM30, SPIS30, TWIM30, TWIS30, UARTE30
    #define NRFX_PRS_BOX_4_ADDR     NRF_UARTE30
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_5_ADDR     NRF_COMP
#elif defined(NRF54LM20A_ENGA_XXAA)
    // SPIM00, SPIS00, UARTE00
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE00
    // SPIM20, SPIS20, TWIM20, TWIS20, UARTE20
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE20
    // SPIM21, SPIS21, TWIM21, TWIS21, UARTE21
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE21
    // SPIM22, SPIS22, TWIM22, TWIS22, UARTE22
    #define NRFX_PRS_BOX_3_ADDR     NRF_UARTE22
    // SPIM23, SPIS23, TWIM23, TWIS23, UARTE23
    #define NRFX_PRS_BOX_4_ADDR     NRF_UARTE23
    // SPIM24, SPIS24, TWIM24, TWIS24, UARTE23
    #define NRFX_PRS_BOX_5_ADDR     NRF_UARTE24
    // SPIM30, SPIS30, TWIM30, TWIS30, UARTE30
    #define NRFX_PRS_BOX_6_ADDR     NRF_UARTE30
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_7_ADDR     NRF_COMP
#elif defined(NRF54LS05B_ENGA_XXAA)
    // SPIM20, SPIS20, TWIM20, TWIS20, UARTE20
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE20
    // SPIM21, SPIS21, TWIM21, TWIS21, UARTE21
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE21
    // SPIM32, SPIS32, TWIM32, TWIS32, UARTE32
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE22
#elif defined(NRF54LV10A_ENGA_XXAA)
    // SPIM20, SPIS20, TWIM20, TWIS20, UARTE20
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE20
    // SPIM21, SPIS21, TWIM21, TWIS21, UARTE21
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE21
    // SPIM30, SPIS30, TWIM30, TWIS30, UARTE30
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE30
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_3_ADDR     NRF_COMP
#elif defined(NRF7120_ENGA_XXAA)
    // SPIM00, UARTE00
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE00
    // SPIM01
    #define NRFX_PRS_BOX_1_ADDR     NRF_SPIM01
    // SPIM02
    #define NRFX_PRS_BOX_2_ADDR     NRF_SPIM02
    // SPIM20, SPIS20, TWIM20, TWIS20, UARTE20
    #define NRFX_PRS_BOX_3_ADDR     NRF_UARTE20
    // SPIM21, SPIS21, TWIM21, TWIS21, UARTE21
    #define NRFX_PRS_BOX_4_ADDR     NRF_UARTE21
    // SPIM22, SPIS22, TWIM22, TWIS22, UARTE22
    #define NRFX_PRS_BOX_5_ADDR     NRF_UARTE22
    // SPIM23, SPIS23, TWIM23, TWIS23, UARTE23
    #define NRFX_PRS_BOX_6_ADDR     NRF_UARTE23
    // SPIM24, SPIS24, TWIM24, TWIS24, UARTE24
    #define NRFX_PRS_BOX_7_ADDR     NRF_UARTE24
    // SPIM30, SPIS30, TWIM30, TWIS30, UARTE30
    #define NRFX_PRS_BOX_8_ADDR     NRF_UARTE30
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_9_ADDR     NRF_COMP
#elif defined(NRF91_SERIES)
    // UARTE0, SPIM0, SPIS0, TWIM0, TWIS0
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE0
    // UARTE1, SPIM1, SPIS1, TWIM1, TWIS1
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE1
    // UARTE2, SPIM2, SPIS2, TWIM2, TWIS2
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE2
    // UARTE3, SPIM3, SPIS3, TWIM3, TWIS3
    #define NRFX_PRS_BOX_3_ADDR     NRF_UARTE3
#elif defined(NRF9230_ENGB_XXAA)
    // SPIM130, SPIS130, TWIM130, TWIS130, UARTE130
    #define NRFX_PRS_BOX_0_ADDR     NRF_UARTE130
    // SPIM131, SPIS131, TWIM131, TWIS131, UARTE131
    #define NRFX_PRS_BOX_1_ADDR     NRF_UARTE131
    // SPIM132, SPIS132, TWIM132, TWIS132, UARTE132
    #define NRFX_PRS_BOX_2_ADDR     NRF_UARTE132
    // SPIM133, SPIS133, TWIM133, TWIS133, UARTE133
    #define NRFX_PRS_BOX_3_ADDR     NRF_UARTE133
    // SPIM134, SPIS134, TWIM134, TWIS134, UARTE134
    #define NRFX_PRS_BOX_4_ADDR     NRF_UARTE134
    // SPIM135, SPIS135, TWIM135, TWIS135, UARTE135
    #define NRFX_PRS_BOX_5_ADDR     NRF_UARTE135
    // SPIM136, SPIS136, TWIM136, TWIS136, UARTE136
    #define NRFX_PRS_BOX_6_ADDR     NRF_UARTE136
    // SPIM137, SPIS137, TWIM137, TWIS137, UARTE137
    #define NRFX_PRS_BOX_7_ADDR     NRF_UARTE137
    // SPIS120, UARTE120
    #define NRFX_PRS_BOX_8_ADDR     NRF_UARTE120
    // COMP, LPCOMP
    #define NRFX_PRS_BOX_9_ADDR     NRF_COMP
#elif !defined(NRF_PRS_BOX_EXT)
    #error "Unknown device."
#endif

/**
 * @brief Function for acquiring shared peripheral resources associated with
 *        the specified peripheral.
 *
 * Certain resources and registers are shared among peripherals that have
 * the same ID (for example: SPI0, SPIM0, SPIS0, TWI0, TWIM0, and TWIS0 in
 * nRF52832). Only one of them can be utilized at a given time. This function
 * reserves proper resources to be used by the specified peripheral.
 * If NRFX_PRS_ENABLED is set to a non-zero value, IRQ handlers for peripherals
 * that are sharing resources with others are implemented by the @ref nrfx_prs
 * module instead of individual drivers. The drivers must then specify their
 * interrupt handling routines and register them by using this function.
 *
 * @param[in] p_base_addr Requested peripheral base pointer.
 * @param[in] irq_handler Interrupt handler to register.
 *
 * @retval NRFX_SUCCESS    If resources were acquired successfully or the
 *                         specified peripheral is not handled by the PRS
 *                         subsystem and there is no need to acquire resources
 *                         for it.
 * @retval NRFX_ERROR_BUSY If resources were already acquired.
 */
nrfx_err_t nrfx_prs_acquire(void       const * p_base_addr,
                            nrfx_irq_handler_t irq_handler);

/**
 * @brief Function for releasing shared resources reserved previously by
 *        @ref nrfx_prs_acquire() for the specified peripheral.
 *
 * @param[in] p_base_addr Released peripheral base pointer.
 */
void nrfx_prs_release(void const * p_base_addr);

/** @} */

/*
 * Declare interrupt handlers for all enabled driver instances in the following format:
 * nrfx_\<periph_name\>_\<idx\>_irq_handler (for example, nrfx_prs_box_0_irq_handler).
 *
 * A specific interrupt handler for the driver instance can be retrieved by using
 * the NRFX_PRS_BOX_INST_HANDLER_GET macro.
 *
 * Here is a sample of using the NRFX_PRS_BOX_INST_HANDLER_GET macro to map an interrupt handler
 * in a Zephyr application:
 *
 * IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PRS_BOX_INST_GET(\<instance_index\>)), \<priority\>,
 *                    NRFX_PRS_BOX_INST_HANDLER_GET(\<instance_index\>), 0, 0);
 */
NRFX_INSTANCE_IRQ_HANDLERS_DECLARE(PRS_BOX_, prs_box)

#ifdef __cplusplus
}
#endif

#endif // NRFX_PRS_H__
