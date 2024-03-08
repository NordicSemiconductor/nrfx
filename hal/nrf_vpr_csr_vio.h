/*
 * Copyright (c) 2023 - 2024, Nordic Semiconductor ASA
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

#ifndef NRF_VPR_CSR_VIO_H__
#define NRF_VPR_CSR_VIO_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrf_vpr_csr_vio_hal VPR CSR VIO HAL
 * @{
 * @ingroup nrf_vpr
 * @brief   Hardware access layer for managing the VPR RISC-V CPU Control
 *          and Status Registers for VPR IO (VPR CSR VIO).
 */

/** @brief Shift sizes for output. */
typedef enum
{
    NRF_VPR_CSR_VIO_OUT_SHIFT_1  = VPRCSR_NORDIC_OUTMODE_SHIFSIZE_SHIFT1,  ///< Shift OUT by 1 bit.
    NRF_VPR_CSR_VIO_OUT_SHIFT_2  = VPRCSR_NORDIC_OUTMODE_SHIFSIZE_SHIFT2,  ///< Shift OUT by 2 bits.
    NRF_VPR_CSR_VIO_OUT_SHIFT_4  = VPRCSR_NORDIC_OUTMODE_SHIFSIZE_SHIFT4,  ///< Shift OUT by 4 bits.
    NRF_VPR_CSR_VIO_OUT_SHIFT_8  = VPRCSR_NORDIC_OUTMODE_SHIFSIZE_SHIFT8,  ///< Shift OUT by 8 bits.
    NRF_VPR_CSR_VIO_OUT_SHIFT_16 = VPRCSR_NORDIC_OUTMODE_SHIFSIZE_SHIFT16, ///< Shift OUT by 16 bits.
} nrf_vpr_csr_vio_out_shift_t;

/** @brief Output mode structure. */
typedef struct
{
    bool                        shift_enable; ///< Enable shift mode.
    nrf_vpr_csr_vio_out_shift_t shift_size;   ///< Shift size.
} nrf_vpr_csr_vio_mode_out_t;

#if !defined(NRF54H20_ENGA_XXAA)
/** @brief Input modes. */
typedef enum
{
    NRF_VPR_CSR_VIO_MODE_IN_CONTINUOUS = VPRCSR_NORDIC_INMODE_MODE_CONTINUOUS, ///< Continuous sampling (if CPU is not sleeping).
    NRF_VPR_CSR_VIO_MODE_IN_EVENT      = VPRCSR_NORDIC_INMODE_MODE_EVENT,      ///< Sampling on Counter 1 event.
} nrf_vpr_csr_vio_mode_in_t;
#endif

/**
 * @brief Function for getting the pin directions mask.
 *
 * @return Mask of pin directions. 0 is input, 1 is output.
 */
NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_dir_get(void);

/**
 * @brief Function for setting the pin directions mask.
 *
 * @param[in] value Mask of pin directions to be set. 0 is input, 1 is output.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_set(uint16_t value);

/**
 * @brief Function for getting the buffered pin directions mask.
 *
 * @return Mask of pin directions. 0 is input, 1 is output.
 */
NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_dir_buffered_get(void);

/**
 * @brief Function for setting the buffered pin directions mask.
 *
 * @param[in] value Mask of pin directions to be set. 0 is input, 1 is output.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_buffered_set(uint16_t value);

/**
 * @brief Function for retrieving the dirty status of buffered pin directions mask.
 *
 * @retval true  Buffer is dirty.
 * @retval fasle Buffer is clean.
 */
NRF_STATIC_INLINE bool nrf_vpr_csr_vio_dir_buffered_dirty_check(void);

/**
 * @brief Function for setting the pin directions toggle mask.
 *
 * @param[in] mask Mask of pin directions to be toggled.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_toggle_set(uint16_t mask);

/**
 * @brief Function for setting the buffered pin directions toggle mask.
 *
 * @param[in] mask Mask of pin directions to be toggled.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_buffered_toggle_set(uint16_t mask);

/**
 * @brief Function for getting the input values.
 *
 * @return Mask of input states. 0 is low, 1 is high.
 */
NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_in_get(void);

#if !defined(NRF54H20_ENGA_XXAA)
/**
 * @brief Function for getting the input mode.
 *
 * @return Input mode.
 */
NRF_STATIC_INLINE nrf_vpr_csr_vio_mode_in_t nrf_vpr_csr_vio_mode_in_get(void);

/**
 * @brief Function for setting the input mode.
 *
 * @param[in] mode Input mode to be set.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_in_set(nrf_vpr_csr_vio_mode_in_t mode);
#else
/**
 * @brief Function for getting the input mode.
 *
 * @return Mask of input modes. 0 is continous sampling, 1 is sampling on event.
 */
NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_mode_in_get(void);

/**
 * @brief Function for setting the input mode.
 *
 * @param[in] value Mask of input modes to be set. 0 is continous sampling, 1 is sampling on event.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_in_set(uint16_t mode);
#endif

/**
 * @brief Function for getting the output values.
 *
 * @return Mask of output states. 0 is low, 1 is high.
 */
NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_out_get(void);

/**
 * @brief Function for setting the output values.
 *
 * @param[in] value Mask of output states to be set. 0 is low, 1 is high.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_set(uint16_t value);

/**
 * @brief Function for getting the buffered output values.
 *
 * @return Mask of output states. 0 is low, 1 is high.
 */
NRF_STATIC_INLINE uint32_t nrf_vpr_csr_vio_out_buffered_get(void);

/**
 * @brief Function for setting the buffered output values.
 *
 * @param[in] value Mask of output states to be set. 0 is low, 1 is high.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_buffered_set(uint32_t value);

/**
 * @brief Function for retrieving the dirty status of buffered output values.
 *
 * @retval true  Buffer is dirty.
 * @retval fasle Buffer is clean.
 */
NRF_STATIC_INLINE bool nrf_vpr_csr_vio_out_buffered_dirty_check(void);

/**
 * @brief Function for setting the output toggle mask.
 *
 * @param[in] mask Mask of output to be toggled.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_toggle_set(uint16_t mask);

/**
 * @brief Function for setting the buffered output toggle mask.
 *
 * @param[in] mask Mask of output to be toggled.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_toggle_buffered_set(uint16_t mask);

/**
 * @brief Function for setting the combined output and buffered output values.
 *
 * @note Lower 16 bits determine the output state, while higher 16 bits determine the buffered output state.
 *
 * @param[in] value Mask of output states to be set. 0 is low, 1 is high.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_combined_set(uint32_t value);

/**
 * @brief Function for setting the combined output and buffered output toggle mask.
 *
 * @note Lower 16 bits determine the output toggle, while higher 16 bits determine the buffered output toggle.
 *
 * @param[in] mask Mask of output to be toggled.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_combined_toggle_set(uint32_t mask);

/**
 * @brief Function for retrieving the dirty status of combined output and buffered output values.
 *
 * @retval true  Buffer is dirty.
 * @retval fasle Buffer is clean.
 */
NRF_STATIC_INLINE bool nrf_vpr_csr_vio_out_combined_dirty_check(void);

/**
 * @brief Function for getting the configuration of output mode.
 *
 * @param[out] p_mode Pointer to the structure to be filled with output mode.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_out_get(nrf_vpr_csr_vio_mode_out_t * p_mode);

/**
 * @brief Function for setting the configuration of output mode.
 *
 * @param[in] p_mode Pointer to the structure with output mode to be set.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_out_set(nrf_vpr_csr_vio_mode_out_t const * p_mode);

/**
 * @brief Function for getting the combined pin directions mask and output values.
 *
 * @note Lower 16 bits determine the output state, while higher 16 bits determine the pin directions.
 *
 * @return Mask of pin directions and output values.
 */
NRF_STATIC_INLINE uint32_t nrf_vpr_csr_vio_dirout_get(void);

/**
 * @brief Function for setting the combined pin directions mask and output values.
 *
 * @note Lower 16 bits determine the output state, while higher 16 bits determine the pin directions.
 *
 * @param[in] value Mask of pin directions and output values.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_set(uint32_t value);

/**
 * @brief Function for getting the combined buffered pin directions mask and buffered output values.
 *
 * @note Lower 16 bits determine the buffered output state, while higher 16 bits determine the buffered pin directions.
 *
 * @return Mask of buffered pin directions and buffered output values.
 */
NRF_STATIC_INLINE uint32_t nrf_vpr_csr_vio_dirout_buffered_get(void);

/**
 * @brief Function for getting the combined buffered pin directions mask and buffered output values.
 *
 * @note Lower 16 bits determine the buffered output state, while higher 16 bits determine the buffered pin directions.
 *
 * @param[in] value Mask of buffered pin directions and buffered output values.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_buffered_set(uint32_t value);

/**
 * @brief Function for retrieving the dirty status of combined buffered pin directions mask and buffered output values.
 *
 * @retval true  Buffer is dirty.
 * @retval fasle Buffer is clean.
 */
NRF_STATIC_INLINE bool nrf_vpr_csr_vio_dirout_buffered_dirty_check(void);

/**
 * @brief Function for setting the combined pin directions and output toggle masks.
 *
 * @note Lower 16 bits determine the output toggle, while higher 16 bits determine the pin directions toggle.
 *
 * @param[in] mask Mask of values to be toggled.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_toggle_set(uint32_t mask);

/**
 * @brief Function for setting the combined buffered pin directions and buffered output toggle masks.
 *
 * @note Lower 16 bits determine the buffered output toggle, while higher 16 bits determine the buffered pin directions toggle.
 *
 * @param[in] mask Mask of values to be toggled.
 */
NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_toggle_buffered_set(uint32_t mask);

#ifndef NRF_DECLARE_ONLY

NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_dir_get(void)
{
    return (uint16_t)nrf_csr_read(VPRCSR_NORDIC_DIR);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_set(uint16_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_DIR, value);
}

NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_dir_buffered_get(void)
{
    return (uint16_t)nrf_csr_read(VPRCSR_NORDIC_DIRB);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_buffered_set(uint16_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_DIRB, value);
}

NRF_STATIC_INLINE bool nrf_vpr_csr_vio_dir_buffered_dirty_check(void)
{
    return ((nrf_csr_read(VPRCSR_NORDIC_DIRBS) & VPRCSR_NORDIC_DIRBS_DIRTYBIT_Msk)
            >> VPRCSR_NORDIC_DIRBS_DIRTYBIT_Pos) == VPRCSR_NORDIC_DIRBS_DIRTYBIT_DIRTY;
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_toggle_set(uint16_t mask)
{
    nrf_csr_write(VPRCSR_NORDIC_DIRTGL, mask);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dir_buffered_toggle_set(uint16_t mask)
{
    nrf_csr_write(VPRCSR_NORDIC_DIRBTGL, mask);
}

NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_in_get(void)
{
    return (uint16_t)nrf_csr_read(VPRCSR_NORDIC_IN);
}

#if !defined(NRF54H20_ENGA_XXAA)
NRF_STATIC_INLINE nrf_vpr_csr_vio_mode_in_t nrf_vpr_csr_vio_mode_in_get(void)
{
    return nrf_csr_read(VPRCSR_NORDIC_INMODE);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_in_set(nrf_vpr_csr_vio_mode_in_t mode)
{
    nrf_csr_write(VPRCSR_NORDIC_INMODE, mode);
}
#else
NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_mode_in_get(void)
{
    return (uint16_t)nrf_csr_read(VPRCSR_NORDIC_INMODE);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_in_set(uint16_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_INMODE, value);
}
#endif

NRF_STATIC_INLINE uint16_t nrf_vpr_csr_vio_out_get(void)
{
    return (uint16_t)nrf_csr_read(VPRCSR_NORDIC_OUT);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_set(uint16_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_OUT, value);
}

NRF_STATIC_INLINE uint32_t nrf_vpr_csr_vio_out_buffered_get(void)
{
    return nrf_csr_read(VPRCSR_NORDIC_OUTB);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_buffered_set(uint32_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_OUTB, value);
}

NRF_STATIC_INLINE bool nrf_vpr_csr_vio_out_buffered_dirty_check(void)
{
    return ((nrf_csr_read(VPRCSR_NORDIC_OUTBS) & VPRCSR_NORDIC_OUTBS_DIRTYBIT_Msk)
            >> VPRCSR_NORDIC_OUTBS_DIRTYBIT_Pos) == VPRCSR_NORDIC_OUTBS_DIRTYBIT_DIRTY;
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_toggle_set(uint16_t mask)
{
    nrf_csr_write(VPRCSR_NORDIC_OUTTGL, mask);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_toggle_buffered_set(uint16_t mask)
{
    nrf_csr_write(VPRCSR_NORDIC_OUTBTGL, mask);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_combined_set(uint32_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_OUTBD, value);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_out_combined_toggle_set(uint32_t mask)
{
    nrf_csr_write(VPRCSR_NORDIC_OUTBDTGL, mask);
}

NRF_STATIC_INLINE bool nrf_vpr_csr_vio_out_combined_dirty_check(void)
{
    return ((nrf_csr_read(VPRCSR_NORDIC_OUTBDS) & VPRCSR_NORDIC_OUTBDS_DIRTYBIT_Msk)
            >> VPRCSR_NORDIC_OUTBDS_DIRTYBIT_Pos) == VPRCSR_NORDIC_OUTBDS_DIRTYBIT_DIRTY;
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_out_get(nrf_vpr_csr_vio_mode_out_t * p_mode)
{
    uint32_t reg = nrf_csr_read(VPRCSR_NORDIC_OUTMODE);

    p_mode->shift_enable = ((reg & VPRCSR_NORDIC_OUTMODE_SHIFTMODE_Msk)
                            >> VPRCSR_NORDIC_OUTMODE_SHIFTMODE_Pos)
                           == VPRCSR_NORDIC_OUTMODE_SHIFTMODE_Enabled ? true : false;
    p_mode->shift_size   = (reg & VPRCSR_NORDIC_OUTMODE_SHIFSIZE_Msk)
                           >> VPRCSR_NORDIC_OUTMODE_SHIFSIZE_Pos;
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_mode_out_set(nrf_vpr_csr_vio_mode_out_t const * p_mode)
{
    uint32_t reg = ((p_mode->shift_enable ? VPRCSR_NORDIC_OUTMODE_SHIFTMODE_Enabled :
                                            VPRCSR_NORDIC_OUTMODE_SHIFTMODE_Disabled)
                    << VPRCSR_NORDIC_OUTMODE_SHIFTMODE_Pos) |
                   ((p_mode->shift_size << VPRCSR_NORDIC_OUTMODE_SHIFSIZE_Pos)
                    & VPRCSR_NORDIC_OUTMODE_SHIFSIZE_Msk);

    nrf_csr_write(VPRCSR_NORDIC_OUTMODE, reg);
}

NRF_STATIC_INLINE uint32_t nrf_vpr_csr_vio_dirout_get(void)
{
    return nrf_csr_read(VPRCSR_NORDIC_DIROUT);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_set(uint32_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_DIROUT, value);
}

NRF_STATIC_INLINE uint32_t nrf_vpr_csr_vio_dirout_buffered_get(void)
{
    return nrf_csr_read(VPRCSR_NORDIC_DIROUTB);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_buffered_set(uint32_t value)
{
    nrf_csr_write(VPRCSR_NORDIC_DIROUTB, value);
}

NRF_STATIC_INLINE bool nrf_vpr_csr_vio_dirout_buffered_dirty_check(void)
{
    return ((nrf_csr_read(VPRCSR_NORDIC_DIROUTBS) & VPRCSR_NORDIC_DIROUTBS_DIRTYBIT_Msk)
            >> VPRCSR_NORDIC_DIROUTBS_DIRTYBIT_Pos) == VPRCSR_NORDIC_DIROUTBS_DIRTYBIT_DIRTY;
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_toggle_set(uint32_t mask)
{
    nrf_csr_write(VPRCSR_NORDIC_DIROUTTGL, mask);
}

NRF_STATIC_INLINE void nrf_vpr_csr_vio_dirout_toggle_buffered_set(uint32_t mask)
{
    nrf_csr_write(VPRCSR_NORDIC_DIROUTBTGL, mask);
}

#endif // NRF_DECLARE_ONLY

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRF_VPR_CSR_VIO_H__
