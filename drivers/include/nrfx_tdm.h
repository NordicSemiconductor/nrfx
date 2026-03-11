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

#ifndef NRFX_TDM_H__
#define NRFX_TDM_H__

#include <nrfx.h>
#include <hal/nrf_tdm.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup nrfx_tdm TDM driver
 * @{
 * @ingroup nrf_tdm
 * @brief   Time Division Multiplexed Audio Interface (TDM) peripheral driver.
 */

/** @brief Parameters used to describe TDM clock prescaling. */
typedef struct
{
    uint32_t base_clock_freq; ///< Freqency of the TDM base clock source.
    uint32_t mck_freq;        ///< Desired TDM Master clock frequency.
    uint32_t sck_freq;        ///< Desired TDM Serial clock frequency.
} nrfx_tdm_clk_params_t;

/** @brief TDM prescalers structure. */
typedef struct
{
    nrf_tdm_mck_div_t mck_div; ///< Master clock divider.
    nrf_tdm_sck_div_t sck_div; ///< Serial clock divider.
} nrfx_tdm_prescalers_t;

/** @brief TDM transfer buffers structure. */
typedef struct
{
    uint32_t       * p_rx_buffer;    ///< Pointer to the buffer for received data.
    uint16_t         rx_buffer_size; ///< Size of the buffer for received data in 32-bit words.
    uint32_t const * p_tx_buffer;    ///< Pointer to the buffer with data to be sent.
    uint16_t         tx_buffer_size; ///< Size of the buffer with data to be sent in 32-bit words.
} nrfx_tdm_buffers_t
;
/** @brief TDM driver configuration structure. */
typedef struct
{
    uint32_t                 sck_pin;         ///< Serial clock pin number.
    uint32_t                 fsync_pin;       ///< Frame Synchronization pin number.
    uint32_t                 mck_pin;         ///< Master clock pin number.
    uint32_t                 sdout_pin;       ///< Serial Data Output pin number.
    uint32_t                 sdin_pin;        ///< Serial Data Input pin number.
    uint8_t                  irq_priority;    ///< Interrupt priority.
    nrf_tdm_mode_t           mode;            ///< TDM mode of operation.
    nrf_tdm_align_t          alignment;       ///< Sample alignment within frame.
    nrf_tdm_swidth_t         sample_width;    ///< Sample width.
    nrf_tdm_channel_mask_t   rx_channel_mask; ///< Mask of enabled RX channels.
    nrf_tdm_channel_mask_t   tx_channel_mask; ///< Mask of enabled TX channels.
    uint8_t                  channel_number;  ///< Number of enabled channels.
    nrf_tdm_channel_delay_t  channel_delay;   ///< Channel delay settings.
    nrf_tdm_polarity_t       sck_polarity;    ///< Serial clock polarity.
    nrf_tdm_polarity_t       fsync_polarity;  ///< Frame Synchronization pulse polarity.
    nrf_tdm_fsync_duration_t fsync_duration;  ///< Frame Synchronization pulse duration.
    uint32_t                 ors;             ///< Over-read sample.
    nrf_tdm_src_t            mck_src;         ///< Master clock source.
    bool                     mck_bypass;      ///< Master clock bypass.
    nrf_tdm_src_t            sck_src;         ///< Serial clock source.
    bool                     sck_bypass;      ///< Serial clock bypass.
    nrfx_tdm_prescalers_t    prescalers;      ///< Clock prescalers.
    bool                     skip_gpio_cfg;   ///< Skip GPIO configuration of pins.
                                              /**< When set to true, the driver does not modify
                                               *   any GPIO parameters of the used pins. Those
                                               *   parameters are supposed to be configured
                                               *   externally before the driver is initialized. */
    bool                     skip_psel_cfg;   ///< Skip pin selection configuration.
                                              /**< When set to true, the driver does not modify
                                               *   pin select registers in the peripheral.
                                               *   Those registers are supposed to be set up
                                               *   externally before the driver is initialized.
                                               *   @note When both GPIO configuration and pin
                                               *   selection are to be skipped, the structure
                                               *   fields that specify pins can be omitted,
                                               *   as they are ignored anyway. */
} nrfx_tdm_config_t;

/**
 * @brief TDM driver default configuration.
 *
 * This configuration sets up TDM with the following options:
 * - Master mode
 * - Left alignment
 * - 16 bit sample width
 * - One RX channel enabled
 * - One TX channel enabled
 * - Delay of one clock pulse
 * - TX on the falling edge of SCK, RX on the rising edge of SCK
 * - Frame starts at falling edge of FSYNC.
 * - Zeros sent as over-read
 * - PCLK32M clock source
 * - MCK frequency divider equal to 8
 * - SCK frequency divider equal to 8
 *
 * @param[in] _sck_pin   Serial clock pin number.
 * @param[in] _fsync_pin Frame Synchronization pin number.
 * @param[in] _mck_pin   Master clock pin number.
 * @param[in] _sdout_pin Serial Data Output pin number.
 * @param[in] _sdin_pin  Serial Data Input pin number.
 */
#define NRFX_TDM_DEFAULT_CONFIG(_sck_pin, _fsync_pin, _mck_pin, _sdout_pin, _sdin_pin) \
{                                                                                      \
    .sck_pin         = _sck_pin,                                                       \
    .fsync_pin       = _fsync_pin,                                                     \
    .mck_pin         = _mck_pin,                                                       \
    .sdout_pin       = _sdout_pin,                                                     \
    .sdin_pin        = _sdin_pin,                                                      \
    .irq_priority    = NRFX_TDM_DEFAULT_CONFIG_IRQ_PRIORITY,                           \
    .mode            = NRF_TDM_MODE_MASTER,                                            \
    .alignment       = NRF_TDM_ALIGN_LEFT,                                             \
    .sample_width    = NRF_TDM_SWIDTH_16BIT,                                           \
    .rx_channel_mask = NRF_TDM_CHANNEL_RX0_MASK,                                       \
    .tx_channel_mask = NRF_TDM_CHANNEL_TX0_MASK,                                       \
    .channel_number  = 2,                                                              \
    .channel_delay   = NRF_TDM_CHANNEL_DELAY_1CK,                                      \
    .sck_polarity    = NRF_TDM_POLARITY_POSEDGE,                                       \
    .fsync_polarity  = NRF_TDM_POLARITY_NEGEDGE,                                       \
    .fsync_duration  = NRF_TDM_FSYNC_DURATION_CHANNEL,                                 \
    .ors             = 0,                                                              \
    .mck_src         = NRF_TDM_SRC_PCLK32M,                                            \
    .sck_src         = NRF_TDM_SRC_PCLK32M,                                            \
    .prescalers      =                                                                 \
    {                                                                                  \
        .mck_div     = NRF_TDM_MCK_DIV_8,                                              \
        .sck_div     = NRF_TDM_SCK_DIV_8,                                              \
    },                                                                                 \
}

/** @brief Maximum number of TDM channels. */
#define NRFX_TDM_NUM_OF_CHANNELS (TDM_CONFIG_CHANNEL_NUM_NUM_Max + 1)

/** @brief Minimum TDM transfer size in bytes. */
#define NRFX_TDM_MIN_TRANSFER_SIZE TDM_MIN_TRANSFER_SIZE

/** @brief TDM status flag indicating that the application must provide buffers that
 *  are to be used in the next part of the transfer. A call to @ref nrfx_tdm_next_buffers_set must
 *  be done before the currently used buffers are completely processed
 *  (that is, the time remaining for supplying the next buffers depends on
 *  the used size of the buffers).
 */
#define NRFX_TDM_STATUS_NEXT_BUFFERS_NEEDED (1UL << 0)

/** @brief TDM status flag indicating that The peripheral has been stopped and
 *  all buffers that were passed to the driver have been released.
 */
#define NRFX_TDM_STATUS_TRANSFER_STOPPED    (1UL << 1)

/**
 * @brief TDM driver data handler type.
 *
 * A data handling function of this type must be specified during the initialization
 * of the driver. The driver will call this function when it finishes transfer using
 * buffers passed to it by the application, and when it needs to be provided
 * with buffers for the next part of the transfer.
 *
 * @note The @c p_released pointer passed to this function is temporary and
 *       will be invalid after the function returns, hence it cannot be stored
 *       and used later. If needed, the pointed content (that is, buffers pointers)
 *       must be copied instead.
 *
 * @param[in] p_released  Pointer to a structure with pointers to buffers
 *                        passed previously to the driver that will no longer
 *                        be accessed by it (they can be now safely released or
 *                        used for another purpose, in particular for a next
 *                        part of the transfer).
 *                        This pointer will be NULL if the application did not
 *                        supply the buffers for the next part of the transfer
 *                        (via a call to @ref nrfx_tdm_next_buffers_set) since
 *                        the previous time the data handler signaled such need.
 *                        This means that data corruption occurred (the previous
 *                        buffers are used for the second time) and no buffers
 *                        can be released at the moment.
 *                        Both pointers in this structure are NULL when the
 *                        handler is called for the first time after a transfer
 *                        is started, because no data has been transferred yet
 *                        at this point. In all successive calls, the pointers
 *                        specify what has been sent (TX) and what has been
 *                        received (RX) in the part of the transfer that has
 *                        just been completed (provided that a given direction
 *                        is enabled, see @ref nrfx_tdm_start).
 *                        @note Since the peripheral is stopped asynchronously,
 *                              buffers that are released after the call to
 *                              @ref nrfx_tdm_stop are not used entirely.
 *                              In this case, only a part (if any) of the TX
 *                              buffer has been actually transmitted and only
 *                              a part (if any) of the RX buffer is filled with
 *                              received data.
 * @param[in] status  Bit field describing the current status of the transfer.
 *                    It can be 0 or a combination of the following flags:
 *                    - @ref NRFX_TDM_STATUS_NEXT_BUFFERS_NEEDED
 *                    - @ref NRFX_TDM_STATUS_TRANSFER_STOPPED
 */
typedef void (* nrfx_tdm_data_handler_t)(nrfx_tdm_buffers_t const * p_released,
                                         uint32_t                   status);

/** @cond Driver internal data. */
typedef struct
{
    nrfx_tdm_data_handler_t handler;
    nrfx_drv_state_t        state;
    bool                    skip_gpio_cfg;
    nrfx_tdm_buffers_t      current_buffers;
    nrfx_tdm_buffers_t      next_buffers;
    bool                    use_rx;
    bool                    use_tx;
    bool                    rx_ready;
    bool                    tx_ready;
    bool                    buffers_needed;
    bool                    buffers_reused;
} nrfx_tdm_control_block_t;
/** @endcond */

/** @brief TDM driver instance data structure. */
typedef struct
{
    NRF_TDM_Type *           p_reg; ///< Pointer to a structure with TDM instance registers.
    nrfx_tdm_control_block_t cb;    ///< Driver internal data.
} nrfx_tdm_t;

/** @brief Macro for creating a TDM driver instance. */
#define NRFX_TDM_INSTANCE(reg)    \
{                                 \
    .p_reg = (NRF_TDM_Type *)reg, \
    .cb = {0},                    \
}

/**
 * @brief Function for initializing the TDM driver.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_config   Pointer to the structure with the initial configuration.
 * @param[in] handler    Data handler provided by the user.
 *
 * @retval 0         Initialization was successful.
 * @retval -EALREADY The driver is already initialized.
 * @retval -EINVAL   The requested combination of configuration
 *                   options is not allowed by the TDM peripheral.
 */
int nrfx_tdm_init(nrfx_tdm_t *              p_instance,
                  nrfx_tdm_config_t const * p_config,
                  nrfx_tdm_data_handler_t   handler);

/**
 * @brief Function for uninitializing the TDM driver.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_tdm_uninit(nrfx_tdm_t * p_instance);

/**
 * @brief Function for checking if the TDM driver instance is initialized.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 *
 * @retval true  Instance is already initialized.
 * @retval false Instance is not initialized.
 */
bool nrfx_tdm_init_check(nrfx_tdm_t const * p_instance);

/**
 * @brief Function for reconfiguring the TDM driver.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_config   Pointer to the structure with the configuration.
 *
 * @retval 0            Reconfiguration was successful.
 * @retval -EINPROGRESS The driver is uninitialized.
 * @retval -EINVAL      The requested combination of configuration
 *                      options is not allowed by the TDM peripheral.
 */
int nrfx_tdm_reconfigure(nrfx_tdm_t *              p_instance,
                         nrfx_tdm_config_t const * p_config);

/**
 * @brief Function for starting the continuous TDM transfer.
 *
 * The TDM data transfer can be performed in one of three modes: RX (reception)
 * only, TX (transmission) only, or in both directions simultaneously.
 * The mode is selected by specifying a proper buffer for a given direction
 * in the call to this function or by passing NULL instead if this direction
 * is to be disabled.
 *
 * The length of the buffer (which is a common value for RX and TX if both
 * directions are enabled) is specified in 32-bit words. One 32-bit memory
 * word can either contain four 8-bit samples, two 16-bit samples, or one
 * right-aligned 24-bit sample sign-extended to a 32-bit value.
 * For a detailed memory mapping for different supported configurations,
 * see the Product Specification.
 *
 * @note Peripherals using EasyDMA (including TDM) require the transfer buffers
 *       to be placed in the Data RAM region. If this condition is not met,
 *       this function will fail with the error code -EACCES.
 *
 * @param[in] p_instance        Pointer to the driver instance structure.
 * @param[in] p_initial_buffers Pointer to a structure specifying the buffers
 *                              to be used in the initial part of the transfer
 *                              (buffers for all consecutive parts are provided
 *                              through the data handler).
 *
 * @retval 0            The operation was successful.
 * @retval -EINPROGRESS The driver has not been initialized.
 * @retval -EALREADY    Transfer has already been already started.
 * @retval -EINVAL      No buffers were provided or the provided transfer length is too short.
 * @retval -EACCES      The provided buffers are not placed in the Data RAM region.
 */
int nrfx_tdm_start(nrfx_tdm_t *               p_instance,
                   nrfx_tdm_buffers_t const * p_initial_buffers);

/**
 * @brief Function for supplying the buffers to be used in the next part of
 *        the transfer.
 *
 * The application must call this function when the data handler receives
 * @ref NRFX_TDM_STATUS_NEXT_BUFFERS_NEEDED in the @c status parameter.
 * The call can be done immediately from the data handler function or later,
 * but it has to be done before the TDM peripheral finishes processing the
 * buffers supplied previously. Otherwise, data corruption will occur.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] p_buffers  Pointer to a structure specifying the buffers
 *                       to be used in the upcoming part of the transfer.
 *
 * @retval 0            If the operation was successful.
 * @retval -EINPROGRESS If the buffers were already supplied or
 *                      the peripheral is currently being stopped.
 * @retval -EINVAL      Required buffers were not provided or the provided
 *                      transfer length is too short.
 * @retval -EACCES      The provided buffers are not placed in the Data RAM region.
 */
int nrfx_tdm_next_buffers_set(nrfx_tdm_t *               p_instance,
                              nrfx_tdm_buffers_t const * p_buffers);

/**
 * @brief Function for stopping the TDM transfer.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] abort      True if transfer is to be stopped immediately,
 *                       false if transfer is to be stopped only after
 *                       currently processed buffers are filled.
 */
void nrfx_tdm_stop(nrfx_tdm_t * p_instance, bool abort);

/**
 * @brief Function for calculating TDM clock prescaler values.
 *
 * Call this function to find suitable value for prescalers in
 * @ref nrfx_tdm_config_t structure.
 *
 * @param[in]  clk_params Parameters used to describe clock prescaling.
 * @param[out] prescalers Prescaler structure pointer to be filled with prescaler values.
 *
 * @retval 0        Suitable prescaler values were found.
 * @retval -EINVAL  No suitable prescaler values were found.
 */
int nrfx_tdm_prescalers_calc(nrfx_tdm_clk_params_t const * clk_params,
                             nrfx_tdm_prescalers_t *       prescalers);

/**
 * @brief Driver interrupt handler.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 */
void nrfx_tdm_irq_handler(nrfx_tdm_t * p_instance);

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_TDM_H__
