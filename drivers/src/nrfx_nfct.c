/*
 * Copyright (c) 2018 - 2026, Nordic Semiconductor ASA
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

#include <nrfx.h>
#include <nrfx_nfct.h>
#include <hal/nrf_ficr.h>

#define NRFX_LOG_MODULE NFCT
#include <nrfx_log.h>

#define FIELD_TIMER_FREQUENCY_HZ NRFX_MHZ_TO_HZ(1)

#if NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) || \
    NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)
#include <nrfx_timer.h>

typedef struct
{
    nrfx_timer_t       timer;                     /**< Timer instance that supports the correct NFC field detection. */
#if NRF_ERRATA_STATIC_CHECK(52, 190) || NRF_ERRATA_STATIC_CHECK(53, 70) || \
    NRF_ERRATA_STATIC_CHECK(54L, 60)
    bool               fieldevents_filter_active; /**< Flag that indicates that the field events are ignored. */
    bool               is_hfclk_on;               /**< HFCLK has started - one of the NFC activation conditions. */
    bool               is_delayed;                /**< Required time delay has passed - one of the NFC activation conditions. */
#elif NRF_ERRATA_STATIC_CHECK(52, 79)
    uint32_t           field_state_cnt;           /**< Counter of the FIELDLOST events. */
#endif
} nrfx_nfct_timer_workaround_t;

#if NRF_ERRATA_STATIC_CHECK(52, 190) || NRF_ERRATA_STATIC_CHECK(53, 70) || \
    NRF_ERRATA_STATIC_CHECK(54L, 60)
    #define NRFX_NFCT_ACTIVATE_DELAY     1000 /**< Minimal delay in us between NFC field detection and activation of NFCT. */
    #define NRFX_NFCT_TIMER_PERIOD       NRFX_NFCT_ACTIVATE_DELAY
#elif  NRF_ERRATA_STATIC_CHECK(52, 79)
    #define NRFX_NFCT_FIELDLOST_THR      7
    #define NRFX_NFCT_FIELD_TIMER_PERIOD 100  /**< Field polling period in us. */
    #define NRFX_NFCT_TIMER_PERIOD       NRFX_NFCT_FIELD_TIMER_PERIOD
#endif

static nrfx_nfct_timer_workaround_t m_timer_workaround =
{
    .timer = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(NRFX_NFCT_CONFIG_TIMER_INSTANCE_ID)),
};
#endif // NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) ||
       // NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)

/* Mask of all possible interrupts that are relevant for data reception. */
#define NRFX_NFCT_RX_INT_MASK (NRF_NFCT_INT_RXFRAMESTART_MASK | \
                               NRF_NFCT_INT_RXFRAMEEND_MASK)

/* Mask of all possible interrupts that are relevant for data transmission. */
#define NRFX_NFCT_TX_INT_MASK (NRF_NFCT_INT_TXFRAMESTART_MASK | \
                               NRF_NFCT_INT_TXFRAMEEND_MASK)


/* Mask of all possible errors from the @ref NRF_NFCT_EVENT_RXERROR event. */
#define NRFX_NFCT_FRAME_STATUS_RX_ALL_MASK (NRF_NFCT_RX_FRAME_STATUS_CRC_MASK    | \
                                            NRF_NFCT_RX_FRAME_STATUS_PARITY_MASK | \
                                            NRF_NFCT_RX_FRAME_STATUS_OVERRUN_MASK)

/* Macros for conversion of bits to bytes. */
#define NRFX_NFCT_BYTES_TO_BITS(_bytes) ((_bytes) << 3UL)
#define NRFX_NFCT_BITS_TO_BYTES(_bits)  ((_bits)  >> 3UL)

/* Macro for checking whether the NFCT interrupt is active. */
#define NRFX_NFCT_EVT_ACTIVE(_name, _mask)                                             \
    (((_mask) & NRFY_EVENT_TO_INT_BITMASK(NRFX_CONCAT_2(NRF_NFCT_EVENT_, _name))) &&   \
     nrfy_nfct_int_enable_check(NRF_NFCT, NRFX_CONCAT_3(NRF_NFCT_INT_, _name, _MASK)))

/* Macro for callback execution. */
#define NRFX_NFCT_CB_HANDLE(_cb, _evt) \
    if (_cb != NULL)                   \
    {                                  \
        _cb(&_evt);                    \
    }

typedef enum
{
    NRFX_NFC_FIELD_STATE_NONE,   /**< Initial value that indicates no NFCT field events. */
    NRFX_NFC_FIELD_STATE_OFF,    /**< The NFCT FIELDLOST event has been set. */
    NRFX_NFC_FIELD_STATE_ON,     /**< The NFCT FIELDDETECTED event has been set. */
    NRFX_NFC_FIELD_STATE_UNKNOWN /**< Both NFCT field events have been set - ambiguous state. */
} nrfx_nfct_field_state_t;

/** @brief NFCT control block. */
typedef struct
{
    nrfx_nfct_config_t config;
    nrfx_drv_state_t   state;
    volatile bool      field_on;
    uint32_t           frame_delay_max;
    uint32_t           frame_delay_min;
} nrfx_nfct_control_block_t;

static nrfx_nfct_control_block_t m_nfct_cb;

static const uint32_t m_status_all_msk = NRF_NFCT_ERROR_FRAMEDELAYTIMEOUT_MASK |
#if NRF_NFCT_HAS_ERROR_FIELD_TOO_STRONG
                                         NRF_NFCT_ERROR_NFCFIELDTOOSTRONG_MASK |
#endif
#if NRF_NFCT_HAS_ERROR_FIELD_TOO_WEAK
                                         NRF_NFCT_ERROR_NFCFIELDTOOWEAK_MASK |
#endif
                                         0;

/**
 * @brief Common part of the setup used for the NFCT initialization and reinitialization.
 */
static void nfct_hw_init_setup(void)
{
    /* Use Window Grid frame delay mode. */
    nrfy_nfct_frame_delay_mode_set(NRF_NFCT, NRF_NFCT_FRAME_DELAY_MODE_WINDOWGRID);

    /* Change the bit frame SDD to 00100 to improve interoperability. */
    nrfy_nfct_sensres_bit_frame_sdd_set(NRF_NFCT, NRF_NFCT_SENSRES_BIT_FRAME_SDD_00100);
}

static void nfct_frame_delay_max_set(bool default_delay)
{
    if (default_delay)
    {
        nrfy_nfct_frame_delay_max_set(NRF_NFCT, NRF_NFCT_FAME_DELAY_MAX_DEFAULT);
    }
    else
    {
        nrfy_nfct_frame_delay_max_set(NRF_NFCT, m_nfct_cb.frame_delay_max);
    }
}

static void nfct_trims_apply(void)
{
#if NRFY_NFCT_HAS_BIAS_CONFIG_TRIM_REG && defined(FICR_TRIM_GLOBAL_NFCT_BIASCFG_VALUE_Msk)
    nrf_nfct_bias_config_t bias_cfg;

    bias_cfg.trim_ibpsr = (NRF_FICR->TRIM.GLOBAL.NFCT.BIASCFG & NFCT_BIASCFG_TRIMIBPSR_Msk)
                          >> NFCT_BIASCFG_TRIMIBPSR_Pos;

    bias_cfg.coarse_ibpsr = (NRF_FICR->TRIM.GLOBAL.NFCT.BIASCFG & NFCT_BIASCFG_COARSEIBPSR_Msk)
                            >> NFCT_BIASCFG_COARSEIBPSR_Pos;

    bias_cfg.reference_volatge = (NRF_FICR->TRIM.GLOBAL.NFCT.BIASCFG &
                                  NFCT_BIASCFG_REFERENCEVOLTAGE_Msk)
                                 >> NFCT_BIASCFG_REFERENCEVOLTAGE_Pos;

    bias_cfg.spare = (NRF_FICR->TRIM.GLOBAL.NFCT.BIASCFG & NFCT_BIASCFG_SPARE_Msk)
                     >>  NFCT_BIASCFG_SPARE_Pos;

    nrfy_nfct_bias_config_set(NRF_NFCT, &bias_cfg);
#endif
}

/** @brief Function for evaluating and handling the NFC field events.
 *
 * @param[in]  field_state  Current field state.
 */
static void nfct_field_event_handler(volatile nrfx_nfct_field_state_t field_state)
{
    nrfx_nfct_evt_t nfct_evt;

#if NRF_ERRATA_STATIC_CHECK(52, 190) || NRF_ERRATA_STATIC_CHECK(53, 70) || \
    NRF_ERRATA_STATIC_CHECK(54L, 60)
    if ((NRF_ERRATA_DYNAMIC_CHECK(52, 190) || NRF_ERRATA_DYNAMIC_CHECK(53, 70) ||
         NRF_ERRATA_DYNAMIC_CHECK(54L, 60)) &&
        m_timer_workaround.fieldevents_filter_active)
    {
        return;
    }
#endif

    if (field_state == NRFX_NFC_FIELD_STATE_UNKNOWN)
    {
        /* Probe NFC field */
        field_state = (nrfx_nfct_field_check()) ? NRFX_NFC_FIELD_STATE_ON :
                                                  NRFX_NFC_FIELD_STATE_OFF;
    }

    /* Field event service. Only take action on field transition -
     * based on the value of m_nfct_cb.field_on
     */
    switch (field_state)
    {
        case NRFX_NFC_FIELD_STATE_ON:
            if (!m_nfct_cb.field_on)
            {
#if NRF_ERRATA_STATIC_CHECK(52, 190) || NRF_ERRATA_STATIC_CHECK(53, 70) || \
    NRF_ERRATA_STATIC_CHECK(54L, 60)
                if (NRF_ERRATA_DYNAMIC_CHECK(52, 190) || NRF_ERRATA_DYNAMIC_CHECK(53, 70) ||
                    NRF_ERRATA_DYNAMIC_CHECK(54L, 60)) {
                    m_timer_workaround.is_hfclk_on               = false;
                    m_timer_workaround.is_delayed                = false;
                    m_timer_workaround.fieldevents_filter_active = true;

                    nrfx_timer_clear(&m_timer_workaround.timer);
                    nrfx_timer_enable(&m_timer_workaround.timer);
                }
#endif
#if NRF_ERRATA_STATIC_CHECK(52, 79)
                if (NRF_ERRATA_DYNAMIC_CHECK(52, 79))
                {
                    nrfx_timer_clear(&m_timer_workaround.timer);
                    nrfx_timer_enable(&m_timer_workaround.timer);
                    m_timer_workaround.field_state_cnt = 0;
                }
#endif

                m_nfct_cb.field_on = true;
                nfct_evt.evt_id    = NRFX_NFCT_EVT_FIELD_DETECTED;
                NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);
            }
            break;

        case NRFX_NFC_FIELD_STATE_OFF:
            if (m_nfct_cb.field_on)
            {
                nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_SENSE);
                nrfy_nfct_int_disable(NRF_NFCT, NRFX_NFCT_RX_INT_MASK | NRFX_NFCT_TX_INT_MASK);
                m_nfct_cb.field_on = false;
                nfct_evt.evt_id    = NRFX_NFCT_EVT_FIELD_LOST;

                if (NRF_ERRATA_DYNAMIC_CHECK(52, 218) || NRF_ERRATA_DYNAMIC_CHECK(53, 71))
                {
                    nfct_frame_delay_max_set(true);
                }

                NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);
            }
            break;

        default:
            /* No implementation required */
            break;
    }
}

#if NRF_ERRATA_STATIC_CHECK(52, 190) || NRF_ERRATA_STATIC_CHECK(53, 70) || \
    NRF_ERRATA_STATIC_CHECK(54L, 60)
static void nfct_activate_check(void)
{
    static bool is_field_validation_pending = false;

    if (is_field_validation_pending)
    {
        is_field_validation_pending                  = false;
        m_timer_workaround.fieldevents_filter_active = false;

        // Check the field status and take action if field is lost.
        nfct_field_event_handler(NRFX_NFC_FIELD_STATE_UNKNOWN);
        return;
    }

    if ((m_timer_workaround.is_hfclk_on) && (m_timer_workaround.is_delayed))
    {
        nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_ACTIVATE);
        is_field_validation_pending = true;

        // Start the timer second time to validate whether the tag has locked to the field.
        nrfx_timer_clear(&m_timer_workaround.timer);
        nrfx_timer_enable(&m_timer_workaround.timer);
    }
}
#endif

#if NRF_ERRATA_STATIC_CHECK(52, 141)
static inline void nfct_reset(void)
{
    nrfy_nfct_parameters_t nfct_params;

    // Save parameter settings before the reset of the NFCT peripheral.
    nrfy_nfct_parameters_save(NRF_NFCT, &nfct_params);

    // Reset the NFCT peripheral.
    *(volatile uint32_t *)0x40005FFC = 0;
    *(volatile uint32_t *)0x40005FFC;
    *(volatile uint32_t *)0x40005FFC = 1;

    // Restore parameter settings after the reset of the NFCT peripheral.
    nrfy_nfct_parameters_restore(NRF_NFCT, &nfct_params);

    // Restore general HW configuration.
    nfct_hw_init_setup();

    // Restore interrupts.
    nrfy_nfct_int_enable(NRF_NFCT, nfct_params.int_enabled);

    // Disable interrupts associated with data exchange.
    nrfy_nfct_int_disable(NRF_NFCT, NRFX_NFCT_RX_INT_MASK | NRFX_NFCT_TX_INT_MASK);

    NRFX_LOG_INFO("Reinitialize");
}
#endif

#if NRF_ERRATA_STATIC_CHECK(52, 79)
static void nfct_field_poll(void)
{
    if (!nrfx_nfct_field_check())
    {
        if (++m_timer_workaround.field_state_cnt > NRFX_NFCT_FIELDLOST_THR)
        {
            nrfx_nfct_evt_t nfct_evt =
            {
                .evt_id = NRFX_NFCT_EVT_FIELD_LOST,
            };

            nrfx_timer_disable(&m_timer_workaround.timer);
            m_nfct_cb.field_on = false;

            nfct_frame_delay_max_set(true);

#if NRF_ERRATA_STATIC_CHECK(52, 141)
            if (NRF_ERRATA_DYNAMIC_CHECK(52, 141))
            {
                nfct_reset();
            }
#endif

            NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);
        }
        return;
    }

    m_timer_workaround.field_state_cnt = 0;
}
#endif

#if NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) || \
    NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)
static void nfct_field_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    (void)p_context;

    if (event_type != NRF_TIMER_EVENT_COMPARE0)
    {
        return;
    }

#if NRF_ERRATA_STATIC_CHECK(52, 190) || NRF_ERRATA_STATIC_CHECK(53, 70) || \
    NRF_ERRATA_STATIC_CHECK(54L, 60)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 190) || NRF_ERRATA_DYNAMIC_CHECK(53, 70) ||
        NRF_ERRATA_DYNAMIC_CHECK(54L, 60))
    {
        m_timer_workaround.is_delayed = true;
        nrfx_timer_disable(&m_timer_workaround.timer);
        nfct_activate_check();
    }
#endif
#if NRF_ERRATA_STATIC_CHECK(52, 79)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 79))
    {
        nfct_field_poll();
    }
#endif
}

static inline int nfct_field_timer_config(uint8_t irq_priority)
{
    int err_code;
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(FIELD_TIMER_FREQUENCY_HZ);
    timer_cfg.interrupt_priority = irq_priority;

    err_code = nrfx_timer_init(&m_timer_workaround.timer,
                               &timer_cfg,
                               nfct_field_timer_handler);
    if (err_code != 0)
    {
        return err_code;
    }

    nrfx_timer_extended_compare(&m_timer_workaround.timer,
                                NRF_TIMER_CC_CHANNEL0,
                                nrfx_timer_us_to_ticks(&m_timer_workaround.timer,
                                                       NRFX_NFCT_TIMER_PERIOD),
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                true);
    return 0;
}
#endif // NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) ||
       // NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)

static inline
nrf_nfct_sensres_nfcid1_size_t nfct_nfcid1_size_to_sensres_size(uint8_t nfcid1_size)
{
    switch (nfcid1_size)
    {
        case NRFX_NFCT_NFCID1_SINGLE_SIZE:
            return NRF_NFCT_SENSRES_NFCID1_SIZE_SINGLE;

        case NRFX_NFCT_NFCID1_DOUBLE_SIZE:
            return NRF_NFCT_SENSRES_NFCID1_SIZE_DOUBLE;

        case NRFX_NFCT_NFCID1_TRIPLE_SIZE:
            return NRF_NFCT_SENSRES_NFCID1_SIZE_TRIPLE;

        default:
            return NRF_NFCT_SENSRES_NFCID1_SIZE_DOUBLE;
    }
}

static inline void nrfx_nfct_rxtx_int_enable(uint32_t rxtx_int_mask)
{
    nrfy_nfct_int_enable(NRF_NFCT, rxtx_int_mask & m_nfct_cb.config.rxtx_int_mask);
}

static void nfct_stop_tx(void)
{
#if NRF_NFCT_HAS_STOPTX_TASK
    nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_STOPTX);
#else
#if defined(NRF52_SERIES)
    *(volatile uint32_t *)0x40005010 = 0x01;
#elif defined(NRF5340_XXAA_APPLICATION) && defined(NRF_TRUSTZONE_NONSECURE)
    *(volatile uint32_t *)0x4002D010 = 0x01;
#elif defined(NRF5340_XXAA_APPLICATION)
    *(volatile uint32_t *)0x5002D010 = 0x01;
#endif
#endif // NRF_NFCT_HAS_STOPTX_TASK
}

int nrfx_nfct_init(nrfx_nfct_config_t const * p_config)
{
    NRFX_ASSERT(p_config);

    int err_code = 0;

    if (m_nfct_cb.state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    nfct_trims_apply();

#if NRFY_NFCT_HAS_PAD_CONFIG_REG
    /* Make sure that NFC pads are configured as NFCT antenna pins. */
    if (!nrfy_nfct_pad_config_enable_check(NRF_NFCT))
    {
        err_code = -EPERM;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        NRFX_LOG_ERROR("NFCT pads are not configured as NFCT antenna pins");
        return err_code;
    }
#endif

    m_nfct_cb.config = *p_config;
    nfct_hw_init_setup();

    nrfy_nfct_int_init(NRF_NFCT, p_config->rxtx_int_mask, p_config->irq_priority, false);

#if NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) || \
    NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 79) || NRF_ERRATA_DYNAMIC_CHECK(52, 190) ||
        NRF_ERRATA_DYNAMIC_CHECK(53, 70) || NRF_ERRATA_DYNAMIC_CHECK(54L, 60))
    {
        err_code = nfct_field_timer_config(p_config->irq_priority);
    }
#endif

    m_nfct_cb.state           = NRFX_DRV_STATE_INITIALIZED;
    m_nfct_cb.field_on        = false;
    m_nfct_cb.frame_delay_max = NRF_NFCT_FAME_DELAY_MAX_DEFAULT;
    m_nfct_cb.frame_delay_min = NRF_NFCT_FAME_DELAY_MIN_DEFAULT;

    NRFX_LOG_INFO("Initialized.");
    return err_code;
}

void nrfx_nfct_uninit(void)
{
    nrfx_nfct_disable();

    nrfy_nfct_int_uninit(NRF_NFCT);

#if NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) || \
    NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)
    if (NRF_ERRATA_DYNAMIC_CHECK(52, 79) || NRF_ERRATA_DYNAMIC_CHECK(52, 190) ||
        NRF_ERRATA_DYNAMIC_CHECK(53, 70) || NRF_ERRATA_DYNAMIC_CHECK(54L, 60))
    {
        nrfx_timer_uninit(&m_timer_workaround.timer);
    }
#endif

    m_nfct_cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    NRFX_LOG_INFO("Uninitialized.");
}

bool nrfx_nfct_init_check(void)
{
    return (m_nfct_cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

void nrfx_nfct_enable(void)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_nfct_error_status_clear(NRF_NFCT, m_status_all_msk);
    nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_SENSE);

    nrfy_nfct_int_enable(NRF_NFCT, NRF_NFCT_INT_FIELDDETECTED_MASK |
                                   NRF_NFCT_INT_ERROR_MASK         |
                                   NRF_NFCT_INT_SELECTED_MASK      |
                                   (NRF_ERRATA_DYNAMIC_CHECK(52, 79) ?
                                    0 : NRF_NFCT_INT_FIELDLOST_MASK));

    NRFX_LOG_INFO("Start");
}

void nrfx_nfct_disable(void)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);

    nrfy_nfct_int_disable(NRF_NFCT, NRF_NFCT_DISABLE_ALL_INT);
    nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_DISABLE);

    NRFX_LOG_INFO("Stop");
}

bool nrfx_nfct_field_check(void)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);

    uint32_t const field_state = nrfy_nfct_field_status_get(NRF_NFCT);

    if (((field_state & NRF_NFCT_FIELD_STATE_PRESENT_MASK) == 0) &&
        ((field_state & NRF_NFCT_FIELD_STATE_LOCK_MASK) == 0))
    {
        /* Field is not active */
        return false;
    }

    return true;
}

int nrfx_nfct_rx(nrfx_nfct_data_desc_t const * p_rx_data)
{
    int err;

    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_rx_data);

    // EasyDMA requires that transfer buffers are placed in DataRAM,
    // signal error if they are not.
    if (!nrf_dma_accessible_check(NRF_NFCT, p_rx_data->p_data))
    {
        err = -EACCES;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err));
        return err;
    }

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

    nrfy_nfct_rxtx_buffer_set(NRF_NFCT,
                              (uint8_t *)p_rx_data->p_data,
                              (uint16_t)p_rx_data->data_size,
                              true);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

    nrfx_nfct_rxtx_int_enable(NRFX_NFCT_RX_INT_MASK);
    nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_ENABLERXDATA);

    return 0;
}

int nrfx_nfct_tx(nrfx_nfct_data_desc_t const * p_tx_data,
                 nrf_nfct_frame_delay_mode_t   delay_mode)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_tx_data);
    NRFX_ASSERT(p_tx_data->p_data);

    int err = 0;

    if (p_tx_data->data_size == 0)
    {
        return -E2BIG;
    }

    // EasyDMA requires that transfer buffers are placed in DataRAM,
    // signal error if they are not.
    if (!nrf_dma_accessible_check(NRF_NFCT, p_tx_data->p_data))
    {
        err = -EACCES;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err));
        return err;
    }

    NRFX_CRITICAL_SECTION_ENTER();

    /* In case when NFC frame transmission has already started, it returns an error. */
    if (nrfy_nfct_event_check(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMESTART) &&
        nrfy_nfct_int_enable_check(NRF_NFCT, NRF_NFCT_INT_TXFRAMESTART_MASK))
    {
        err = -EBUSY;
    }
    else
    {
        /* In case when Tx operation was scheduled with delay, stop scheduled Tx operation. */
        nfct_stop_tx();

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

        nrfy_nfct_rxtx_buffer_set(NRF_NFCT,
                                  (uint8_t *)p_tx_data->p_data,
                                  (uint16_t)p_tx_data->data_size,
                                  false);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

        nrfy_nfct_tx_bits_set(NRF_NFCT, (uint16_t)NRFX_NFCT_BYTES_TO_BITS(p_tx_data->data_size));
        nrfy_nfct_frame_delay_mode_set(NRF_NFCT, (nrf_nfct_frame_delay_mode_t) delay_mode);
        nfct_frame_delay_max_set(false);

        nrfx_nfct_rxtx_int_enable(NRFX_NFCT_TX_INT_MASK);
        nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_STARTTX);
    }

    NRFX_CRITICAL_SECTION_EXIT();

    if (err == 0)
    {
        NRFX_LOG_INFO("Tx start");
    }

    return err;
}

int nrfx_nfct_bits_tx(nrfx_nfct_data_desc_t const * p_tx_data,
                      nrf_nfct_frame_delay_mode_t   delay_mode)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_tx_data);
    NRFX_ASSERT(p_tx_data->p_data);

    int err = 0;

    if (p_tx_data->data_size == 0)
    {
        return -E2BIG;
    }

    // EasyDMA requires that transfer buffers are placed in DataRAM,
    // signal error if they are not.
    if (!nrf_dma_accessible_check(NRF_NFCT, p_tx_data->p_data))
    {
        err = -EACCES;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err));
        return err;
    }

    /* Get buffer length, add additional byte if bits go beyond last whole byte */
    uint32_t buffer_length = NRFX_NFCT_BITS_TO_BYTES(p_tx_data->data_size);
    if (p_tx_data->data_size & NFCT_TXD_AMOUNT_TXDATABITS_Msk)
    {
        ++buffer_length;
    }

    NRFX_CRITICAL_SECTION_ENTER();

    /* In case when NFC frame transmission has already started, it returns an error. */
    if (nrfy_nfct_event_check(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMESTART) &&
        nrfy_nfct_int_enable_check(NRF_NFCT, NRF_NFCT_INT_TXFRAMESTART_MASK))
    {
        err = -EBUSY;
    }
    else
    {
        /* In case when Tx operation was scheduled with delay, stop scheduled Tx operation. */
        nfct_stop_tx();

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

        nrfy_nfct_rxtx_buffer_set(NRF_NFCT,
                                  (uint8_t *)p_tx_data->p_data,
                                  (uint16_t)buffer_length,
                                  false);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

        nrfy_nfct_tx_bits_set(NRF_NFCT, (uint16_t)p_tx_data->data_size);
        nrfy_nfct_frame_delay_mode_set(NRF_NFCT, (nrf_nfct_frame_delay_mode_t)delay_mode);
        nfct_frame_delay_max_set(false);

        nrfx_nfct_rxtx_int_enable(NRFX_NFCT_TX_INT_MASK);
        nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_STARTTX);
    }

    NRFX_CRITICAL_SECTION_EXIT();

    if (err == 0)
    {
        NRFX_LOG_INFO("Tx start");
    }

    return err;
}

void nrfx_nfct_state_force(nrfx_nfct_state_t state)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);

#if NRF_ERRATA_STATIC_CHECK(52, 190) || NRF_ERRATA_STATIC_CHECK(53, 70) || \
    NRF_ERRATA_STATIC_CHECK(54L, 60)
    if ((NRF_ERRATA_DYNAMIC_CHECK(52, 190) || NRF_ERRATA_DYNAMIC_CHECK(53, 70) ||
         NRF_ERRATA_DYNAMIC_CHECK(54L, 60)) &&
        (state == NRFX_NFCT_STATE_ACTIVATED))
    {
        m_timer_workaround.is_hfclk_on = true;
        /* NFCT will be activated based on additional conditions */
        nfct_activate_check();
        return;
    }
#endif
    nrfy_nfct_task_trigger(NRF_NFCT, (nrf_nfct_task_t) state);
}

void nrfx_nfct_init_substate_force(nrfx_nfct_active_state_t sub_state)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);

    if (sub_state == NRFX_NFCT_ACTIVE_STATE_DEFAULT)
    {
#if defined(NFCT_SLEEPSTATE_SLEEPSTATE_Msk)
        if (nrf_nfct_sleep_state_get(NRF_NFCT) == NRF_NFCT_SLEEP_STATE_SLEEP_A)
#else
        if (((*(uint32_t volatile *)(0x40005420)) & 0x1UL) == (1UL))
#endif
        {
            // Default state is SLEEP_A
            nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_GOSLEEP);
        }
        else
        {
            // Default state is IDLE
            nrfy_nfct_task_trigger(NRF_NFCT, NRF_NFCT_TASK_GOIDLE);
        }
    }
    else
    {
        nrfy_nfct_task_trigger(NRF_NFCT, (nrf_nfct_task_t) sub_state);
    }

    nfct_frame_delay_max_set(true);

    /* Disable TX/RX here (will be enabled at SELECTED) */
    nrfy_nfct_int_disable(NRF_NFCT, NRFX_NFCT_RX_INT_MASK | NRFX_NFCT_TX_INT_MASK);
}

int nrfx_nfct_parameter_set(nrfx_nfct_param_t const * p_param)
{
    NRFX_ASSERT(p_param);

    switch (p_param->id)
    {
        case NRFX_NFCT_PARAM_ID_FDT:
        {
            uint32_t delay     = p_param->data.fdt;
            uint32_t delay_thr = NRF_NFCT_FRAME_DELAY_MAX_MAX_VALUE;
            uint32_t delay_max;

            delay_max = (delay > delay_thr) ? delay_thr : delay;
            if (delay_max < m_nfct_cb.frame_delay_min)
            {
                return -EINVAL;
            }

            m_nfct_cb.frame_delay_max = delay_max;
            break;
        }

        case NRFX_NFCT_PARAM_ID_FDT_MIN:
        {
            uint32_t delay = p_param->data.fdt_min;
            uint32_t delay_thr = NRF_NFCT_FRAME_DELAY_MAX_MAX_VALUE;
            uint32_t delay_min;

            delay_min = (delay > delay_thr) ? delay_thr : delay;
            if (delay_min > m_nfct_cb.frame_delay_max)
            {
                return -EINVAL;
            }

            m_nfct_cb.frame_delay_min = delay_min;
            nrfy_nfct_frame_delay_min_set(NRF_NFCT, (uint16_t)m_nfct_cb.frame_delay_min);
            break;
        }

        case NRFX_NFCT_PARAM_ID_SEL_RES:
            if (p_param->data.sel_res_protocol > NRF_NFCT_SELRES_PROTOCOL_NFCDEP_T4AT)
            {
                return -EINVAL;
            }

            nrfy_nfct_selres_protocol_set(NRF_NFCT,
                    (nrf_nfct_selres_protocol_t) p_param->data.sel_res_protocol);
            break;

        case NRFX_NFCT_PARAM_ID_NFCID1:
        {
            nrf_nfct_sensres_nfcid1_size_t id_size_mask;

            id_size_mask = nfct_nfcid1_size_to_sensres_size(p_param->data.nfcid1.id_size);
            nrfy_nfct_nfcid1_set(NRF_NFCT, p_param->data.nfcid1.p_id, id_size_mask);
            break;
        }

        default:
            break;
    }

    return 0;
}

int nrfx_nfct_nfcid1_default_bytes_get(uint8_t * const p_nfcid1_buff,
                                              uint32_t        nfcid1_buff_len)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);
    NRFX_ASSERT(p_nfcid1_buff);

    uint32_t tag_header[3];

    if ((nfcid1_buff_len != NRFX_NFCT_NFCID1_SINGLE_SIZE) &&
        (nfcid1_buff_len != NRFX_NFCT_NFCID1_DOUBLE_SIZE) &&
        (nfcid1_buff_len != NRFX_NFCT_NFCID1_TRIPLE_SIZE))
    {
        return -E2BIG;
    }

#if (NRF_FICR_HAS_NFC_TAGHEADER || NRF_FICR_HAS_NFC_TAGHEADER_ARRAY) && \
    !defined(NRF_TRUSTZONE_NONSECURE)
    tag_header[0] = nrf_ficr_nfc_tagheader_get(NRF_FICR, 0);
    tag_header[1] = nrf_ficr_nfc_tagheader_get(NRF_FICR, 1);
    tag_header[2] = nrf_ficr_nfc_tagheader_get(NRF_FICR, 2);
#else
    /* IC manufacturer ID byte, set to Nordic Semiconductor value. */
    tag_header[0] = 0x5F;
    tag_header[1] = 0;
    tag_header[2] = 0;
#endif

    p_nfcid1_buff[0] = (uint8_t) (tag_header[0] >> 0);
    p_nfcid1_buff[1] = (uint8_t) (tag_header[0] >> 8);
    p_nfcid1_buff[2] = (uint8_t) (tag_header[0] >> 16);
    p_nfcid1_buff[3] = (uint8_t) (tag_header[1] >> 0);

    if (nfcid1_buff_len != NRFX_NFCT_NFCID1_SINGLE_SIZE)
    {
        p_nfcid1_buff[4] = (uint8_t) (tag_header[1] >> 8);
        p_nfcid1_buff[5] = (uint8_t) (tag_header[1] >> 16);
        p_nfcid1_buff[6] = (uint8_t) (tag_header[1] >> 24);

        if (nfcid1_buff_len == NRFX_NFCT_NFCID1_TRIPLE_SIZE)
        {
            p_nfcid1_buff[7] = (uint8_t) (tag_header[2] >> 0);
            p_nfcid1_buff[8] = (uint8_t) (tag_header[2] >> 8);
            p_nfcid1_buff[9] = (uint8_t) (tag_header[2] >> 16);
        }
        else if (NRF_ERRATA_DYNAMIC_CHECK(52, 181) && (p_nfcid1_buff[3] == 0x88))
        {
            p_nfcid1_buff[3] |= 0x11;
        }
    }

    return 0;
}

void nrfx_nfct_autocolres_enable(void)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);

#if NRF_NFCT_HAS_AUTOCOLRES_CONFIG_REG
    nrf_nfct_autocolres_enable(NRF_NFCT);
#else
    (*(uint32_t volatile *)(0x4000559C)) &= (~(0x1UL));
#endif
}

void nrfx_nfct_autocolres_disable(void)
{
    NRFX_ASSERT(m_nfct_cb.state == NRFX_DRV_STATE_INITIALIZED);

#if NRF_NFCT_HAS_AUTOCOLRES_CONFIG_REG
    nrf_nfct_autocolres_disable(NRF_NFCT);
#else
    (*(uint32_t volatile *)(0x4000559C)) |= (0x1UL);
#endif
}

void nrfx_nfct_irq_handler(void)
{
    nrfx_nfct_field_state_t current_field = NRFX_NFC_FIELD_STATE_NONE;
    uint32_t evt_mask = nrfy_nfct_events_process(NRF_NFCT,
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_FIELDDETECTED)  |
                                        (NRF_ERRATA_DYNAMIC_CHECK(52, 79) ? 0 :
                                            NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_FIELDLOST)) |
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_FIELDLOST)      |
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_RXFRAMESTART)   |
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_RXFRAMEEND)     |
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_SELECTED)       |
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_ERROR)          |
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_TXFRAMESTART)   |
                                        NRFY_EVENT_TO_INT_BITMASK(NRF_NFCT_EVENT_TXFRAMEEND));

    if (NRFX_NFCT_EVT_ACTIVE(FIELDDETECTED, evt_mask))
    {
        current_field = NRFX_NFC_FIELD_STATE_ON;

        NRFX_LOG_DEBUG("Field detected");
    }

    if (!NRF_ERRATA_DYNAMIC_CHECK(52, 79) && NRFX_NFCT_EVT_ACTIVE(FIELDLOST, evt_mask))
    {
        current_field = (current_field == NRFX_NFC_FIELD_STATE_NONE) ?
                        NRFX_NFC_FIELD_STATE_OFF : NRFX_NFC_FIELD_STATE_UNKNOWN;

        NRFX_LOG_DEBUG("Field lost");
    }

    /* Perform actions if any FIELD event is active */
    if (current_field != NRFX_NFC_FIELD_STATE_NONE)
    {
        nfct_field_event_handler(current_field);
    }

    if (NRFX_NFCT_EVT_ACTIVE(RXFRAMESTART, evt_mask))
    {
        nrfx_nfct_evt_t nfct_evt =
        {
            .evt_id = NRFX_NFCT_EVT_RX_FRAMESTART
        };

        NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);
    }

    if (NRFX_NFCT_EVT_ACTIVE(RXFRAMEEND, evt_mask))
    {
        nrfx_nfct_evt_t nfct_evt =
        {
            .evt_id = NRFX_NFCT_EVT_RX_FRAMEEND
        };

        /* Take into account only the number of whole bytes. */
        nfct_evt.params.rx_frameend.rx_status         = 0;
        nfct_evt.params.rx_frameend.rx_data.p_data    = nrfy_nfct_rxtx_buffer_get(NRF_NFCT);
        nfct_evt.params.rx_frameend.rx_data.data_size =
            NRFX_NFCT_BITS_TO_BYTES(nrfy_nfct_rx_bits_get(NRF_NFCT, true));

        if (nrfy_nfct_event_check(NRF_NFCT, NRF_NFCT_EVENT_RXERROR))
        {
            nrfy_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXERROR);

            nfct_evt.params.rx_frameend.rx_status =
                (nrfy_nfct_rx_frame_status_get(NRF_NFCT) & NRFX_NFCT_FRAME_STATUS_RX_ALL_MASK);

            NRFX_LOG_DEBUG("Rx error (0x%x)", (unsigned int) nfct_evt.params.rx_frameend.rx_status);

            /* Clear rx frame status */
            nrfy_nfct_rx_frame_status_clear(NRF_NFCT, NRFX_NFCT_FRAME_STATUS_RX_ALL_MASK);
        }

        NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);

        NRFX_LOG_DEBUG("Rx fend");
    }

    if (NRFX_NFCT_EVT_ACTIVE(SELECTED, evt_mask))
    {
        /* Clear also RX END and RXERROR events because SW does not take care of
           commands that were received before selecting the tag. */
        nrfy_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXFRAMEEND);
        nrfy_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_RXERROR);
        nrfy_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMESTART);
        nrfy_nfct_event_clear(NRF_NFCT, NRF_NFCT_EVENT_TXFRAMEEND);

        nfct_frame_delay_max_set(false);

        /* At this point any previous error status can be ignored. */
        nrfy_nfct_rx_frame_status_clear(NRF_NFCT, NRFX_NFCT_FRAME_STATUS_RX_ALL_MASK);
        nrfy_nfct_error_status_clear(NRF_NFCT, m_status_all_msk);

        nrfx_nfct_evt_t nfct_evt =
        {
            .evt_id = NRFX_NFCT_EVT_SELECTED
        };
        NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);

        NRFX_LOG_DEBUG("Selected");
    }

    if (NRFX_NFCT_EVT_ACTIVE(ERROR, evt_mask))
    {
        uint32_t err_status = nrfy_nfct_error_status_get(NRF_NFCT);

        nrfx_nfct_evt_t nfct_evt =
        {
            .evt_id = NRFX_NFCT_EVT_ERROR
        };

        /* Clear FRAMEDELAYTIMEOUT error (expected HW behaviour) when SLP_REQ command was received. */
        if (err_status & NRF_NFCT_ERROR_FRAMEDELAYTIMEOUT_MASK)
        {
            nrfy_nfct_error_status_clear(NRF_NFCT, NRF_NFCT_ERROR_FRAMEDELAYTIMEOUT_MASK);

            nfct_evt.params.error.reason = NRFX_NFCT_ERROR_FRAMEDELAYTIMEOUT;
            NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);
        }

        /* Report any other error. */
        err_status &= (uint32_t)~NRF_NFCT_ERROR_FRAMEDELAYTIMEOUT_MASK;
        if (err_status)
        {
            NRFX_LOG_DEBUG("Error (0x%x)", (unsigned int) err_status);
        }

        /* Clear error status. */
        nrfy_nfct_error_status_clear(NRF_NFCT, m_status_all_msk);
    }

    if (NRFX_NFCT_EVT_ACTIVE(TXFRAMESTART, evt_mask))
    {
        if (m_nfct_cb.config.cb != NULL)
        {
            nrfx_nfct_evt_t nfct_evt;

            nfct_evt.evt_id                                 = NRFX_NFCT_EVT_TX_FRAMESTART;
            nfct_evt.params.tx_framestart.tx_data.p_data    = nrfy_nfct_rxtx_buffer_get(NRF_NFCT);
            nfct_evt.params.tx_framestart.tx_data.data_size =
                NRFX_NFCT_BITS_TO_BYTES(nrfy_nfct_tx_bits_get(NRF_NFCT));

            m_nfct_cb.config.cb(&nfct_evt);
        }
    }

    if (NRFX_NFCT_EVT_ACTIVE(TXFRAMEEND, evt_mask))
    {
        nrfx_nfct_evt_t nfct_evt =
        {
            .evt_id = NRFX_NFCT_EVT_TX_FRAMEEND
        };

        /* Ignore any frame transmission until a new TX is scheduled by nrfx_nfct_tx() */
        nrfy_nfct_int_disable(NRF_NFCT, NRFX_NFCT_TX_INT_MASK);

        NRFX_NFCT_CB_HANDLE(m_nfct_cb.config.cb, nfct_evt);

        NRFX_LOG_DEBUG("Tx fend");
    }
}

#if NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) || \
    NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)
void nrfx_nfct_workaround_timer_handler(void)
{
    nrfx_timer_irq_handler(&m_timer_workaround.timer);
}
#endif // NRF_ERRATA_STATIC_CHECK(52, 79) || NRF_ERRATA_STATIC_CHECK(52, 190) ||
       // NRF_ERRATA_STATIC_CHECK(53, 70) || NRF_ERRATA_STATIC_CHECK(54L, 60)
