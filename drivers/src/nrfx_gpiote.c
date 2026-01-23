/*
 * Copyright (c) 2015 - 2026, Nordic Semiconductor ASA
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
#include <nrfx_gpiote.h>
#include <helpers/nrfx_flag32_allocator.h>
#include "nrf_bitmask.h"
#include <string.h>

#define NRFX_LOG_MODULE GPIOTE
#include <nrfx_log.h>

/* Macro returning mask of pins in the port */
#define GPIO_PIN_MASK(periph, prefix, i, _) NRFX_CONCAT(periph, prefix, i, _FEATURE_PINS_PRESENT)

#if !defined(GPIOTE_SPARSE_PINS)
#define GPIO_WITH_GAP(periph, prefix, i, _)               \
    (((1 << NRF_GPIO_PIN_NUM(periph, prefix, i, _)) - 1) != GPIO_PIN_MASK(periph, prefix, i, _))

#if NRFX_FOREACH_PRESENT(P, GPIO_WITH_GAP, (+), (0), _)
#error "Pin gaps in GPIO ports not supported"
#endif
#endif

/* Macro returns true if port has 32 pins. */
#define GPIO_IS_FULL_PORT(periph, prefix, i, _) \
    (NRFX_CONCAT(periph, prefix, i, _PIN_NUM) == 32)

/* Macro return true if all ports has 32 pins. In that case pin numbers are continuous. */
#define FULL_PORTS_PRESENT (NRFX_FOREACH_PRESENT(P, GPIO_IS_FULL_PORT, (&&), (1), _))

/*
 * 2 bytes are dedicated for each pin to store it's current state.
 *
 * +--------+-------+-----------------------+-----+--------+--------+---------+-------+
 * | 0      | 1     | 2-4                   | 5   | 6      | 7      | 8-12    | 13-15 |
 * +--------+-------+-----------------------+-----+--------+--------+---------+-------+
 * | in use | dir   | nrfx_gpiote_trigger_t | te  | skip   | N/A    |8:       | TE    |
 * | 0: no  | 0:in  |                       | used| config |        | present | index |
 * | 1: yes | 1:out |                       |     |        |        |9-12:    | (when |
 * |        |       |                       |     |        |        | handler |  used)|
 * |        |       |                       |     |        |        | index   |       |
 * +--------+-------+-----------------------+-----+--------+--------+---------+-------+
 *
 */

/* Flags content when pin is not used by the driver. */
#define PIN_FLAG_NOT_USED 0

#define PIN_FLAG_IN_USE NRFX_BIT(0)

#define PIN_FLAG_DIR_MASK NRFX_BIT(1)

/* Flag indicating output direction. */
#define PIN_FLAG_OUTPUT PIN_FLAG_DIR_MASK

/* Macro checks if pin is output. */
#define PIN_FLAG_IS_OUTPUT(flags) ((flags & PIN_FLAG_DIR_MASK) == PIN_FLAG_OUTPUT)

/* Trigger mode field. It stores the information about a trigger type. If trigger
 * is not enabled, it holds information about task usage and pin direction. */
#define PIN_FLAG_TRIG_MODE_OFFSET 2UL
#define PIN_FLAG_TRIG_MODE_BITS 3UL
#define PIN_FLAG_TRIG_MODE_MASK \
        (NRFX_BIT_MASK(PIN_FLAG_TRIG_MODE_BITS) << PIN_FLAG_TRIG_MODE_OFFSET)
NRFX_STATIC_ASSERT(NRFX_GPIOTE_TRIGGER_MAX <= NRFX_BIT(PIN_FLAG_TRIG_MODE_BITS));

/* Macro sets trigger mode field. */
#define PIN_FLAG_TRIG_MODE_SET(trigger) (trigger << PIN_FLAG_TRIG_MODE_OFFSET)

/* Macro gets trigger mode from pin flags. */
#define PIN_FLAG_TRIG_MODE_GET(flags) \
        (nrfx_gpiote_trigger_t)((flags & PIN_FLAG_TRIG_MODE_MASK) >> PIN_FLAG_TRIG_MODE_OFFSET)

#define PIN_FLAG_TE_USED        NRFX_BIT(5)
#define PIN_FLAG_SKIP_CONFIG    NRFX_BIT(6)

#define PIN_FLAG_HANDLER_PRESENT NRFX_BIT(8)

#define PIN_HANDLER_ID_SHIFT 9UL
#define PIN_HANDLER_ID_BITS 4UL
#define PIN_HANDLER_ID_MASK (NRFX_BIT_MASK(PIN_HANDLER_ID_BITS) << PIN_HANDLER_ID_SHIFT)
#define PIN_HANDLER_MASK (PIN_FLAG_HANDLER_PRESENT | PIN_HANDLER_ID_MASK)

/* Macro for encoding handler index into the flags. */
#define PIN_FLAG_HANDLER(x) \
        (PIN_FLAG_HANDLER_PRESENT | ((x) << PIN_HANDLER_ID_SHIFT))

/* Pin in use but no handler attached. */
#define PIN_FLAG_NO_HANDLER -1

/* Macro for getting handler index from flags. -1 is returned when no handler */
#define PIN_GET_HANDLER_ID(flags) \
        ((flags & PIN_FLAG_HANDLER_PRESENT) \
         ? (int32_t)((flags & PIN_HANDLER_ID_MASK) >> PIN_HANDLER_ID_SHIFT) \
         : PIN_FLAG_NO_HANDLER)

#define PIN_HANDLER_MAX_COUNT NRFX_BIT_MASK(PIN_HANDLER_ID_BITS)
NRFX_STATIC_ASSERT(NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS <= PIN_HANDLER_MAX_COUNT);

#define PIN_TE_ID_SHIFT 13UL
#define PIN_TE_ID_BITS 3UL
#define PIN_TE_ID_MASK (NRFX_BIT_MASK(PIN_TE_ID_BITS) << PIN_TE_ID_SHIFT)

/* Validate that field is big enough for number of channels. */
NRFX_STATIC_ASSERT((NRFX_BIT(PIN_TE_ID_BITS)) >= GPIOTE_CH_NUM);

/* Macro for encoding Task/Event index into the flags. */
#define PIN_FLAG_TE_ID(x) \
        (PIN_FLAG_TE_USED | (((x) << PIN_TE_ID_SHIFT) & PIN_TE_ID_MASK))

/* Macro for getting Task/Event index from flags. */
#define PIN_GET_TE_ID(flags) ((flags & PIN_TE_ID_MASK) >> PIN_TE_ID_SHIFT)

#define GPIOTE_PORT_INDEX(periph_name, prefix, i, _) i,
#define GPIOTE_PORTS_INDEX_LIST                                                \
  NRFX_FOREACH_PRESENT(P, GPIOTE_PORT_INDEX, (), (), _)

typedef struct
{
    /* Number of GPIOTE channels. */
    uint32_t channels_number;

    /* Mask of available ports for GPIOTE instance. */
    uint32_t available_gpio_ports;

#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
    uint8_t  group_idx;
    bool     port_supported;
    bool     dynamic_chan_supported;
#endif
} gpiote_config_t;

#if !defined(__NRFX_DOXYGEN__)
#if (defined(NRF_GPIOTE) || defined(NRF_GPIOTE0)) && !defined(NRFX_GPIOTE0_CHANNELS_USED)
/* Bitmask that defines GPIOTE0 channels that are reserved for use outside of the nrfx library. */
#define NRFX_GPIOTE0_CHANNELS_USED 0UL
#endif

#if defined(NRF_GPIOTE1) && !defined(NRFX_GPIOTE1_CHANNELS_USED)
/* Bitmask that defines GPIOTE1 channels that are reserved for use outside of the nrfx library. */
#define NRFX_GPIOTE1_CHANNELS_USED 0UL
#endif

#if defined(NRF_GPIOTE20) && !defined(NRFX_GPIOTE20_CHANNELS_USED)
/* Bitmask that defines GPIOTE20 channels that are reserved for use outside of the nrfx library. */
#define NRFX_GPIOTE20_CHANNELS_USED 0UL
#endif

#if defined(NRF_GPIOTE30) && !defined(NRFX_GPIOTE30_CHANNELS_USED)
/* Bitmask that defines GPIOTE30 channels that are reserved for use outside of the nrfx library. */
#define NRFX_GPIOTE30_CHANNELS_USED 0UL
#endif

#if defined(NRF_GPIOTE130) && !defined(NRFX_GPIOTE130_CHANNELS_USED)
/* Bitmask that defines GPIOTE130 channels that are reserved for use outside of the nrfx library. */
#define NRFX_GPIOTE130_CHANNELS_USED 0UL
#endif

#if defined(NRF_GPIOTE131) && !defined(NRFX_GPIOTE131_CHANNELS_USED)
/* Bitmask that defines GPIOTE131 channels that are reserved for use outside of the nrfx library. */
#define NRFX_GPIOTE131_CHANNELS_USED 0UL
#endif
#endif // !defined(__NRFX_DOXYGEN__)

#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
#define GPIOTE_VAR_INIT(prefix, idx)                                                    \
    .group_idx = idx == 0 ? 0 : NRF_GPIOTE_IRQ_GROUP,                                   \
    .port_supported = idx != 0,                                                         \
    .dynamic_chan_supported = idx != 0,
#else
#define GPIOTE_VAR_INIT(prefix, idx)
#endif

#define _NRFX_GPIOTE_CONFIG_INITIALIZER(periph_name, prefix, idx, _)                    \
    [NRFX_CONCAT(NRFX_, periph_name, idx, _INST_IDX)] = {                               \
        .channels_number = NRFX_CONCAT_3(periph_name, idx, _CH_NUM),                    \
        .available_gpio_ports = NRFX_CONCAT_3(periph_name, idx, _AVAILABLE_GPIO_PORTS), \
        GPIOTE_VAR_INIT(prefix, idx)                                                    \
    },

#define _NRFX_GPIOTE_CHANNEL_INITIALIZER(periph_name, prefix, idx, _)                   \
    [NRFX_CONCAT(NRFX_, periph_name, idx, _INST_IDX)] =                                 \
        (nrfx_atomic_t)NRFX_GPIOTE_APP_CHANNELS_MASK(idx),

/* Mask for tracking GPIOTE channel allocation. */
static nrfx_atomic_t available_channels_mask[GPIOTE_COUNT] = {
    NRFX_FOREACH_INDEXED_PRESENT(GPIOTE, _NRFX_GPIOTE_CHANNEL_INITIALIZER, (), ())
};

static const gpiote_config_t m_config[GPIOTE_COUNT] = {
    NRFX_FOREACH_INDEXED_PRESENT(GPIOTE, _NRFX_GPIOTE_CONFIG_INITIALIZER, (), ())
};

#if defined(NRF_GPIO_LATCH_PRESENT) || (!FULL_PORTS_PRESENT)
static const uint8_t ports[GPIO_COUNT] = GPIO_PORT_NUM_LIST;
#endif

static const uint8_t port_index[GPIO_COUNT] = {GPIOTE_PORTS_INDEX_LIST};

#define GPIO_PORT_OFFSET(i, _) \
    NRFX_COND_CODE_1(NRFX_INSTANCE_PRESENT(NRFX_CONCAT(P, i)),(NRFX_CONCAT(P, i, _PIN_NUM)), (0))

static uint8_t get_pin_idx(nrfx_gpiote_pin_t pin)
{
#if FULL_PORTS_PRESENT
    // If all ports have 32 pins then array ordering matches pin ordering.
    return (uint8_t)pin;
#else
    // Possible instances must be explicitely listed as NRFX_LISTIFY cannot be nested.
    static const uint8_t port_offset[] = {
        0,
        NRFX_LISTIFY(1, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(2, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(3, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(4, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(5, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(6, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(7, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(8, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(9, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(10, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(11, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(12, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(13, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(14, GPIO_PORT_OFFSET, (+), _),
        NRFX_LISTIFY(15, GPIO_PORT_OFFSET, (+), _),
    };

    return port_offset[pin >> 5] + (pin & 0x1F);
#endif
}

/** @brief Function for getting pointer to instance channel mask.
 *
 * Function is optimized for case when there is only one GPIOTE instance.
 *
 * @param[in] idx Instance index.
 *
 * @return Configure structure.
 */
static nrfx_atomic_t * get_channel_mask(uint32_t idx)
{
    if (GPIOTE_COUNT == 1)
    {
        return &available_channels_mask[0];
    }
    else
    {
        return &available_channels_mask[idx];
    }
}

/** @brief Function for getting instance configuration structure.
 *
 * Function is optimized for case when there is only one GPIOTE instance.
 *
 * @param[in] idx Instance index.
 *
 * @return Configure structure.
 */
static const gpiote_config_t * get_config(uint32_t idx)
{
    if (GPIOTE_COUNT == 1)
    {
        return &m_config[0];
    }
    else
    {
        return &m_config[idx];
    }
}

/** @brief Checks if pin is in use by a given GPIOTE instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin.
 *
 * @return True if pin is in use.
 */
static bool pin_in_use(nrfx_gpiote_t const * p_instance, uint32_t pin)
{
    return p_instance->cb.pin_flags[get_pin_idx(pin)] & PIN_FLAG_IN_USE;
}

/** @brief Check if Task/Event is used by a given GPIOTE instance.
 *
 * Assuming that pin is in use.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin.
 *
 * @return True if pin uses GPIOTE task/event.
 */
static bool pin_in_use_by_te(nrfx_gpiote_t const * p_instance, uint32_t pin)
{
    return p_instance->cb.pin_flags[get_pin_idx(pin)] & PIN_FLAG_TE_USED;
}

/** @brief Check if pin has trigger for a given GPIOTE instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin.
 *
 * @return True if pin has trigger.
 */
static bool pin_has_trigger(nrfx_gpiote_t const * p_instance, uint32_t pin)
{
    return PIN_FLAG_TRIG_MODE_GET(
               p_instance->cb.pin_flags[get_pin_idx(pin)]) !=
           NRFX_GPIOTE_TRIGGER_NONE;
}

/** @brief Check if pin is output for a given GPIOTE instance.
 *
 * Assuming that pin is in use.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin.
 *
 * @return True if pin is output.
 */
static bool pin_is_output(nrfx_gpiote_t const * p_instance, uint32_t pin)
{
    return PIN_FLAG_IS_OUTPUT(p_instance->cb.pin_flags[get_pin_idx(pin)]);
}

/** @brief Check if pin is output controlled by GPIOTE task for a given GPIOTE instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin.
 *
 * @return True if pin is task output.
 */
static bool pin_is_task_output(nrfx_gpiote_t const * p_instance, uint32_t pin)
{
    return pin_is_output(p_instance, pin) && pin_in_use_by_te(p_instance, pin);
}

/** @brief Check if pin is used by the driver and configured as input for a given GPIOTE instance.
 *
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] pin        Absolute pin.
 *
 * @return True if pin is configured as input.
 */
static bool pin_is_input(nrfx_gpiote_t const * p_instance, uint32_t pin)
{
    return !pin_is_output(p_instance, pin);
}

/* Convert polarity enum (HAL) to trigger enum. */
static nrfx_gpiote_trigger_t gpiote_polarity_to_trigger(nrf_gpiote_polarity_t polarity)
{
   return (nrfx_gpiote_trigger_t)polarity;
}

/* Convert trigger enum to polarity enum (HAL). */
static nrf_gpiote_polarity_t gpiote_trigger_to_polarity(nrfx_gpiote_trigger_t trigger)
{
    return (nrf_gpiote_polarity_t)trigger;
}

/* Returns gpiote TE channel associated with the pin */
static uint8_t pin_te_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    return PIN_GET_TE_ID(p_instance->cb.pin_flags[get_pin_idx(pin)]);
}

static bool is_level(nrfx_gpiote_trigger_t trigger)
{
    return trigger >= NRFX_GPIOTE_TRIGGER_LOW;
}

static bool handler_in_use(nrfx_gpiote_t const * p_instance, int32_t handler_id)
{
    for (uint32_t i = 0; i < NRF_GPIO_MAX_PIN_NUMBER; i++)
    {
        if (PIN_GET_HANDLER_ID(p_instance->cb.pin_flags[i]) == handler_id)
        {
            return true;
        }
    }
    return false;
}

/* Function clears pin handler flag and releases handler slot if handler+context
 * pair is not used by other pin. */
static void release_handler(nrfx_gpiote_t * p_instance, nrfx_gpiote_pin_t pin)
{
    uint8_t idx = get_pin_idx(pin);
    int32_t handler_id = PIN_GET_HANDLER_ID(p_instance->cb.pin_flags[idx]);

    if (handler_id == PIN_FLAG_NO_HANDLER)
    {
        return;
    }

    p_instance->cb.pin_flags[idx] &= (uint16_t)~PIN_HANDLER_MASK;

    /* Check if other pin is using same handler and release handler only if handler
     * is not used by others.
     */
    if (!handler_in_use(p_instance, handler_id))
    {
        p_instance->cb.handlers[handler_id].handler = NULL;
        int err = nrfx_flag32_free(&p_instance->cb.available_evt_handlers, (uint8_t)handler_id);
        (void)err;
        NRFX_ASSERT(err == 0);
    }
}

/* Function releases the handler associated with the pin and sets GPIOTE channel
 * configuration to default if it was used with the pin.
 */
static void pin_handler_trigger_uninit(nrfx_gpiote_t * p_instance, nrfx_gpiote_pin_t pin)
{
    if (pin_in_use_by_te(p_instance, pin))
    {
        /* te to default */
        nrfy_gpiote_te_default(p_instance->p_reg, pin_te_get(p_instance, pin));
    }
    else
    {
#if !defined(NRF_GPIO_LATCH_PRESENT)
        nrf_bitmask_bit_clear(pin, (uint8_t *)p_instance->cb.port_pins);
#endif
    }

    release_handler(p_instance, pin);
    p_instance->cb.pin_flags[get_pin_idx(pin)] = PIN_FLAG_NOT_USED;
}

/* Function disabling sense level for the given pin
 * or disabling interrupts and events for GPIOTE channel if it was used with the pin.
 */
static void pin_trigger_disable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    if (pin_in_use_by_te(p_instance, pin) && pin_is_input(p_instance, pin))
    {
        uint8_t ch = pin_te_get(p_instance, pin);

#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
        nrf_gpiote_int_group_disable(p_instance->p_reg,
                                     get_config(p_instance->cb.drv_inst_idx)->group_idx,
                                     NRFX_BIT(ch));
#else
        nrfy_gpiote_int_disable(p_instance->p_reg, NRFX_BIT(ch));
#endif
        nrfy_gpiote_event_disable(p_instance->p_reg, ch);
    }
    else
    {
        nrfy_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
    }
}

static nrf_gpiote_event_t pin_in_event_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_input(p_instance, pin));
    NRFX_ASSERT(pin_has_trigger(p_instance, pin));

    if (pin_in_use_by_te(p_instance, pin))
    {
        return nrfy_gpiote_in_event_get((uint8_t)pin_te_get(p_instance, pin));
    }

    return NRF_GPIOTE_EVENT_PORT;
}

static int32_t find_handler(nrfx_gpiote_t const *           p_instance,
                            nrfx_gpiote_interrupt_handler_t handler,
                            void *                          p_context)
{
    for (int32_t i = 0; i < NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS; i++)
    {
        if ((p_instance->cb.handlers[i].handler == handler) &&
            (p_instance->cb.handlers[i].p_context == p_context))
        {
            return i;
        }
    }

    return -1;
}

/** @brief Set new handler, if handler was not previously set allocate it. */
static int pin_handler_set(nrfx_gpiote_t *                 p_instance,
                           nrfx_gpiote_pin_t               pin,
                           nrfx_gpiote_interrupt_handler_t handler,
                           void *                          p_context)
{
    int32_t handler_id;

    release_handler(p_instance, pin);
    if (!handler)
    {
        return 0;
    }

    handler_id = find_handler(p_instance, handler, p_context);
    /* Handler not found, new must be allocated. */
    if (handler_id < 0)
    {
        handler_id = nrfx_flag32_alloc(&p_instance->cb.available_evt_handlers);
        if (handler_id < 0)
        {
            return handler_id;
        }
    }

    p_instance->cb.handlers[handler_id].handler = handler;
    p_instance->cb.handlers[handler_id].p_context = p_context;
    p_instance->cb.pin_flags[get_pin_idx(pin)] |=
        (uint16_t)PIN_FLAG_HANDLER((uint8_t)handler_id);

    return 0;
}

static inline nrf_gpio_pin_sense_t get_initial_sense(nrfx_gpiote_t const * p_instance,
                                                     nrfx_gpiote_pin_t     pin)
{
    nrfx_gpiote_trigger_t trigger = PIN_FLAG_TRIG_MODE_GET(
        p_instance->cb.pin_flags[get_pin_idx(pin)]);
    nrf_gpio_pin_sense_t sense;

    if (trigger == NRFX_GPIOTE_TRIGGER_LOW)
    {
        sense = NRF_GPIO_PIN_SENSE_LOW;
    }
    else if (trigger == NRFX_GPIOTE_TRIGGER_HIGH)
    {
        sense = NRF_GPIO_PIN_SENSE_HIGH;
    }
    else
    {
        /* If edge detection start with sensing opposite state. */
        sense = nrfy_gpio_pin_read(pin) ? NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;
    }

    return sense;
}

/* Return handler associated with given pin or null. */
static nrfx_gpiote_handler_config_t const * channel_handler_get(nrfx_gpiote_control_block_t * p_cb,
                                                                nrfx_gpiote_pin_t             pin)
{
    int32_t handler_id = PIN_GET_HANDLER_ID(p_cb->pin_flags[get_pin_idx(pin)]);

    if (handler_id == PIN_FLAG_NO_HANDLER)
    {
        return NULL;
    }

    return &p_cb->handlers[handler_id];
}

/* Function for deinitializing the specified pin. */
static int pin_uninit(nrfx_gpiote_t * p_instance, nrfx_gpiote_pin_t pin)
{
    if (!pin_in_use(p_instance, pin))
    {
        return -EINVAL;
    }

    pin_trigger_disable(p_instance, pin);
    pin_handler_trigger_uninit(p_instance, pin);
    nrfy_gpio_cfg_default(pin);

    return 0;
}

/* Function for deinitializing the specified pin if present. */
static void pin_cond_uninit(nrfx_gpiote_t * p_instance, uint32_t pin)
{
    if (nrfy_gpio_pin_present_check(pin))
    {
        int err_code = pin_uninit(p_instance, pin);

        if (err_code != 0)
        {
            NRFX_LOG_WARNING("Function: %s, error code: %s.",
                             __func__,
                             NRFX_LOG_ERROR_STRING_GET(err_code));
        }
    }
}

static int gpiote_input_configure(nrfx_gpiote_t *                        p_instance,
                                  nrfx_gpiote_pin_t                      pin,
                                  nrfx_gpiote_input_pin_config_t const * p_config)
{
    int err;
    uint8_t idx = get_pin_idx(pin);
    nrfx_gpiote_control_block_t *cb = &p_instance->cb;

    if (p_config->p_pull_config)
    {
        if (pin_is_task_output(p_instance, pin))
        {
            return -EINVAL;
        }

        nrf_gpio_pin_dir_t dir = NRF_GPIO_PIN_DIR_INPUT;
        nrf_gpio_pin_input_t input_connect = NRF_GPIO_PIN_INPUT_CONNECT;

        nrfy_gpio_reconfigure(pin, &dir, &input_connect, p_config->p_pull_config, NULL, NULL);

        cb->pin_flags[idx] &= (uint16_t)~PIN_FLAG_OUTPUT;
        cb->pin_flags[idx] |= PIN_FLAG_IN_USE;
    }

    if (p_config->p_trigger_config)
    {
        nrfx_gpiote_trigger_t trigger = p_config->p_trigger_config->trigger;
        bool use_evt = p_config->p_trigger_config->p_in_channel ? true : false;

        if (pin_is_output(p_instance, pin))
        {
            if (use_evt)
            {
                return -EINVAL;
            }
        }
        else
        {
            cb->pin_flags[idx] &=
                (uint16_t) ~(PIN_TE_ID_MASK | PIN_FLAG_TE_USED);
            if (use_evt) {
                bool edge = trigger <= NRFX_GPIOTE_TRIGGER_TOGGLE;

                /* IN event used. */
                if (!edge)
                {
                    /* IN event supports only edge trigger. */
                    return -EINVAL;
                }

                uint8_t ch = *p_config->p_trigger_config->p_in_channel;

                if (trigger == NRFX_GPIOTE_TRIGGER_NONE)
                {
                    nrfy_gpiote_te_default(p_instance->p_reg, ch);
                }
                else
                {
                    nrf_gpiote_polarity_t polarity = gpiote_trigger_to_polarity(trigger);

                    nrfy_gpiote_event_disable(p_instance->p_reg, ch);
                    nrfy_gpiote_event_configure(p_instance->p_reg, ch, pin, polarity);
                    cb->ch_pin[ch] = (uint16_t)pin;
                    cb->pin_flags[idx] |= (uint16_t)PIN_FLAG_TE_ID(ch);
                }
            }
        }
#if !defined(NRF_GPIO_LATCH_PRESENT)
        if (use_evt || trigger == NRFX_GPIOTE_TRIGGER_NONE)
        {
            nrf_bitmask_bit_clear(pin, (uint8_t *)cb->port_pins);
        }
        else
        {
#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
            if (!get_config(p_instance->drv_inst_idx)->port_supported)
            {
                return -EINVAL;
            }
#endif
            nrf_bitmask_bit_set(pin, (uint8_t *)cb->port_pins);
        }
#endif
        cb->pin_flags[idx] &= (uint16_t)~PIN_FLAG_TRIG_MODE_MASK;
        cb->pin_flags[idx] |= (uint16_t)PIN_FLAG_TRIG_MODE_SET(trigger);
    }

    if (p_config->p_handler_config)
    {
        err = pin_handler_set(p_instance,
                              pin,
                              p_config->p_handler_config->handler,
                              p_config->p_handler_config->p_context);
    }
    else
    {
        err = 0;
    }

    return err;
}

static int gpiote_output_configure(nrfx_gpiote_t *                     p_instance,
                                   nrfx_gpiote_pin_t                   pin,
                                   nrfx_gpiote_output_config_t const * p_config,
                                   nrfx_gpiote_task_config_t const *   p_task_config)
{
    uint8_t idx = get_pin_idx(pin);

    if (p_config)
    {
        /* Cannot configure pin to output if pin was using TE event. */
        if (pin_is_input(p_instance, pin) && pin_in_use_by_te(p_instance, pin))
        {
            return -EINVAL;
        }

        /* If reconfiguring to output pin that has trigger configured then accept
         * only when input is still connected. */
        if (pin_has_trigger(p_instance, pin) &&
            (p_config->input_connect == NRF_GPIO_PIN_INPUT_DISCONNECT))
        {
            return -EINVAL;
        }

        nrf_gpio_pin_dir_t dir = NRF_GPIO_PIN_DIR_OUTPUT;

        nrfy_gpio_reconfigure(pin, &dir, &p_config->input_connect, &p_config->pull,
                              &p_config->drive, NULL);

        p_instance->cb.pin_flags[idx] |= PIN_FLAG_IN_USE | PIN_FLAG_OUTPUT;
    }

    if (p_task_config)
    {
        if (pin_is_input(p_instance, pin))
        {
            return -EINVAL;
        }

        uint32_t ch = p_task_config->task_ch;

        nrfy_gpiote_te_default(p_instance->p_reg, ch);
        p_instance->cb.pin_flags[idx] &=
            (uint16_t) ~(PIN_FLAG_TE_USED | PIN_TE_ID_MASK);
        if (p_task_config->polarity != NRF_GPIOTE_POLARITY_NONE)
        {
            nrfy_gpiote_task_configure(p_instance->p_reg, ch, pin,
                                       p_task_config->polarity,
                                       p_task_config->init_val);
            p_instance->cb.pin_flags[idx] |= (uint16_t)PIN_FLAG_TE_ID(ch);
        }
    }

    return 0;
}

static void gpiote_global_callback_set(nrfx_gpiote_t *                 p_instance,
                                       nrfx_gpiote_interrupt_handler_t handler,
                                       void *                          p_context)
{
    p_instance->cb.global_handler.handler = handler;
    p_instance->cb.global_handler.p_context = p_context;
}

static int gpiote_channel_get(nrfx_gpiote_t const * p_instance,
                              nrfx_gpiote_pin_t     pin,
                              uint8_t *             p_channel)
{
    NRFX_ASSERT(p_channel);

    if (pin_in_use_by_te(p_instance, pin))
    {
        *p_channel = PIN_GET_TE_ID(p_instance->cb.pin_flags[get_pin_idx(pin)]);
        return 0;
    }
    else
    {
        return -EINVAL;
    }
}

static int gpiote_init(nrfx_gpiote_t * p_instance, uint8_t interrupt_priority)
{
    nrfx_gpiote_control_block_t * p_cb = &p_instance->cb;
    gpiote_config_t const * p_cfg = get_config(p_instance->cb.drv_inst_idx);
    int err_code = 0;

    if (p_cb->state != NRFX_DRV_STATE_UNINITIALIZED)
    {
        err_code = -EALREADY;
        NRFX_LOG_WARNING("Function: %s, error code: %s.",
                         __func__,
                         NRFX_LOG_ERROR_STRING_GET(err_code));
        return err_code;
    }

    memset(p_cb->pin_flags, 0, sizeof(p_cb->pin_flags));

#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
    uint32_t int_mask = p_cfg->port_supported ? NRF_GPIOTE_INT_PORT_MASK : 0;
#else
    uint32_t int_mask = NRF_GPIOTE_INT_PORT_MASK;
#endif

    nrfy_gpiote_int_init(p_instance->p_reg,
                         int_mask,
                         interrupt_priority,
                         false,
                         p_cfg->channels_number);

    p_cb->state = NRFX_DRV_STATE_INITIALIZED;
    p_cb->available_evt_handlers = NRFX_BIT_MASK(NRFX_GPIOTE_CONFIG_NUM_OF_EVT_HANDLERS);

    NRFX_LOG_INFO("channels_number: %d, available_channels_mask: 0x%x",
                  (int)p_cfg->channels_number,
                  (int)*get_channel_mask(p_instance->cb.drv_inst_idx));

    NRFX_LOG_INFO("Function: %s, error code: %s.",
                  __func__,
                  NRFX_LOG_ERROR_STRING_GET(err_code));
    return err_code;
}

static bool gpiote_init_check(nrfx_gpiote_t const * p_instance)
{
    return (p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);
}

static void gpiote_uninit(nrfx_gpiote_t * p_instance)
{
    NRFX_ASSERT(p_instance->cb.state != NRFX_DRV_STATE_UNINITIALIZED);

#if FULL_PORTS_PRESENT
    // Simple iteration for simple case to save memory
    for (size_t i = 0; i < NRF_GPIO_MAX_PIN_NUMBER; i++)
    {
        pin_cond_uninit(p_instance, i);
    }
#else
#define _PORT_LEN(periph, prefix, i, _) NRFX_CONCAT(periph, prefix, i, _PIN_NUM),
    static const uint8_t port_lens[] =
    {
        NRFX_FOREACH_PRESENT(P, _PORT_LEN, (), (), _)
    };

    // Iterate over all pins in all ports.
    for (size_t i = 0; i < NRFX_ARRAY_SIZE(ports); i++)
    {
        for (size_t j = 0; j < port_lens[i]; j++)
        {
            pin_cond_uninit(p_instance, 32 * ports[i] + j);
        }
    }
#undef _PORT_LEN
#endif

    p_instance->cb.state = NRFX_DRV_STATE_UNINITIALIZED;
    p_instance->cb.global_handler.handler = NULL;
    NRFX_LOG_INFO("Uninitialized.");
}

static int pin_channel_free(nrfx_gpiote_t * p_instance, uint8_t channel)
{
#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
    const gpiote_config_t * p_config = get_config(p_instance->cb.drv_inst_idx);

    if (!p_config->dynamic_chan_supported)
    {
        return -ENOTSUP;
    }
#endif

    return nrfx_flag32_free(get_channel_mask(p_instance->cb.drv_inst_idx), channel);
}

static int pin_channel_alloc(nrfx_gpiote_t * p_instance)
{
    NRFX_LOG_INFO("available_channels_mask = 0x%x",
                  (int)*get_channel_mask(p_instance->cb.drv_inst_idx));
#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
    const gpiote_config_t * p_config = get_config(p_instance->cb.drv_inst_idx);

    if (!p_config->dynamic_chan_supported)
    {
        return -ENOTSUP;
    }
#endif

    return nrfx_flag32_alloc(get_channel_mask(p_instance->cb.drv_inst_idx));
}

static void pin_out_set(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_output(p_instance, pin) && !pin_in_use_by_te(p_instance, pin));

    nrfy_gpio_pin_set(pin);
}

static void pin_out_clear(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_output(p_instance, pin) && !pin_in_use_by_te(p_instance, pin));

    nrfy_gpio_pin_clear(pin);
}

static void pin_out_toggle(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_output(p_instance, pin) && !pin_in_use_by_te(p_instance, pin));

    nrfy_gpio_pin_toggle(pin);
}

static void pin_out_task_enable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    (void)pin_is_task_output; /* Add to avoid compiler warnings when asserts disabled.*/
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(p_instance, pin));

    nrfy_gpiote_task_enable(p_instance->p_reg, (uint32_t)pin_te_get(p_instance, pin));
}

static void pin_out_task_disable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(p_instance, pin));

    nrfy_gpiote_task_disable(p_instance->p_reg, (uint32_t)pin_te_get(p_instance, pin));
}

static nrf_gpiote_task_t pin_out_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(p_instance, pin));

    return nrfy_gpiote_out_task_get((uint8_t)pin_te_get(p_instance, pin));
}

static uint32_t pin_out_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_task_t task = pin_out_task_get(p_instance, pin);
    return nrfy_gpiote_task_address_get(p_instance->p_reg, task);
}

#if defined(GPIOTE_FEATURE_SET_PRESENT)
static nrf_gpiote_task_t pin_set_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(p_instance, pin));

    return nrfy_gpiote_set_task_get((uint8_t)pin_te_get(p_instance, pin));
}

static uint32_t pin_set_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_task_t task = pin_set_task_get(p_instance, pin);
    return nrfy_gpiote_task_address_get(p_instance->p_reg, task);
}
#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if defined(GPIOTE_FEATURE_CLR_PRESENT)
static nrf_gpiote_task_t pin_clr_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(p_instance, pin));

    return nrfy_gpiote_clr_task_get((uint8_t)pin_te_get(p_instance, pin));
}
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)

static void pin_out_task_force(nrfx_gpiote_t const * p_instance,
                               nrfx_gpiote_pin_t     pin,
                               uint8_t               state)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output(p_instance, pin));

    nrf_gpiote_outinit_t init_val =
        state ? NRF_GPIOTE_INITIAL_VALUE_HIGH : NRF_GPIOTE_INITIAL_VALUE_LOW;
    nrfy_gpiote_task_force(p_instance->p_reg, (uint32_t)pin_te_get(p_instance, pin), init_val);
}

static void pin_out_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_is_task_output((nrfx_gpiote_t const *)p_instance, pin));

    nrf_gpiote_task_t task = nrfy_gpiote_out_task_get((uint8_t)pin_te_get(p_instance, pin));
    nrfy_gpiote_task_trigger(p_instance->p_reg, task);
}

#if defined(GPIOTE_FEATURE_SET_PRESENT)
static void pin_set_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(p_instance, pin));
    NRFX_ASSERT(pin_in_use_by_te(p_instance, pin));

    nrf_gpiote_task_t task = nrfy_gpiote_set_task_get((uint8_t)pin_te_get(p_instance, pin));
    nrfy_gpiote_task_trigger(p_instance->p_reg, task);
}
#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if  defined(GPIOTE_FEATURE_CLR_PRESENT)
static void pin_clr_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    NRFX_ASSERT(pin_in_use(p_instance, pin));
    NRFX_ASSERT(pin_in_use_by_te(p_instance, pin));

    nrf_gpiote_task_t task = nrfy_gpiote_clr_task_get((uint8_t)pin_te_get(p_instance, pin));
    nrfy_gpiote_task_trigger(p_instance->p_reg, task);
}
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)

static void pin_trigger_enable(nrfx_gpiote_t const * p_instance,
                               nrfx_gpiote_pin_t     pin,
                               bool                  int_enable)
{
    NRFX_ASSERT(pin_has_trigger(p_instance, pin));

    if (pin_in_use_by_te(p_instance, pin) && pin_is_input(p_instance, pin))
    {
        uint8_t ch = pin_te_get(p_instance, pin);

        nrfy_gpiote_event_clear(p_instance->p_reg, nrf_gpiote_in_event_get(ch));
        nrfy_gpiote_event_enable(p_instance->p_reg, ch);
        if (int_enable)
        {
#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
            nrf_gpiote_int_group_enable(p_instance->p_reg,
                                        get_config(p_instance->cb.drv_inst_idx)->group_idx,
                                        NRFX_BIT(ch));
#else
            nrfy_gpiote_int_enable(p_instance->p_reg, NRFX_BIT(ch));
#endif
        }
    }
    else
    {
        if (!nrfy_gpiote_int_enable_check(p_instance->p_reg, (uint32_t)NRF_GPIOTE_INT_PORT_MASK))
        {
            nrfy_gpiote_int_enable(p_instance->p_reg, (uint32_t)NRF_GPIOTE_INT_PORT_MASK);
        }

        NRFX_ASSERT(int_enable);
        nrfy_gpio_cfg_sense_set(pin, get_initial_sense(p_instance, pin));
    }
}

bool nrfx_gpiote_in_is_set(nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(nrfy_gpio_pin_present_check(pin));
    return nrfy_gpio_pin_read(pin) ? true : false;
}

int nrfx_gpiote_input_configure(nrfx_gpiote_t *                        p_instance,
                                nrfx_gpiote_pin_t                      pin,
                                nrfx_gpiote_input_pin_config_t const * p_config)
{
    NRFX_ASSERT(p_instance);

    return gpiote_input_configure(p_instance, pin, p_config);
}

int nrfx_gpiote_output_configure(nrfx_gpiote_t *                     p_instance,
                                 nrfx_gpiote_pin_t                   pin,
                                 nrfx_gpiote_output_config_t const * p_config,
                                 nrfx_gpiote_task_config_t const *   p_task_config)
{
    NRFX_ASSERT(p_instance);

    return gpiote_output_configure(p_instance, pin, p_config, p_task_config);
}

void nrfx_gpiote_global_callback_set(nrfx_gpiote_t *                 p_instance,
                                     nrfx_gpiote_interrupt_handler_t handler,
                                     void *                          p_context)
{
    NRFX_ASSERT(p_instance);

    gpiote_global_callback_set(p_instance, handler, p_context);
}

int nrfx_gpiote_channel_get(nrfx_gpiote_t const * p_instance,
                            nrfx_gpiote_pin_t     pin,
                            uint8_t *             p_channel)
{
    NRFX_ASSERT(p_instance);

    return gpiote_channel_get(p_instance, pin, p_channel);
}

uint32_t nrfx_gpiote_channels_number_get(nrfx_gpiote_t const * p_instance)
{
    NRFX_ASSERT(p_instance);

    return get_config(p_instance->cb.drv_inst_idx)->channels_number;
}

int nrfx_gpiote_init(nrfx_gpiote_t * p_instance, uint8_t interrupt_priority)
{
    NRFX_ASSERT(p_instance);

    return gpiote_init(p_instance, interrupt_priority);
}

bool nrfx_gpiote_init_check(nrfx_gpiote_t const * p_instance)
{
    NRFX_ASSERT(p_instance);

    return gpiote_init_check(p_instance);
}

void nrfx_gpiote_uninit(nrfx_gpiote_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    gpiote_uninit(p_instance);
}

int nrfx_gpiote_pin_uninit(nrfx_gpiote_t * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    return pin_uninit(p_instance, pin);
}

int nrfx_gpiote_channel_free(nrfx_gpiote_t * p_instance, uint8_t channel)
{
    NRFX_ASSERT(p_instance);

    return pin_channel_free(p_instance, channel);
}

int nrfx_gpiote_channel_alloc(nrfx_gpiote_t * p_instance, uint8_t * p_channel)
{
    NRFX_ASSERT(p_instance);
    int rv = pin_channel_alloc(p_instance);
    if (rv < 0)
    {
        return rv;
    }

    *p_channel = (uint8_t)rv;
    return 0;
}

void nrfx_gpiote_out_set(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    pin_out_set(p_instance, pin);
}

void nrfx_gpiote_out_clear(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    pin_out_clear(p_instance, pin);
}

void nrfx_gpiote_out_toggle(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    pin_out_toggle(p_instance, pin);
}

void nrfx_gpiote_out_task_enable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    pin_out_task_enable(p_instance, pin);
}

void nrfx_gpiote_out_task_disable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    pin_out_task_disable(p_instance, pin);
}

nrf_gpiote_task_t nrfx_gpiote_out_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    return pin_out_task_get(p_instance, pin);
}

uint32_t nrfx_gpiote_out_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    return pin_out_task_address_get(p_instance, pin);
}

#if defined(GPIOTE_FEATURE_SET_PRESENT)
nrf_gpiote_task_t nrfx_gpiote_set_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    return pin_set_task_get(p_instance, pin);
}

uint32_t nrfx_gpiote_set_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    return pin_set_task_address_get(p_instance, pin);
}
#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if defined(GPIOTE_FEATURE_CLR_PRESENT)
nrf_gpiote_task_t nrfx_gpiote_clr_task_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    return pin_clr_task_get(p_instance, pin);
}

uint32_t nrfx_gpiote_clr_task_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    nrf_gpiote_task_t task = pin_clr_task_get(p_instance, pin);
    return nrfy_gpiote_task_address_get(p_instance->p_reg, task);
}
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)

void nrfx_gpiote_out_task_force(nrfx_gpiote_t const * p_instance,
                                nrfx_gpiote_pin_t     pin,
                                uint8_t               state)
{
    NRFX_ASSERT(p_instance);

    pin_out_task_force(p_instance, pin, state);
}

void nrfx_gpiote_out_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    pin_out_task_trigger(p_instance, pin);
}

#if defined(GPIOTE_FEATURE_SET_PRESENT)
void nrfx_gpiote_set_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    pin_set_task_trigger(p_instance, pin);
}
#endif // defined(GPIOTE_FEATURE_SET_PRESENT)

#if  defined(GPIOTE_FEATURE_CLR_PRESENT)
void nrfx_gpiote_clr_task_trigger(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    pin_clr_task_trigger(p_instance, pin);
}
#endif // defined(GPIOTE_FEATURE_CLR_PRESENT)

void nrfx_gpiote_trigger_enable(nrfx_gpiote_t const * p_instance,
                                nrfx_gpiote_pin_t     pin,
                                bool                  int_enable)
{
    NRFX_ASSERT(p_instance);

    pin_trigger_enable(p_instance, pin, int_enable);
}

void nrfx_gpiote_trigger_disable(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    NRFX_ASSERT(p_instance);

    pin_trigger_disable(p_instance, pin);
}

nrf_gpiote_event_t nrfx_gpiote_in_event_get(nrfx_gpiote_t const * p_instance,
                                            nrfx_gpiote_pin_t     pin)
{
    return pin_in_event_get(p_instance, pin);
}

uint32_t nrfx_gpiote_in_event_address_get(nrfx_gpiote_t const * p_instance, nrfx_gpiote_pin_t pin)
{
    nrf_gpiote_event_t event = pin_in_event_get(p_instance, pin);
    return nrfy_gpiote_event_address_get(p_instance->p_reg, event);
}

static void call_handler(nrfx_gpiote_control_block_t * p_cb,
                         nrfx_gpiote_pin_t             pin,
                         nrfx_gpiote_trigger_t         trigger)
{
    nrfx_gpiote_handler_config_t const * handler = channel_handler_get(p_cb, pin);

    if (handler)
    {
        handler->handler(pin, trigger, handler->p_context);
    }
    if (p_cb->global_handler.handler)
    {
        p_cb->global_handler.handler(pin, trigger, p_cb->global_handler.p_context);
    }
}

static void next_sense_cond_call_handler(nrfx_gpiote_control_block_t * p_cb,
                                         nrfx_gpiote_pin_t             pin,
                                         nrfx_gpiote_trigger_t         trigger,
                                         nrf_gpio_pin_sense_t          sense)
{
    if (is_level(trigger))
    {
        call_handler(p_cb, pin, trigger);
        if (nrfy_gpio_pin_sense_get(pin) == sense)
        {
            /* The sensing mechanism needs to be reenabled here so that the PORT event
             * is generated again for the pin if it stays at the sensed level. */
            nrfy_gpio_cfg_sense_set(pin, NRF_GPIO_PIN_NOSENSE);
            nrfy_gpio_cfg_sense_set(pin, sense);
        }
    }
    else
    {
        /* Reconfigure sense to the opposite level, so the internal PINx.DETECT signal
         * can be deasserted. Therefore PORT event can be generated again,
         * unless some other PINx.DETECT signal is still active. */
        nrf_gpio_pin_sense_t next_sense = (sense == NRF_GPIO_PIN_SENSE_HIGH) ?
                NRF_GPIO_PIN_SENSE_LOW : NRF_GPIO_PIN_SENSE_HIGH;

        nrfy_gpio_cfg_sense_set(pin, next_sense);

        /* Invoke user handler only if the sensed pin level matches its polarity
         * configuration. Call handler unconditionally in case of toggle trigger or
         * level trigger. */
        if ((trigger == NRFX_GPIOTE_TRIGGER_TOGGLE) ||
            (sense == NRF_GPIO_PIN_SENSE_HIGH && trigger == NRFX_GPIOTE_TRIGGER_LOTOHI) ||
            (sense == NRF_GPIO_PIN_SENSE_LOW && trigger == NRFX_GPIOTE_TRIGGER_HITOLO))
        {
            call_handler(p_cb, pin, trigger);
        }
    }
}

#if defined(NRF_GPIO_LATCH_PRESENT)
static bool latch_pending_read_and_check(uint32_t * latch, uint32_t available_gpio_ports)
{
    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        if (nrf_bitmask_bit_is_set(port_index[port_idx], &available_gpio_ports))
        {
            nrfy_gpio_latches_read_and_clear(port_idx, 1, &latch[port_idx]);

            if (latch[port_idx])
            {
                /* If any of the latch bits is still set, it means another edge has been captured
                * before or during the interrupt processing. Therefore event-processing loop
                * should be executed again. */
                return true;
            }
        }
    }
    return false;
}

static void port_event_handle(NRF_GPIOTE_Type *             p_gpiote,
                              nrfx_gpiote_control_block_t * p_cb,
                              const gpiote_config_t *       p_config)
{
    uint32_t latch[GPIO_COUNT] = {0};

    latch_pending_read_and_check(latch, p_config->available_gpio_ports);

    do {
        for (uint32_t i = 0; i < GPIO_COUNT; i++)
        {
            if (nrf_bitmask_bit_is_set(port_index[i], &p_config->available_gpio_ports))
            {
                while (latch[i])
                {
                    uint32_t pin = NRF_CTZ(latch[i]);

                    /* Convert to absolute value. */
                    nrfx_gpiote_pin_t abs_pin = NRF_PIN_PORT_TO_PIN_NUMBER(pin, ports[i]);
                    nrf_gpio_pin_sense_t sense;
                    nrfx_gpiote_trigger_t trigger =
                        PIN_FLAG_TRIG_MODE_GET(p_cb->pin_flags[get_pin_idx(abs_pin)]);

                    nrf_bitmask_bit_clear(pin, &latch[i]);
                    sense = nrfy_gpio_pin_sense_get(abs_pin);

                    next_sense_cond_call_handler(p_cb, abs_pin, trigger, sense);
                    /* Try to clear LATCH bit corresponding to currently processed pin.
                    * This may not succeed if the pin's state changed during the interrupt processing
                    * and now it matches the new sense configuration. In such case,
                    * the pin will be processed again in another iteration of the outer loop. */
                    nrfy_gpio_pin_latch_clear(abs_pin);
                }
            }
        }

        /* All pins have been handled, clear PORT, check latch again in case
         * something came between deciding to exit and clearing PORT event. */
        (void)nrfy_gpiote_events_process(p_gpiote, (uint32_t)NRF_GPIOTE_INT_PORT_MASK, 0);
    } while (latch_pending_read_and_check(latch, p_config->available_gpio_ports));
}

#else

static bool input_read_and_check(uint32_t * input,
                                 uint32_t * pins_to_check,
                                 uint32_t   available_gpio_ports)
{
    bool process_inputs_again;
    uint32_t new_input[GPIO_COUNT];

    process_inputs_again = false;
    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        if (nrf_bitmask_bit_is_set(port_index[port_idx], &available_gpio_ports))
        {
            nrfy_gpio_ports_read(port_idx, 1, &new_input[port_idx]);

            /* Execute XOR to find out which inputs have changed. */
            uint32_t input_diff = input[port_idx] ^ new_input[port_idx];
            input[port_idx] = new_input[port_idx];
            if (input_diff)
            {
                /* If any differences among inputs were found, mark those pins
                * to be processed again. */
                pins_to_check[port_idx] &= input_diff;
                process_inputs_again = true;
            }
            else
            {
                pins_to_check[port_idx] = 0;
            }
        }
    }
    return process_inputs_again;
}

static void port_event_handle(NRF_GPIOTE_Type *             p_gpiote,
                              nrfx_gpiote_control_block_t * p_cb,
                              const gpiote_config_t *       p_config)
{
    uint32_t pins_to_check[GPIO_COUNT] = {0};
    uint32_t input[GPIO_COUNT] = {0};
    uint8_t rel_pin;
    nrfx_gpiote_pin_t pin;
    nrfx_gpiote_trigger_t trigger;

    for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
    {
        if (nrf_bitmask_bit_is_set(port_index[port_idx], &p_config->available_gpio_ports))
        {
            nrfy_gpio_ports_read(port_idx, 1, &input[port_idx]);
            pins_to_check[port_idx] = p_cb->port_pins[port_idx];
        }
    }

    do {
        for (uint32_t i = 0; i < GPIO_COUNT; i++)
        {
            while (pins_to_check[i])
            {
                nrf_gpio_pin_sense_t sense;
                bool pin_state;

                rel_pin = (uint8_t)NRF_CTZ(pins_to_check[i]);
                pins_to_check[i] &= ~NRFX_BIT(rel_pin);
                /* Absolute */
                pin = rel_pin + 32 * i;

                trigger = PIN_FLAG_TRIG_MODE_GET(p_cb->pin_flags[get_pin_idx(pin)]);
                sense = nrfy_gpio_pin_sense_get(pin);
                pin_state = nrf_bitmask_bit_is_set(pin, input);

                /* Process pin further only if its state matches its sense level. */
                if ((pin_state && (sense == NRF_GPIO_PIN_SENSE_HIGH)) ||
                    (!pin_state && (sense == NRF_GPIO_PIN_SENSE_LOW)) )
                {
                    next_sense_cond_call_handler(p_cb, pin, trigger, sense);
                }
            }
        }

        for (uint32_t port_idx = 0; port_idx < GPIO_COUNT; port_idx++)
        {
            if (nrf_bitmask_bit_is_set(port_index[port_idx], &p_config->available_gpio_ports))
            {
                /* All pins used with PORT must be rechecked because it's content and
                 * number of port pins may have changed during handler execution. */
                pins_to_check[port_idx] = p_cb->port_pins[port_idx];

                /* Small trick to continue check if input level is equal to the trigger:
                * Set input to the opposite level. If input equals trigger level that
                * it will be set in pins_to_check. */

                uint32_t pin_mask = pins_to_check[port_idx];

                while (pin_mask)
                {
                    rel_pin = (uint8_t)NRF_CTZ(pin_mask);
                    pin_mask &= ~NRFX_BIT(rel_pin);
                    pin = rel_pin + 32 * port_idx;
                    if (nrfy_gpio_pin_sense_get(pin) != NRF_GPIO_PIN_NOSENSE)
                    {
                        trigger = PIN_FLAG_TRIG_MODE_GET(p_cb->pin_flags[get_pin_idx(pin)]);
                        if (trigger == NRFX_GPIOTE_TRIGGER_HIGH)
                        {
                            input[port_idx] &= ~NRFX_BIT(rel_pin);
                        }
                        else if (trigger == NRFX_GPIOTE_TRIGGER_LOW)
                        {
                            input[port_idx] |= NRFX_BIT(rel_pin);
                        }
                    }
                }
            }
        }

        (void)nrfy_gpiote_events_process(p_gpiote, (uint32_t)NRF_GPIOTE_INT_PORT_MASK, 0);
    } while (input_read_and_check(input, pins_to_check, p_config->available_gpio_ports));
}
#endif // defined(NRF_GPIO_LATCH_PRESENT)

static void gpiote_evt_handle(NRF_GPIOTE_Type *             p_gpiote,
                              nrfx_gpiote_control_block_t * p_cb,
                              uint32_t                      mask)
{
    while (mask)
    {
        uint32_t ch = NRF_CTZ(mask);
        mask &= ~NRFX_BIT(ch);
        nrfx_gpiote_pin_t pin = p_cb->ch_pin[ch];
        nrf_gpiote_polarity_t polarity = nrfy_gpiote_event_polarity_get(p_gpiote, ch);

        call_handler(p_cb, pin, gpiote_polarity_to_trigger(polarity));
    }
}

static void irq_handler(NRF_GPIOTE_Type * p_gpiote, nrfx_gpiote_control_block_t * p_cb)
{
    uint32_t instance_idx = p_cb->drv_inst_idx;
    const gpiote_config_t * p_config = get_config(instance_idx);
    /* Collect status of all GPIOTE pin events. Processing is done once all are collected and cleared.*/
#if NRFX_CHECK(NRFX_GPIOTE_CONFIG_NONUNIFORM_INSTANCES)
    uint32_t enabled_in_events = nrf_gpiote_int_group_enable_check(p_gpiote,
                                                                   p_config->group_idx,
                                                                   NRF_GPIOTE_INT_IN_MASK);
    uint32_t enabled_events = enabled_in_events |
                              (p_config->port_supported ? NRF_GPIOTE_INT_PORT_MASK : 0);
#else
    uint32_t enabled_events = nrf_gpiote_int_enable_check(p_gpiote, NRF_GPIOTE_INT_IN_MASK) |
                                                          NRF_GPIOTE_INT_PORT_MASK;
#endif
    uint32_t evt_mask = nrfy_gpiote_events_process(p_gpiote,
                                                   enabled_events,
                                                   p_config->channels_number);

    /* Handle PORT event. */
    if (evt_mask & (uint32_t)NRF_GPIOTE_INT_PORT_MASK)
    {
        port_event_handle(p_gpiote, p_cb, p_config);
        evt_mask &= ~(uint32_t)NRF_GPIOTE_INT_PORT_MASK;
    }

    /* Process pin events. */
    gpiote_evt_handle(p_gpiote, p_cb, evt_mask);
}

void nrfx_gpiote_irq_handler(nrfx_gpiote_t * p_instance)
{
    NRFX_ASSERT(p_instance);

    irq_handler(p_instance->p_reg, &p_instance->cb);
}
