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

#ifndef NRFX_VPR_VIO_PINS_H__
#define NRFX_VPR_VIO_PINS_H__

#include <nrfx.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @cond Internal macro helpers used to facilitate the mapping. */

/* Internal symbol specifying the maximum pin number. */
#define _NRFX_VPR_VIO_PORT_PIN_MAX_COUNT 32

/* Internal symbol specifying the maximum port number. */
#define _NRFX_VPR_VIO_PORT_MAX_COUNT 16

/* Internal macro for getting the name of the symbol that holds the mapping of a pin. */
#define _NRFX_VPR_VIO_PIN_SYMBOL(vpr_idx, port, pin) \
    NRFX_CONCAT(VPR, vpr_idx, _VIO_P, port, _PIN, pin)

/* Internal macro used by NRFX_VPR_VIO_PORT_MASK_GET. */
#define _NRFX_VPR_VIO_PIN_BIT(pin, vpr_idx, port) NRFX_VPR_VIO_PIN_BIT_GET(vpr_idx, port, pin)

/*
 * Internal macro used by NRFX_VPR_VIO_PORT_PIN_NUM_LOWEST.
 *
 * It expands to the pin number followed by a comma, or to nothing when the pin is not
 * accessible by the VIO. Listifying it therefore yields a comma-separated list of the
 * numbers of all accessible pins, in ascending order.
 */
#define _NRFX_VPR_VIO_PIN_NUM_LIST_PIN_FIRST(pin, vpr_idx, port)         \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin), \
        (pin, ), ())

/*
 * Internal macro used NRFX_VPR_VIO_PORT_PIN_NUM_HIGHEST.
 *
 * It expands to the pin number preceded by a comma, or to nothing when the pin is not
 * accessible by the VIO. Listifying it therefore yields a comma-separated list of the
 * numbers of all accessible pins, in ascending order.
 */
#define _NRFX_VPR_VIO_PIN_NUM_LIST_COMMA_FIRST(pin, vpr_idx, port)       \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin), \
        (, pin), ())

/*
 * Internal macro used by NRFX_VPR_VIO_PORT_ACCESSIBLE_CHECK.
 *
 * It expands to "1" followed by a comma, or to nothing when the pin is not accessible by
 * the VIO. Listifying it therefore yields a comma-separated list of "1" for each pin.
 */
#define _NRFX_VPR_VIO_PORT_ACCESSIBLE_LIST(pin, vpr_idx, port)           \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin), \
        (1, ), ())

/*
 * Internal macro used by NRFX_VPR_VIO_MAPPING_DEFINED_CHECK.
 *
 * It expands to "1" followed by a comma, or to nothing when the port is not accessible by
 * the VIO. Listifying it therefore yields a comma-separated list of "1" for each port.
 *
 * Note: The macro cannot be passed as the first argument to NRFX_LISTIFY because
 *       NRFX_LISTIFY is used internally by the macro and nesting is not allowed.
 */
#define _NRFX_VPR_VIO_MAPPING_DEFINED_LIST(port, vpr_idx)               \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PORT_ACCESSIBLE_CHECK(vpr_idx, port), \
        (1, ), ())

/* Internal macro used by _NRFX_VPR_VIO_PORTS_LIST. */
#define _NRFX_VPR_VIO_PORTS_LIST_FILTER(port, vpr_idx)                  \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PORT_ACCESSIBLE_CHECK(vpr_idx, port), \
                (, port), ())

/*
 * Internal macro for generating a comma separated list of ports
 * that can be accessed by a VPR VIO.
 */
#define _NRFX_VPR_VIO_PORTS_LIST(vpr_idx)                                  \
    _NRFX_DROP_ARG1(                                                       \
        NRFX_FOR_EACH_FIXED_ARG(_NRFX_VPR_VIO_PORTS_LIST_FILTER,           \
                                (), vpr_idx,                               \
                                NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_MAX_COUNT, \
                                             _NRFX_GET_ARG1, (,))))

/* Internal macro used by _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET_PROCESS. */
#define _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET_PAIR(pin, port, vpr_idx)  \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin), \
                     (, (port, pin)), ())

/* Internal macro used by __NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET. */
#define _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET_PROCESS(port, vpr_idx)         \
    _NRFX_DROP_ARG1(NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT,            \
                                 _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET_PAIR, \
                                 (), port, vpr_idx))

/* Internal macro used by _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET. */
#define __NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET(vpr_idx, ...)               \
    NRFX_FOR_EACH_FIXED_ARG(_NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET_PROCESS, \
                            (,), vpr_idx, __VA_ARGS__)

/*
 * Internal macro for generating a list of port-pin combinations
 * that can be accessed by the specified VIO. The result is a comma separated list of
 * tuples of the form (<port>, <pin>).
 */
#define _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_LIST(vpr_idx) \
    __NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_GET(vpr_idx, _NRFX_VPR_VIO_PORTS_LIST(vpr_idx))

/*
 * Internal macro used by _NRFX_VPR_VIO_FOR_EACH_PIN_EXPAND.
 *
 * The macro invokes user-defined macro F.
 */
#define _NRFX_VPR_VIO_FOR_EACH_PIN_EXEC(port, pin, vpr_idx, F, ...) \
    F(vpr_idx, port, pin)

/*
 * Internal macro used by _NRFX_VPR_VIO_FOR_EACH_PIN_DEBRACKET.
 *
 * The macro expands (port, pin) and (vpr_idx, F) tuples into
 * individual arguments needed for _NRFX_VPR_VIO_FOR_EACH_PIN_EXEC.
 */
#define _NRFX_VPR_VIO_FOR_EACH_PIN_EXPAND(port_pin_pair, vpr_idx_F_pair) \
    _NRFX_VPR_VIO_FOR_EACH_PIN_EXEC(port_pin_pair, vpr_idx_F_pair, _)

/*
 * Internal macro used by NRFX_VPR_VIO_FOR_EACH_PIN.
 *
 * The macro is called inside an NRFX_FOR_EACH_FIXED_ARG and as its first argument is
 * passed a (port, pin) tuple, and as the second (fixed) argument (vpr_idx, F) tuple.
 */
#define _NRFX_VPR_VIO_FOR_EACH_PIN_DEBRACKET(port_pin_pair, vpr_idx_F_pair) \
    _NRFX_VPR_VIO_FOR_EACH_PIN_EXPAND(NRFX_DEBRACKET port_pin_pair,         \
                                      NRFX_DEBRACKET vpr_idx_F_pair)

/*
 * Internal macro used by _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_EXPAND.
 *
 * The macro invokes user-defined macro F if the VIO pin index is equal to the
 * user-specified VIO index. The result enclosed in parenthesis to later insert
 * the separator in-between.
 */
#define _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_EXEC(port, pin, vpr_idx, vio_idx, F, ...)        \
    NRFX_COND_CODE_1(                                                                   \
        NRFX_IS_EQ(vio_idx,                                                             \
                   NRFX_COND_CODE_1(NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin), \
                                    (NRFX_VPR_VIO_PIN_INDEX_GET(vpr_idx, port, pin)),   \
                                    (_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT))),               \
                  (, (F(vpr_idx, port, pin))), ())

/*
 * Internal macro used by _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_DEBRACKET.
 *
 * The macro expands (port, pin) and (vpr_idx, vio_index, F) tuples into
 * individual arguments needed for _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_EXEC.
 */
#define _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_EXPAND(port_pin_pair, vpr_vio_F_tuple) \
    _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_EXEC(port_pin_pair, vpr_vio_F_tuple, _)

/*
 * Internal macro used by _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_COMMA_SEP.
 *
 * The macro is called inside an NRFX_FOR_EACH_FIXED_ARG and as its first argument is
 * passed a (port, pin) tuple, and as the second (fixed) argument (vpr_idx, vio_idx, F) tuple.
 */
#define _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_DEBRACKET(port_pin_pair, vpr_vio_F_tuple) \
    _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_EXPAND(NRFX_DEBRACKET port_pin_pair,          \
                                          NRFX_DEBRACKET vpr_vio_F_tuple)

/* Internal macro used by _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_COMMA_SEP_DEBRACKET. */
#define _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_COMMA_SEP(vpr_idx, vio_idx, F)            \
    _NRFX_DROP_ARG1(                                                             \
        NRFX_FOR_EACH_FIXED_ARG(_NRFX_VPR_VIO_FOR_EACH_VIO_PIN_DEBRACKET,        \
                                (), (vpr_idx, vio_idx, F),                       \
                                _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_LIST(vpr_idx)))

/* Internal macro used by NRFX_VPR_VIO_FOR_EACH_VIO_PIN. */
#define _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_COMMA_SEP_DEBRACKET(elem) \
    NRFX_COND_CODE_0(NRFX_IS_EMPTY(elem), (NRFX_DEBRACKET elem), ())

/* Internal macro used by _NRFX_VPR_VIO_LIST_CONTAINS_NUM. */
#define _NRFX_VPR_VIO_LIST_CONTAINS_NUM_I(elem, val) \
    NRFX_COND_CODE_1(NRFX_IS_EQ(elem, val), (1,), ())

/* Internal macro used by X_VPR_VIO_GET_NUM_COMMA_IF_PRESENT. */
#define _NRFX_VPR_VIO_LIST_CONTAINS_NUM(val, ...)                      \
    NRFX_NOT(                                                          \
        NRFX_IS_EMPTY(                                                 \
            NRFX_FOR_EACH_FIXED_ARG(_NRFX_VPR_VIO_LIST_CONTAINS_NUM_I, \
            (), val, __VA_ARGS__)))

/* Internal macro used by _NRFX_VPR_VIO_LIST_GET. */
#define _NRFX_VPR_VIO_GET_NUM_COMMA_IF_PRESENT(val, ...) \
    NRFX_COND_CODE_1(_NRFX_VPR_VIO_LIST_CONTAINS_NUM(val, __VA_ARGS__), (val,), ())

/* Internal macro used by NRFX_VPR_VIO_PRESENT_COUNT_GET. */
#define _NRFX_VPR_VIO_LIST_GET(vpr_idx)                          \
    NRFX_LISTIFY(16, _NRFX_VPR_VIO_GET_NUM_COMMA_IF_PRESENT, (), \
                 NRFX_VPR_VIO_FOR_EACH_PIN(vpr_idx, NRFX_VPR_VIO_PIN_INDEX_GET, (,)))

/* Internal macro used by _NRFX_VPR_VIO_PORT_LIST_GET. */
#define _NRFX_VPR_VIO_PORT_GET_NUM_COMMA_IF_PRESENT(val, ...) \
    NRFX_COND_CODE_1(_NRFX_VPR_VIO_LIST_CONTAINS_NUM(val, __VA_ARGS__), (val,), ())

/* Internal macro used by NRFX_VPR_VIO_MAPPED_PORT_COUNT_GET. */
#define _NRFX_VPR_VIO_PORT_LIST_GET(vpr_idx)                      \
    NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_MAX_COUNT,                    \
                 _NRFX_VPR_VIO_PORT_GET_NUM_COMMA_IF_PRESENT, (), \
                 NRFX_VPR_VIO_FOR_EACH_PIN(vpr_idx, _NRFX_GET_ARG2, (,)))
/*
 * Verify assumptions about the range of possible pin and port numbers:
 *  - P<X>_PIN_NUM_MAX must be less than _NRFX_VPR_VIO_PORT_PIN_MAX_COUNT
 *    for each port number X.
 *  - _NRFX_VPR_VIO_PORT_MAX_COUNT must be less than the maximum port number.
 *  - NRFX_IS_EQ must support the entire range of possible pin numbers.
 */

/* Internal macro used in an assertion. */
#define _NRFX_VPR_VIO_ASSERT_PORT_PIN_NUM(periph, prefix, i, _)       \
    NRFX_STATIC_ASSERT(NRFX_CONCAT(periph, prefix, i, _PIN_NUM_MAX) < \
                       _NRFX_VPR_VIO_PORT_PIN_MAX_COUNT);

NRFX_FOREACH_PRESENT(P, _NRFX_VPR_VIO_ASSERT_PORT_PIN_NUM, (), (), _)

/* Internal macro used in an assertion. */
#define _NRFX_VPR_VIO_MAX_PORT_NUM_LIST(periph, prefix, i, _) i,

/* Internal macro used in an assertion. */
#define _NRFX_VPR_VIO_MAX_PORT_NUM \
    _NRFX_GET_ARG2(                \
        NRFX_REVERSE_ARGS(         \
            NRFX_FOREACH_PRESENT(P, _NRFX_VPR_VIO_MAX_PORT_NUM_LIST, (), (), _) _))

NRFX_STATIC_ASSERT(_NRFX_VPR_VIO_MAX_PORT_NUM < _NRFX_VPR_VIO_PORT_MAX_COUNT);

/* Internal macro used in an assertion. */
#define _NRFX_VPR_VIO_PORT_PIN_MAX_COUNT_LESS_1 \
    _NRFX_GET_ARG1(                             \
        NRFX_REVERSE_ARGS(                      \
            NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT, _NRFX_GET_ARG1, (,))))

NRFX_STATIC_ASSERT(NRFX_IS_EQ(_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT_LESS_1,
                              _NRFX_VPR_VIO_PORT_PIN_MAX_COUNT_LESS_1) == 1);

/** @endcond */

/**
 * @defgroup nrfx_vpr_vio_pins Helper layer for mapping VPR VIO pin indices
 * @{
 * @ingroup nrfx
 * @brief Helper layer that provides the mapping between GPIO pins and bit indices in
 *        VPR VIO registers.
 *
 * The mapping is resolved entirely by the preprocessor. Therefore it can be used in constant
 * expressions, such as static assertions or array sizes, and it occupies neither code nor data
 * memory.
 *
 * The VPR instance is specified by its index, for example @c 121, and not by a
 * pointer to its registers.
 *
 * The mapping itself is declared per SoC in @c nrfx_soc_defines.h.
 */

#if defined(VPR_IDX_FLPR) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol specifying the index of the VPR instance for FLPR. */
#define NRF_VPR_IDX_FLPR VPR_IDX_FLPR
#endif

#if defined(VPR_IDX_PPR) || defined(__NRFX_DOXYGEN__)
/** @brief Symbol specifying the index of the VPR instance for PPR. */
#define NRF_VPR_IDX_PPR VPR_IDX_PPR
#endif

/** @brief Symbol specifying a pin that is not accessible by the VIO. */
#define NRFX_VPR_VIO_PIN_NOT_PRESENT UINT8_MAX

/**
 * @brief Macro for checking whether a pin is accessible by the VIO of a given VPR instance.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 * @param[in] port    Port number of the pin.
 * @param[in] pin     Pin number within the port.
 *
 * @retval 1 If the pin is accessible by the VIO.
 * @retval 0 If the pin is not accessible by the VIO.
 */
#define NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin) \
    NRFX_ARG_HAS_PARENTHESIS(_NRFX_VPR_VIO_PIN_SYMBOL(vpr_idx, port, pin))

/**
 * @brief Macro for getting the VIO bit index of a given pin.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 * @param[in] port    Port number of the pin.
 * @param[in] pin     Pin number within the port.
 *
 * @return Index of the bit in the VIO registers that corresponds to the specified pin, or
 *         @ref NRFX_VPR_VIO_PIN_NOT_PRESENT if the pin is not accessible by the VIO.
 */
#define NRFX_VPR_VIO_PIN_INDEX_GET(vpr_idx, port, pin)                   \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin), \
        (NRFX_DEBRACKET _NRFX_VPR_VIO_PIN_SYMBOL(vpr_idx, port, pin)),   \
        (NRFX_VPR_VIO_PIN_NOT_PRESENT))

/**
 * @brief Macro for getting the VIO register bit mask of a given pin.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 * @param[in] port    Port number of the pin.
 * @param[in] pin     Pin number within the port.
 *
 * @return Mask with the bit that corresponds to the specified pin set, or 0 if the pin is not
 *         accessible by the VIO.
 */
#define NRFX_VPR_VIO_PIN_BIT_GET(vpr_idx, port, pin)                     \
    NRFX_COND_CODE_1(NRFX_VPR_VIO_PIN_PRESENT_CHECK(vpr_idx, port, pin), \
        ((1UL << _NRFX_VPR_VIO_PIN_SYMBOL(vpr_idx, port, pin))),         \
        (0UL))

/**
 * @brief Macro for getting the VIO register bit mask of all pins of a given port.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 * @param[in] port    Port number.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY, which cannot be nested. Therefore,
 *       using this macro within another @ref NRFX_LISTIFY invocation results
 *       in a compilation error.
 *
 * @return Mask with the bits that correspond to all pins of the specified port that are
 *         accessible by the VIO set, or 0 if none of the pins of the port are accessible by the VIO.
 */
#define NRFX_VPR_VIO_PORT_MASK_GET(vpr_idx, port)                               \
    (NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT, _NRFX_VPR_VIO_PIN_BIT, (|), \
                  vpr_idx, port))

/**
 * @brief Macro for getting the pin with the lowest number which is accessible by
 *        the VIO of the specified VPR instance.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 * @param[in] port    Port number.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY, which cannot be nested. Therefore,
 *       using this macro within another @ref NRFX_LISTIFY invocation results
 *       in a compilation error.
 *
 * @return Pin number, or @ref NRFX_VPR_VIO_PIN_NOT_PRESENT if none of the pins of the port
 *         are accessible by the VIO.
 */
#define NRFX_VPR_VIO_PORT_PIN_NUM_LOWEST(vpr_idx, port)    \
    _NRFX_GET_ARG1(                                        \
        NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT,     \
                     _NRFX_VPR_VIO_PIN_NUM_LIST_PIN_FIRST, \
                     (), vpr_idx, port)                    \
        NRFX_VPR_VIO_PIN_NOT_PRESENT)

/**
 * @brief Macro for getting the pin with the highest number which is accessible by
 *        the VIO of the specified VPR instance.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 * @param[in] port    Port number.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY and @ref NRFX_FOR_EACH macros,
 *       which cannot be nested. Therefore, using this macro within another invocation
 *       of @ref NRFX_LISTIFY or @ref NRFX_FOR_EACH results in a compilation error.
 *
 * @return Pin number, or @ref NRFX_VPR_VIO_PIN_NOT_PRESENT if none of the pins of the port
 *         are accessible by the VIO.
 */
#define NRFX_VPR_VIO_PORT_PIN_NUM_HIGHEST(vpr_idx, port)         \
    _NRFX_GET_ARG1(                                              \
        NRFX_REVERSE_ARGS(NRFX_VPR_VIO_PIN_NOT_PRESENT           \
            NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT,       \
                         _NRFX_VPR_VIO_PIN_NUM_LIST_COMMA_FIRST, \
                         (), vpr_idx, port)))

/**
 * @brief Macro for checking whether the specified port is accessible by
 *        the VIO of the specified VPR instance.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 * @param[in] port    Port number.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY, which cannot be nested. Therefore,
 *       using this macro within another @ref NRFX_LISTIFY invocation results
 *       in a compilation error.
 *
 * @retval 1 If the port can be accessed.
 * @retval 0 If the port cannot be accessed.
 */
#define NRFX_VPR_VIO_PORT_ACCESSIBLE_CHECK(vpr_idx, port) \
    _NRFX_GET_ARG1(                                       \
        NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_PIN_MAX_COUNT,    \
                     _NRFX_VPR_VIO_PORT_ACCESSIBLE_LIST,  \
                     (), vpr_idx, port)                   \
        0                                                 \
    )

/**
 * @brief Macro for checking whether there is a definition of VIO pin mapping
 *        associated with the specified VPR instance.
 *
 * @param[in] vpr_idx Index of the VPR instance, for example @c 121.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY and @ref NRFX_FOR_EACH macros,
 *       which cannot be nested. Therefore, using this macro within another invocation
 *       of @ref NRFX_LISTIFY or @ref NRFX_FOR_EACH results in a compilation error.
 *
 * @retval 1 If the mapping is defined.
 * @retval 0 If the mapping is not defined.
 */
#define NRFX_VPR_VIO_MAPPING_DEFINED_CHECK(vpr_idx)                        \
    _NRFX_GET_ARG1(                                                        \
        NRFX_FOR_EACH_FIXED_ARG(_NRFX_VPR_VIO_MAPPING_DEFINED_LIST,        \
                                (), vpr_idx,                               \
                                NRFX_LISTIFY(_NRFX_VPR_VIO_PORT_MAX_COUNT, \
                                             _NRFX_GET_ARG1, (,)))         \
        0                                                                  \
    )

/**
 * @brief Macro for calling a macro @p F for each GPIO pin that can be accessed by
 *        the specified VPR VIO instance.
 *
 * Example:
 *
 * @code
 * struct vpr_pin_info
 * {
 *     uint8_t port;
 *     uint8_t pin;
 * };
 *
 * #define GET_PIN_INFO(_vpr_idx, _port, _pin) { .port = _port, .pin = _pin }
 *
 * struct vpr_pin_info my_vio_pins[] =
 * {
 *     NRFX_VPR_VIO_FOR_EACH_PIN(121, GET_PIN_INFO, (,))
 * };
 * @endcode
 *
 * This expands to:
 *
 * @code
 * struct vpr_pin_info my_vio_pins[] =
 * {
 *     { .port = 1, .pin = 8 },
 *     { .port = 1, .pin = 9 },
 *     ...
 * };
 * @endcode
 *
 * @param[in] vpr_idx Index of the VPR instance.
 * @param[in] F       Macro to invoke. The macro is passed three arguments: VPR index, port, and pin.
 * @param[in] sep     Separator (e.g. comma or semicolon). Must be in parentheses.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY and @ref NRFX_FOR_EACH macros,
 *       which cannot be nested. Therefore, using this macro within another invocation
 *       of @ref NRFX_LISTIFY or @ref NRFX_FOR_EACH results in a compilation error.
 */
#define NRFX_VPR_VIO_FOR_EACH_PIN(vpr_idx, F, sep)                                   \
    NRFX_COND_CODE_1(                                                                \
        NRFX_VPR_VIO_MAPPING_DEFINED_CHECK(vpr_idx),                                 \
        (NRFX_FOR_EACH_FIXED_ARG(_NRFX_VPR_VIO_FOR_EACH_PIN_DEBRACKET,               \
                                 sep, (vpr_idx, F),                                  \
                                 _NRFX_VPR_VIO_ACCESSIBLE_PORT_PINS_LIST(vpr_idx))), \
        ())

/**
 * @brief Macro for calling a macro @p F for each GPIO pin corresponding to a given VIO index
 *        in the specified VPR VIO instance.
 *
 * Example:
 *
 * @code
 * #define GET_PIN_INFO(_vpr_idx, _port, _pin) { .port = _port, .pin = _pin }
 *
 * struct vpr_pin_info my_vio_pins[] =
 * {
 *     NRFX_VPR_VIO_FOR_EACH_VIO_PIN(121, 3, GET_PIN_INFO, (,))
 * };
 * @endcode
 *
 * This expands to:
 *
 * @code
 * struct vpr_pin_info my_vio_pins[] =
 * {
 *     { .port = 1, .pin = 11 },
 *     { .port = 6, .pin = 5 },
 *     ...
 * };
 * @endcode
 *
 * @param[in] vpr_idx Index of the VPR instance.
 * @param[in] vio_idx Index in the VIO registers of the specified VPR instance.
 * @param[in] F       Macro to invoke. The macro is passed three arguments: VPR index, port, and pin.
 * @param[in] sep     Separator (e.g. comma or semicolon). Must be in parentheses.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY and @ref NRFX_FOR_EACH macros,
 *       which cannot be nested. Therefore, using this macro within another invocation
 *       of @ref NRFX_LISTIFY or @ref NRFX_FOR_EACH results in a compilation error.
 */
#define NRFX_VPR_VIO_FOR_EACH_VIO_PIN(vpr_idx, vio_idx, F, sep)            \
    NRFX_FOR_EACH(_NRFX_VPR_VIO_FOR_EACH_VIO_PIN_COMMA_SEP_DEBRACKET, sep, \
                  _NRFX_VPR_VIO_FOR_EACH_VIO_PIN_COMMA_SEP(vpr_idx, vio_idx, F))

/**
 * @brief Macro for checking whether there exists a mapping for a given VIO index in
 *        the specified VPR VIO instance.
 *
 * @param[in] vpr_idx Index of the VPR instance.
 * @param[in] vio_idx Index in the VIO registers of the specified VPR instance.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY and @ref NRFX_FOR_EACH macros,
 *       which cannot be nested. Therefore, using this macro within another invocation
 *       of @ref NRFX_LISTIFY or @ref NRFX_FOR_EACH results in a compilation error.
 *
 * @retval 1 If the mapping exists.
 * @retval 0 If the mapping doesn't exist.
 */
#define NRFX_VPR_VIO_PRESENT_CHECK(vpr_idx, vio_idx) \
    NRFX_NOT(                                        \
        NRFX_IS_EMPTY(                               \
            NRFX_VPR_VIO_FOR_EACH_VIO_PIN(vpr_idx, vio_idx, _NRFX_GET_ARG1, ())))

/**
 * @brief Macro for getting the number of VIO indices for which pin mapping has been defined.
 *
 * @param[in] vpr_idx Index of the VPR instance.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY, which cannot be nested. Therefore,
 *       using this macro within another @ref NRFX_LISTIFY invocation results
 *       in a compilation error.
 *
 * @return Number of VIO indices.
 */
#define NRFX_VPR_VIO_PRESENT_COUNT_GET(vpr_idx) \
    NRFX_NUM_VA_ARGS_LESS_1(_NRFX_VPR_VIO_LIST_GET(vpr_idx))

/**
 * @brief Macro for getting the number of GPIO ports for which pin mapping has been defined.
 *
 * @param[in] vpr_idx Index of the VPR instance.
 *
 * @note This macro internally uses @ref NRFX_LISTIFY, which cannot be nested. Therefore,
 *       using this macro within another @ref NRFX_LISTIFY invocation results
 *       in a compilation error.
 *
 * @return Number of VIO indices.
 */
#define NRFX_VPR_VIO_MAPPED_PORT_COUNT_GET(vpr_idx) \
    NRFX_NUM_VA_ARGS_LESS_1(_NRFX_VPR_VIO_PORT_LIST_GET(vpr_idx))

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_VPR_VIO_PINS_H__
