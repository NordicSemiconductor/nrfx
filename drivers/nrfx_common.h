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

#ifndef NRFX_COMMON_H__
#define NRFX_COMMON_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include <nrf.h>
#include "nrfx_utils.h"
#include <nrf_peripherals.h>
#include <nrf_mem.h>
#include "nrfx_ext.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__CORTEX_M) || defined(__NRFX_DOXYGEN__)
#define ISA_ARM     1
#elif defined(__VPR_REV)
#define ISA_RISCV   1
#else
#define ISA_UNKNOWN 1
#endif

#if defined(ISA_RISCV)
#define __STATIC_INLINE __attribute__((always_inline)) static inline
#endif

#ifndef NRFX_STATIC_INLINE
#ifdef NRFX_DECLARE_ONLY
#define NRFX_STATIC_INLINE
#else
#define NRFX_STATIC_INLINE __STATIC_INLINE
#endif
#endif // NRFX_STATIC_INLINE

#define NRFY_STATIC_INLINE __STATIC_INLINE

#ifndef NRF_STATIC_INLINE
#ifdef NRF_DECLARE_ONLY
#define NRF_STATIC_INLINE
#else
#define NRF_STATIC_INLINE __STATIC_INLINE
#endif
#endif // NRF_STATIC_INLINE

/**
 * @defgroup nrfx_common Common module
 * @{
 * @ingroup nrfx
 * @brief Common module.
 */

/** @brief Symbol specifying major number of the current nrfx version. */
#define NRFX_RELEASE_VER_MAJOR 3

/** @brief Symbol specifying minor number of the current nrfx version. */
#define NRFX_RELEASE_VER_MINOR 14

/** @brief Symbol specifying micro number of the current nrfx version. */
#define NRFX_RELEASE_VER_MICRO 0

/**
 * @brief Macro for checking if the specified identifier is defined and it has
 *        a non-zero value.
 *
 * Normally, preprocessors treat all undefined identifiers as having the value
 * zero. However, some tools, like static code analyzers, can issue a warning
 * when such identifier is evaluated. This macro gives the possibility to suppress
 * such warnings only in places where this macro is used for evaluation, not in
 * the whole analyzed code.
 */
#define NRFX_CHECK(module_enabled) NRFX_IS_ENABLED(module_enabled)

/**
 * @brief Macro for checking if the nrfx version is greater than or equal
 *        to the specified version.
 *
 * @note Current nrfx version is specified with the following symbols:
 *       - @ref NRFX_RELEASE_VER_MAJOR
 *       - @ref NRFX_RELEASE_VER_MINOR
 *       - @ref NRFX_RELEASE_VER_MICRO
 *
 * @param[in] major Major version.
 * @param[in] minor Minor version.
 * @param[in] micro Micro version.
 *
 * @retval true  Current nrfx version is greater than or equal to the specified version.
 * @retval false Current nrfx version is smaller than the specified version.
 */
#define NRFX_RELEASE_VER_AT_LEAST(major, minor, micro) \
    (((NRFX_RELEASE_VER_MAJOR > (major))) ||                                      \
     ((NRFX_RELEASE_VER_MAJOR == major) && (NRFX_RELEASE_VER_MINOR > (minor))) || \
     ((NRFX_RELEASE_VER_MAJOR == major) && (NRFX_RELEASE_VER_MINOR == minor) &&   \
      (NRFX_RELEASE_VER_MICRO >= (micro))))

/**
 * @brief Macro for checking if the configured API version is greater than or equal
 *        to the specified API version.
 *
 * @note API version to be used is configured using following symbols:
 *       - @ref NRFX_CONFIG_API_VER_MAJOR
 *       - @ref NRFX_CONFIG_API_VER_MINOR
 *       - @ref NRFX_CONFIG_API_VER_MICRO
 *
 * @param[in] major Major API version.
 * @param[in] minor Minor API version.
 * @param[in] micro Micro API version.
 *
 * @retval true  Configured API version is greater than or equal to the specified API version.
 * @retval false Configured API version is smaller than the specified API version.
 */
#define NRFX_API_VER_AT_LEAST(major, minor, micro) \
    (((NRFX_CONFIG_API_VER_MAJOR > (major))) ||                                         \
     ((NRFX_CONFIG_API_VER_MAJOR == major) && (NRFX_CONFIG_API_VER_MINOR > (minor))) || \
     ((NRFX_CONFIG_API_VER_MAJOR == major) && (NRFX_CONFIG_API_VER_MINOR == minor) &&   \
      (NRFX_CONFIG_API_VER_MICRO >= (micro))))

/**
 * @brief Macro for creating unsigned integer with bit position @p x set.
 *
 * @param[in] x Bit position to be set.
 *
 * @return Unsigned integer with requested bit position set.
 */
#define NRFX_BIT(x) (1UL << (x))

/**
 * @brief Macro for returning bit mask or 0 if @p x is 0.
 *
 * @param[in] x Bit mask size. Bit mask has bits 0 through x-1 (inclusive) set.
 *
 * @return Bit mask.
 */
#define NRFX_BIT_MASK(x) (((x) == 32) ? UINT32_MAX : ((1UL << (x)) - 1))

/**
 * @brief Macro for returning size in bits for given size in bytes.
 *
 * @param[in] x Size in bytes.
 *
 * @return Size in bits.
 */
#define NRFX_BIT_SIZE(x) ((x) << 3)

/**
 * @brief Macro for concatenating two tokens in macro expansion.
 *
 * @note This macro is expanded in two steps so that tokens given as macros
 *       themselves are fully expanded before they are merged.
 *
 * @param[in] p1 First token.
 * @param[in] p2 Second token.
 *
 * @return The two tokens merged into one, unless they cannot together form
 *         a valid token (in such case, the preprocessor issues a warning and
 *         does not perform the concatenation).
 *
 * @sa NRFX_CONCAT_3
 */
#define NRFX_CONCAT_2(p1, p2) NRFX_CONCAT_2_(p1, p2)

/** @brief Internal macro used by @ref NRFX_CONCAT_2 to perform the expansion in two steps. */
#define NRFX_CONCAT_2_(p1, p2) p1 ## p2

/**
 * @brief Macro for concatenating three tokens in macro expansion.
 *
 * @note This macro is expanded in two steps so that tokens given as macros
 *       themselves are fully expanded before they are merged.
 *
 * @param[in] p1 First token.
 * @param[in] p2 Second token.
 * @param[in] p3 Third token.
 *
 * @return The three tokens merged into one, unless they cannot together form
 *         a valid token (in such case, the preprocessor issues a warning and
 *         does not perform the concatenation).
 *
 * @sa NRFX_CONCAT_2
 */
#define NRFX_CONCAT_3(p1, p2, p3) NRFX_CONCAT_3_(p1, p2, p3)

/** @brief Internal macro used by @ref NRFX_CONCAT_3 to perform the expansion in two steps. */
#define NRFX_CONCAT_3_(p1, p2, p3) p1 ## p2 ## p3

/**
 * @brief Macro for computing the absolute value of an integer number.
 *
 * @param[in] a Input value.
 *
 * @return Absolute value.
 */
#define NRFX_ABS(a) ((a) < (0) ? -(a) : (a))

/**
 * @brief Macro for checking whether any of the instance of the specified peripheral supports a given feature.
 *
 * Macro checks flags set in \<device\>_peripherals.h file.
 *
 * Macro supports check on instances with following names:
 * - \<periph_name\>0 - \<periph_name\>255 - e.g. SPIM0, SPIM255
 * - \<periph_name\>00 - \<periph_name\>099 - e.g. SPIM00, SPIM099
 * - \<periph_name\>000 - \<periph_name\>009 - e.g. SPIM000, SPIM009
 *
 * @param[in] periph_name  Peripheral name, e.g. SPIM.
 * @param[in] feature_name Feature flag name suffix following an instance name, e.g.
 *                         _FEATURE_HARDWARE_CSN_PRESENT.
 *
 * @retval 1 At least one instance on current device supports a given feature.
 * @retval 0 None of peripheral instances supports a given feature.
 */
#define NRFX_FEATURE_PRESENT(periph_name, feature_name)                                            \
        NRFX_COND_CODE_0(NRFX_CONCAT(0,                                                            \
                            _NRFX_FEATURE_PRESENT(periph_name, feature_name, 256),                 \
                            _NRFX_FEATURE_PRESENT(NRFX_CONCAT(periph_name, 0), feature_name, 100), \
                            _NRFX_FEATURE_PRESENT(NRFX_CONCAT(periph_name, 00), feature_name, 10)  \
                         ),                                                                        \
                        (0), (1))

/**
 * @brief Macro for resolving provided user macro for enabled instances of a driver.
 *
 * Macro checks if driver instances are enabled for all potential instaces of a
 * peripheral. It takes peripheral name and checks whether NRFX_\<peripheral\>\<id\>_ENABLED
 * is set to 1 and if yes then provided macro is evaluated for given instance.
 *
 * Macro supports check on instances with following names:
 * - \<periph_name\>0 - \<periph_name\>255 - e.g. SPIM0, SPIM255
 * - \<periph_name\>00 - \<periph_name\>099 - e.g. SPIM00, SPIM099
 * - \<periph_name\>000 - \<periph_name\>009 - e.g. SPIM000, SPIM009
 *
 * @param[in] periph_name Peripheral name, e.g. SPIM.
 * @param[in] macro       Macro which is resolved if driver instance is enabled. Macro has following
 *                        arguments: macro(periph_name, prefix, i, ...).
 * @param[in] sep         Separator added between all evaluations, in parentheses.
 * @param[in] off_code    Code injected for disabled instances, in parentheses.
 */
#define NRFX_FOREACH_ENABLED(periph_name, macro, sep, off_code, ...)                  \
        NRFX_LISTIFY(256, _NRFX_EVAL_IF_ENABLED, sep,                                 \
                     off_code, periph_name, , macro, __VA_ARGS__) NRFX_DEBRACKET sep  \
        NRFX_LISTIFY(100, _NRFX_EVAL_IF_ENABLED, sep,                                 \
                     off_code, periph_name, 0, macro, __VA_ARGS__) NRFX_DEBRACKET sep \
        NRFX_LISTIFY(10, _NRFX_EVAL_IF_ENABLED, sep,                                  \
                     off_code, periph_name, 00, macro, __VA_ARGS__)

/**
 * @brief Macro for resolving provided user macro for present instances of a peripheral.
 *
 * Macro checks if peripheral instances are present by checking if there is a token
 * NRF_\<periph_name\>\<id\> defined with wrapped in parenthesis value.
 *
 * Macro supports check on instances with following names:
 * - \<periph_name\>0 - \<periph_name\>255 - e.g. SPIM0, SPIM255
 * - \<periph_name\>00 - \<periph_name\>099 - e.g. SPIM00, SPIM099
 * - \<periph_name\>000 - \<periph_name\>009 - e.g. SPIM000, SPIM009
 * - \<periph_name\> - e.g. SPIM
 *
 * @param[in] periph_name Peripheral name, e.g. SPIM.
 * @param[in] macro       Macro which is resolved if peripheral instance is present.
 *                        Macro has following arguments: macro(periph_name, prefix, i, ...).
 * @param[in] sep         Separator added between all evaluations, in parentheses.
 * @param[in] off_code    Code injected for disabled instances, in parentheses.
 */
#define NRFX_FOREACH_PRESENT(periph_name, macro, sep, off_code, ...)                   \
        NRFX_LISTIFY(256, _NRFX_EVAL_IF_PRESENT, sep,                                  \
                     off_code, periph_name, , macro, __VA_ARGS__) NRFX_DEBRACKET sep   \
        NRFX_LISTIFY(100, _NRFX_EVAL_IF_PRESENT, sep,                                  \
                     off_code, periph_name, 0, macro, __VA_ARGS__) NRFX_DEBRACKET sep  \
        NRFX_LISTIFY(10, _NRFX_EVAL_IF_PRESENT, sep,                                   \
                     off_code, periph_name, 00, macro, __VA_ARGS__) NRFX_DEBRACKET sep \
        _NRFX_EVAL_IF_PRESENT(, off_code, periph_name, , macro, __VA_ARGS__)

/**
 * @brief Macro for resolving provided user macro on concatenated peripheral name
 *        and instance index.
 *
 * Execute provided macro with single argument <instance\>
 * that is the concatenation of @p periph_name, @p prefix and @p i.
 *
 * @param[in] i           Instance index.
 * @param[in] periph_name Peripheral name, e.g. SPIM.
 * @param[in] prefix      Prefix added before instance index, e.g. some device has
 *                        instances named like SPIM00. First 0 is passed here as prefix.
 * @param[in] macro       Macro which is executed.
 * @param[in] ...         Variable length arguments passed to the @p macro. Macro has following
 *                        arguments: macro(instance, ...).
 */
#define NRFX_INSTANCE_CONCAT(periph_name, prefix, i, macro, ...) \
      macro(NRFX_CONCAT(periph_name, prefix, i), __VA_ARGS__)

/**
 * @brief Macro for creating a content for enum which is listing enabled driver instances.
 *
 * It creates comma separated list of entries like NRFX_\<instance_name\>_INST_IDX,
 * e.g. (NRFX_SPIM0_INST_IDX) for all enabled instances (NRFX_\<instance_name\>_ENABLED
 * is set to 1). It should be called within enum declaration. Created enum is used
 * by the driver to index all enabled instances of the driver.
 *
 * @param[in] periph_name Peripheral name (e.g. SPIM).
 */
#define NRFX_INSTANCE_ENUM_LIST(periph_name) \
        NRFX_FOREACH_ENABLED(periph_name, _NRFX_INST_ENUM, (), ())

/**
 * @brief Macro for creating an interrupt handler for all enabled driver instances.
 *
 * Macro creates a set of functions which calls generic @p irq_handler function with two parameters:
 * - peripheral instance register pointer
 * - pointer to a control block structure associated with the given instance
 *
 * Generic interrupt handler function with above mentioned parameters named @p irq_handler
 * must be implemented in the driver.
 *
 * @note Handlers are using enum which should be generated using @ref NRFX_INSTANCE_ENUM_LIST.
 *
 * @param[in] periph_name       Peripheral name, e.g. SPIM.
 * @param[in] periph_name_small Peripheral name written with small letters, e.g. spim.
 */
#define NRFX_INSTANCE_IRQ_HANDLERS(periph_name, periph_name_small) \
    NRFX_FOREACH_ENABLED(periph_name, _NRFX_IRQ_HANDLER, (), (), periph_name_small)

/**
 * @brief Macro for creating an interrupt handler for all enabled driver instances
 *        with the specified extra parameter.
 *
 * Macro creates set of function which calls generic @p irq_handler function with three parameters:
 * - peripheral instance register pointer
 * - pointer to a control block structure associated with the given instance
 * - provided @p ext_macro called with peripheral name suffix (e.g. 01 for TIMER01)
 *
 * Generic interrupt handler function with above mentioned parameters named @p irq_handler
 * must be implemented in the driver.
 *
 * @note Handlers are using enum which should be generated using @ref NRFX_INSTANCE_ENUM_LIST.
 *
 * @param[in] periph_name       Peripheral name, e.g. SPIM.
 * @param[in] periph_name_small Peripheral name written with small letters, e.g. rtc.
 * @param[in] ext_macro         External macro to be executed for each instance.
 */
#define NRFX_INSTANCE_IRQ_HANDLERS_EXT(periph_name, periph_name_small, ext_macro) \
    NRFX_FOREACH_ENABLED(periph_name, _NRFX_IRQ_HANDLER_EXT, (), (), periph_name_small, ext_macro)

/**
 * @brief Macro for declaring an interrupt handler for all enabled driver instances.
 *
 * Macro creates set of function declarations. It is intended to be used in the driver header.
 *
 * @param[in] periph_name       Peripheral name, e.g. SPIM.
 * @param[in] periph_name_small Peripheral name written with small letters, e.g. spim.
 */
#define NRFX_INSTANCE_IRQ_HANDLERS_DECLARE(periph_name, periph_name_small) \
    NRFX_FOREACH_ENABLED(periph_name, _NRFX_IRQ_HANDLER_DECLARE, (), (), periph_name_small)

/**
 * @brief Macro for generating comma-separated list of interrupt handlers for all
 *        enabled driver instances.
 *
 * Interrupt handlers are generated using @ref NRFX_INSTANCE_IRQ_HANDLERS.
 * It is intended to be used to create a list which is used for passing an interrupt
 * handler function to the PRS driver.
 *
 * @param[in] periph_name       Peripheral name, e.g. SPIM.
 * @param[in] periph_name_small Peripheral name written with small letters, e.g. spim.
 */
#define NRFX_INSTANCE_IRQ_HANDLERS_LIST(periph_name, periph_name_small) \
    NRFX_FOREACH_ENABLED(periph_name, _NRFX_IRQ_HANDLER_LIST, (), (), periph_name_small)

/**
 * @brief Macro for checking if given peripheral instance is present on the target.
 *
 * Macro utilizes the fact that for each existing instance a define is created which points to
 * the memory mapped register set casted to a register set structure. It is wrapped in parenthesis
 * and existance of parethesis wrapping is used to determine if instance exists. It if does not
 * exist then token (e.g. NRF_SPIM10) is undefined so it does not have parenthesis wrapping.
 *
 * Since macro returns literal 1 it can be used by other macros.
 *
 * @param[in] _inst Instance, .e.g SPIM10.
 *
 * @retval 1 If instance is present.
 * @retval 0 If instance is not present.
 */
#define NRFX_INSTANCE_PRESENT(_inst) NRFX_ARG_HAS_PARENTHESIS(NRFX_CONCAT(NRF_, _inst))

/**
 * @brief Macro for getting the smaller value between two arguments.
 *
 * @param[in] a First argument.
 * @param[in] b Second argument.
 *
 * @return Smaller value between two arguments.
 */
#define NRFX_MIN(a, b) ((a) < (b) ? (a) : (b))

/**
 * @brief Macro for getting the larger value between two arguments.
 *
 * @param[in] a First argument.
 * @param[in] b Second argument.
 *
 * @return Larger value between two arguments.
 */
#define NRFX_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * @brief Macro for checking if a given value is in a given range.
 *
 * @note @p val is evaluated twice.
 *
 * @param[in] val A value to be checked.
 * @param[in] min The lower bound (inclusive).
 * @param[in] max The upper bound (inclusive).
 *
 * @retval true  The value is in the given range.
 * @retval false The value is out of the given range.
 */
#define NRFX_IN_RANGE(val, min, max) ((val) >= (min) && (val) <= (max))

/**
 * @brief Macro for performing rounded integer division (as opposed to
 *        truncating the result).
 *
 * @param[in] a Numerator.
 * @param[in] b Denominator.
 *
 * @return Rounded (integer) result of dividing @c a by @c b.
 */
#define NRFX_ROUNDED_DIV(a, b) \
    ((((a) < 0) ^ ((b) < 0)) ? (((a) - (b) / 2) / (b)) : (((a) + (b) / 2) / (b)))

/**
 * @brief Macro for performing integer division, making sure the result is rounded up.
 *
 * @details A typical use case for this macro is to compute the number of objects
 *          with size @c b required to hold @c a number of bytes.
 *
 * @param[in] a Numerator.
 * @param[in] b Denominator.
 *
 * @return Integer result of dividing @c a by @c b, rounded up.
 */
#define NRFX_CEIL_DIV(a, b) ((((a) - 1) / (b)) + 1)

/**
 * @brief Macro for getting the number of elements in an array.
 *
 * @param[in] array Name of the array.
 *
 * @return Array element count.
 */
#define NRFX_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/**
 * @brief Macro for getting the offset (in bytes) from the beginning of a structure
 *        of the specified type to its specified member.
 *
 * @param[in] type   Structure type.
 * @param[in] member Structure member whose offset is searched for.
 *
 * @return Member offset in bytes.
 */
#define NRFX_OFFSETOF(type, member) ((size_t) & (((type *)0)->member))

/**
 * @brief Macro for checking whether given number is power of 2.
 *
 * @param[in] val Tested value.
 *
 * @retval true  The value is power of 2.
 * @retval false The value is not power of 2.
 */
#define NRFX_IS_POWER_OF_TWO(val) (((val) != 0) && ((val) & ((val) - 1)) == 0)

/**
 * @brief Macro for checking whether a given number is even.
 *
 * @param[in] val Tested value.
 *
 * @retval true  The value is even.
 * @retval false The value is odd.
 */
#define NRFX_IS_EVEN(val) (((val) % 2)  == 0)

/**
 * @brief Macro for checking if given lengths of EasyDMA transfers do not exceed
 *        the limit of the specified peripheral.
 *
 * @param[in] peripheral Peripheral to check the lengths against.
 * @param[in] length1    First length to be checked.
 * @param[in] length2    Second length to be checked (pass 0 if not needed).
 *
 * @retval true  The length of buffers does not exceed the limit of the specified peripheral.
 * @retval false The length of buffers exceeds the limit of the specified peripheral.
 */
#define NRFX_EASYDMA_LENGTH_VALIDATE(peripheral, length1, length2)            \
    (((length1) < (1U << NRFX_CONCAT_2(peripheral, _EASYDMA_MAXCNT_SIZE))) && \
     ((length2) < (1U << NRFX_CONCAT_2(peripheral, _EASYDMA_MAXCNT_SIZE))))

/**
 * @brief Macro for waiting until condition is met.
 *
 * @param[in]  condition Condition to meet.
 * @param[in]  attempts  Maximum number of condition checks. Must not be 0.
 * @param[in]  delay_us  Delay between consecutive checks, in microseconds.
 * @param[out] result    Boolean variable to store the result of the wait process.
 *                       Set to true if the condition is met or false otherwise.
 */
#define NRFX_WAIT_FOR(condition, attempts, delay_us, result) \
do {                                                         \
    result =  false;                                         \
    uint32_t remaining_attempts = (attempts);                \
    do {                                                     \
           if (condition)                                    \
           {                                                 \
               result =  true;                               \
               break;                                        \
           }                                                 \
           NRFX_DELAY_US(delay_us);                          \
    } while (--remaining_attempts);                          \
} while(0)

/**
 * @brief Macro for getting the ID number of the specified peripheral.
 *
 * For peripherals in Nordic SoCs, there is a direct relationship between their
 * ID numbers and their base addresses. See the chapter "Peripheral interface"
 * (section "Peripheral ID") in the Product Specification.
 *
 * @param[in] base_addr Peripheral base address or pointer.
 *
 * @return ID number associated with the specified peripheral.
 */
#define NRFX_PERIPHERAL_ID_GET(base_addr) (uint16_t)(((uint32_t)(base_addr) >> 12) & 0x000001FF)

/**
 * @brief Macro for getting the interrupt number assigned to a specific
 *        peripheral.
 *
 * For peripherals in Nordic SoCs, the IRQ number assigned to a peripheral is
 * equal to its ID number. See the chapter "Peripheral interface" (sections
 * "Peripheral ID" and "Interrupts") in the Product Specification.
 *
 * @warning This macro is valid only for peripherals with a single interrupt line.
 *
 * @param[in] base_addr Peripheral base address or pointer.
 *
 * @return Interrupt number associated with the specified peripheral.
 */
#define NRFX_IRQ_NUMBER_GET(base_addr) NRFX_PERIPHERAL_ID_GET(base_addr)

/**
 * @brief Macro for converting frequency in kHz to Hz.
 *
 * @param[in] freq Frequency value in kHz.
 *
 * @return Number of Hz in @p freq kHz.
 */
#define NRFX_KHZ_TO_HZ(freq) ((freq) * 1000)

/**
 * @brief Macro for converting frequency in MHz to Hz.
 *
 * @param[in] freq Frequency value in MHz.
 *
 * @return Number of Hz in @p freq MHz.
 */
#define NRFX_MHZ_TO_HZ(freq) ((freq) * 1000 * 1000)

/** @brief IRQ handler type. */
typedef void (* nrfx_irq_handler_t)(void);

/** @brief Driver state. */
typedef enum
{
    NRFX_DRV_STATE_UNINITIALIZED, ///< Uninitialized.
    NRFX_DRV_STATE_INITIALIZED,   ///< Initialized but powered off.
    NRFX_DRV_STATE_POWERED_ON,    ///< Initialized and powered on.
} nrfx_drv_state_t;

/**
 * @brief Function for checking if an object is placed in the Data RAM region.
 *
 * Several peripherals (the ones using EasyDMA) require the transfer buffers
 * to be placed in the Data RAM region. This function can be used to check if
 * this condition is met.
 *
 * @param[in] p_object Pointer to an object whose location is to be checked.
 *
 * @retval true  The pointed object is located in the Data RAM region.
 * @retval false The pointed object is not located in the Data RAM region.
 */
NRF_STATIC_INLINE bool nrfx_is_in_ram(void const * p_object);

/**
 * @brief Function for checking if an object is aligned to a 32-bit word
 *
 * Several peripherals (the ones using EasyDMA) require the transfer buffers
 * to be aligned to a 32-bit word. This function can be used to check if
 * this condition is met.
 *
 * @param[in] p_object  Pointer to an object whose location is to be checked.
 *
 * @retval true  The pointed object is aligned to a 32-bit word.
 * @retval false The pointed object is not aligned to a 32-bit word.
 */
NRF_STATIC_INLINE bool nrfx_is_word_aligned(void const * p_object);

/**
 * @brief Function for getting the interrupt number for the specified peripheral.
 *
 * @warning This function is valid only for peripherals with a single interrupt line.
 *
 * @param[in] p_reg Peripheral base pointer.
 *
 * @return Interrupt number associated with the pointed peripheral.
 */
NRF_STATIC_INLINE IRQn_Type nrfx_get_irq_number(void const * p_reg);

/**
 * @brief Function for converting an INTEN register bit position to the
 *        corresponding event identifier.
 *
 * The event identifier is the offset between the event register address and
 * the peripheral base address, and is equal (thus, can be directly cast) to
 * the corresponding value of the enumerated type from HAL (nrf_*_event_t).
 *
 * @param[in] bit INTEN register bit position.
 *
 * @return Event identifier.
 *
 * @sa nrfx_event_to_bitpos
 */
NRF_STATIC_INLINE uint32_t nrfx_bitpos_to_event(uint32_t bit);

/**
 * @brief Function for converting an event identifier to the corresponding
 *        INTEN register bit position.
 *
 * The event identifier is the offset between the event register address and
 * the peripheral base address, and is equal (thus, can be directly cast) to
 * the corresponding value of the enumerated type from HAL (nrf_*_event_t).
 *
 * @param[in] event Event identifier.
 *
 * @return INTEN register bit position.
 *
 * @sa nrfx_bitpos_to_event
 */
NRF_STATIC_INLINE uint32_t nrfx_event_to_bitpos(uint32_t event);

#ifndef NRF_DECLARE_ONLY

NRF_STATIC_INLINE bool nrfx_is_in_ram(void const * p_object)
{
    return ((((uint32_t)p_object) & 0xE0000000u) == 0x20000000u);
}

NRF_STATIC_INLINE bool nrfx_is_word_aligned(void const * p_object)
{
    return ((((uint32_t)p_object) & 0x3u) == 0u);
}

NRF_STATIC_INLINE IRQn_Type nrfx_get_irq_number(void const * p_reg)
{
    return (IRQn_Type)NRFX_IRQ_NUMBER_GET(p_reg);
}

NRF_STATIC_INLINE uint32_t nrfx_bitpos_to_event(uint32_t bit)
{
    static const uint32_t event_reg_offset = 0x100u;
    return event_reg_offset + (bit * sizeof(uint32_t));
}

NRF_STATIC_INLINE uint32_t nrfx_event_to_bitpos(uint32_t event)
{
    static const uint32_t event_reg_offset = 0x100u;
    return (event - event_reg_offset) / sizeof(uint32_t);
}

#endif // NRF_DECLARE_ONLY

/** @} */

#ifdef __cplusplus
}
#endif

#endif // NRFX_COMMON_H__
