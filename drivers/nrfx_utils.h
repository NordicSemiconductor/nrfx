/*
 * Copyright (c) 2022 - 2025, Nordic Semiconductor ASA
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

#ifndef NRFX_UTILS_H__
#define NRFX_UTILS_H__

#include "nrfx_utils_internal.h"

/**
 * @defgroup nrfx_utils Preprocessor utility macros
 * @{
 * @ingroup nrfx
 * @brief Preprocessor utility macros.
 */

/**
 * @brief Macro for inserting code depending on whether @p _flag exists and expands to 1 or not.
 *
 * To prevent the preprocessor from treating commas as argument
 * separators, the @p _if_1_code and @p _else_code expressions must be
 * inside brackets/parentheses: <tt>()</tt>. These are stripped away
 * during macro expansion.
 *
 * Example:
 *
 *     NRFX_COND_CODE_1(CONFIG_FLAG, (uint32_t x;), (there_is_no_flag();))
 *
 * If @p CONFIG_FLAG is defined to 1, this expands to:
 *
 *     uint32_t x;
 *
 * It expands to <tt>there_is_no_flag();</tt> otherwise.
 *
 * This could be used as an alternative to:
 *
 *     #if defined(CONFIG_FLAG) && (CONFIG_FLAG == 1)
 *     #define MAYBE_DECLARE(x) uint32_t x
 *     #else
 *     #define MAYBE_DECLARE(x) there_is_no_flag()
 *     #endif
 *
 *     MAYBE_DECLARE(x);
 *
 * However, the advantage of COND_CODE_1() is that code is resolved in
 * place where it is used, while the @p \#if method defines @p
 * MAYBE_DECLARE on two lines and requires it to be invoked again on a
 * separate line. This makes COND_CODE_1() more concise and also
 * sometimes more useful when used within another macro's expansion.
 *
 * @note @p _flag can be the result of preprocessor expansion,
 *       however @p _if_1_code is only expanded if @p _flag expands
 *       to the integer literal 1. Integer expressions that evaluate
 *       to 1, e.g. after doing some arithmetic, will not work.
 *
 * @param[in] _flag      Evaluated flag
 * @param[in] _if_1_code Result if @p _flag expands to 1; must be in parentheses
 * @param[in] _else_code Result otherwise; must be in parentheses
 */
#define NRFX_COND_CODE_1(_flag, _if_1_code, _else_code) \
    _NRFX_COND_CODE_1(_flag, _if_1_code, _else_code)

/**
 * @brief Macro for inserting code depending on whether @p _flag exists and expands to 0 or not.
 *
 * This is like @ref NRFX_COND_CODE_1(), except that it tests whether @p _flag
 * expands to the integer literal 0. It expands to @p _if_0_code if
 * so, and @p _else_code otherwise; both of these must be enclosed in
 * parentheses.
 *
 * @param[in] _flag      Evaluated flag
 * @param[in] _if_0_code Result if @p _flag expands to 0; must be in parentheses
 * @param[in] _else_code Result otherwise; must be in parentheses
 */
#define NRFX_COND_CODE_0(_flag, _if_0_code, _else_code) \
    _NRFX_COND_CODE_0(_flag, _if_0_code, _else_code)

/**
 * @brief Macro for checking for macro definition in compiler-visible expressions
 *
 * It has the effect of taking a macro value that may be defined to "1"
 * or may not be defined at all and turning it into a literal
 * expression that can be handled by the C compiler instead of just
 * the preprocessor.
 *
 * That is, it works similarly to <tt>\#if defined(CONFIG_FOO)</tt>
 * except that its expansion is a C expression. Thus, much <tt>\#ifdef</tt>
 * usage can be replaced with equivalents like:
 *
 *     if (IS_ENABLED(CONFIG_FOO)) {
 *             do_something_with_foo
 *     }
 *
 * This is cleaner since the compiler can generate errors and warnings
 * for @p do_something_with_foo even when @p CONFIG_FOO is undefined.
 *
 * @param[in] config_macro Macro to check
 *
 * @return 1 if @p config_macro is defined to 1, 0 otherwise (including
 *         if @p config_macro is not defined)
 */
#define NRFX_IS_ENABLED(config_macro) _NRFX_IS_ENABLED1(config_macro)

/**
 * @brief Macro for generating a sequence of code with configurable separator.
 *
 * Example:
 *
 *     #define FOO(i, _) MY_PWM ## i
 *     { NRFX_LISTIFY(PWM_COUNT, FOO, (,)) }
 *
 * The above two lines expand to:
 *
 *    { MY_PWM0 , MY_PWM1 }
 *
 * @param[in] LEN The length of the sequence. Must be an integer literal less
 *                than 255.
 * @param[in] F   A macro function that accepts at least two arguments:
 *                <tt>F(i, ...)</tt>. @p F is called repeatedly in the expansion.
 *                Its first argument @p i is the index in the sequence, and
 *                the variable list of arguments passed to LISTIFY are passed
 *                through to @p F.
 * @param[in] sep Separator (e.g. comma or semicolon). Must be in parentheses;
 *            this is required to enable providing a comma as separator.
 *
 * @note Calling NRFX_LISTIFY with undefined arguments has undefined behavior.
 */
#define NRFX_LISTIFY(LEN, F, sep, ...) \
    NRFX_CONCAT_2(_NRFX_LISTIFY_, LEN)(F, sep, __VA_ARGS__)

/**
 * @brief Macro for checking if input argument is empty.
 *
 * Empty means that nothing is provided or provided value is resolved to nothing
 * (e.g. empty define).
 *
 * Macro idea is taken from P99 which is under Apache 2.0 license and described by
 * Jens Gustedt https://gustedt.wordpress.com/2010/06/08/detect-empty-macro-arguments/
 *
 * @param arg Argument.
 *
 * @retval 1 if argument is empty.
 * @retval 0 if argument is not empty.
 */
#define NRFX_IS_EMPTY(arg) _NRFX_IS_EMPTY(arg)

/**
 * @brief Macro for calculating number of arguments in the variable arguments list minus one.
 *
 * @param[in] ... List of arguments
 *
 * @return Number of variadic arguments in the argument list, minus one
 */
#define NRFX_NUM_VA_ARGS_LESS_1(...) \
        _NRFX_NUM_VA_ARGS_LESS_1_IMPL(__VA_ARGS__, 63, 62, 61, \
                    60, 59, 58, 57, 56, 55, 54, 53, 52, 51, \
                    50, 49, 48, 47, 46, 45, 44, 43, 42, 41, \
                    40, 39, 38, 37, 36, 35, 34, 33, 32, 31, \
                    30, 29, 28, 27, 26, 25, 24, 23, 22, 21, \
                    20, 19, 18, 17, 16, 15, 14, 13, 12, 11, \
                    10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, ~)

/**
 * @brief Macro for concatenating multiple arguments.
 *
 * Support up to 8 arguments.
 *
 * @param[in] ... Arguments to concatenate.
 */
#define NRFX_CONCAT(...) \
    NRFX_CONCAT_2(_NRFX_CONCAT_, NRFX_NUM_VA_ARGS_LESS_1(__VA_ARGS__))(__VA_ARGS__)

/**
 * @brief Macro for checking if argument starts with opening round bracket and contains matching
 *        closing bracket (parenthesis).
 *
 * @param[in] x Input argument.
 *
 * @retval 1 If input argument starts with opening bracket and contains closing bracket.
 * @retval 0 If input argument does not match above mentioned condition.
 */
#define NRFX_ARG_HAS_PARENTHESIS(x) _NRFX_GET_ARG3(_NRFX_EVAL(_NRFX_ARG_HAS_PARENTHESIS x, 1, 0))

/**
 * @brief Macro for calling a macro @p F on each provided argument with a given
 *        separator between each call.
 *
 * Example:
 *
 *     #define F(x) int a##x
 *     NRFX_FOR_EACH(F, (;), 4, 5, 6);
 *
 * This expands to:
 *
 *     int a4;
 *     int a5;
 *     int a6;
 *
 * @param F Macro to invoke
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses;
 *            this is required to enable providing a comma as a separator.
 * @param ... Variable argument list. The macro @p F is invoked as
 *            <tt>F(element)</tt> for each element in the list.
 */
#define NRFX_FOR_EACH(F, sep, ...) \
    _NRFX_FOR_EACH(F, sep, NRFX_REVERSE_ARGS(__VA_ARGS__))

/**
 * @brief Call macro @p F on each provided argument, with the argument's index
 *        as an additional parameter.
 *
 * This is like @ref NRFX_FOR_EACH(), except @p F should be a macro which takes two
 * arguments: <tt>F(index, variable_arg)</tt>.
 *
 * Example:
 *
 *     #define F(idx, x) int a##idx = x
 *     NRFX_FOR_EACH_IDX(F, (;), 4, 5, 6);
 *
 * This expands to:
 *
 *     int a0 = 4;
 *     int a1 = 5;
 *     int a2 = 6;
 *
 * @param F Macro to invoke
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses;
 *            this is required to enable providing a comma as a separator.
 * @param ... Variable argument list. The macro @p F is invoked as
 *            <tt>F(index, element)</tt> for each element in the list.
 */
#define NRFX_FOR_EACH_IDX(F, sep, ...) \
    _NRFX_FOR_EACH_IDX(F, sep, NRFX_REVERSE_ARGS(__VA_ARGS__))

/**
 * @brief Macro for calling macro @p F on each provided argument, with an additional fixed
 *        argument as a parameter.
 *
 * This is like @ref NRFX_FOR_EACH(), except @p F should be a macro which takes two
 * arguments: <tt>F(variable_arg, fixed_arg)</tt>.
 *
 * Example:
 *
 *     static void func(int val, void *dev);
 *     NRFX_FOR_EACH_FIXED_ARG(func, (;), dev, 4, 5, 6);
 *
 * This expands to:
 *
 *     func(4, dev);
 *     func(5, dev);
 *     func(6, dev);
 *
 * @param F Macro to invoke
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses;
 *            this is required to enable providing a comma as a separator.
 * @param fixed_arg Fixed argument passed to @p F as the second macro parameter.
 * @param ... Variable argument list. The macro @p F is invoked as
 *            <tt>F(element, fixed_arg)</tt> for each element in the list.
 */
#define NRFX_FOR_EACH_FIXED_ARG(F, sep, fixed_arg, ...) \
    _NRFX_FOR_EACH_FIXED_ARG(F, sep, fixed_arg, NRFX_REVERSE_ARGS(__VA_ARGS__))

/**
 * @brief Macro from calling macro @p F for each variable argument with an index and fixed
 *        argument
 *
 * This is like the combination of @ref NRFX_FOR_EACH_IDX() with @ref NRFX_FOR_EACH_FIXED_ARG().
 *
 * Example:
 *
 *     #define F(idx, x, fixed_arg) int fixed_arg##idx = x
 *     NRFX_FOR_EACH_IDX_FIXED_ARG(F, (;), a, 4, 5, 6);
 *
 * This expands to:
 *
 *     int a0 = 4;
 *     int a1 = 5;
 *     int a2 = 6;
 *
 * @param F Macro to invoke
 * @param sep Separator (e.g. comma or semicolon). Must be in parentheses;
 *            This is required to enable providing a comma as a separator.
 * @param fixed_arg Fixed argument passed to @p F as the third macro parameter.
 * @param ... Variable list of arguments. The macro @p F is invoked as
 *            <tt>F(index, element, fixed_arg)</tt> for each element in
 *            the list.
 */
#define NRFX_FOR_EACH_IDX_FIXED_ARG(F, sep, fixed_arg, ...) \
    _NRFX_FOR_EACH_IDX_FIXED_ARG(F, sep, fixed_arg, NRFX_REVERSE_ARGS(__VA_ARGS__))

/**
 * @brief Macro for reversing arguments order.
 *
 * @param ... Variable argument list.
 *
 * @return Input arguments in reversed order.
 */
#define NRFX_REVERSE_ARGS(...) \
    _NRFX_FOR_EACH_ENGINE(_NRFX_FOR_EACH_EXEC, (,), NRFX_EVAL, _, __VA_ARGS__)


/**
 * @brief Macro for getting the highest value from input arguments.
 *
 * It is similar to @ref NRFX_MAX but accepts a variable number of arguments.
 *
 * @note Input arguments must be numeric variables.
 *
 * @param ... Variable argument list.
 *
 * @return Highest value from the input list.
 */
#define NRFX_MAX_N(...) \
    NRFX_EVAL(NRFX_FOR_EACH(_NRFX_MAX_P1, (), __VA_ARGS__) 0 \
              NRFX_FOR_EACH(_NRFX_MAX_P2, (), __VA_ARGS__))

/**
 * @brief Macro for getting a position of a bit in a bit mask with only one bit set.
 *
 * Macro shall be used only on fixed value so that it can be resolved at compile time.
 *
 * @param _bitmask 32 bit mask with one bit set.
 *
 * @return Index of a bit.
 */
#define NRFX_BITMASK_TO_BITPOS(_bitmask) \
    (_bitmask == NRFX_BIT(0) ? 0 : \
     _bitmask == NRFX_BIT(1) ? 1 : \
     _bitmask == NRFX_BIT(2) ? 2 : \
     _bitmask == NRFX_BIT(3) ? 3 : \
     _bitmask == NRFX_BIT(4) ? 4 : \
     _bitmask == NRFX_BIT(5) ? 5 : \
     _bitmask == NRFX_BIT(6) ? 6 : \
     _bitmask == NRFX_BIT(7) ? 7 : \
     _bitmask == NRFX_BIT(8) ? 8 : \
     _bitmask == NRFX_BIT(9) ? 9 : \
     _bitmask == NRFX_BIT(10) ? 10 : \
     _bitmask == NRFX_BIT(11) ? 11 : \
     _bitmask == NRFX_BIT(12) ? 12 : \
     _bitmask == NRFX_BIT(13) ? 13 : \
     _bitmask == NRFX_BIT(14) ? 14 : \
     _bitmask == NRFX_BIT(15) ? 15 : \
     _bitmask == NRFX_BIT(16) ? 16 : \
     _bitmask == NRFX_BIT(17) ? 17 : \
     _bitmask == NRFX_BIT(18) ? 18 : \
     _bitmask == NRFX_BIT(19) ? 19 : \
     _bitmask == NRFX_BIT(20) ? 20 : \
     _bitmask == NRFX_BIT(21) ? 21 : \
     _bitmask == NRFX_BIT(22) ? 22 : \
     _bitmask == NRFX_BIT(23) ? 23 : \
     _bitmask == NRFX_BIT(24) ? 24 : \
     _bitmask == NRFX_BIT(25) ? 25 : \
     _bitmask == NRFX_BIT(26) ? 26 : \
     _bitmask == NRFX_BIT(27) ? 27 : \
     _bitmask == NRFX_BIT(28) ? 28 : \
     _bitmask == NRFX_BIT(29) ? 29 : \
     _bitmask == NRFX_BIT(30) ? 30 : 31)

/** @} */

#endif /* NRFX_UTILS_H__ */
