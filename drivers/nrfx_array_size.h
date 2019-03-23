/*

The MIT License (MIT)

Copyright (c)

All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/
#ifndef NRFX_ARRAY_SIZE_H__
#define NRFX_ARRAY_SIZE_H__

#ifndef __has_feature
    #define __has_feature(x) 0 // Compatibility with non-clang compilers.
#endif

#if __cplusplus >= 199711L
    //#pragma message "using Ivan J. Johnson's ARRAY_SIZE2"

    // Works on older compilers, even Visual C++ 6....
    // Created by Ivan J. Johnson, March 06, 2007
    // See http://drdobbs.com/cpp/197800525?pgno=1
    //
    // Pseudocode:
    // if x is not an array
    //   issue a compile-time error
    // else
    //   use the traditional (non-typesafe) C99 COUNTOF expression
    //
    // If the argument is any of:
    //    object of class type, such as an std::vector
    //    floating-point type
    //    function pointer
    //    pointer-to-member
    // then the first reinterpret_cast<> is not legal (compiler error)
    //
    // The type for check1 is chosen and named to help understand
    // the cause of the error, because the class name is likely to
    // appear in the compiler error message.
    //
    // If check1 succeeds, then the argument must be one of:
    //    an integral type
    //    an enumerated type
    //    a pointer to an object
    //    an array
    //
    // Check2 expands approximately to sizeof(check_type(x, &x)),
    // where check_type is an overloaded function.
    // Because this is purely a compile-time computation,
    // the function is never really called or even implemented,
    // but it lets the compiler apply overload resolution,
    // which allows further type discrimination.
    // There are three possibilities to consider:
    //    x is an integral type or enumerated type.
    //      In this case, neither of the two function overloads
    //      is a match, resulting in a compiler error.
    //    x is a pointer to an object.
    //      In this case, the first argument to check_type()
    //      is a pointer and the second one is a pointer-to-pointer.
    //      The best function match is the first overload of check_type,
    //      the one that returns an incomplete type (Is_pointer).
    //      However, because Is_pointer is an incomplete type,
    //      sizeof(Is_pointer) is not a valid expression,
    //      resulting in a compiler error.
    //    x is an array.
    //      In this case, the first argument to check_type()
    //      is an array and the second is a pointer-to-array.
    //      A pointer-to-array is *NOT* convertible to a
    //      pointer-to-pointer, so the first overload of
    //      check_type() is not a match.
    //      However, an array IS convertible to a pointer,
    //      and a pointer-to-array already is a pointer.
    //      Any pointer is convertible to a void*,
    //      so the second function overload is a match.
    //      That overload returns a complete type (Is_array).
    //      Because it's a complete type,
    //      sizeof(Is_array) is a valid expression.
    // Thus, the compiler has EXCLUDED every possible type
    // except arrays via compilation errors before reaching
    // the third line.
    // Moreover, check1 and check2 are reduced to the value zero,
    // while the third line is the old type-unsafe C-style macro,
    // now made entirely type-safe.
    // 
    // Additional benefits:
    // The result is itself constexpr
    // 
    //
    
    /**
     * @brief Macro for getting the number of elements in an array.
     *
     * @param array  Name of the array.
     *
     * @return Array element count.
     */
    #define NRFX_ARRAY_SIZE(array) ( \
       0 * sizeof(reinterpret_cast<const ::Bad_arg_to_COUNTOF*>(array)) + /*check1*/ \
       0 * sizeof(::Bad_arg_to_COUNTOF::check_type((array), &(array)))  + /*check2*/ \
       sizeof(array) / sizeof((array)[0])                                 /* eval */ \
       )

    struct Bad_arg_to_COUNTOF {
       class Is_pointer; // incomplete
       class Is_array {};
       template <typename T>
       static Is_pointer check_type(const T*, const T* const*);
       static Is_array check_type(const void*, const void*);
    };

#elif __cplusplus >= 201103L ||  /* any compiler claiming C++11 support */ \
    _MSC_VER >= 1900 ||          /* Visual C++ 2015 or higher           */ \
    __has_feature(cxx_constexpr) /* CLang versions supporting constexp  */

    //#pragma message "C++11 version ARRAY_SIZE2"

    namespace detail
    {
        template <typename T, std::size_t N>
        constexpr std::size_t countof(T const (&)[N]) noexcept
        {
            return N;
        }
    } // namespace detail
    /**
     * @brief Macro for getting the number of elements in an array.
     *
     * @param array  Name of the array.
     *
     * @return Array element count.
     */
    #define NRFX_ARRAY_SIZE(array) detail::countof(array)

#elif _MSC_VER // Visual C++ fallback

    //#pragma message "using Microsoft Visual C++ intrinsic ARRAY_SIZE2"
    /**
     * @brief Macro for getting the number of elements in an array.
     *
     * @param array  Name of the array.
     *
     * @return Array element count.
     */
    #define NRFX_ARRAY_SIZE(array) _countof(array)

#elif __cplusplus >= 199711L && ( /* C++ 98 trick */ \
    defined(__INTEL_COMPILER) ||                     \
    defined(__clang__) ||                            \
    (defined(__GNUC__) && (                          \
        (__GNUC__ > 4) ||                            \
        (__GNUC__ == 4 && __GNUC_MINOR__ >= 4)       \
    )))

    //#pragma message "C++98 version ARRAY_SIZE2"

    template <typename T, std::size_t N>
    char(&_ArraySizeHelperRequiresArray(T(&)[N]))[N];
    /**
     * @brief Macro for getting the number of elements in an array.
     *
     * @param array  Name of the array.
     *
     * @return Array element count.
     */
    #define NRFX_ARRAY_SIZE(array) sizeof(_ArraySizeHelperRequiresArray(array))

#else

    //#pragma message "Using type-unsafe version of ARRAY_SIZE2"
    // This is the worst-case scenario macro.
    // While it is valid C, it is NOT typesafe.
    // For example, if the parameter array is a pointer instead of array,
    // the compiler will SILENTLY give a (likely) incorrect result. 
    /**
     * @brief Macro for getting the number of elements in an array.
     *
     * @param array  Name of the array.
     *
     * @return Array element count.
     */
    #define NRFX_ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#endif


#endif // NRFX_ARRAY_SIZE_H__

