/*
Copyright (c) 2009-2024 ARM Limited. All rights reserved.

    SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the License); you may
not use this file except in compliance with the License.
You may obtain a copy of the License at

    www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an AS IS BASIS, WITHOUT
WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

NOTICE: This file has been modified by Nordic Semiconductor ASA.
 */

#include <stdint.h>

#include <nrf_mem.h>
#include <compiler_abstraction.h>

/*---------------------------------------------------------------------------
  Stack and heap definitions
 *---------------------------------------------------------------------------*/

#ifndef NRF_SKIP_STACK_DECLARATION
    extern uint8_t __STACK[] __STACK_ATTRIBUTES(__STACK_ALIGNMENT);
    uint8_t __STACK[__STACK_SIZE] __STACK_ATTRIBUTES(__STACK_ALIGNMENT);
    #define __STACK_BASE  ((uint32_t)(__STACK + __STACK_SIZE))
    #define __STACK_LIMIT ((uint32_t)(__STACK))
#endif

#if !defined(NRF_SKIP_HEAP_DECLARATION) && __HEAP_SIZE > 0
    extern uint8_t __HEAP[] __HEAP_ATTRIBUTES(__HEAP_ALIGNMENT);
    uint8_t __HEAP[__HEAP_SIZE] __HEAP_ATTRIBUTES(__HEAP_ALIGNMENT);
    #define __HEAP_LIMIT ((uint32_t)(__HEAP + __HEAP_SIZE))
    #define __HEAP_BASE  ((uint32_t)(__HEAP))
#endif

/*---------------------------------------------------------------------------
  Define __PROGRAM_START to avoid conflict with CMSIS
 *---------------------------------------------------------------------------*/
#define __PROGRAM_START

/*---------------------------------------------------------------------------
  Interrupt vector tables
 *---------------------------------------------------------------------------*/

#include <nrf.h>

__VECTOR_TABLE_ATTRIBUTE extern const VECTOR_TABLE_Type __VECTOR_TABLE[];

#include <nrf_vectors.h>

/*---------------------------------------------------------------------------
  Memory initializer structs
 *---------------------------------------------------------------------------*/

#if ! defined (__ARMCC_VERSION) && (defined( __GNUC__ ) ||  defined( __clang__ ))

#define DO_GNU_MEM_INIT
/** Source: https://github.com/ARM-software/CMSIS_5/blob/develop/CMSIS/Core/Include/cmsis_gcc.h */
typedef struct __copy_table {
    uint32_t const * src;
    uint32_t * dest;
    uint32_t wlen;
} __copy_table_t;

typedef struct __zero_table {
    uint32_t * dest;
    uint32_t wlen;
} __zero_table_t;

__STATIC_FORCEINLINE void copy_region(const __copy_table_t * table)
{
    for (uint32_t i = 0; i < table->wlen; ++i)
    {
        table->dest[i] = table->src[i];
    }
}

__STATIC_FORCEINLINE void zero_region(const __zero_table_t * table)
{
    for (uint32_t i = 0; i < table->wlen; ++i)
    {
        table->dest[i] = 0;
    }
}

__STATIC_FORCEINLINE void GNUInitializeMemories()
{
    /* Perform C memory initialization */
    #ifndef NRF_SKIP_VARIABLE_INIT
        
        extern const __copy_table_t __copy_table_start__;
        extern const __copy_table_t __copy_table_end__;

        for (__copy_table_t const* pTable = &__copy_table_start__; pTable < &__copy_table_end__; ++pTable) {
            copy_region(pTable);
        }
    #endif

    #ifndef NRF_SKIP_ZERO_INIT
        extern const __zero_table_t __zero_table_start__;
        extern const __zero_table_t __zero_table_end__;
        for (__zero_table_t const* pTable = &__zero_table_start__; pTable < &__zero_table_end__; ++pTable) {
            zero_region(pTable);
        }
    #endif

    #ifdef NRF_VECTORS_IN_RAM
        /* Copy vector table from code to data region */
        extern const uint32_t __vectors_load_start;
        extern uint32_t __vectors_start;
        extern uint32_t __vectors_end;

        const __copy_table_t vector_copy = {
            &__vectors_load_start,
            &__vectors_start,
            (&__vectors_end - &__vectors_start) / 4
        };

        copy_region(&vector_copy);
        
        SCB->VTOR = &__vectors_start;
    #endif
}
#endif

/*---------------------------------------------------------------------------
  Implement newlib heap monitor
 *---------------------------------------------------------------------------*/
#ifdef NRF_ENABLE_HEAP_LIMIT

    #include <stddef.h>
    #include <errno.h>

    void * _sbrk (ptrdiff_t incr)
    {
        extern char   __heap_base;  /* Defined by the linker.  */
        extern char   __heap_limit; /* Defined by the linker.  */
        static char * heap_top = NULL;
        
        if (heap_top == NULL)
            heap_top = & __heap_base;

        char * prev_heap_top = heap_top;

        if ((heap_top + incr > (char *)GET_SP()) || (heap_top + incr > &__heap_limit))
        {
            errno = ENOMEM;
            return (void *) -1;
        }

        heap_top += incr;

        return (void *) prev_heap_top;
    }
    
#endif
#if defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)

    /* ArmClang stack / heap setup method. */
    void __attribute__((naked)) __user_setup_stackheap()
    {
        __asm(
        "    ldr     r0, = %0\n" /* __STACK_BASE */
        "    mov     sp, r0\n"
        #if __HEAP_SIZE != 0
            "    ldr     r0, = %1 \n" /* __HEAP_BASE */
            "    ldr     r2, = %2 \n" /* __HEAP_LIMIT */
        #else
            "    mov     r0, #0\n"
            "    mov     r2, #0\n"
        #endif
        "    bx      lr\n"
            :: "i"(__STACK_BASE), "i"(__HEAP_BASE),  "i"(__HEAP_LIMIT));
    }
#endif


/*---------------------------------------------------------------------------
  Reset Handler called on controller reset
 *---------------------------------------------------------------------------*/
extern __NO_RETURN void __START(void);


__RESET_HANDLER_ATTRIBUTE void Reset_Handler(void)
{
#ifdef __ARM_ARCH
    #ifndef NRF_NO_STACK_INIT
        __set_PSP((uint32_t)(__STACK_BASE));
        #if __ARM_ARCH >= 8
            __set_MSPLIM((uint32_t)(__STACK_LIMIT));
            __set_PSPLIM((uint32_t)(__STACK_LIMIT));
        #endif
    #endif
#else
	__asm__ __volatile__ (   					\
            ".option push\n"					\
            ".option norelax\n"					\
            "la gp, __global_pointer$ \n"		\
            ".option pop\n"						\
        ::: "memory");

    #ifndef NRF_NO_STACK_INIT
        __set_SP((uint32_t)(__STACK_BASE));
    #endif
    #if !defined(NRF_NO_MTVT_CONFIG) && defined(__MTVT_PRESENT) && __MTVT_PRESENT
        /* Configure machine trap vector table register */
        csr_write(CSR_MTVT, __VECTOR_TABLE);

        /* Setup machine trap vector in CSR and clear cause register (hardfault exceptions) */
        csr_write(CSR_MTVEC, Trap_Handler);
        csr_write(CSR_MCAUSE, 0);
    #endif
#endif

    NRFPreInit();

    SystemInit(); /* CMSIS System Initialization */

#ifdef DO_GNU_MEM_INIT
    GNUInitializeMemories();
#endif

    /* Configure vector table offset register */
#ifdef NRF_VTOR_CONFIG
    SCB->VTOR = NRF_VTOR_CONFIG;
#endif

    __START();
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *---------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1)
        ;
}

#ifdef INITIALIZE_USER_SECTIONS
void InitializeUserMemorySections()
{
    extern const copy_region_t __start_nrf_sections;
    copy_memory_region(&__start_nrf_sections);
}
#endif

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#pragma clang diagnostic pop
#endif
