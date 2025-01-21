/*

Copyright (c) 2009-2025 ARM Limited. All rights reserved.

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

/* NOTE: Template files (including this one) are application specific and therefore expected to 
   be copied into the application project folder prior to its use! */

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_peripherals.h"
#include "nrf91_erratas.h"
#include "system_nrf91.h"
#include "system_nrf91_approtect.h"

/*lint ++flb "Enter library region" */

void SystemStoreFICRNS(void);
void SystemLockFICRNS(void);

#define __SYSTEM_CLOCK_DEFAULT      (64000000UL)     /*!< nRF91 Application core uses a fixed System Clock Frequency of 64MHz */

#define TRACE_PIN_CNF_VALUE (   (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) | \
                                (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | \
                                (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | \
                                (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | \
                                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) )
#define TRACE_TRACECLK_PIN   (21ul)
#define TRACE_TRACEDATA0_PIN (22ul)
#define TRACE_TRACEDATA1_PIN (23ul)
#define TRACE_TRACEDATA2_PIN (24ul)
#define TRACE_TRACEDATA3_PIN (25ul)

#if defined ( __CC_ARM )
    uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_DEFAULT;  
#elif defined ( __ICCARM__ )
    __root uint32_t SystemCoreClock = __SYSTEM_CLOCK_DEFAULT;
#elif defined ( __GNUC__ )
    uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_DEFAULT;
#endif

/* Errata are only handled in secure mode since they usually need access to FICR. */
#if !defined(NRF_TRUSTZONE_NONSECURE)
    #if !defined(NRF_SKIP_UICR_HFXO_WORKAROUND)
        static bool uicr_HFXOSRC_erased(void);
        static bool uicr_HFXOCNT_erased(void);
    #endif
    static bool is_empty_word(uint32_t const volatile * word);
#endif

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = __SYSTEM_CLOCK_DEFAULT;
}

void SystemInit(void)
{
    #if !defined(NRF_TRUSTZONE_NONSECURE)
        /* Perform Secure-mode initialization routines. */

        /* Set all ARM SAU regions to NonSecure if TrustZone extensions are enabled.
        * Nordic SPU should handle Secure Attribution tasks */
        #if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
          SAU->CTRL |= (1ul << SAU_CTRL_ALLNS_Pos);
        #endif
        
        #if NRF91_ERRATA_6_ENABLE_WORKAROUND
            /* Workaround for Errata 6 "POWER: SLEEPENTER and SLEEPEXIT events asserted after pin reset" found at the Errata document
                for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf91_errata_6()){
                NRF_POWER_S->EVENTS_SLEEPENTER = (POWER_EVENTS_SLEEPENTER_EVENTS_SLEEPENTER_NotGenerated << POWER_EVENTS_SLEEPENTER_EVENTS_SLEEPENTER_Pos);
                NRF_POWER_S->EVENTS_SLEEPEXIT = (POWER_EVENTS_SLEEPEXIT_EVENTS_SLEEPEXIT_NotGenerated << POWER_EVENTS_SLEEPEXIT_EVENTS_SLEEPEXIT_Pos);
            }
        #endif
        
        #if NRF91_ERRATA_14_ENABLE_WORKAROUND
            /* Workaround for Errata 14 "REGULATORS: LDO mode at startup" found at the Errata document
                for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf91_errata_14()){
                *((volatile uint32_t *)0x50004A38ul) = 0x01ul;
                NRF_REGULATORS_S->DCDCEN = REGULATORS_DCDCEN_DCDCEN_Enabled << REGULATORS_DCDCEN_DCDCEN_Pos;
            }
        #endif

        #if NRF91_ERRATA_15_ENABLE_WORKAROUND
            /* Workaround for Errata 15 "REGULATORS: LDO mode at startup" found at the Errata document
                for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf91_errata_15()){
                NRF_REGULATORS_S->DCDCEN = REGULATORS_DCDCEN_DCDCEN_Enabled << REGULATORS_DCDCEN_DCDCEN_Pos;
            }
        #endif

        #if NRF91_ERRATA_20_ENABLE_WORKAROUND
            /* Workaround for Errata 20 "RAM content cannot be trusted upon waking up from System ON Idle or System OFF mode" found at the Errata document
                for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf91_errata_20()){
                *((volatile uint32_t *)0x5003AEE4ul) = 0xEul;
            }
        #endif

        #if NRF91_ERRATA_31_ENABLE_WORKAROUND
            /* Workaround for Errata 31 "XOSC32k Startup Failure" found at the Errata document
                for your device located at https://infocenter.nordicsemi.com/index.jsp  */
            if (nrf91_errata_31()){
                *((volatile uint32_t *)0x5000470Cul) = 0x0ul;
                *((volatile uint32_t *)0x50004710ul) = 0x1ul;
            }
        #endif

        #if !defined(NRF_SKIP_FICR_NS_COPY_TO_RAM)
            SystemStoreFICRNS();
        #endif

        /* Trimming of the device. Copy all the trimming values from FICR into the target addresses. Trim
         until one ADDR is not initialized. */
            
        for (uint32_t index = 0ul; index < 256ul && !is_empty_word(&NRF_FICR_S->TRIMCNF[index].ADDR); index++){
          #if defined ( __ICCARM__ )
              #pragma diag_suppress=Pa082
          #endif
          *(volatile uint32_t *)NRF_FICR_S->TRIMCNF[index].ADDR = NRF_FICR_S->TRIMCNF[index].DATA;
          #if defined ( __ICCARM__ )
              #pragma diag_default=Pa082
          #endif
        }

        #if !defined(NRF_SKIP_UICR_HFXO_WORKAROUND)
            bool irq_disabled = __get_PRIMASK() == 1; 
            uint32_t uicr_erased_value;
            uint32_t uicr_new_value;
            /* Set UICR->HFXOSRC and UICR->HFXOCNT to working defaults if UICR was erased */
            if (uicr_HFXOSRC_erased() || uicr_HFXOCNT_erased()) {
                __DSB();
                /* Wait for pending NVMC operations to finish */
                while (NRF_NVMC_S->READY != NVMC_READY_READY_Ready);

                /* Enable write mode in NVMC */
                NRF_NVMC_S->CONFIG = NVMC_CONFIG_WEN_Wen;
                while (NRF_NVMC_S->READY != NVMC_READY_READY_Ready);

                if (!irq_disabled && nrf91_errata_7()){
                    __disable_irq();
                }

                if (uicr_HFXOSRC_erased()){
                    /* Write default value to UICR->HFXOSRC */
                    uicr_erased_value = NRF_UICR_S->HFXOSRC;
                    uicr_new_value = (uicr_erased_value & ~UICR_HFXOSRC_HFXOSRC_Msk) | UICR_HFXOSRC_HFXOSRC_TCXO;
                    NRF_UICR_S->HFXOSRC = uicr_new_value;
                    __DSB();
                    while (NRF_NVMC_S->READY != NVMC_READY_READY_Ready);
                }

                if (uicr_HFXOCNT_erased()){
                    /* Write default value to UICR->HFXOCNT */
                    uicr_erased_value = NRF_UICR_S->HFXOCNT;
                    uicr_new_value = (uicr_erased_value & ~UICR_HFXOCNT_HFXOCNT_Msk) | 0x20;
                    NRF_UICR_S->HFXOCNT = uicr_new_value;
                    __DSB();
                    while (NRF_NVMC_S->READY != NVMC_READY_READY_Ready);
                }
                
                if(!irq_disabled && nrf91_errata_7()){
                    __enable_irq();
                }

                /* Enable read mode in NVMC */
                NRF_NVMC_S->CONFIG = NVMC_CONFIG_WEN_Ren;
                while (NRF_NVMC_S->READY != NVMC_READY_READY_Ready);

                /* Reset to apply clock select update */
                NVIC_SystemReset();
            }
        #endif

        /* Enable Trace functionality. If ENABLE_TRACE is not defined, TRACE pins will be used as GPIOs (see Product
           Specification to see which ones). */
        #if defined (ENABLE_TRACE)
            // Enable Trace And Debug peripheral
            NRF_TAD_S->ENABLE = TAD_ENABLE_ENABLE_Msk;
            NRF_TAD_S->TASKS_CLOCKSTART = TAD_TASKS_CLOCKSTART_TASKS_CLOCKSTART_Msk;

            // Set up Trace pads SPU firewall
            NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACECLK_PIN);
            NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA0_PIN);
            NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA1_PIN);
            NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA2_PIN);
            NRF_SPU_S->GPIOPORT[0].PERM &= ~(1ul << TRACE_TRACEDATA3_PIN);

            // Configure trace port pads
            NRF_P0_S->PIN_CNF[TRACE_TRACECLK_PIN] =   TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA0_PIN] = TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA1_PIN] = TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA2_PIN] = TRACE_PIN_CNF_VALUE;
            NRF_P0_S->PIN_CNF[TRACE_TRACEDATA3_PIN] = TRACE_PIN_CNF_VALUE;

            // Select trace pins
            NRF_TAD_S->PSEL.TRACECLK   = TRACE_TRACECLK_PIN;
            NRF_TAD_S->PSEL.TRACEDATA0 = TRACE_TRACEDATA0_PIN;
            NRF_TAD_S->PSEL.TRACEDATA1 = TRACE_TRACEDATA1_PIN;
            NRF_TAD_S->PSEL.TRACEDATA2 = TRACE_TRACEDATA2_PIN;
            NRF_TAD_S->PSEL.TRACEDATA3 = TRACE_TRACEDATA3_PIN;

            // Set trace port speed to 32 MHz
            NRF_TAD_S->TRACEPORTSPEED = TAD_TRACEPORTSPEED_TRACEPORTSPEED_32MHz;

            *((volatile uint32_t *)(0xE0053000ul)) = 0x00000001ul;

            *((volatile uint32_t *)(0xE005AFB0ul))  = 0xC5ACCE55ul;
            *((volatile uint32_t *)(0xE005A000ul)) &= 0xFFFFFF00ul;
            *((volatile uint32_t *)(0xE005A004ul))  = 0x00000009ul;
            *((volatile uint32_t *)(0xE005A000ul))  = 0x00000303ul;
            *((volatile uint32_t *)(0xE005AFB0ul))  = 0x00000000ul;

            *((volatile uint32_t *)(0xE005BFB0ul))  = 0xC5ACCE55ul;
            *((volatile uint32_t *)(0xE005B000ul)) &= 0xFFFFFF00ul;
            *((volatile uint32_t *)(0xE005B004ul))  = 0x00003000ul;
            *((volatile uint32_t *)(0xE005B000ul))  = 0x00000308ul;
            *((volatile uint32_t *)(0xE005BFB0ul))  = 0x00000000ul;

            *((volatile uint32_t *)(0xE0058FB0ul)) = 0xC5ACCE55ul;
            *((volatile uint32_t *)(0xE0058000ul)) = 0x00000000ul;
            *((volatile uint32_t *)(0xE0058004ul)) = 0x00000000ul;
            *((volatile uint32_t *)(0xE0058FB0ul)) = 0x00000000ul;

            /* Rom table does not list ETB, or TPIU base addresses.
             * Some debug probes may require manual configuration of these peripherals to enable tracing.
             * ETB_BASE = 0xE0051000
             * TPIU_BASE = 0xE0054000
             */
        #endif

        /* Allow Non-Secure code to run FPU instructions. 
         * If only the secure code should control FPU power state these registers should be configured accordingly in the secure application code. */
        SCB->NSACR |= (3UL << 10ul);

        nrf91_handle_approtect();
    #endif
    
    /* Enable the FPU if the compiler used floating point unit instructions. __FPU_USED is a MACRO defined by the
    * compiler. Since the FPU consumes energy, remember to disable FPU use in the compiler if floating point unit
    * operations are not used in your code. */
    #if (__FPU_USED == 1)
        SCB->CPACR |= (3UL << 20ul) | (3UL << 22ul);
        __DSB();
        __ISB();
    #endif
}


#if !defined(NRF_TRUSTZONE_NONSECURE)

    #if !defined(NRF_SKIP_UICR_HFXO_WORKAROUND)
        bool uicr_HFXOCNT_erased()
        {
            bool irq_disabled = __get_PRIMASK() == 1; 
            if (!irq_disabled && nrf91_errata_7()){
                __disable_irq();
            }
            bool is_empty = is_empty_word(&NRF_UICR_S->HFXOCNT); 
            if (!irq_disabled && nrf91_errata_7()){
                __enable_irq();
            }
            return is_empty;
        }
        
        
        bool uicr_HFXOSRC_erased()
        {
            bool irq_disabled = __get_PRIMASK() == 1; 
            if (!irq_disabled && nrf91_errata_7()){
                __disable_irq();
            }
            uint32_t HFXOSRC_readout = NRF_UICR_S->HFXOSRC;
            __DSB();
            bool erased = (HFXOSRC_readout & UICR_HFXOSRC_HFXOSRC_Msk) != UICR_HFXOSRC_HFXOSRC_TCXO;
            if (!irq_disabled && nrf91_errata_7()){
                __enable_irq();
            }
            return erased;
        }
    #endif
    
    bool is_empty_word(uint32_t const volatile * word)
    {
        uint32_t val = *word;
        __DSB();
        return val == 0xFFFFFFFFul;
    }
#endif


/* Workaround to allow NS code to access FICR. Override NRF_FICR_NS to move FICR_NS buffer. */
#define FICR_SIZE 0x1000ul
#define RAM_BASE 0x20000000ul
#define RAM_END  0x2FFFFFFFul

/* Copy FICR_S to FICR_NS RAM region */
void SystemStoreFICRNS()
{
    if ((uint32_t)NRF_FICR_NS < RAM_BASE || (uint32_t)NRF_FICR_NS + FICR_SIZE > RAM_END)
    {
        /* FICR_NS is not in RAM. */
        return;
    }
    /* Copy FICR to NS-accessible RAM block. */
    volatile uint32_t * from            = (volatile uint32_t *)((uint32_t)NRF_FICR_S + (FICR_SIZE - sizeof(uint32_t)));
    volatile uint32_t * to              = (volatile uint32_t *)((uint32_t)NRF_FICR_NS + (FICR_SIZE - sizeof(uint32_t)));
    volatile uint32_t * copy_from_end   = (volatile uint32_t *)NRF_FICR_S;
    while (from >= copy_from_end)
    {
        *(to--) = *(from--);
    }

    /* Make RAM region NS. */
    uint32_t ram_region = ((uint32_t)NRF_FICR_NS - (uint32_t)RAM_BASE) / SPU_RAMREGION_SIZE;
    __DSB();
    NRF_SPU_S->RAMREGION[ram_region].PERM &= ~(1ul << SPU_RAMREGION_PERM_SECATTR_Pos);
}

/* Block write and execute access to FICR RAM region */
void SystemLockFICRNS()
{
    if ((uint32_t)NRF_FICR_NS < RAM_BASE || (uint32_t)NRF_FICR_NS + FICR_SIZE > RAM_END)
    {
        /* FICR_NS is not in RAM. */
        return;
    }

    uint32_t ram_region = ((uint32_t)NRF_FICR_NS - (uint32_t)RAM_BASE) / SPU_RAMREGION_SIZE;
    __DSB();
    NRF_SPU_S->RAMREGION[ram_region].PERM &=
        ~(
            (1ul << SPU_RAMREGION_PERM_WRITE_Pos) |
            (1ul << SPU_RAMREGION_PERM_EXECUTE_Pos)
        );
    NRF_SPU_S->RAMREGION[ram_region].PERM |= 1ul << SPU_RAMREGION_PERM_LOCK_Pos;
}

/*lint --flb "Leave library region" */
