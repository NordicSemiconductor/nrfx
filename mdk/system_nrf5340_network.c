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
#include "nrf53_erratas.h"
#include "system_nrf53.h"
#include "system_nrf53_approtect.h"

/*lint ++flb "Enter library region" */


#define __SYSTEM_CLOCK_DEFAULT      (64000000UL)     /*!< NRF5340 network core uses a fixed System Clock Frequency of 64MHz */

#if defined ( __CC_ARM ) || defined ( __GNUC__ )
    uint32_t SystemCoreClock __attribute__((used)) = __SYSTEM_CLOCK_DEFAULT;
#elif defined ( __ICCARM__ )
    __root uint32_t SystemCoreClock = __SYSTEM_CLOCK_DEFAULT;
#endif

void SystemCoreClockUpdate(void)
{
    SystemCoreClock = __SYSTEM_CLOCK_DEFAULT;
}

void SystemInit(void)
{
    /* Trimming of the device. Copy all the trimming values from FICR into the target addresses. Trim
     until one ADDR is not initialized. */
    uint32_t index = 0;
    for (index = 0; index < 32ul && NRF_FICR_NS->TRIMCNF[index].ADDR != 0xFFFFFFFFul; index++){
        #if defined ( __ICCARM__ )
            /* IAR will complain about the order of volatile pointer accesses. */
            #pragma diag_suppress=Pa082
        #endif
        *((volatile uint32_t *)NRF_FICR_NS->TRIMCNF[index].ADDR) = NRF_FICR_NS->TRIMCNF[index].DATA;
        #if defined ( __ICCARM__ )
            #pragma diag_default=Pa082
        #endif
    }

    #if NRF53_ERRATA_49_ENABLE_WORKAROUND
        /* Workaround for Errata 49 "SLEEPENTER and SLEEPEXIT events asserted after pin reset" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
        if (nrf53_errata_49())
        {
            if (NRF_RESET_NS->RESETREAS & RESET_RESETREAS_RESETPIN_Msk)
            {
                NRF_POWER_NS->EVENTS_SLEEPENTER = 0ul;
                NRF_POWER_NS->EVENTS_SLEEPEXIT = 0ul;
            }
        }
    #endif

    #if NRF53_ERRATA_55_ENABLE_WORKAROUND
        /* Workaround for Errata 55 "Bits in RESETREAS are set when they should not be" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/index.jsp  */
        if (nrf53_errata_55())
        {
            if (NRF_RESET_NS->RESETREAS & RESET_RESETREAS_RESETPIN_Msk){
                NRF_RESET_NS->RESETREAS = ~RESET_RESETREAS_RESETPIN_Msk;
            }
        }
    #endif

    #if NRF53_ERRATA_160_ENABLE_WORKAROUND
        if (nrf53_errata_160())
        {
            *((volatile uint32_t *)0x41002118ul) = 0x7Ful;
            *((volatile uint32_t *)0x41080E04ul) = 0x0ul;
            *((volatile uint32_t *)0x41080E08ul) = 0x0ul;
            *((volatile uint32_t *)0x41002124ul) = 0x0ul;
            *((volatile uint32_t *)0x4100212Cul) = 0x0ul;
            *((volatile uint32_t *)0x41101110ul) = 0x0ul;
        }
    #endif

    /* Handle fw-branch APPROTECT setup. */
    nrf53_handle_approtect();
}

/*lint --flb "Leave library region" */
