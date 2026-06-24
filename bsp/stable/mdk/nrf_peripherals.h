/*

Copyright (c) 2010 - 2026, Nordic Semiconductor ASA All rights reserved.

SPDX-License-Identifier: BSD-3-Clause

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. Neither the name of Nordic Semiconductor ASA nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef NRF_PERIPHERALS_H__
#define NRF_PERIPHERALS_H__

#ifdef __ZEPHYR__
#include <mdk_config.h>
#endif

/*lint ++flb "Enter library region */

/* Device selection for peripheral includes. */
#if defined(NRF51)
    #include "nrf51/nrf51_peripherals.h"

#elif defined (NRF52805_XXAA)
    #include "nrf52/nrf52805/nrf52805_peripherals.h"
#elif defined(NRF52810_XXAA)
    #include "nrf52/nrf52810/nrf52810_peripherals.h"
#elif defined(NRF52811_XXAA)
    #include "nrf52/nrf52811/nrf52811_peripherals.h"
#elif defined(NRF52820_XXAA)
    #include "nrf52/nrf52820/nrf52820_peripherals.h"
#elif defined(NRF52832_XXAA) || defined(NRF52832_XXAB)
    #include "nrf52/nrf52832/nrf52832_peripherals.h"
#elif defined (NRF52833_XXAA)
    #include "nrf52/nrf52833/nrf52833_peripherals.h"
#elif defined(NRF52840_XXAA)
    #include "nrf52/nrf52840/nrf52840_peripherals.h"

#elif defined (NRF5340_XXAA_APPLICATION)
    #include "nrf53/nrf5340/nrf5340_application_peripherals.h"
#elif defined (NRF5340_XXAA_NETWORK)
    #include "nrf53/nrf5340/nrf5340_network_peripherals.h"

#elif defined (NRF54H20_XXAA)
    #include "nrf54h/nrf54h20/nrf54h20_peripherals.h"

#elif defined (NRF54L05_XXAA)
    #include "nrf54l/nrf54l05/nrf54l05_peripherals.h"

#elif defined (NRF54LV10A_XXAA)
    #include "nrf54l/nrf54lv10a/nrf54lv10a_peripherals.h"

#elif defined (NRF54L10_XXAA)
    #include "nrf54l/nrf54l10/nrf54l10_peripherals.h"

#elif defined (NRF54L15_XXAA)
    #include "nrf54l/nrf54l15/nrf54l15_peripherals.h"

#elif defined (NRF54LM20A_XXAA)
    #include "nrf54l/nrf54lm20a/nrf54lm20a_peripherals.h"

#elif defined (NRF54LM20B_XXAA)
    #include "nrf54l/nrf54lm20b/nrf54lm20b_peripherals.h"

#elif defined (NRF54LS05B_XXAA)
    #include "nrf54l/nrf54ls05b/nrf54ls05b_peripherals.h"

#elif defined (NRF7120_ENGA_XXAA)
    #include "nrf71/nrf7120_enga/nrf7120_enga_peripherals.h"

#elif defined(NRF9120_XXAA)
    #include "nrf91/nrf9120/nrf9120_peripherals.h"
#elif defined(NRF9160_XXAA)
    #include "nrf91/nrf9160/nrf9160_peripherals.h"

#elif defined (NRF9230_ENGB_XXAA)
    #include "nrf92/nrf9230_engb/nrf9230_engb_peripherals.h"

#elif defined (NRF54LC10A_XXAA)
    #include "nrf54l/nrf54lc10a/nrf54lc10a_peripherals.h"

#elif defined (NRF54LS05A_XXAA)
    #include "nrf54l/nrf54ls05a/nrf54ls05a_peripherals.h"

#elif defined (NRF9220_XXAA)
    #include "nrf92/nrf9220/nrf9220_peripherals.h"

/* Ending device selection for peripheral includes. */
#else
    #error "Device must be defined. See nrf_peripherals.h."
#endif

/*lint --flb "Leave library region" */

#endif // NRF_PERIPHERALS_H__
