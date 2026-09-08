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

#ifndef NRF_VECTORS_H_
#define NRF_VECTORS_H_

#if defined(NRF51422_XXAA)
    #include "nrf51/nrf51422/nrf51422_vectors.h"
#elif defined(NRF51422_XXAB)
    #include "nrf51/nrf51422/nrf51422_vectors.h"
#elif defined(NRF51422_XXAC)
    #include "nrf51/nrf51422/nrf51422_vectors.h"
#elif defined(NRF51801_XXAB)
    #include "nrf51/nrf51801/nrf51801_vectors.h"
#elif defined(NRF51802_XXAA)
    #include "nrf51/nrf51802/nrf51802_vectors.h"
#elif defined(NRF51822_XXAA)
    #include "nrf51/nrf51822/nrf51822_vectors.h"
#elif defined(NRF51822_XXAB)
    #include "nrf51/nrf51822/nrf51822_vectors.h"
#elif defined(NRF51822_XXAC)
    #include "nrf51/nrf51822/nrf51822_vectors.h"
#elif defined(NRF51824_XXAA)
    #include "nrf51/nrf51824/nrf51824_vectors.h"
#elif defined(NRF52805_XXAA)
    #include "nrf52/nrf52805/nrf52805_vectors.h"
#elif defined(NRF52810_XXAA)
    #include "nrf52/nrf52810/nrf52810_vectors.h"
#elif defined(NRF52811_XXAA)
    #include "nrf52/nrf52811/nrf52811_vectors.h"
#elif defined(NRF52820_XXAA)
    #include "nrf52/nrf52820/nrf52820_vectors.h"
#elif defined(NRF52832_XXAA)
    #include "nrf52/nrf52832/nrf52832_vectors.h"
#elif defined(NRF52832_XXAB)
    #include "nrf52/nrf52832/nrf52832_vectors.h"
#elif defined(NRF52833_XXAA)
    #include "nrf52/nrf52833/nrf52833_vectors.h"
#elif defined(NRF52840_XXAA)
    #include "nrf52/nrf52840/nrf52840_vectors.h"
#elif defined(NRF5340_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf53/nrf5340/nrf5340_application_vectors.h"
    #endif
    #if defined(NRF_NETWORK)
        #include "nrf53/nrf5340/nrf5340_network_vectors.h"
    #endif
#elif defined(NRF54H20_XXAA)
    #if defined(NRF_SECURE)
        #include "nrf54h/nrf54h20/nrf54h20_secure_vectors.h"
    #endif
    #if defined(NRF_APPLICATION)
        #include "nrf54h/nrf54h20/nrf54h20_application_vectors.h"
    #endif
    #if defined(NRF_RADIOCORE)
        #include "nrf54h/nrf54h20/nrf54h20_radiocore_vectors.h"
    #endif
    #if defined(NRF_SYSCTRL)
        #include "nrf54h/nrf54h20/nrf54h20_sysctrl_vectors.h"
    #endif
    #if defined(NRF_PPR)
        #include "nrf54h/nrf54h20/nrf54h20_ppr_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54h/nrf54h20/nrf54h20_flpr_vectors.h"
    #endif
#elif defined(NRF54L05_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l/nrf54l05/nrf54l05_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l/nrf54l05/nrf54l05_flpr_vectors.h"
    #endif
#elif defined(NRF54L10_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l/nrf54l10/nrf54l10_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l/nrf54l10/nrf54l10_flpr_vectors.h"
    #endif
#elif defined(NRF54L15_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l/nrf54l15/nrf54l15_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l/nrf54l15/nrf54l15_flpr_vectors.h"
    #endif
#elif defined(NRF54LC10A_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l/nrf54lc10a/nrf54lc10a_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l/nrf54lc10a/nrf54lc10a_flpr_vectors.h"
    #endif
#elif defined(NRF54LM20A_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l/nrf54lm20a/nrf54lm20a_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l/nrf54lm20a/nrf54lm20a_flpr_vectors.h"
    #endif
#elif defined(NRF54LM20B_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l/nrf54lm20b/nrf54lm20b_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l/nrf54lm20b/nrf54lm20b_flpr_vectors.h"
    #endif
#elif defined(NRF54LS05A_XXAA)
    #include "nrf54l/nrf54ls05a/nrf54ls05a_application_vectors.h"
#elif defined(NRF54LS05B_XXAA)
    #include "nrf54l/nrf54ls05b/nrf54ls05b_application_vectors.h"
#elif defined(NRF54LV10A_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l/nrf54lv10a/nrf54lv10a_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l/nrf54lv10a/nrf54lv10a_flpr_vectors.h"
    #endif
#elif defined(NRF7120_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf71/nrf7120/nrf7120_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf71/nrf7120/nrf7120_flpr_vectors.h"
    #endif
    #if defined(NRF_LMAC)
        #include "nrf71/nrf7120/nrf7120_lmac_vectors.h"
    #endif
    #if defined(NRF_UMAC)
        #include "nrf71/nrf7120/nrf7120_umac_vectors.h"
    #endif
#elif defined(NRF7120_ENGA_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf71/nrf7120_enga/nrf7120_enga_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf71/nrf7120_enga/nrf7120_enga_flpr_vectors.h"
    #endif
    #if defined(NRF_LMAC)
        #include "nrf71/nrf7120_enga/nrf7120_enga_lmac_vectors.h"
    #endif
    #if defined(NRF_UMAC)
        #include "nrf71/nrf7120_enga/nrf7120_enga_umac_vectors.h"
    #endif
#elif defined(NRF7120E_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf71/nrf7120e/nrf7120e_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf71/nrf7120e/nrf7120e_flpr_vectors.h"
    #endif
    #if defined(NRF_LMAC)
        #include "nrf71/nrf7120e/nrf7120e_lmac_vectors.h"
    #endif
    #if defined(NRF_UMAC)
        #include "nrf71/nrf7120e/nrf7120e_umac_vectors.h"
    #endif
#elif defined(NRF7120E_ENGA_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf71/nrf7120e_enga/nrf7120e_enga_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf71/nrf7120e_enga/nrf7120e_enga_flpr_vectors.h"
    #endif
    #if defined(NRF_LMAC)
        #include "nrf71/nrf7120e_enga/nrf7120e_enga_lmac_vectors.h"
    #endif
    #if defined(NRF_UMAC)
        #include "nrf71/nrf7120e_enga/nrf7120e_enga_umac_vectors.h"
    #endif
#elif defined(NRF9120_XXAA)
    #include "nrf91/nrf9120/nrf9120_vectors.h"
#elif defined(NRF9160_XXAA)
    #include "nrf91/nrf9160/nrf9160_vectors.h"
#elif defined(NRF9220_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf92/nrf9220/nrf9220_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf92/nrf9220/nrf9220_flpr_vectors.h"
    #endif
    #if defined(NRF_PPR)
        #include "nrf92/nrf9220/nrf9220_ppr_vectors.h"
    #endif
#elif defined(NRF9230_ENGB_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf92/nrf9230_engb/nrf9230_engb_application_vectors.h"
    #endif
    #if defined(NRF_RADIOCORE)
        #include "nrf92/nrf9230_engb/nrf9230_engb_radiocore_vectors.h"
    #endif
    #if defined(NRF_PPR)
        #include "nrf92/nrf9230_engb/nrf9230_engb_ppr_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf92/nrf9230_engb/nrf9230_engb_flpr_vectors.h"
    #endif
#else
    #error "Device must be defined. See nrf_vectors.h."
#endif

#endif
