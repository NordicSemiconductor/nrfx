/*
Copyright (c) 2010 - 2024, Nordic Semiconductor ASA All rights reserved.

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
    #include "nrf51422_vectors.h"
#elif defined(NRF51422_XXAB)
    #include "nrf51422_vectors.h"
#elif defined(NRF51422_XXAC)
    #include "nrf51422_vectors.h"
#elif defined(NRF51801_XXAB)
    #include "nrf51801_vectors.h"
#elif defined(NRF51802_XXAA)
    #include "nrf51802_vectors.h"
#elif defined(NRF51822_XXAA)
    #include "nrf51822_vectors.h"
#elif defined(NRF51822_XXAB)
    #include "nrf51822_vectors.h"
#elif defined(NRF51822_XXAC)
    #include "nrf51822_vectors.h"
#elif defined(NRF51824_XXAA)
    #include "nrf51824_vectors.h"
#elif defined(NRF52805_XXAA)
    #include "nrf52805_vectors.h"
#elif defined(NRF52810_XXAA)
    #include "nrf52810_vectors.h"
#elif defined(NRF52811_XXAA)
    #include "nrf52811_vectors.h"
#elif defined(NRF52820_XXAA)
    #include "nrf52820_vectors.h"
#elif defined(NRF52832_XXAA)
    #include "nrf52832_vectors.h"
#elif defined(NRF52832_XXAB)
    #include "nrf52832_vectors.h"
#elif defined(NRF52833_XXAA)
    #include "nrf52833_vectors.h"
#elif defined(NRF52840_XXAA)
    #include "nrf52840_vectors.h"
#elif defined(NRF5340_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf5340_application_vectors.h"
    #endif
    #if defined(NRF_NETWORK)
        #include "nrf5340_network_vectors.h"
    #endif
#elif defined(NRF9120_XXAA)
    #include "nrf9120_vectors.h"
#elif defined(NRF9160_XXAA)
    #include "nrf9160_vectors.h"
#elif defined(NRF54H20_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54h20_application_vectors.h"
    #endif
    #if defined(NRF_RADIOCORE)
        #include "nrf54h20_radiocore_vectors.h"
    #endif
    #if defined(NRF_PPR)
        #include "nrf54h20_ppr_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54h20_flpr_vectors.h"
    #endif
#elif defined(NRF54H20_ENGA_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54h20_enga_application_vectors.h"
    #endif
    #if defined(NRF_RADIOCORE)
        #include "nrf54h20_enga_radiocore_vectors.h"
    #endif
    #if defined(NRF_PPR)
        #include "nrf54h20_enga_ppr_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54h20_enga_flpr_vectors.h"
    #endif
#elif defined(NRF54L15_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l15_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l15_flpr_vectors.h"
    #endif
#elif defined(NRF54L15_ENGA_XXAA)
    #if defined(NRF_APPLICATION)
        #include "nrf54l15_enga_application_vectors.h"
    #endif
    #if defined(NRF_FLPR)
        #include "nrf54l15_enga_flpr_vectors.h"
    #endif
#else
    #error "Device must be defined. See nrf_vectors.h."
#endif

#endif
