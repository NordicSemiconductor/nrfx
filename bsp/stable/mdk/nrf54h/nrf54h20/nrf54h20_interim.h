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

#ifndef NRF54H20_INTERIM_H__
#define NRF54H20_INTERIM_H__

#if defined(NRF54H20_XXAA)

    #define NRF_DOMAIN_COUNT    NRF_DOMAIN_GLOBAL + 1
    #define NRF_PROCESSOR_COUNT NRF_PROCESSOR_FLPR + 1


    #define ADDRESS_REGION_Pos        (29UL)
    #define ADDRESS_REGION_Msk        (0x7UL << ADDRESS_REGION_Pos)
    #define ADDRESS_SECURITY_Pos      (28UL)
    #define ADDRESS_SECURITY_Msk      (0x1UL << ADDRESS_SECURITY_Pos)
    #define ADDRESS_DOMAIN_Pos        (24UL)
    #define ADDRESS_DOMAIN_Msk        (0xFUL << ADDRESS_DOMAIN_Pos)
    #define ADDRESS_BUS_Pos           (16UL)
    #define ADDRESS_BUS_Msk           (0xFFUL << ADDRESS_BUS_Pos)
    #define ADDRESS_SLAVE_Pos         (12UL)
    #define ADDRESS_SLAVE_Msk         (0xFUL << ADDRESS_SLAVE_Pos)
    #define ADDRESS_PERIPHID_Pos      (12UL)
    #define ADDRESS_PERIPHID_Msk      (0x7FFUL << ADDRESS_PERIPHID_Pos)

    typedef enum
    {
        NRF_REGION_PROGRAM      = 0,
        NRF_REGION_DATA         = 1,
        NRF_REGION_PERIPHERALS  = 2,
        NRF_REGION_EXTMEM       = 3,
        NRF_REGION_EXTMEM_ENC   = 4,
        NRF_REGION_STM          = 5,
        NRF_REGION_CPU_INTERNAL = 7,
    } nrf_region_t;

    #define GPIOTE_INT_COUNT 7

    #define GPIOTE_CH_NUM   8
    #define GPIOTE130_CH_NUM (GPIOTE130_GPIOTE_NCHANNELS_MAX + 1UL)
    #define GPIOTE130_AVAILABLE_GPIO_PORTS 0x207UL
    #define GPIOTE_PORT_NUM GPIOTE_EVENTS_PORT_MaxCount
    #define GPIOTE_FEATURE_SET_PRESENT
    #define GPIOTE_FEATURE_CLR_PRESENT

    #define VPR_CLIC_PRIO_COUNT 4
    #define VPR_VEVIF_EVENT_MaxCount 32

    #define GLOBAL_IRQN_START (96)
    #define GLOBAL_IRQN_MAX   (480)

    #define SPU000_PERIPH_COUNT 16
    #define SPU010_PERIPH_COUNT 16
    #define SPU020_PERIPH_COUNT 16
    #define SPU030_PERIPH_COUNT 16

    #define SAADC_CH_NUM 8
    #define SAADC_EASYDMA_MAXCNT_SIZE 15

    #define LPCOMP_REFSEL_RESOLUTION 16

    #if !defined(DPPI_PRESENT)
        #define DPPI_PRESENT
    #endif
    #define DPPI_GROUP_NUM 2
    #if defined(NRF_RADIOCORE)
        #define DPPI020_CH_NUM 16
        #define DPPI020_GROUP_NUM 0
        #define DPPI030_CH_NUM 9
        #define DPPI030_GROUP_NUM 1
    #endif

    #define GLOBAL_IPCT_CH_NUM 8
    #if defined(NRF_RADIOCORE)
        #define LOCAL_IPCT_NUM 8
    #elif defined(NRF_APPLICATION)
        #define LOCAL_IPCT_NUM 4
    #endif

    #if defined(NRF_PPR) || defined(NRF_FLPR)
        #define MVDMA_JOBLISTCOUNT 4
    #else
        #define MVDMA_JOBLISTCOUNT 1
    #endif

    #define MVDMA_AXI_BUS_WIDTH 8

    #define MPC_MASTER_PORTS_MaxCount (32UL)

    #if defined(NRF_TRUSTZONE_NONSECURE)
        #if defined(NRF_APPLICATION)
            #define GRTC_IRQ_GROUP 2
            #define GPIOTE_IRQ_GROUP 2
        #elif defined(NRF_RADIOCORE)
            #define GRTC_IRQ_GROUP 4
            #define GPIOTE_IRQ_GROUP 4
        #endif
    #elif defined(NRF_PPR)
        #define GRTC_IRQ_GROUP 8
        #define GPIOTE_IRQ_GROUP 2
    #elif defined(NRF_FLPR)
        #define GRTC_IRQ_GROUP 9
        #define GPIOTE_IRQ_GROUP 2
    #else
        #if defined(NRF_APPLICATION)
            #define GRTC_IRQ_GROUP 3
            #define GPIOTE_IRQ_GROUP 3
        #elif defined(NRF_RADIOCORE)
            #define GRTC_IRQ_GROUP 5
            #define GPIOTE_IRQ_GROUP 5
        #endif
    #endif

    #define P0_PIN_NUM P0_PIN_NUM_SIZE
    #define P1_PIN_NUM P1_PIN_NUM_SIZE
    #define P2_PIN_NUM P2_PIN_NUM_SIZE
    #define P6_PIN_NUM P6_PIN_NUM_SIZE
    #define P7_PIN_NUM P7_PIN_NUM_SIZE
    #define P9_PIN_NUM P9_PIN_NUM_SIZE

    #define DPPI_CH_NUM 8

    #undef ETM_TRCRSCTLR_MaxCount

    #define ETM_TRCRSCTLR_MaxCount (32UL)                          /*!< Max size of TRCRSCTLR[32] array.            */

    #define EASYVDMA_PRESENT

    #define RTC_CC_NUM    RTC_CC_NUM_SIZE
    #define RTC130_CC_NUM RTC130_CC_NUM_SIZE
    #define RTC131_CC_NUM RTC131_CC_NUM_SIZE

    #define TIMER020_MAX_SIZE TIMER020_MAX_SIZE_SIZE
    #define TIMER021_MAX_SIZE TIMER021_MAX_SIZE_SIZE
    #define TIMER022_MAX_SIZE TIMER022_MAX_SIZE_SIZE
    #define TIMER120_MAX_SIZE TIMER120_MAX_SIZE_SIZE
    #define TIMER121_MAX_SIZE TIMER121_MAX_SIZE_SIZE
    #define TIMER130_MAX_SIZE TIMER130_MAX_SIZE_SIZE
    #define TIMER131_MAX_SIZE TIMER131_MAX_SIZE_SIZE
    #define TIMER132_MAX_SIZE TIMER132_MAX_SIZE_SIZE
    #define TIMER133_MAX_SIZE TIMER133_MAX_SIZE_SIZE
    #define TIMER134_MAX_SIZE TIMER134_MAX_SIZE_SIZE
    #define TIMER135_MAX_SIZE TIMER135_MAX_SIZE_SIZE
    #define TIMER136_MAX_SIZE TIMER136_MAX_SIZE_SIZE
    #define TIMER137_MAX_SIZE TIMER137_MAX_SIZE_SIZE

    #define TIMER020_CC_NUM TIMER020_CC_NUM_SIZE
    #define TIMER021_CC_NUM TIMER021_CC_NUM_SIZE
    #define TIMER022_CC_NUM TIMER022_CC_NUM_SIZE
    #define TIMER120_CC_NUM TIMER120_CC_NUM_SIZE
    #define TIMER121_CC_NUM TIMER121_CC_NUM_SIZE
    #define TIMER130_CC_NUM TIMER130_CC_NUM_SIZE
    #define TIMER131_CC_NUM TIMER131_CC_NUM_SIZE
    #define TIMER132_CC_NUM TIMER132_CC_NUM_SIZE
    #define TIMER133_CC_NUM TIMER133_CC_NUM_SIZE
    #define TIMER134_CC_NUM TIMER134_CC_NUM_SIZE
    #define TIMER135_CC_NUM TIMER135_CC_NUM_SIZE
    #define TIMER136_CC_NUM TIMER136_CC_NUM_SIZE
    #define TIMER137_CC_NUM TIMER137_CC_NUM_SIZE

    #define DPPIC020_CH_NUM DPPIC020_CH_NUM_SIZE
    #define DPPIC030_CH_NUM DPPIC030_CH_NUM_SIZE
    #define DPPIC120_CH_NUM DPPIC120_CH_NUM_SIZE
    #define DPPIC130_CH_NUM DPPIC130_CH_NUM_SIZE
    #define DPPIC131_CH_NUM DPPIC131_CH_NUM_SIZE
    #define DPPIC132_CH_NUM DPPIC132_CH_NUM_SIZE
    #define DPPIC133_CH_NUM DPPIC133_CH_NUM_SIZE
    #define DPPIC134_CH_NUM DPPIC134_CH_NUM_SIZE
    #define DPPIC135_CH_NUM DPPIC135_CH_NUM_SIZE
    #define DPPIC136_CH_NUM DPPIC136_CH_NUM_SIZE

    #define DPPIC020_GROUP_NUM DPPIC020_GROUP_NUM_SIZE
    #define DPPIC030_GROUP_NUM DPPIC030_GROUP_NUM_SIZE
    #define DPPIC120_GROUP_NUM DPPIC120_GROUP_NUM_SIZE
    #define DPPIC130_GROUP_NUM DPPIC130_GROUP_NUM_SIZE
    #define DPPIC131_GROUP_NUM DPPIC131_GROUP_NUM_SIZE
    #define DPPIC132_GROUP_NUM DPPIC132_GROUP_NUM_SIZE
    #define DPPIC133_GROUP_NUM DPPIC133_GROUP_NUM_SIZE
    #define DPPIC134_GROUP_NUM DPPIC134_GROUP_NUM_SIZE
    #define DPPIC135_GROUP_NUM DPPIC135_GROUP_NUM_SIZE
    #define DPPIC136_GROUP_NUM DPPIC136_GROUP_NUM_SIZE

    #define EGU020_CH_NUM EGU020_CH_NUM_SIZE
    #define EGU130_CH_NUM EGU130_CH_NUM_SIZE


#endif

#endif // NRF54H20_INTERIM_H__
