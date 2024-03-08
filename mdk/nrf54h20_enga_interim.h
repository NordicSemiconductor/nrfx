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

#ifndef NRF54H20_ENGA_INTERIM_H__
#define NRF54H20_ENGA_INTERIM_H__

#include "haltium_interim.h"

#if defined(NRF54H20_ENGA_XXAA)

    #if defined(NRF_TRUSTZONE_NONSECURE)
        #if defined(NRF_APPLICATION)
            #define GRTC_IRQ_GROUP 2
            #define GPIOTE_IRQ_GROUP 2
        #elif defined(NRF_RADIOCORE)
            #define GRTC_IRQ_GROUP 4
            #define GPIOTE_IRQ_GROUP 4
        #else
            #error Unknown core.
        #endif
    #elif defined(NRF_PPR) || defined(NRF_FLPR)
        #define GRTC_IRQ_GROUP 2
        #define GPIOTE_IRQ_GROUP 2
    #else
        #if defined(NRF_APPLICATION)
            #define GRTC_IRQ_GROUP 3
            #define GPIOTE_IRQ_GROUP 3
        #elif defined(NRF_RADIOCORE)
            #define GRTC_IRQ_GROUP 5
            #define GPIOTE_IRQ_GROUP 5
        #else
            #error Unknown core.
        #endif
    #endif

    #define EASYVDMA_PRESENT
    #define VDMADESCRIPTOR_CONFIG_CNT_Pos (0UL)        /*!< Position of CNT field.                                               */
    #define VDMADESCRIPTOR_CONFIG_CNT_Msk (0xFFFFFFUL << VDMADESCRIPTOR_CONFIG_CNT_Pos) /*!< Bit mask of CNT field.              */
    #define VDMADESCRIPTOR_CONFIG_ATTRIBUTE_Pos (24UL) /*!< Position of ATTRIBUTE field.                                         */

    #define SPIS120_EASYDMA_MAXCNT_SIZE 16
    #define SPIS130_EASYDMA_MAXCNT_SIZE 16
    #define SPIS131_EASYDMA_MAXCNT_SIZE 16
    #define SPIS132_EASYDMA_MAXCNT_SIZE 16
    #define SPIS133_EASYDMA_MAXCNT_SIZE 16
    #define SPIS134_EASYDMA_MAXCNT_SIZE 16
    #define SPIS135_EASYDMA_MAXCNT_SIZE 16
    #define SPIS136_EASYDMA_MAXCNT_SIZE 16
    #define SPIS137_EASYDMA_MAXCNT_SIZE 16

    #define TWIM130_EASYDMA_MAXCNT_SIZE 16
    #define TWIM131_EASYDMA_MAXCNT_SIZE 16
    #define TWIM132_EASYDMA_MAXCNT_SIZE 16
    #define TWIM133_EASYDMA_MAXCNT_SIZE 16
    #define TWIM134_EASYDMA_MAXCNT_SIZE 16
    #define TWIM135_EASYDMA_MAXCNT_SIZE 16
    #define TWIM136_EASYDMA_MAXCNT_SIZE 16
    #define TWIM137_EASYDMA_MAXCNT_SIZE 16
            
    #define TWIS130_EASYDMA_MAXCNT_SIZE 16
    #define TWIS131_EASYDMA_MAXCNT_SIZE 16
    #define TWIS132_EASYDMA_MAXCNT_SIZE 16
    #define TWIS133_EASYDMA_MAXCNT_SIZE 16
    #define TWIS134_EASYDMA_MAXCNT_SIZE 16
    #define TWIS135_EASYDMA_MAXCNT_SIZE 16
    #define TWIS136_EASYDMA_MAXCNT_SIZE 16
    #define TWIS137_EASYDMA_MAXCNT_SIZE 16

    #define SPIM120_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM120_MAX_DATARATE 32                      /*!< (unspecified)                                                        */
    #define SPIM120_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM120_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM120_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM120_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM120_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM120_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM120_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM120_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM120_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM120_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM120_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM120_CORE_FREQUENCY 320                   /*!< Peripheral core frequency is 320 MHz.                                */
    #define SPIM120_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM120_PRESCALER_DIVISOR_RANGE_MIN 4        /*!< (unspecified)                                                        */
    #define SPIM120_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM120_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM120_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM120_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM120_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM120_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM120_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM120_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM120_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM121_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM121_MAX_DATARATE 32                      /*!< (unspecified)                                                        */
    #define SPIM121_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM121_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM121_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM121_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM121_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM121_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM121_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM121_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM121_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM121_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM121_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM121_CORE_FREQUENCY 320                   /*!< Peripheral core frequency is 320 MHz.                                */
    #define SPIM121_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM121_PRESCALER_DIVISOR_RANGE_MIN 4        /*!< (unspecified)                                                        */
    #define SPIM121_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM121_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM121_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM121_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM121_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM121_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM121_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM121_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM121_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM130_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM130_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM130_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM130_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM130_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM130_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM130_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM130_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM130_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM130_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM130_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM130_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM130_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM130_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM130_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM130_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM130_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM130_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM130_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM130_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM130_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM130_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM130_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM130_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM130_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM131_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM131_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM131_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM131_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM131_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM131_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM131_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM131_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM131_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM131_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM131_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM131_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM131_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM131_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM131_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM131_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM131_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM131_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM131_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM131_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM131_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM131_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM131_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM131_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM131_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM132_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM132_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM132_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM132_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM132_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM132_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM132_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM132_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM132_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM132_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM132_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM132_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM132_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM132_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM132_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM132_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM132_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM132_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM132_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM132_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM132_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM132_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM132_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM132_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM132_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM133_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM133_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM133_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM133_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM133_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM133_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM133_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM133_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM133_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM133_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM133_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM133_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM133_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM133_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM133_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM133_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM133_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM133_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM133_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM133_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM133_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM133_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM133_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM133_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM133_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM134_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM134_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM134_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM134_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM134_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM134_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM134_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM134_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM134_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM134_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM134_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM134_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM134_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM134_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM134_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM134_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM134_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM134_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM134_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM134_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM134_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM134_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM134_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM134_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM134_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM135_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM135_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM135_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM135_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM135_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM135_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM135_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM135_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM135_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM135_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM135_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM135_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM135_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM135_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM135_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM135_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM135_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM135_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM135_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM135_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM135_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM135_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM135_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM135_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM135_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM136_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM136_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM136_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM136_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM136_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM136_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM136_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM136_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM136_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM136_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM136_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM136_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM136_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM136_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM136_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM136_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM136_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM136_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM136_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM136_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM136_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM136_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM136_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM136_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM136_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */

    #define SPIM137_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */
    #define SPIM137_MAX_DATARATE 8                       /*!< (unspecified)                                                        */
    #define SPIM137_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM137_EASYDMA_MAXCNT_MAX 14                /*!< (unspecified)                                                        */
    #define SPIM137_EASYDMA_MAXCNT_SIZE 15               /*!< (unspecified)                                                        */
    #define SPIM137_FEATURE_HARDWARE_CSN_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM137_FEATURE_HARDWARE_DCX_PRESENT 1       /*!< (unspecified)                                                        */
    #define SPIM137_FEATURE_RXDELAY_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM137_STALL_STATUS_PRESENT 1               /*!< (unspecified)                                                        */
    #define SPIM137_STALL_STATUS_TX_PRESENT 1            /*!< (unspecified)                                                        */
    #define SPIM137_NUM_CHIPSELECT_MIN 0                 /*!< (unspecified)                                                        */
    #define SPIM137_NUM_CHIPSELECT_MAX 0                 /*!< (unspecified)                                                        */
    #define SPIM137_NUM_CHIPSELECT_SIZE 1                /*!< (unspecified)                                                        */
    #define SPIM137_CORE_FREQUENCY 16                    /*!< Peripheral core frequency is 16 MHz.                                 */
    #define SPIM137_PRESCALER_PRESENT 1                  /*!< (unspecified)                                                        */
    #define SPIM137_PRESCALER_DIVISOR_RANGE_MIN 2        /*!< (unspecified)                                                        */
    #define SPIM137_PRESCALER_DIVISOR_RANGE_MAX 126      /*!< (unspecified)                                                        */
    #define SPIM137_PRESCALER_DIVISOR_RANGE_SIZE 127     /*!< (unspecified)                                                        */
    #define SPIM137_RXDELAY_VALUE_RANGE_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM137_RXDELAY_VALUE_RANGE_MAX 7            /*!< (unspecified)                                                        */
    #define SPIM137_RXDELAY_VALUE_RANGE_SIZE 8           /*!< (unspecified)                                                        */
    #define SPIM137_RXDELAY_RESET_VALUE 2                /*!< (unspecified)                                                        */
    #define SPIM137_RXDELAY_FIELD_WIDTH_MIN 0            /*!< (unspecified)                                                        */
    #define SPIM137_RXDELAY_FIELD_WIDTH_MAX 2            /*!< (unspecified)                                                        */
    #define SPIM137_RXDELAY_FIELD_WIDTH_SIZE 3           /*!< (unspecified)                                                        */


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

    #define UARTE120_EASYDMA_MAXCNT_SIZE UARTE120_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE130_EASYDMA_MAXCNT_SIZE UARTE130_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE131_EASYDMA_MAXCNT_SIZE UARTE131_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE132_EASYDMA_MAXCNT_SIZE UARTE132_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE133_EASYDMA_MAXCNT_SIZE UARTE133_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE134_EASYDMA_MAXCNT_SIZE UARTE134_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE135_EASYDMA_MAXCNT_SIZE UARTE135_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE136_EASYDMA_MAXCNT_SIZE UARTE136_EASYDMA_MAXCNT_SIZE_SIZE
    #define UARTE137_EASYDMA_MAXCNT_SIZE UARTE137_EASYDMA_MAXCNT_SIZE_SIZE

    #define I2S130_EASYDMA_MAXCNT_SIZE I2S130_EASYDMA_MAXCNT_SIZE_SIZE
    #define I2S131_EASYDMA_MAXCNT_SIZE I2S131_EASYDMA_MAXCNT_SIZE_SIZE

    #define P0_PIN_NUM P0_PIN_NUM_SIZE
    #define P1_PIN_NUM P1_PIN_NUM_SIZE
    #define P2_PIN_NUM P2_PIN_NUM_SIZE
    #define P6_PIN_NUM P6_PIN_NUM_SIZE
    #define P7_PIN_NUM P7_PIN_NUM_SIZE
    #define P9_PIN_NUM P9_PIN_NUM_SIZE

    #define EGU020_CH_NUM EGU020_CH_NUM_SIZE

#endif

#endif // NRF54H20_ENGA_INTERIM_H__
