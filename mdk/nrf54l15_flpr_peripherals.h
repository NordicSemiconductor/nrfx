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

#ifndef NRF54L15_FLPR_PERIPHERALS_H
#define NRF54L15_FLPR_PERIPHERALS_H

#ifdef __cplusplus
    extern "C" {
#endif

#include <stdbool.h>
/* Domain definition */
#define NRF_DOMAIN NRF_DOMAIN_FLPR

/*VPR CSR registers*/
#define VPRCSR_PRESENT 1
#define VPRCSR_COUNT 1

#define VPRCSR_HARTNUM 0                             /*!< HARTNUM: 0                                                           */
#define VPRCSR_MCLICBASERESET 0x00001000             /*!< MCLICBASE: 0x00001000                                                */
#define VPRCSR_MULDIV 2                              /*!< MULDIV: 2                                                            */
#define VPRCSR_HIBERNATE 1                           /*!< HIBERNATE: 1                                                         */
#define VPRCSR_DBG 1                                 /*!< DBG: 1                                                               */
#define VPRCSR_REMAP 0                               /*!< Code patching (REMAP): 0                                             */
#define VPRCSR_BUSWIDTH 64                           /*!< BUSWIDTH: 64                                                         */
#define VPRCSR_BKPT 2                                /*!< BKPT: 2                                                              */
#define VPRCSR_VIOPINS 0x00000000                    /*!< CSR VIOPINS value: 0x00000000                                        */
#define VPRCSR_VEVIF_NTASKS_MIN 0                    /*!< VEVIF tasks: 0..31                                                   */
#define VPRCSR_VEVIF_NTASKS_MAX 31                   /*!< VEVIF tasks: 0..31                                                   */
#define VPRCSR_VEVIF_NTASKS_SIZE 32                  /*!< VEVIF tasks: 0..31                                                   */
#define VPRCSR_VEVIF_TASKS_MASK 0xFFFFFFFF           /*!< Mask of supported VEVIF tasks: 0xFFFFFFFF                            */
#define VPRCSR_VEVIF_NDPPI_MIN 0                     /*!< VEVIF DPPI channels: 0..31                                           */
#define VPRCSR_VEVIF_NDPPI_MAX 31                    /*!< VEVIF DPPI channels: 0..31                                           */
#define VPRCSR_VEVIF_NDPPI_SIZE 32                   /*!< VEVIF DPPI channels: 0..31                                           */
#define VPRCSR_VEVIF_NEVENTS_MIN 0                   /*!< VEVIF events: 0..31                                                  */
#define VPRCSR_VEVIF_NEVENTS_MAX 31                  /*!< VEVIF events: 0..31                                                  */
#define VPRCSR_VEVIF_NEVENTS_SIZE 32                 /*!< VEVIF events: 0..31                                                  */
#define VPRCSR_BEXT 0                                /*!< Bit-Manipulation extension: 0                                        */
#define VPRCSR_CACHE_EN 1                            /*!< (unspecified)                                                        */
#define VPRCSR_OUTMODE_VPR1_2 1                      /*!< (unspecified)                                                        */
#define VPRCSR_VPR_BUS_PRIO 1                        /*!< (unspecified)                                                        */
#define VPRCSR_NMIMPID_VPR1_3_3 0                    /*!< (unspecified)                                                        */

/*VPR CLIC registers*/
#define CLIC_PRESENT 1
#define CLIC_COUNT 1

#define VPRCLIC_IRQ_COUNT 32
#define VPRCLIC_IRQNUM_MIN 0                         /*!< Supported interrupts (IRQNUM): 0..479                                */
#define VPRCLIC_IRQNUM_MAX 479                       /*!< Supported interrupts (IRQNUM): 0..479                                */
#define VPRCLIC_IRQNUM_SIZE 480                      /*!< Supported interrupts (IRQNUM): 0..479                                */
#define VPRCLIC_CLIC_NTASKS_MIN 0                    /*!< VEVIF tasks: 0..31                                                   */
#define VPRCLIC_CLIC_NTASKS_MAX 31                   /*!< VEVIF tasks: 0..31                                                   */
#define VPRCLIC_CLIC_NTASKS_SIZE 32                  /*!< VEVIF tasks: 0..31                                                   */
#define VPRCLIC_CLIC_TASKS_MASK 0xFFFFFFFF           /*!< Mask of supported VEVIF tasks: 0xFFFFFFFF                            */
#define VPRCLIC_COUNTER_IRQ_NUM 32                   /*!< VPR counter (CNT0) interrupt handler number (COUNTER_IRQ_NUM): 32    */
#define VPRCLIC_CLIC_VPR_1_2 1                       /*!< (unspecified)                                                        */

/*Factory Information Configuration Registers*/
#define FICR_PRESENT 1
#define FICR_COUNT 1

/*User Information Configuration Registers*/
#define UICR_PRESENT 1
#define UICR_COUNT 1

/*Factory Information Configuration Registers*/
#define SICR_PRESENT 1
#define SICR_COUNT 1

/*CRACENCORE*/
#define CRACENCORE_PRESENT 1
#define CRACENCORE_COUNT 1

#define CRACENCORE_CRYPTMSTRDMAREGS 1                /*!< (unspecified)                                                        */
#define CRACENCORE_CRYPTMSTRHWREGS 1                 /*!< (unspecified)                                                        */
#define CRACENCORE_RNGCONTROLREGS 1                  /*!< (unspecified)                                                        */
#define CRACENCORE_PKREGS 1                          /*!< (unspecified)                                                        */
#define CRACENCORE_IKGREGS 1                         /*!< (unspecified)                                                        */
#define CRACENCORE_RNGDATAREGS 1                     /*!< (unspecified)                                                        */
#define CRACENCORE_PKDATAMEMORYREGS 1                /*!< (unspecified)                                                        */
#define CRACENCORE_PKUCODEREGS 1                     /*!< (unspecified)                                                        */
#define CRACENCORE_CRACENRESETVALUES 1               /*!< (unspecified)                                                        */
#define CRACENCORE_SHA3RESETVALUES 0                 /*!< (unspecified)                                                        */
#define CRACENCORE_PKE_DATA_MEMORY 0x51808000        /*!< (unspecified)                                                        */
#define CRACENCORE_PKE_DATA_MEMORY_SIZE 17408        /*!< (unspecified)                                                        */
#define CRACENCORE_PKE_CODE_MEMORY 0x5180C000        /*!< (unspecified)                                                        */
#define CRACENCORE_PKE_CODE_MEMORY_SIZE 5120         /*!< (unspecified)                                                        */

/*System protection unit*/
#define SPU_PRESENT 1
#define SPU_COUNT 4

#define SPU00_BELLS 0                                /*!< (unspecified)                                                        */
#define SPU00_IPCT 0                                 /*!< (unspecified)                                                        */
#define SPU00_DPPI 1                                 /*!< (unspecified)                                                        */
#define SPU00_GPIOTE 0                               /*!< (unspecified)                                                        */
#define SPU00_GRTC 0                                 /*!< (unspecified)                                                        */
#define SPU00_GPIO 1                                 /*!< (unspecified)                                                        */
#define SPU00_CRACEN 1                               /*!< (unspecified)                                                        */
#define SPU00_MRAMC 0                                /*!< (unspecified)                                                        */
#define SPU00_COEXC 0                                /*!< (unspecified)                                                        */
#define SPU00_ANTSWC 0                               /*!< (unspecified)                                                        */
#define SPU00_TDD 0                                  /*!< (unspecified)                                                        */
#define SPU00_SLAVE_BITS 4                           /*!< SLAVE_BITS=4 (number of address bits required to represent the
                                                          peripheral slave index)*/

#define SPU10_BELLS 0                                /*!< (unspecified)                                                        */
#define SPU10_IPCT 0                                 /*!< (unspecified)                                                        */
#define SPU10_DPPI 1                                 /*!< (unspecified)                                                        */
#define SPU10_GPIOTE 0                               /*!< (unspecified)                                                        */
#define SPU10_GRTC 0                                 /*!< (unspecified)                                                        */
#define SPU10_GPIO 0                                 /*!< (unspecified)                                                        */
#define SPU10_CRACEN 0                               /*!< (unspecified)                                                        */
#define SPU10_MRAMC 0                                /*!< (unspecified)                                                        */
#define SPU10_COEXC 0                                /*!< (unspecified)                                                        */
#define SPU10_ANTSWC 0                               /*!< (unspecified)                                                        */
#define SPU10_TDD 0                                  /*!< (unspecified)                                                        */
#define SPU10_SLAVE_BITS 4                           /*!< SLAVE_BITS=4 (number of address bits required to represent the
                                                          peripheral slave index)*/

#define SPU20_BELLS 0                                /*!< (unspecified)                                                        */
#define SPU20_IPCT 0                                 /*!< (unspecified)                                                        */
#define SPU20_DPPI 1                                 /*!< (unspecified)                                                        */
#define SPU20_GPIOTE 1                               /*!< (unspecified)                                                        */
#define SPU20_GRTC 1                                 /*!< (unspecified)                                                        */
#define SPU20_GPIO 1                                 /*!< (unspecified)                                                        */
#define SPU20_CRACEN 0                               /*!< (unspecified)                                                        */
#define SPU20_MRAMC 0                                /*!< (unspecified)                                                        */
#define SPU20_COEXC 0                                /*!< (unspecified)                                                        */
#define SPU20_ANTSWC 0                               /*!< (unspecified)                                                        */
#define SPU20_TDD 0                                  /*!< (unspecified)                                                        */
#define SPU20_SLAVE_BITS 4                           /*!< SLAVE_BITS=4 (number of address bits required to represent the
                                                          peripheral slave index)*/

#define SPU30_BELLS 0                                /*!< (unspecified)                                                        */
#define SPU30_IPCT 0                                 /*!< (unspecified)                                                        */
#define SPU30_DPPI 1                                 /*!< (unspecified)                                                        */
#define SPU30_GPIOTE 1                               /*!< (unspecified)                                                        */
#define SPU30_GRTC 0                                 /*!< (unspecified)                                                        */
#define SPU30_GPIO 1                                 /*!< (unspecified)                                                        */
#define SPU30_CRACEN 0                               /*!< (unspecified)                                                        */
#define SPU30_MRAMC 0                                /*!< (unspecified)                                                        */
#define SPU30_COEXC 0                                /*!< (unspecified)                                                        */
#define SPU30_ANTSWC 0                               /*!< (unspecified)                                                        */
#define SPU30_TDD 0                                  /*!< (unspecified)                                                        */
#define SPU30_SLAVE_BITS 4                           /*!< SLAVE_BITS=4 (number of address bits required to represent the
                                                          peripheral slave index)*/

/*Memory Privilege Controller*/
#define MPC_PRESENT 1
#define MPC_COUNT 1

#define MPC00_EXTEND_CLOCK_REQ 0                     /*!< (unspecified)                                                        */
#define MPC00_RTCHOKE 0                              /*!< (unspecified)                                                        */
#define MPC00_OVERRIDE_GRAN 4096                     /*!< The override region granularity is 4096 bytes                        */

/*Distributed programmable peripheral interconnect controller*/
#define DPPIC_PRESENT 1
#define DPPIC_COUNT 4

#define DPPIC00_HASCHANNELGROUPS 1                   /*!< (unspecified)                                                        */
#define DPPIC00_CH_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define DPPIC00_CH_NUM_MAX 7                         /*!< (unspecified)                                                        */
#define DPPIC00_CH_NUM_SIZE 8                        /*!< (unspecified)                                                        */
#define DPPIC00_GROUP_NUM_MIN 0                      /*!< (unspecified)                                                        */
#define DPPIC00_GROUP_NUM_MAX 1                      /*!< (unspecified)                                                        */
#define DPPIC00_GROUP_NUM_SIZE 2                     /*!< (unspecified)                                                        */

#define DPPIC10_HASCHANNELGROUPS 1                   /*!< (unspecified)                                                        */
#define DPPIC10_CH_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define DPPIC10_CH_NUM_MAX 23                        /*!< (unspecified)                                                        */
#define DPPIC10_CH_NUM_SIZE 24                       /*!< (unspecified)                                                        */
#define DPPIC10_GROUP_NUM_MIN 0                      /*!< (unspecified)                                                        */
#define DPPIC10_GROUP_NUM_MAX 5                      /*!< (unspecified)                                                        */
#define DPPIC10_GROUP_NUM_SIZE 6                     /*!< (unspecified)                                                        */

#define DPPIC20_HASCHANNELGROUPS 1                   /*!< (unspecified)                                                        */
#define DPPIC20_CH_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define DPPIC20_CH_NUM_MAX 15                        /*!< (unspecified)                                                        */
#define DPPIC20_CH_NUM_SIZE 16                       /*!< (unspecified)                                                        */
#define DPPIC20_GROUP_NUM_MIN 0                      /*!< (unspecified)                                                        */
#define DPPIC20_GROUP_NUM_MAX 5                      /*!< (unspecified)                                                        */
#define DPPIC20_GROUP_NUM_SIZE 6                     /*!< (unspecified)                                                        */

#define DPPIC30_HASCHANNELGROUPS 1                   /*!< (unspecified)                                                        */
#define DPPIC30_CH_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define DPPIC30_CH_NUM_MAX 3                         /*!< (unspecified)                                                        */
#define DPPIC30_CH_NUM_SIZE 4                        /*!< (unspecified)                                                        */
#define DPPIC30_GROUP_NUM_MIN 0                      /*!< (unspecified)                                                        */
#define DPPIC30_GROUP_NUM_MAX 1                      /*!< (unspecified)                                                        */
#define DPPIC30_GROUP_NUM_SIZE 2                     /*!< (unspecified)                                                        */

/*PPIB APB registers*/
#define PPIB_PRESENT 1
#define PPIB_COUNT 8

#define PPIB00_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB00_NTASKSEVENTS_MAX 7                    /*!< (unspecified)                                                        */
#define PPIB00_NTASKSEVENTS_SIZE 8                   /*!< (unspecified)                                                        */

#define PPIB01_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB01_NTASKSEVENTS_MAX 7                    /*!< (unspecified)                                                        */
#define PPIB01_NTASKSEVENTS_SIZE 8                   /*!< (unspecified)                                                        */

#define PPIB10_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB10_NTASKSEVENTS_MAX 7                    /*!< (unspecified)                                                        */
#define PPIB10_NTASKSEVENTS_SIZE 8                   /*!< (unspecified)                                                        */

#define PPIB11_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB11_NTASKSEVENTS_MAX 15                   /*!< (unspecified)                                                        */
#define PPIB11_NTASKSEVENTS_SIZE 16                  /*!< (unspecified)                                                        */

#define PPIB20_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB20_NTASKSEVENTS_MAX 7                    /*!< (unspecified)                                                        */
#define PPIB20_NTASKSEVENTS_SIZE 8                   /*!< (unspecified)                                                        */

#define PPIB21_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB21_NTASKSEVENTS_MAX 15                   /*!< (unspecified)                                                        */
#define PPIB21_NTASKSEVENTS_SIZE 16                  /*!< (unspecified)                                                        */

#define PPIB22_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB22_NTASKSEVENTS_MAX 3                    /*!< (unspecified)                                                        */
#define PPIB22_NTASKSEVENTS_SIZE 4                   /*!< (unspecified)                                                        */

#define PPIB30_NTASKSEVENTS_MIN 0                    /*!< (unspecified)                                                        */
#define PPIB30_NTASKSEVENTS_MAX 3                    /*!< (unspecified)                                                        */
#define PPIB30_NTASKSEVENTS_SIZE 4                   /*!< (unspecified)                                                        */

/*Key management unit*/
#define KMU_PRESENT 1
#define KMU_COUNT 1

#define KMU_KEYSLOTNUM 256                           /*!< Number of keyslots                                                   */
#define KMU_KEYSLOTBITS 128                          /*!< Number of bits per keyslot                                           */

/*Accelerated Address Resolver*/
#define AAR_PRESENT 1
#define AAR_COUNT 1

#define AAR00_DMAERROR 1                             /*!< (unspecified)                                                        */

/*AES CCM Mode Encryption*/
#define CCM_PRESENT 1
#define CCM_COUNT 1

#define CCM00_AMOUNTREG 0                            /*!< (unspecified)                                                        */
#define CCM00_ONTHEFLYDECRYPTION 0                   /*!< (unspecified)                                                        */
#define CCM00_DMAERROR 1                             /*!< (unspecified)                                                        */

/*AES ECB Mode Encryption*/
#define ECB_PRESENT 1
#define ECB_COUNT 1

#define ECB00_AMOUNTREG 0                            /*!< (unspecified)                                                        */
#define ECB00_DMAERROR 1                             /*!< (unspecified)                                                        */

/*CRACEN*/
#define CRACEN_PRESENT 1
#define CRACEN_COUNT 1

#define CRACEN_CRYPTOACCELERATOR 1                   /*!< (unspecified)                                                        */
#define CRACEN_SEEDRAMLOCK 0                         /*!< (unspecified)                                                        */
#define CRACEN_SPLITKEYRAMLOCK 1                     /*!< (unspecified)                                                        */
#define CRACEN_SEEDALIGNED 1                         /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SEED 0x51810000         /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SEED_SIZE 64            /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_AES_KEY0 0x51810040     /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_AES_KEY0_SIZE 32        /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_AES_KEY1 0x51810060     /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_AES_KEY1_SIZE 32        /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY0 0x51810080     /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY0_SIZE 16        /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY1 0x51810090     /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY1_SIZE 16        /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY2 0x518100A0     /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY2_SIZE 16        /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY3 0x518100B0     /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_SM4_KEY3_SIZE 16        /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_RESERVED 0x518100C0     /*!< (unspecified)                                                        */
#define CRACEN_PROTECTED_RAM_RESERVED_SIZE 64        /*!< (unspecified)                                                        */
#define CRACEN_PKEDATA 0x51808000                    /*!< Must be read and written using aligned access, i.e. using an operation
                                                          where a word-aligned address is used for a word, or a halfword-aligned
                                                          address is used for a halfword access.*/
#define CRACEN_PKECODE 0x5180C000                    /*!< Must be read and written using aligned access, i.e. using an operation
                                                          where a word-aligned address is used for a word, or a halfword-aligned
                                                          address is used for a halfword access.*/

/*Serial Peripheral Interface Master with EasyDMA*/
#define SPIM_PRESENT 1
#define SPIM_COUNT 5

#define SPIM00_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */
#define SPIM00_MAX_DATARATE 32                       /*!< (unspecified)                                                        */
#define SPIM00_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM00_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIM00_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIM00_FEATURE_HARDWARE_CSN_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM00_FEATURE_HARDWARE_DCX_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM00_FEATURE_RXDELAY_PRESENT 1             /*!< (unspecified)                                                        */
#define SPIM00_STALL_STATUS_PRESENT 0                /*!< (unspecified)                                                        */
#define SPIM00_STALL_STATUS_TX_PRESENT 1             /*!< (unspecified)                                                        */
#define SPIM00_NUM_CHIPSELECT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM00_NUM_CHIPSELECT_MAX 0                  /*!< (unspecified)                                                        */
#define SPIM00_NUM_CHIPSELECT_SIZE 1                 /*!< (unspecified)                                                        */
#define SPIM00_CORE_FREQUENCY 128                    /*!< Peripheral core frequency is 128 MHz.                                */
#define SPIM00_PRESCALER_PRESENT 1                   /*!< (unspecified)                                                        */
#define SPIM00_PRESCALER_DIVISOR_RANGE_MIN 4         /*!< (unspecified)                                                        */
#define SPIM00_PRESCALER_DIVISOR_RANGE_MAX 126       /*!< (unspecified)                                                        */
#define SPIM00_PRESCALER_DIVISOR_RANGE_SIZE 127      /*!< (unspecified)                                                        */
#define SPIM00_RXDELAY_VALUE_RANGE_MIN 0             /*!< (unspecified)                                                        */
#define SPIM00_RXDELAY_VALUE_RANGE_MAX 7             /*!< (unspecified)                                                        */
#define SPIM00_RXDELAY_VALUE_RANGE_SIZE 8            /*!< (unspecified)                                                        */
#define SPIM00_RXDELAY_RESET_VALUE 2                 /*!< (unspecified)                                                        */
#define SPIM00_RXDELAY_FIELD_WIDTH_MIN 0             /*!< (unspecified)                                                        */
#define SPIM00_RXDELAY_FIELD_WIDTH_MAX 2             /*!< (unspecified)                                                        */
#define SPIM00_RXDELAY_FIELD_WIDTH_SIZE 3            /*!< (unspecified)                                                        */

#define SPIM20_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */
#define SPIM20_MAX_DATARATE 8                        /*!< (unspecified)                                                        */
#define SPIM20_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM20_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIM20_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIM20_FEATURE_HARDWARE_CSN_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM20_FEATURE_HARDWARE_DCX_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM20_FEATURE_RXDELAY_PRESENT 1             /*!< (unspecified)                                                        */
#define SPIM20_STALL_STATUS_PRESENT 0                /*!< (unspecified)                                                        */
#define SPIM20_STALL_STATUS_TX_PRESENT 0             /*!< (unspecified)                                                        */
#define SPIM20_NUM_CHIPSELECT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM20_NUM_CHIPSELECT_MAX 0                  /*!< (unspecified)                                                        */
#define SPIM20_NUM_CHIPSELECT_SIZE 1                 /*!< (unspecified)                                                        */
#define SPIM20_CORE_FREQUENCY 16                     /*!< Peripheral core frequency is 16 MHz.                                 */
#define SPIM20_PRESCALER_PRESENT 1                   /*!< (unspecified)                                                        */
#define SPIM20_PRESCALER_DIVISOR_RANGE_MIN 2         /*!< (unspecified)                                                        */
#define SPIM20_PRESCALER_DIVISOR_RANGE_MAX 126       /*!< (unspecified)                                                        */
#define SPIM20_PRESCALER_DIVISOR_RANGE_SIZE 127      /*!< (unspecified)                                                        */
#define SPIM20_RXDELAY_VALUE_RANGE_MIN 0             /*!< (unspecified)                                                        */
#define SPIM20_RXDELAY_VALUE_RANGE_MAX 7             /*!< (unspecified)                                                        */
#define SPIM20_RXDELAY_VALUE_RANGE_SIZE 8            /*!< (unspecified)                                                        */
#define SPIM20_RXDELAY_RESET_VALUE 1                 /*!< (unspecified)                                                        */
#define SPIM20_RXDELAY_FIELD_WIDTH_MIN 0             /*!< (unspecified)                                                        */
#define SPIM20_RXDELAY_FIELD_WIDTH_MAX 2             /*!< (unspecified)                                                        */
#define SPIM20_RXDELAY_FIELD_WIDTH_SIZE 3            /*!< (unspecified)                                                        */

#define SPIM21_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */
#define SPIM21_MAX_DATARATE 8                        /*!< (unspecified)                                                        */
#define SPIM21_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM21_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIM21_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIM21_FEATURE_HARDWARE_CSN_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM21_FEATURE_HARDWARE_DCX_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM21_FEATURE_RXDELAY_PRESENT 1             /*!< (unspecified)                                                        */
#define SPIM21_STALL_STATUS_PRESENT 0                /*!< (unspecified)                                                        */
#define SPIM21_STALL_STATUS_TX_PRESENT 0             /*!< (unspecified)                                                        */
#define SPIM21_NUM_CHIPSELECT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM21_NUM_CHIPSELECT_MAX 0                  /*!< (unspecified)                                                        */
#define SPIM21_NUM_CHIPSELECT_SIZE 1                 /*!< (unspecified)                                                        */
#define SPIM21_CORE_FREQUENCY 16                     /*!< Peripheral core frequency is 16 MHz.                                 */
#define SPIM21_PRESCALER_PRESENT 1                   /*!< (unspecified)                                                        */
#define SPIM21_PRESCALER_DIVISOR_RANGE_MIN 2         /*!< (unspecified)                                                        */
#define SPIM21_PRESCALER_DIVISOR_RANGE_MAX 126       /*!< (unspecified)                                                        */
#define SPIM21_PRESCALER_DIVISOR_RANGE_SIZE 127      /*!< (unspecified)                                                        */
#define SPIM21_RXDELAY_VALUE_RANGE_MIN 0             /*!< (unspecified)                                                        */
#define SPIM21_RXDELAY_VALUE_RANGE_MAX 7             /*!< (unspecified)                                                        */
#define SPIM21_RXDELAY_VALUE_RANGE_SIZE 8            /*!< (unspecified)                                                        */
#define SPIM21_RXDELAY_RESET_VALUE 1                 /*!< (unspecified)                                                        */
#define SPIM21_RXDELAY_FIELD_WIDTH_MIN 0             /*!< (unspecified)                                                        */
#define SPIM21_RXDELAY_FIELD_WIDTH_MAX 2             /*!< (unspecified)                                                        */
#define SPIM21_RXDELAY_FIELD_WIDTH_SIZE 3            /*!< (unspecified)                                                        */

#define SPIM22_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */
#define SPIM22_MAX_DATARATE 8                        /*!< (unspecified)                                                        */
#define SPIM22_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM22_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIM22_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIM22_FEATURE_HARDWARE_CSN_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM22_FEATURE_HARDWARE_DCX_PRESENT 0        /*!< (unspecified)                                                        */
#define SPIM22_FEATURE_RXDELAY_PRESENT 1             /*!< (unspecified)                                                        */
#define SPIM22_STALL_STATUS_PRESENT 0                /*!< (unspecified)                                                        */
#define SPIM22_STALL_STATUS_TX_PRESENT 0             /*!< (unspecified)                                                        */
#define SPIM22_NUM_CHIPSELECT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM22_NUM_CHIPSELECT_MAX 0                  /*!< (unspecified)                                                        */
#define SPIM22_NUM_CHIPSELECT_SIZE 1                 /*!< (unspecified)                                                        */
#define SPIM22_CORE_FREQUENCY 16                     /*!< Peripheral core frequency is 16 MHz.                                 */
#define SPIM22_PRESCALER_PRESENT 1                   /*!< (unspecified)                                                        */
#define SPIM22_PRESCALER_DIVISOR_RANGE_MIN 2         /*!< (unspecified)                                                        */
#define SPIM22_PRESCALER_DIVISOR_RANGE_MAX 126       /*!< (unspecified)                                                        */
#define SPIM22_PRESCALER_DIVISOR_RANGE_SIZE 127      /*!< (unspecified)                                                        */
#define SPIM22_RXDELAY_VALUE_RANGE_MIN 0             /*!< (unspecified)                                                        */
#define SPIM22_RXDELAY_VALUE_RANGE_MAX 7             /*!< (unspecified)                                                        */
#define SPIM22_RXDELAY_VALUE_RANGE_SIZE 8            /*!< (unspecified)                                                        */
#define SPIM22_RXDELAY_RESET_VALUE 1                 /*!< (unspecified)                                                        */
#define SPIM22_RXDELAY_FIELD_WIDTH_MIN 0             /*!< (unspecified)                                                        */
#define SPIM22_RXDELAY_FIELD_WIDTH_MAX 2             /*!< (unspecified)                                                        */
#define SPIM22_RXDELAY_FIELD_WIDTH_SIZE 3            /*!< (unspecified)                                                        */

#define SPIM30_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */
#define SPIM30_MAX_DATARATE 8                        /*!< (unspecified)                                                        */
#define SPIM30_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM30_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIM30_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIM30_FEATURE_HARDWARE_CSN_PRESENT 1        /*!< (unspecified)                                                        */
#define SPIM30_FEATURE_HARDWARE_DCX_PRESENT 0        /*!< (unspecified)                                                        */
#define SPIM30_FEATURE_RXDELAY_PRESENT 1             /*!< (unspecified)                                                        */
#define SPIM30_STALL_STATUS_PRESENT 0                /*!< (unspecified)                                                        */
#define SPIM30_STALL_STATUS_TX_PRESENT 0             /*!< (unspecified)                                                        */
#define SPIM30_NUM_CHIPSELECT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIM30_NUM_CHIPSELECT_MAX 0                  /*!< (unspecified)                                                        */
#define SPIM30_NUM_CHIPSELECT_SIZE 1                 /*!< (unspecified)                                                        */
#define SPIM30_CORE_FREQUENCY 16                     /*!< Peripheral core frequency is 16 MHz.                                 */
#define SPIM30_PRESCALER_PRESENT 1                   /*!< (unspecified)                                                        */
#define SPIM30_PRESCALER_DIVISOR_RANGE_MIN 2         /*!< (unspecified)                                                        */
#define SPIM30_PRESCALER_DIVISOR_RANGE_MAX 126       /*!< (unspecified)                                                        */
#define SPIM30_PRESCALER_DIVISOR_RANGE_SIZE 127      /*!< (unspecified)                                                        */
#define SPIM30_RXDELAY_VALUE_RANGE_MIN 0             /*!< (unspecified)                                                        */
#define SPIM30_RXDELAY_VALUE_RANGE_MAX 7             /*!< (unspecified)                                                        */
#define SPIM30_RXDELAY_VALUE_RANGE_SIZE 8            /*!< (unspecified)                                                        */
#define SPIM30_RXDELAY_RESET_VALUE 1                 /*!< (unspecified)                                                        */
#define SPIM30_RXDELAY_FIELD_WIDTH_MIN 0             /*!< (unspecified)                                                        */
#define SPIM30_RXDELAY_FIELD_WIDTH_MAX 2             /*!< (unspecified)                                                        */
#define SPIM30_RXDELAY_FIELD_WIDTH_SIZE 3            /*!< (unspecified)                                                        */

/*SPI Slave*/
#define SPIS_PRESENT 1
#define SPIS_COUNT 5

#define SPIS00_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIS00_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIS00_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIS00_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define SPIS20_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIS20_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIS20_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIS20_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define SPIS21_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIS21_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIS21_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIS21_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define SPIS22_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIS22_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIS22_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIS22_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define SPIS30_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define SPIS30_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define SPIS30_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define SPIS30_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

/*UART with EasyDMA*/
#define UARTE_PRESENT 1
#define UARTE_COUNT 5

#define UARTE00_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
#define UARTE00_EASYDMA_MAXCNT_MAX 7                 /*!< (unspecified)                                                        */
#define UARTE00_EASYDMA_MAXCNT_SIZE 8                /*!< (unspecified)                                                        */
#define UARTE00_TIMEOUT_INTERRUPT 1                  /*!< (unspecified)                                                        */
#define UARTE00_CONFIGURABLE_DATA_FRAME_SIZE 1       /*!< (unspecified)                                                        */
#define UARTE00_CORE_FREQUENCY 128                   /*!< Peripheral clock frequency is 128 MHz.                               */
#define UARTE00_CORE_CLOCK_128 1                     /*!< (unspecified)                                                        */
#define UARTE00_SHORTS_ENDTX_STOPTX 1                /*!< (unspecified)                                                        */
#define UARTE00_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */

#define UARTE20_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
#define UARTE20_EASYDMA_MAXCNT_MAX 7                 /*!< (unspecified)                                                        */
#define UARTE20_EASYDMA_MAXCNT_SIZE 8                /*!< (unspecified)                                                        */
#define UARTE20_TIMEOUT_INTERRUPT 1                  /*!< (unspecified)                                                        */
#define UARTE20_CONFIGURABLE_DATA_FRAME_SIZE 1       /*!< (unspecified)                                                        */
#define UARTE20_CORE_FREQUENCY 16                    /*!< Peripheral clock frequency is 16 MHz.                                */
#define UARTE20_CORE_CLOCK_16 1                      /*!< (unspecified)                                                        */
#define UARTE20_SHORTS_ENDTX_STOPTX 1                /*!< (unspecified)                                                        */
#define UARTE20_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */

#define UARTE21_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
#define UARTE21_EASYDMA_MAXCNT_MAX 7                 /*!< (unspecified)                                                        */
#define UARTE21_EASYDMA_MAXCNT_SIZE 8                /*!< (unspecified)                                                        */
#define UARTE21_TIMEOUT_INTERRUPT 1                  /*!< (unspecified)                                                        */
#define UARTE21_CONFIGURABLE_DATA_FRAME_SIZE 1       /*!< (unspecified)                                                        */
#define UARTE21_CORE_FREQUENCY 16                    /*!< Peripheral clock frequency is 16 MHz.                                */
#define UARTE21_CORE_CLOCK_16 1                      /*!< (unspecified)                                                        */
#define UARTE21_SHORTS_ENDTX_STOPTX 1                /*!< (unspecified)                                                        */
#define UARTE21_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */

#define UARTE22_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
#define UARTE22_EASYDMA_MAXCNT_MAX 7                 /*!< (unspecified)                                                        */
#define UARTE22_EASYDMA_MAXCNT_SIZE 8                /*!< (unspecified)                                                        */
#define UARTE22_TIMEOUT_INTERRUPT 1                  /*!< (unspecified)                                                        */
#define UARTE22_CONFIGURABLE_DATA_FRAME_SIZE 1       /*!< (unspecified)                                                        */
#define UARTE22_CORE_FREQUENCY 16                    /*!< Peripheral clock frequency is 16 MHz.                                */
#define UARTE22_CORE_CLOCK_16 1                      /*!< (unspecified)                                                        */
#define UARTE22_SHORTS_ENDTX_STOPTX 1                /*!< (unspecified)                                                        */
#define UARTE22_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */

#define UARTE30_EASYDMA_MAXCNT_MIN 0                 /*!< (unspecified)                                                        */
#define UARTE30_EASYDMA_MAXCNT_MAX 7                 /*!< (unspecified)                                                        */
#define UARTE30_EASYDMA_MAXCNT_SIZE 8                /*!< (unspecified)                                                        */
#define UARTE30_TIMEOUT_INTERRUPT 1                  /*!< (unspecified)                                                        */
#define UARTE30_CONFIGURABLE_DATA_FRAME_SIZE 1       /*!< (unspecified)                                                        */
#define UARTE30_CORE_FREQUENCY 16                    /*!< Peripheral clock frequency is 16 MHz.                                */
#define UARTE30_CORE_CLOCK_16 1                      /*!< (unspecified)                                                        */
#define UARTE30_SHORTS_ENDTX_STOPTX 1                /*!< (unspecified)                                                        */
#define UARTE30_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                  */

/*Voltage glitch detectors*/
#define GLITCHDET_PRESENT 1
#define GLITCHDET_COUNT 1

/*RRAM controller GLITCH detector*/
#define RRAMC_PRESENT 1
#define RRAMC_COUNT 1

#define RRAMC_NRRAMWORDSIZE 128                      /*!< RRAM word size : 128 bits per wordline                               */
#define RRAMC_NWRITEBUFSIZE 32                       /*!< Maximum write buffer size : 32                                       */
#define RRAMC_REGION0ADDR_WRITABLE 0                 /*!< (unspecified)                                                        */
#define RRAMC_REGION0SIZE 4                          /*!< (unspecified)                                                        */
#define RRAMC_REGION0SIZE_WRITABLE 0                 /*!< (unspecified)                                                        */
#define RRAMC_REGION0READ 1                          /*!< (unspecified)                                                        */
#define RRAMC_REGION0READ_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION0WRITE 1                         /*!< (unspecified)                                                        */
#define RRAMC_REGION0WRITE_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION0EXECUTE 0                       /*!< (unspecified)                                                        */
#define RRAMC_REGION0EXECUTE_WRITABLE 0              /*!< (unspecified)                                                        */
#define RRAMC_REGION0SECURE 0                        /*!< (unspecified)                                                        */
#define RRAMC_REGION0SECURE_WRITABLE 0               /*!< (unspecified)                                                        */
#define RRAMC_REGION0OWNER 0                         /*!< (unspecified)                                                        */
#define RRAMC_REGION0OWNER_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION0WRITEONCE 1                     /*!< (unspecified)                                                        */
#define RRAMC_REGION0WRITEONCE_WRITABLE 0            /*!< (unspecified)                                                        */
#define RRAMC_REGION0LOCK 0                          /*!< (unspecified)                                                        */
#define RRAMC_REGION0LOCK_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION1ADDR_WRITABLE 0                 /*!< (unspecified)                                                        */
#define RRAMC_REGION1SIZE 4                          /*!< (unspecified)                                                        */
#define RRAMC_REGION1SIZE_WRITABLE 0                 /*!< (unspecified)                                                        */
#define RRAMC_REGION1READ 1                          /*!< (unspecified)                                                        */
#define RRAMC_REGION1READ_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION1WRITE 1                         /*!< (unspecified)                                                        */
#define RRAMC_REGION1WRITE_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION1EXECUTE 0                       /*!< (unspecified)                                                        */
#define RRAMC_REGION1EXECUTE_WRITABLE 0              /*!< (unspecified)                                                        */
#define RRAMC_REGION1SECURE 1                        /*!< (unspecified)                                                        */
#define RRAMC_REGION1SECURE_WRITABLE 1               /*!< (unspecified)                                                        */
#define RRAMC_REGION1OWNER 0                         /*!< (unspecified)                                                        */
#define RRAMC_REGION1OWNER_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION1WRITEONCE 1                     /*!< (unspecified)                                                        */
#define RRAMC_REGION1WRITEONCE_WRITABLE 0            /*!< (unspecified)                                                        */
#define RRAMC_REGION1LOCK 0                          /*!< (unspecified)                                                        */
#define RRAMC_REGION1LOCK_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION2ADDR_WRITABLE 0                 /*!< (unspecified)                                                        */
#define RRAMC_REGION2SIZE 8                          /*!< (unspecified)                                                        */
#define RRAMC_REGION2SIZE_WRITABLE 0                 /*!< (unspecified)                                                        */
#define RRAMC_REGION2READ 1                          /*!< (unspecified)                                                        */
#define RRAMC_REGION2READ_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION2WRITE 1                         /*!< (unspecified)                                                        */
#define RRAMC_REGION2WRITE_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION2EXECUTE 0                       /*!< (unspecified)                                                        */
#define RRAMC_REGION2EXECUTE_WRITABLE 0              /*!< (unspecified)                                                        */
#define RRAMC_REGION2SECURE 1                        /*!< (unspecified)                                                        */
#define RRAMC_REGION2SECURE_WRITABLE 0               /*!< (unspecified)                                                        */
#define RRAMC_REGION2OWNER 2                         /*!< (unspecified)                                                        */
#define RRAMC_REGION2OWNER_WRITABLE 0                /*!< (unspecified)                                                        */
#define RRAMC_REGION2WRITEONCE 0                     /*!< (unspecified)                                                        */
#define RRAMC_REGION2WRITEONCE_WRITABLE 1            /*!< (unspecified)                                                        */
#define RRAMC_REGION2LOCK 0                          /*!< (unspecified)                                                        */
#define RRAMC_REGION2LOCK_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION3ADDR_WRITABLE 0                 /*!< (unspecified)                                                        */
#define RRAMC_REGION3SIZE 0                          /*!< (unspecified)                                                        */
#define RRAMC_REGION3SIZE_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION3READ 1                          /*!< (unspecified)                                                        */
#define RRAMC_REGION3READ_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION3WRITE 1                         /*!< (unspecified)                                                        */
#define RRAMC_REGION3WRITE_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION3EXECUTE 1                       /*!< (unspecified)                                                        */
#define RRAMC_REGION3EXECUTE_WRITABLE 1              /*!< (unspecified)                                                        */
#define RRAMC_REGION3SECURE 1                        /*!< (unspecified)                                                        */
#define RRAMC_REGION3SECURE_WRITABLE 1               /*!< (unspecified)                                                        */
#define RRAMC_REGION3OWNER 0                         /*!< (unspecified)                                                        */
#define RRAMC_REGION3OWNER_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION3WRITEONCE 0                     /*!< (unspecified)                                                        */
#define RRAMC_REGION3WRITEONCE_WRITABLE 1            /*!< (unspecified)                                                        */
#define RRAMC_REGION3LOCK 0                          /*!< (unspecified)                                                        */
#define RRAMC_REGION3LOCK_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION4ADDR_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION4SIZE 0                          /*!< (unspecified)                                                        */
#define RRAMC_REGION4SIZE_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION4READ 1                          /*!< (unspecified)                                                        */
#define RRAMC_REGION4READ_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_REGION4WRITE 1                         /*!< (unspecified)                                                        */
#define RRAMC_REGION4WRITE_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION4EXECUTE 1                       /*!< (unspecified)                                                        */
#define RRAMC_REGION4EXECUTE_WRITABLE 1              /*!< (unspecified)                                                        */
#define RRAMC_REGION4SECURE 1                        /*!< (unspecified)                                                        */
#define RRAMC_REGION4SECURE_WRITABLE 1               /*!< (unspecified)                                                        */
#define RRAMC_REGION4OWNER 0                         /*!< (unspecified)                                                        */
#define RRAMC_REGION4OWNER_WRITABLE 1                /*!< (unspecified)                                                        */
#define RRAMC_REGION4WRITEONCE 0                     /*!< (unspecified)                                                        */
#define RRAMC_REGION4WRITEONCE_WRITABLE 1            /*!< (unspecified)                                                        */
#define RRAMC_REGION4LOCK 0                          /*!< (unspecified)                                                        */
#define RRAMC_REGION4LOCK_WRITABLE 1                 /*!< (unspecified)                                                        */
#define RRAMC_GLITCHDETECTORS 0                      /*!< (unspecified)                                                        */

/*VPR peripheral registers*/
#define VPR_PRESENT 1
#define VPR_COUNT 1

#define VPR00_INIT_PC_RESET_VALUE 0x00000000         /*!< Boot vector (INIT_PC_RESET_VALUE): 0x00000000                        */
#define VPR00_VPR_START_RESET_VALUE 0                /*!< Self-booting (VPR_START_RESET_VALUE): 0                              */
#define VPR00_RAM_BASE_ADDR 0x00000000               /*!< VPR RAM base address (RAM_BASE_ADDR): 0x00000000                     */
#define VPR00_RAM_SZ 0                               /*!< VPR RAM size (RAM_SZ): 0 (Value in bytes is computed as 2^(RAM size))*/
#define VPR00_RETAINED 0                             /*!< Retain registers in Deep Sleep mode: 0                               */
#define VPR00_VPRSAVEDCTX 1                          /*!< (unspecified)                                                        */
#define VPR00_VPRSAVEADDR 0x00000000                 /*!< VPR context save address: 0x00000000                                 */
#define VPR00_VPRREMAPADDRVTOB 0x00000000            /*!< VPR remap address: 0x00000000                                        */
#define VPR00_VEVIF_NTASKS_MIN 0                     /*!< VEVIF tasks: 0..31                                                   */
#define VPR00_VEVIF_NTASKS_MAX 31                    /*!< VEVIF tasks: 0..31                                                   */
#define VPR00_VEVIF_NTASKS_SIZE 32                   /*!< VEVIF tasks: 0..31                                                   */
#define VPR00_VEVIF_TASKS_MASK 0xFFFFFFFF            /*!< Mask of supported VEVIF tasks: 0xFFFFFFFF                            */
#define VPR00_VEVIF_NDPPI_MIN 0                      /*!< VEVIF DPPI channels: 0..31                                           */
#define VPR00_VEVIF_NDPPI_MAX 31                     /*!< VEVIF DPPI channels: 0..31                                           */
#define VPR00_VEVIF_NDPPI_SIZE 32                    /*!< VEVIF DPPI channels: 0..31                                           */
#define VPR00_VEVIF_NEVENTS_MIN 0                    /*!< VEVIF events: 0..31                                                  */
#define VPR00_VEVIF_NEVENTS_MAX 31                   /*!< VEVIF events: 0..31                                                  */
#define VPR00_VEVIF_NEVENTS_SIZE 32                  /*!< VEVIF events: 0..31                                                  */
#define VPR00_DEBUGGER_OFFSET 1024                   /*!< Debugger interface register offset: 0x5004C400                       */

/*GPIO Port*/
#define GPIO_PRESENT 1
#define GPIO_COUNT 3

#define P2_CTRLSEL_MAP1 0                            /*!< (unspecified)                                                        */
#define P2_CTRLSEL_MAP2 1                            /*!< (unspecified)                                                        */
#define P2_PIN_NUM_MIN 0                             /*!< (unspecified)                                                        */
#define P2_PIN_NUM_MAX 10                            /*!< (unspecified)                                                        */
#define P2_PIN_NUM_SIZE 11                           /*!< (unspecified)                                                        */
#define P2_PINS_PRESENT 2047                         /*!< (unspecified)                                                        */
#define P2_DRIVECTRL 0                               /*!< (unspecified)                                                        */
#define P2_RETAIN 0                                  /*!< (unspecified)                                                        */
#define P2_PWRCTRL 0                                 /*!< (unspecified)                                                        */
#define P2_PWRCTRL_SEPARATE_REG 0                    /*!< (unspecified)                                                        */
#define P2_VSS_FLOAT_DFT 0                           /*!< (unspecified)                                                        */
#define P2_PIN_OWNER_SEC 0                           /*!< (unspecified)                                                        */
#define P2_WIFI_CORE_PRESENT 0                       /*!< (unspecified)                                                        */
#define P2_RETAIN_PER_PIN 0                          /*!< (unspecified)                                                        */
#define P2_CLOCKPIN 0                                /*!< (unspecified)                                                        */

#define P1_CTRLSEL_MAP1 0                            /*!< (unspecified)                                                        */
#define P1_CTRLSEL_MAP2 1                            /*!< (unspecified)                                                        */
#define P1_PIN_NUM_MIN 0                             /*!< (unspecified)                                                        */
#define P1_PIN_NUM_MAX 16                            /*!< (unspecified)                                                        */
#define P1_PIN_NUM_SIZE 17                           /*!< (unspecified)                                                        */
#define P1_PINS_PRESENT 131071                       /*!< (unspecified)                                                        */
#define P1_DRIVECTRL 0                               /*!< (unspecified)                                                        */
#define P1_RETAIN 0                                  /*!< (unspecified)                                                        */
#define P1_PWRCTRL 0                                 /*!< (unspecified)                                                        */
#define P1_PWRCTRL_SEPARATE_REG 0                    /*!< (unspecified)                                                        */
#define P1_VSS_FLOAT_DFT 0                           /*!< (unspecified)                                                        */
#define P1_PIN_OWNER_SEC 0                           /*!< (unspecified)                                                        */
#define P1_WIFI_CORE_PRESENT 0                       /*!< (unspecified)                                                        */
#define P1_RETAIN_PER_PIN 0                          /*!< (unspecified)                                                        */
#define P1_CLOCKPIN 0                                /*!< (unspecified)                                                        */

#define P0_CTRLSEL_MAP1 0                            /*!< (unspecified)                                                        */
#define P0_CTRLSEL_MAP2 1                            /*!< (unspecified)                                                        */
#define P0_PIN_NUM_MIN 0                             /*!< (unspecified)                                                        */
#define P0_PIN_NUM_MAX 6                             /*!< (unspecified)                                                        */
#define P0_PIN_NUM_SIZE 7                            /*!< (unspecified)                                                        */
#define P0_PINS_PRESENT 127                          /*!< (unspecified)                                                        */
#define P0_DRIVECTRL 0                               /*!< (unspecified)                                                        */
#define P0_RETAIN 0                                  /*!< (unspecified)                                                        */
#define P0_PWRCTRL 0                                 /*!< (unspecified)                                                        */
#define P0_PWRCTRL_SEPARATE_REG 0                    /*!< (unspecified)                                                        */
#define P0_VSS_FLOAT_DFT 0                           /*!< (unspecified)                                                        */
#define P0_PIN_OWNER_SEC 0                           /*!< (unspecified)                                                        */
#define P0_WIFI_CORE_PRESENT 0                       /*!< (unspecified)                                                        */
#define P0_RETAIN_PER_PIN 0                          /*!< (unspecified)                                                        */
#define P0_CLOCKPIN 0                                /*!< (unspecified)                                                        */

/*Control access port*/
#define CTRLAPPERI_PRESENT 1
#define CTRLAPPERI_COUNT 1

/*Trace and debug control*/
#define TAD_PRESENT 1
#define TAD_COUNT 1

#define TAD_TADFORCEON 0                             /*!< (unspecified)                                                        */
#define TAD_TAD_HAS_TASKS 0                          /*!< (unspecified)                                                        */
#define TAD_PDREQCLR 1                               /*!< (unspecified)                                                        */

/*Timer/Counter*/
#define TIMER_PRESENT 1
#define TIMER_COUNT 7

#define TIMER00_CC_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define TIMER00_CC_NUM_MAX 5                         /*!< (unspecified)                                                        */
#define TIMER00_CC_NUM_SIZE 6                        /*!< (unspecified)                                                        */
#define TIMER00_MAX_SIZE_MIN 0                       /*!< (unspecified)                                                        */
#define TIMER00_MAX_SIZE_MAX 31                      /*!< (unspecified)                                                        */
#define TIMER00_MAX_SIZE_SIZE 32                     /*!< (unspecified)                                                        */
#define TIMER00_PCLK_MHZ 128                         /*!< Peripheral clock frequency (PCLK) is 128 MHz                         */
#define TIMER00_PCLK_VARIABLE 1                      /*!< (unspecified)                                                        */

#define TIMER10_CC_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define TIMER10_CC_NUM_MAX 7                         /*!< (unspecified)                                                        */
#define TIMER10_CC_NUM_SIZE 8                        /*!< (unspecified)                                                        */
#define TIMER10_MAX_SIZE_MIN 0                       /*!< (unspecified)                                                        */
#define TIMER10_MAX_SIZE_MAX 31                      /*!< (unspecified)                                                        */
#define TIMER10_MAX_SIZE_SIZE 32                     /*!< (unspecified)                                                        */
#define TIMER10_PCLK_MHZ 32                          /*!< Peripheral clock frequency (PCLK) is 32 MHz                          */
#define TIMER10_PCLK_VARIABLE 0                      /*!< (unspecified)                                                        */

#define TIMER20_CC_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define TIMER20_CC_NUM_MAX 5                         /*!< (unspecified)                                                        */
#define TIMER20_CC_NUM_SIZE 6                        /*!< (unspecified)                                                        */
#define TIMER20_MAX_SIZE_MIN 0                       /*!< (unspecified)                                                        */
#define TIMER20_MAX_SIZE_MAX 31                      /*!< (unspecified)                                                        */
#define TIMER20_MAX_SIZE_SIZE 32                     /*!< (unspecified)                                                        */
#define TIMER20_PCLK_MHZ 16                          /*!< Peripheral clock frequency (PCLK) is 16 MHz                          */
#define TIMER20_PCLK_VARIABLE 0                      /*!< (unspecified)                                                        */

#define TIMER21_CC_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define TIMER21_CC_NUM_MAX 5                         /*!< (unspecified)                                                        */
#define TIMER21_CC_NUM_SIZE 6                        /*!< (unspecified)                                                        */
#define TIMER21_MAX_SIZE_MIN 0                       /*!< (unspecified)                                                        */
#define TIMER21_MAX_SIZE_MAX 31                      /*!< (unspecified)                                                        */
#define TIMER21_MAX_SIZE_SIZE 32                     /*!< (unspecified)                                                        */
#define TIMER21_PCLK_MHZ 16                          /*!< Peripheral clock frequency (PCLK) is 16 MHz                          */
#define TIMER21_PCLK_VARIABLE 0                      /*!< (unspecified)                                                        */

#define TIMER22_CC_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define TIMER22_CC_NUM_MAX 5                         /*!< (unspecified)                                                        */
#define TIMER22_CC_NUM_SIZE 6                        /*!< (unspecified)                                                        */
#define TIMER22_MAX_SIZE_MIN 0                       /*!< (unspecified)                                                        */
#define TIMER22_MAX_SIZE_MAX 31                      /*!< (unspecified)                                                        */
#define TIMER22_MAX_SIZE_SIZE 32                     /*!< (unspecified)                                                        */
#define TIMER22_PCLK_MHZ 16                          /*!< Peripheral clock frequency (PCLK) is 16 MHz                          */
#define TIMER22_PCLK_VARIABLE 0                      /*!< (unspecified)                                                        */

#define TIMER23_CC_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define TIMER23_CC_NUM_MAX 5                         /*!< (unspecified)                                                        */
#define TIMER23_CC_NUM_SIZE 6                        /*!< (unspecified)                                                        */
#define TIMER23_MAX_SIZE_MIN 0                       /*!< (unspecified)                                                        */
#define TIMER23_MAX_SIZE_MAX 31                      /*!< (unspecified)                                                        */
#define TIMER23_MAX_SIZE_SIZE 32                     /*!< (unspecified)                                                        */
#define TIMER23_PCLK_MHZ 16                          /*!< Peripheral clock frequency (PCLK) is 16 MHz                          */
#define TIMER23_PCLK_VARIABLE 0                      /*!< (unspecified)                                                        */

#define TIMER24_CC_NUM_MIN 0                         /*!< (unspecified)                                                        */
#define TIMER24_CC_NUM_MAX 5                         /*!< (unspecified)                                                        */
#define TIMER24_CC_NUM_SIZE 6                        /*!< (unspecified)                                                        */
#define TIMER24_MAX_SIZE_MIN 0                       /*!< (unspecified)                                                        */
#define TIMER24_MAX_SIZE_MAX 31                      /*!< (unspecified)                                                        */
#define TIMER24_MAX_SIZE_SIZE 32                     /*!< (unspecified)                                                        */
#define TIMER24_PCLK_MHZ 16                          /*!< Peripheral clock frequency (PCLK) is 16 MHz                          */
#define TIMER24_PCLK_VARIABLE 0                      /*!< (unspecified)                                                        */

/*Real-time counter*/
#define RTC_PRESENT 1
#define RTC_COUNT 2

#define RTC10_CC_NUM_MIN 0                           /*!< (unspecified)                                                        */
#define RTC10_CC_NUM_MAX 3                           /*!< (unspecified)                                                        */
#define RTC10_CC_NUM_SIZE 4                          /*!< (unspecified)                                                        */

#define RTC30_CC_NUM_MIN 0                           /*!< (unspecified)                                                        */
#define RTC30_CC_NUM_MAX 3                           /*!< (unspecified)                                                        */
#define RTC30_CC_NUM_SIZE 4                          /*!< (unspecified)                                                        */

/*Event generator unit*/
#define EGU_PRESENT 1
#define EGU_COUNT 2

#define EGU10_PEND 0                                 /*!< (unspecified)                                                        */
#define EGU10_CH_NUM_MIN 0                           /*!< (unspecified)                                                        */
#define EGU10_CH_NUM_MAX 15                          /*!< (unspecified)                                                        */
#define EGU10_CH_NUM_SIZE 16                         /*!< (unspecified)                                                        */

#define EGU20_PEND 0                                 /*!< (unspecified)                                                        */
#define EGU20_CH_NUM_MIN 0                           /*!< (unspecified)                                                        */
#define EGU20_CH_NUM_MAX 5                           /*!< (unspecified)                                                        */
#define EGU20_CH_NUM_SIZE 6                          /*!< (unspecified)                                                        */

/*2.4 GHz radio*/
#define RADIO_PRESENT 1
#define RADIO_COUNT 1

#define RADIO_IRQ_COUNT 2
#define RADIO_WHITENINGPOLY 1                        /*!< (unspecified)                                                        */
#define RADIO_ADPLLCOMPANION_INCLUDE_DMA 0           /*!< (unspecified)                                                        */

/*I2C compatible Two-Wire Master Interface with EasyDMA*/
#define TWIM_PRESENT 1
#define TWIM_COUNT 4

#define TWIM20_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIM20_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIM20_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIM20_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define TWIM21_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIM21_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIM21_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIM21_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define TWIM22_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIM22_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIM22_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIM22_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define TWIM30_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIM30_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIM30_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIM30_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

/*I2C compatible Two-Wire Slave Interface with EasyDMA*/
#define TWIS_PRESENT 1
#define TWIS_COUNT 4

#define TWIS20_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIS20_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIS20_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIS20_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define TWIS21_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIS21_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIS21_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIS21_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define TWIS22_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIS22_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIS22_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIS22_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

#define TWIS30_EASYDMA_MAXCNT_MIN 0                  /*!< (unspecified)                                                        */
#define TWIS30_EASYDMA_MAXCNT_MAX 15                 /*!< (unspecified)                                                        */
#define TWIS30_EASYDMA_MAXCNT_SIZE 16                /*!< (unspecified)                                                        */
#define TWIS30_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                   */

/*Memory configuration*/
#define MEMCONF_PRESENT 1
#define MEMCONF_COUNT 1

#define MEMCONF_RETTRIM 1                            /*!< (unspecified)                                                        */
#define MEMCONF_REPAIR 0                             /*!< (unspecified)                                                        */
#define MEMCONF_POWER 1                              /*!< (unspecified)                                                        */
#define MEMCONF_RET2 1                               /*!< (unspecified)                                                        */

/*Pulse Density Modulation (Digital Microphone) Interface*/
#define PDM_PRESENT 1
#define PDM_COUNT 2

#define PDM20_SAMPLE16 0                             /*!< (unspecified)                                                        */
#define PDM20_SAMPLE48 1                             /*!< (unspecified)                                                        */
#define PDM20_MCLKCONFIG_TEMPADDRESS 1               /*!< (unspecified)                                                        */
#define PDM20_PRESCALER_PRESENT 1                    /*!< (unspecified)                                                        */
#define PDM20_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 1 /*!< (unspecified)                                                    */

#define PDM21_SAMPLE16 0                             /*!< (unspecified)                                                        */
#define PDM21_SAMPLE48 1                             /*!< (unspecified)                                                        */
#define PDM21_MCLKCONFIG_TEMPADDRESS 1               /*!< (unspecified)                                                        */
#define PDM21_PRESCALER_PRESENT 1                    /*!< (unspecified)                                                        */
#define PDM21_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 1 /*!< (unspecified)                                                    */

/*Pulse width modulation unit*/
#define PWM_PRESENT 1
#define PWM_COUNT 3

#define PWM20_IDLE_OUT 1                             /*!< (unspecified)                                                        */
#define PWM20_COMPARE_MATCH 1                        /*!< (unspecified)                                                        */
#define PWM20_FEATURES_V2 0                          /*!< (unspecified)                                                        */
#define PWM20_NO_FEATURES_V2 1                       /*!< (unspecified)                                                        */
#define PWM20_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 1 /*!< (unspecified)                                                    */

#define PWM21_IDLE_OUT 1                             /*!< (unspecified)                                                        */
#define PWM21_COMPARE_MATCH 1                        /*!< (unspecified)                                                        */
#define PWM21_FEATURES_V2 0                          /*!< (unspecified)                                                        */
#define PWM21_NO_FEATURES_V2 1                       /*!< (unspecified)                                                        */
#define PWM21_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 1 /*!< (unspecified)                                                    */

#define PWM22_IDLE_OUT 1                             /*!< (unspecified)                                                        */
#define PWM22_COMPARE_MATCH 1                        /*!< (unspecified)                                                        */
#define PWM22_FEATURES_V2 0                          /*!< (unspecified)                                                        */
#define PWM22_NO_FEATURES_V2 1                       /*!< (unspecified)                                                        */
#define PWM22_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 1 /*!< (unspecified)                                                    */

/*Analog to Digital Converter*/
#define SAADC_PRESENT 1
#define SAADC_COUNT 1

#define SAADC_PSEL_V2 1                              /*!< (unspecified)                                                        */
#define SAADC_TASKS_CALIBRATEGAIN 0                  /*!< (unspecified)                                                        */
#define SAADC_SAMPLERATE_CC_VALUERANGE_MIN 8         /*!< (unspecified)                                                        */
#define SAADC_SAMPLERATE_CC_VALUERANGE_MAX 2047      /*!< (unspecified)                                                        */
#define SAADC_SAMPLERATE_CC_VALUERANGE_SIZE 2048     /*!< (unspecified)                                                        */
#define SAADC_TACQ_VALUE_RANGE_MIN 1                 /*!< (unspecified)                                                        */
#define SAADC_TACQ_VALUE_RANGE_MAX 319               /*!< (unspecified)                                                        */
#define SAADC_TACQ_VALUE_RANGE_SIZE 320              /*!< (unspecified)                                                        */
#define SAADC_TCONV_VALUE_RANGE_MIN 1                /*!< (unspecified)                                                        */
#define SAADC_TCONV_VALUE_RANGE_MAX 7                /*!< (unspecified)                                                        */
#define SAADC_TCONV_VALUE_RANGE_SIZE 8               /*!< (unspecified)                                                        */
#define SAADC_EASYDMA_CURRENT_AMOUNT_REGISTER_INCLUDED 0 /*!< (unspecified)                                                    */

/*NFC-A compatible radio NFC-A compatible radio*/
#define NFCT_PRESENT 1
#define NFCT_COUNT 1

#define NFCT_NFCTFIELDDETCFG_RESET 1                 /*!< Reset value of register NFCTFIELDDETCFG: 1                           */

/*Temperature Sensor*/
#define TEMP_PRESENT 1
#define TEMP_COUNT 1

/*GPIO Tasks and Events*/
#define GPIOTE_PRESENT 1
#define GPIOTE_COUNT 2

#define GPIOTE20_IRQ_COUNT 2
#define GPIOTE20_GPIOTE_NCHANNELS_MIN 0              /*!< Number of GPIOTE channels: 0..7                                      */
#define GPIOTE20_GPIOTE_NCHANNELS_MAX 7              /*!< Number of GPIOTE channels: 0..7                                      */
#define GPIOTE20_GPIOTE_NCHANNELS_SIZE 8             /*!< Number of GPIOTE channels: 0..7                                      */
#define GPIOTE20_GPIOTE_NPORTEVENTS_MIN 0            /*!< Number of GPIOTE port events: 0..0                                   */
#define GPIOTE20_GPIOTE_NPORTEVENTS_MAX 0            /*!< Number of GPIOTE port events: 0..0                                   */
#define GPIOTE20_GPIOTE_NPORTEVENTS_SIZE 1           /*!< Number of GPIOTE port events: 0..0                                   */
#define GPIOTE20_GPIOTE_NINTERRUPTS_MIN 0            /*!< Number of GPIOTE interrupts: 0..1                                    */
#define GPIOTE20_GPIOTE_NINTERRUPTS_MAX 1            /*!< Number of GPIOTE interrupts: 0..1                                    */
#define GPIOTE20_GPIOTE_NINTERRUPTS_SIZE 2           /*!< Number of GPIOTE interrupts: 0..1                                    */
#define GPIOTE20_HAS_PORT_EVENT 1                    /*!< (unspecified)                                                        */

#define GPIOTE30_IRQ_COUNT 2
#define GPIOTE30_GPIOTE_NCHANNELS_MIN 0              /*!< Number of GPIOTE channels: 0..3                                      */
#define GPIOTE30_GPIOTE_NCHANNELS_MAX 3              /*!< Number of GPIOTE channels: 0..3                                      */
#define GPIOTE30_GPIOTE_NCHANNELS_SIZE 4             /*!< Number of GPIOTE channels: 0..3                                      */
#define GPIOTE30_GPIOTE_NPORTEVENTS_MIN 0            /*!< Number of GPIOTE port events: 0..0                                   */
#define GPIOTE30_GPIOTE_NPORTEVENTS_MAX 0            /*!< Number of GPIOTE port events: 0..0                                   */
#define GPIOTE30_GPIOTE_NPORTEVENTS_SIZE 1           /*!< Number of GPIOTE port events: 0..0                                   */
#define GPIOTE30_GPIOTE_NINTERRUPTS_MIN 0            /*!< Number of GPIOTE interrupts: 0..1                                    */
#define GPIOTE30_GPIOTE_NINTERRUPTS_MAX 1            /*!< Number of GPIOTE interrupts: 0..1                                    */
#define GPIOTE30_GPIOTE_NINTERRUPTS_SIZE 2           /*!< Number of GPIOTE interrupts: 0..1                                    */
#define GPIOTE30_HAS_PORT_EVENT 1                    /*!< (unspecified)                                                        */

/*Tamper controller*/
#define TAMPC_PRESENT 1
#define TAMPC_COUNT 1

#define TAMPC_PROTECT_INTRESETEN_CTRL_VALUE_RESET 0  /*!< Reset value of field VALUE in register PROTECT.INTRESETEN.CTRL: 0    */

/*Inter-IC Sound*/
#define I2S_PRESENT 1
#define I2S_COUNT 1

#define I2S20_EASYDMA_MAXCNT_SIZE_MIN 0              /*!< (unspecified)                                                        */
#define I2S20_EASYDMA_MAXCNT_SIZE_MAX 13             /*!< (unspecified)                                                        */
#define I2S20_EASYDMA_MAXCNT_SIZE_SIZE 14            /*!< (unspecified)                                                        */
#define I2S20_CLKCONFIG 0                            /*!< (unspecified)                                                        */

/*Quadrature Decoder*/
#define QDEC_PRESENT 1
#define QDEC_COUNT 2

/*Global Real-time counter*/
#define GRTC_PRESENT 1
#define GRTC_COUNT 1

#define GRTC_IRQ_COUNT 4
#define GRTC_MSBWIDTH_MIN 0                          /*!< Width of the RTCOUNTERH, RTCOMPAREH and RTCOMPARESYNCH registers :
                                                          0..14*/
#define GRTC_MSBWIDTH_MAX 14                         /*!< Width of the RTCOUNTERH, RTCOMPAREH and RTCOMPARESYNCH registers :
                                                          0..14*/
#define GRTC_MSBWIDTH_SIZE 15                        /*!< Width of the RTCOUNTERH, RTCOMPAREH and RTCOMPARESYNCH registers :
                                                          0..14*/
#define GRTC_NCC_MIN 0                               /*!< Number of compare/capture registers : 0..11                          */
#define GRTC_NCC_MAX 11                              /*!< Number of compare/capture registers : 0..11                          */
#define GRTC_NCC_SIZE 12                             /*!< Number of compare/capture registers : 0..11                          */
#define GRTC_NTIMEOUT_MIN 0                          /*!< Width of the TIMEOUT register : 0..15                                */
#define GRTC_NTIMEOUT_MAX 15                         /*!< Width of the TIMEOUT register : 0..15                                */
#define GRTC_NTIMEOUT_SIZE 16                        /*!< Width of the TIMEOUT register : 0..15                                */
#define GRTC_NDOMAIN_MIN 0                           /*!< Number of domains at the KEEPRUNNING register: 0..15                 */
#define GRTC_NDOMAIN_MAX 15                          /*!< Number of domains at the KEEPRUNNING register: 0..15                 */
#define GRTC_NDOMAIN_SIZE 16                         /*!< Number of domains at the KEEPRUNNING register: 0..15                 */
#define GRTC_GRTC_NINTERRUPTS_MIN 0                  /*!< Number of GRTC interrupts : 0..3                                     */
#define GRTC_GRTC_NINTERRUPTS_MAX 3                  /*!< Number of GRTC interrupts : 0..3                                     */
#define GRTC_GRTC_NINTERRUPTS_SIZE 4                 /*!< Number of GRTC interrupts : 0..3                                     */
#define GRTC_PWMREGS 1                               /*!< (unspecified)                                                        */
#define GRTC_CLKOUTREG 1                             /*!< (unspecified)                                                        */
#define GRTC_CLKSELREG 1                             /*!< (unspecified)                                                        */
#define GRTC_CLKSELLFLPRC 0                          /*!< (unspecified)                                                        */
#define GRTC_CCADD_WRITE_ONLY 0                      /*!< (unspecified)                                                        */

/*Comparator*/
#define COMP_PRESENT 1
#define COMP_COUNT 1

/*Low-power comparator*/
#define LPCOMP_PRESENT 1
#define LPCOMP_COUNT 1

/*Watchdog Timer*/
#define WDT_PRESENT 1
#define WDT_COUNT 2

#define WDT30_ALLOW_STOP 1                           /*!< (unspecified)                                                        */

#define WDT31_ALLOW_STOP 1                           /*!< (unspecified)                                                        */

/*Clock management*/
#define CLOCK_PRESENT 1
#define CLOCK_COUNT 1

#define CLOCK_XOTUNE 1                               /*!< (unspecified)                                                        */

/*Power control*/
#define POWER_PRESENT 1
#define POWER_COUNT 1

#define POWER_CONSTLATSTAT 1                         /*!< (unspecified)                                                        */

/*Reset control*/
#define RESET_PRESENT 1
#define RESET_COUNT 1

/*Oscillator control*/
#define OSCILLATORS_PRESENT 1
#define OSCILLATORS_COUNT 1

/*Voltage regulators*/
#define REGULATORS_PRESENT 1
#define REGULATORS_COUNT 1


#ifdef __cplusplus
}
#endif
#endif /* NRF54L15_FLPR_PERIPHERALS_H */

