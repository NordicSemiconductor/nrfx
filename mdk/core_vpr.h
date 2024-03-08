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

#ifndef _CORE_VPR_H
#define _CORE_VPR_H

#ifndef __ASSEMBLY__
    #include <stdint.h>
#endif

#include "riscv_encoding.h"

#define __MTVT_PRESENT 1

/* =========================================================================================================================== */
/* ================                                  CSR declaration                                          ================ */
/* =========================================================================================================================== */
#define CSR_MCYCLE        0xB00
#define CSR_MINSTRET      0xB02
#define CSR_MCYCLEH       0xB80
#define CSR_MINSTRETH     0xB82
#define CSR_MVENDORID     0xF11
#define CSR_MARCHID       0xF12
#define CSR_MIMPID        0xF13
#define CSR_MHARTID       0xF14
#define CSR_MSTATUS       0x300
#define CSR_MISA          0x301
#define CSR_MEDELEG       0x302
#define CSR_MIDELEG       0x303
#define CSR_MIE           0x304
#define CSR_MTVEC         0x305
#define CSR_MCOUNTEREN    0x306
#define CSR_MTVT          0x307
#define CSR_MSCRATCH      0x340
#define CSR_MEPC          0x341
#define CSR_MCAUSE        0x342
#define CSR_MTVAL         0x343
#define CSR_MIP           0x344
#define CSR_MNXTI         0x345
#define CSR_MINTSTATUS    0x346
#define CSR_MSCRATCHCSW   0x348
#define CSR_MSCRATCHCSWL  0x349
#define CSR_MCLICBASE     0x350
#define CSR_DCSR          0x7b0
#define CSR_DPC           0x7b1
#define CSR_UCYCLE        0xc00
#define CSR_UINSTRET      0xc02
#define CSR_UCYCLEH       0xc80
#define CSR_UINSTRETH     0xc82

// Nordic custom CSRs
#define CSR_NORDIC_CTRL   0x7c0
#define CSR_NORDIC_SLEEP  0x7c1
#define CSR_NORDIC_DISFTR 0x7c2
#define CSR_NORDIC_VIOPINS 0x7c3

#define VTIM_CNTMODE0     0x7d0
#define VTIM_CNTMODE1     0x7d1
#define VTIM_CNT          0x7d2
#define VTIM_CNTTOP       0x7d3
#define VTIM_CNTADD       0x7d4
#define VTIM_CNT0         0x7d5
#define VTIM_CNTADD0      0x7d6
#define VTIM_CNT1         0x7d7
#define VTIM_CNTADD1      0x7d8

#define VIO_OUT           0x7e0
#define VIO_DIR           0x7e1
#define VIO_IN            0x7e2
#define VIO_INMODE        0x7e3
#define VIO_OUTB          0x7e4
#define VIO_DIRB          0x7e5
#define VIO_DIROUT        0x7e6
#define VIO_DIROUTB       0x7e7
#define VIO_OUTTGL        0x7e8
#define VIO_DIRTGL        0x7e9
#define VIO_OUTBTGL       0x7ea
#define VIO_DIRBTGL       0x7eb
#define VIO_DIROUTTGL     0x7ec
#define VIO_DIROUTBTGL    0x7ed
#define VIO_OUTBS         0x7ee
#define VIO_DIRBS         0x7ef  
#define VIO_DIROUTBS      0x7f0

#define VEVIF_VTASKS      0x7f1
#define VEVIF_VSUBSCRIBE  0x7f2
#define VEVIF_VEVENTS     0x7f3
#define VEVIF_VPUBLISH    0x7f4
#define VEVIF_VINTEN      0x7f5
#define VEVIF_VEVENTSB    0x7f6
#define VEVIF_VEVENTSBS   0x7f7

#endif /* _CORE_VPR_H */