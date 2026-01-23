/*
 * Copyright (c) 2017 - 2026, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <nrfx.h>
#include "nrfx_prs.h"

#define NRFX_LOG_MODULE PRS
#include <nrfx_log.h>

#define LOG_FUNCTION_EXIT(level, ret_code)            \
    NRFX_LOG_##level("Function: %s, error code: %s.", \
        __func__,                                     \
        NRFX_LOG_ERROR_STRING_GET(ret_code))

#define NRF_PRS_ELSE_IF(periph_name, prefix, i, p_reg) \
    else if (p_reg == NRFX_CONCAT(NRFX_PRS_BOX_, i, _ADDR))                   \
    {                                                                              \
        return &NRFX_CONCAT(m_prs_box_, i);                         \
    }
typedef struct {
    nrfx_irq_handler_t     handler;
    bool                   acquired;
    void *                 p_instance;
} prs_box_t;

#define PRS_BOX_DEFINE(periph_name, prefix, n, _)                \
    static prs_box_t m_prs_box_##n = { .handler = NULL,          \
                                       .acquired = false,        \
                                       .p_instance = NULL };     \
    void nrfx_prs_box_##n##_irq_handler(void)                    \
    {                                                            \
        NRFX_ASSERT(m_prs_box_##n.handler);                      \
        m_prs_box_##n.handler(m_prs_box_##n.p_instance);         \
    }

NRFX_FOREACH_ENABLED(PRS_BOX_, PRS_BOX_DEFINE, (), (), _)

static prs_box_t * prs_box_get(void const * p_base_addr)
{
    if (0) {}
    NRFX_FOREACH_ENABLED(PRS_BOX_, NRF_PRS_ELSE_IF, (), (), p_base_addr)
    else
    {
        return NULL;
    }
}

int nrfx_prs_acquire(void const *       p_base_addr,
                     nrfx_irq_handler_t irq_handler,
                     void *             p_instance)
{
    NRFX_ASSERT(p_base_addr);
    int ret_code;

    prs_box_t * p_box = prs_box_get(p_base_addr);
    if (p_box != NULL)
    {
        bool busy = false;

        NRFX_CRITICAL_SECTION_ENTER();
        if (p_box->acquired)
        {
            busy = true;
        }
        else
        {
            p_box->acquired   = true;
            p_box->handler    = irq_handler;
            p_box->p_instance = p_instance;
        }
        NRFX_CRITICAL_SECTION_EXIT();

        if (busy)
        {
            ret_code = -EBUSY;
            LOG_FUNCTION_EXIT(WARNING, ret_code);
            return ret_code;
        }
    }

    ret_code = 0;
    LOG_FUNCTION_EXIT(INFO, ret_code);
    return ret_code;
}

void nrfx_prs_release(void const * p_base_addr)
{
    NRFX_ASSERT(p_base_addr);

    prs_box_t * p_box = prs_box_get(p_base_addr);
    if (p_box != NULL)
    {
        p_box->handler  = NULL;
        p_box->acquired = false;
        p_box->p_instance = NULL;
    }
}
