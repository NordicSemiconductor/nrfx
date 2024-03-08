/*
 * Copyright (c) 2023 - 2024, Nordic Semiconductor ASA
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

#include <helpers/nrfx_ram_ctrl.h>

typedef struct
{
    uint8_t block:4;
    uint8_t section:4;
} ram_block_section_t;

typedef union
{
    ram_block_section_t decoded;
    uint8_t             raw;
} ram_unit_t;

#if !defined(NRF_MEMORY_RAM_BASE) && defined(NRF_MEMORY_RAM0_BASE)
#define NRF_MEMORY_RAM_BASE NRF_MEMORY_RAM0_BASE
#endif

#define RAM_NON_UNIFORM_SECTION_DECLARE(i, _block, _section) {.decoded = {_block, _section}}

#if defined(NRF51)
#define RAM_SECTION_UNIT_SIZE          8192
#define RAM_UNIFORM_BLOCKS             4
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 1
#define RAM_UNIFORM_SECTIONS_TOTAL     4
#error "Unsupported device."
#elif defined(NRF52805_XXAA) || defined(NRF52810_XXAA) || defined(NRF52811_XXAA)
#define RAM_SECTION_UNIT_SIZE          4096
#define RAM_UNIFORM_BLOCKS             3
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 2
#define RAM_UNIFORM_SECTIONS_TOTAL     6
#elif defined(NRF52820_XXAA)
#define RAM_SECTION_UNIT_SIZE          4096
#define RAM_UNIFORM_BLOCKS             4
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 2
#define RAM_UNIFORM_SECTIONS_TOTAL     8
#elif defined(NRF52832_XXAA)
#define RAM_SECTION_UNIT_SIZE          4096
#define RAM_UNIFORM_BLOCKS             8
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 2
#define RAM_UNIFORM_SECTIONS_TOTAL     16
#elif defined(NRF52833_XXAA)
#define RAM_SECTION_UNIT_SIZE          4096
#define RAM_UNIFORM_BLOCKS             8
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 2
#define RAM_UNIFORM_SECTIONS_TOTAL     16
#define RAM_NON_UNIFORM_BLOCK_INDEX    8
#define RAM_NON_UNIFORM_BLOCK_UNITS    8
#define RAM_NON_UNIFORM_SECTIONS                                                               \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 0), /* Section 0 of block 8 - 8 * 4 kB units. */ \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 1)  /* Section 1 of block 8 - 8 * 4 kB units. */
#elif defined(NRF52840_XXAA)
#define RAM_SECTION_UNIT_SIZE          4096
#define RAM_UNIFORM_BLOCKS             8
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 2
#define RAM_UNIFORM_SECTIONS_TOTAL     16
#define RAM_NON_UNIFORM_BLOCK_INDEX    8
#define RAM_NON_UNIFORM_BLOCK_UNITS    8
#define RAM_NON_UNIFORM_SECTIONS                                                               \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 0), /* Section 0 of block 8 - 8 * 4 kB units. */ \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 1), /* Section 1 of block 8 - 8 * 4 kB units. */ \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 2), /* Section 2 of block 8 - 8 * 4 kB units. */ \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 3), /* Section 3 of block 8 - 8 * 4 kB units. */ \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 4), /* Section 4 of block 8 - 8 * 4 kB units. */ \
    NRFX_LISTIFY(RAM_NON_UNIFORM_BLOCK_UNITS,                                                  \
                 RAM_NON_UNIFORM_SECTION_DECLARE, (,),                                         \
                 RAM_NON_UNIFORM_BLOCK_INDEX, 5)  /* Section 5 of block 8 - 8 * 4 kB units. */
#elif defined(NRF5340_XXAA_APPLICATION)
#define RAM_SECTION_UNIT_SIZE          4096
#define RAM_UNIFORM_BLOCKS             8
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 16
#define RAM_UNIFORM_SECTIONS_TOTAL     128
#elif defined(NRF5340_XXAA_NETWORK)
#define RAM_SECTION_UNIT_SIZE          4096
#define RAM_UNIFORM_BLOCKS             4
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 4
#define RAM_UNIFORM_SECTIONS_TOTAL     16
#elif defined(NRF54H20_ENGA_XXAA)
#define RAM_SECTION_UNIT_SIZE          (32UL * 1024UL)
#define RAM_UNIFORM_BLOCKS             1
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 1
#define RAM_UNIFORM_SECTIONS_TOTAL     1
#elif defined(NRF54L15_XXAA) || defined(NRF54L15_ENGA_XXAA)
#define RAM_SECTION_UNIT_SIZE          (16UL * 1024UL)
#define RAM_NON_UNIFORM_SECTIONS                                      \
    NRFX_LISTIFY(4, RAM_NON_UNIFORM_SECTION_DECLARE, (,),             \
                 0, 0), /* Section 0 of block 0 - 4 * 16 kB units. */ \
    NRFX_LISTIFY(4, RAM_NON_UNIFORM_SECTION_DECLARE, (,),             \
                 0, 1), /* Section 1 of block 0 - 4 * 16 kB units. */ \
    NRFX_LISTIFY(2, RAM_NON_UNIFORM_SECTION_DECLARE, (,),             \
                 1, 0), /* Section 0 of block 1 - 2 * 16 kB units. */ \
    NRFX_LISTIFY(2, RAM_NON_UNIFORM_SECTION_DECLARE, (,),             \
                 1, 1), /* Section 1 of block 1 - 2 * 16 kB units. */ \
    NRFX_LISTIFY(1, RAM_NON_UNIFORM_SECTION_DECLARE, (,),             \
                 1, 2), /* Section 2 of block 1 - 1 * 16 kB units. */ \
    NRFX_LISTIFY(1, RAM_NON_UNIFORM_SECTION_DECLARE, (,),             \
                 1, 3)  /* Section 3 of block 1 - 1 * 16 kB units. */
#elif defined(NRF9120_XXAA) || defined(NRF9160_XXAA)
#define RAM_SECTION_UNIT_SIZE          8192
#define RAM_UNIFORM_BLOCKS             8
#define RAM_UNIFORM_SECTIONS_PER_BLOCK 4
#define RAM_UNIFORM_SECTIONS_TOTAL     32
#elif !defined(RAM_SECTION_UNIT_SIZE)
#error "Unsupported device."
#endif

#define RAM_UNIFORM_SECTION_DECLARE(i, ...) \
    {.decoded = {i / RAM_UNIFORM_SECTIONS_PER_BLOCK, i % RAM_UNIFORM_SECTIONS_PER_BLOCK}}

#if defined(RAM_NON_UNIFORM_SECTIONS)
static const ram_unit_t unit_to_block_section_lut[] =
{
#if defined(RAM_UNIFORM_SECTIONS_TOTAL)
    NRFX_LISTIFY(RAM_UNIFORM_SECTIONS_TOTAL, RAM_UNIFORM_SECTION_DECLARE, (,)),
#endif
    RAM_NON_UNIFORM_SECTIONS,
};
#endif

typedef void (* ram_ctrl_block_section_op_t)(uint8_t  block_idx,
                                             uint32_t section_mask,
                                             bool     enable);

static void ram_ctrl_block_section_power_enable_set(uint8_t  block_idx,
                                                    uint32_t section_mask,
                                                    bool     enable)
{
    nrfx_ram_ctrl_section_power_mask_enable_set(block_idx, section_mask, enable);
}

static void ram_ctrl_block_section_retention_enable_set(uint8_t  block_idx,
                                                        uint32_t section_mask,
                                                        bool     enable)
{
    nrfx_ram_ctrl_section_retention_mask_enable_set(block_idx, section_mask, enable);
}

static void ram_ctrl_block_section_iterate(void const *                p_object,
                                           size_t                      length,
                                           bool                        enable,
                                           ram_ctrl_block_section_op_t handler)
{
    NRFX_ASSERT(nrfx_is_in_ram(p_object));
    NRFX_ASSERT(length);

    size_t rel_obj_adr    = (size_t)p_object - NRF_MEMORY_RAM_BASE;
    size_t obj_start_addr = rel_obj_adr;
    size_t obj_end_addr   = rel_obj_adr + length;

    /* Handle case when the object is aligned to section boundaries,
     * which could cause additional section being incorrectly iterated over. */
    obj_end_addr--;

    size_t ram_unit_start_idx = obj_start_addr / RAM_SECTION_UNIT_SIZE;
    size_t ram_unit_end_idx   = obj_end_addr / RAM_SECTION_UNIT_SIZE;

    uint8_t block;
    uint8_t section;
    ram_unit_t prev_ram_unit = {.raw = UINT8_MAX};
    for (size_t idx = ram_unit_start_idx; idx <= ram_unit_end_idx; idx++)
    {
#if defined(RAM_NON_UNIFORM_SECTIONS)
        ram_unit_t const * p_ram_unit = &unit_to_block_section_lut[idx];
        block = p_ram_unit->decoded.block;
        section = p_ram_unit->decoded.section;

        ram_unit_t prev_ram_unit_copy = prev_ram_unit;
        prev_ram_unit.raw = p_ram_unit->raw;
        if (p_ram_unit->raw == prev_ram_unit_copy.raw)
        {
            continue;
        }
#else
        block   = (uint8_t)idx / RAM_UNIFORM_SECTIONS_PER_BLOCK;
        section = idx % RAM_UNIFORM_SECTIONS_PER_BLOCK;
        (void)prev_ram_unit;
#endif
        handler(block, 1UL << section, enable);
    }
}

void nrfx_ram_ctrl_power_enable_set(void const * p_object, size_t length, bool enable)
{
    ram_ctrl_block_section_iterate(p_object,
                                   length,
                                   enable,
                                   ram_ctrl_block_section_power_enable_set);
}

void nrfx_ram_ctrl_retention_enable_set(void const * p_object, size_t length, bool enable)
{
    ram_ctrl_block_section_iterate(p_object,
                                   length,
                                   enable,
                                   ram_ctrl_block_section_retention_enable_set);
}
