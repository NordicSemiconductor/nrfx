/*
 * Copyright (c) 2020 - 2023, Nordic Semiconductor ASA
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

#ifndef NRF_COMMON_H__
#define NRF_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NRFX_EVENT_READBACK_ENABLED
#define NRFX_EVENT_READBACK_ENABLED 1
#endif

#ifndef NRFX_CONFIG_API_VER_MAJOR
#define NRFX_CONFIG_API_VER_MAJOR 3
#endif

#ifndef NRFX_CONFIG_API_VER_MINOR
#define NRFX_CONFIG_API_VER_MINOR 0
#endif

#ifndef NRFX_CONFIG_API_VER_MICRO
#define NRFX_CONFIG_API_VER_MICRO 0
#endif

#if defined(ISA_RISCV)
#define RISCV_FENCE(p, s) __asm__ __volatile__ ("fence " #p "," #s : : : "memory")
#endif

#ifndef NRF_SUBSCRIBE_PUBLISH_ENABLE
#define NRF_SUBSCRIBE_PUBLISH_ENABLE (0x01UL << 31UL)
#endif

#if defined(NRFX_CLZ)
#define NRF_CLZ(value) NRFX_CLZ(value)
#elif defined(ISA_ARM)
#define NRF_CLZ(value) __CLZ(value)
#else
#define NRF_CLZ(value) __builtin_clz(value)
#endif

#if defined(NRFX_CTZ)
#define NRF_CTZ(value) NRFX_CTZ(value)
#elif defined(ISA_ARM)
#define NRF_CTZ(value) __CLZ(__RBIT(value))
#else
#define NRF_CTZ(value) __builtin_ctz(value)
#endif

/** @brief Macro for extracting relative pin number from the absolute pin number. */
#define NRF_PIN_NUMBER_TO_PIN(pin) ((pin) & 0x1F)

/** @brief Macro for extracting port number from the absolute pin number. */
#define NRF_PIN_NUMBER_TO_PORT(pin) ((pin) >> 5)

/** @brief Macro for extracting absolute pin number from the relative pin and port numbers. */
#define NRF_PIN_PORT_TO_PIN_NUMBER(pin, port) (((pin) & 0x1F) | ((port) << 5))

/**
 * @brief Function for checking if an object is accesible by EasyDMA of given peripheral instance.
 *
 * Peripherals that use EasyDMA require buffers to be placed in certain memory regions.
 *
 * @param[in] p_reg    Peripheral base pointer.
 * @param[in] p_object Pointer to an object whose location is to be checked.
 *
 * @retval true  The pointed object is located in the memory region accessible by EasyDMA.
 * @retval false The pointed object is not located in the memory region accessible by EasyDMA.
 */
NRF_STATIC_INLINE bool nrf_dma_accessible_check(void const * p_reg, void const * p_object);

NRF_STATIC_INLINE void nrf_barrier_w(void);

NRF_STATIC_INLINE void nrf_barrier_r(void);

NRF_STATIC_INLINE void nrf_barrier_rw(void);

#ifndef NRF_DECLARE_ONLY

NRF_STATIC_INLINE void nrf_event_readback(void * p_event_reg)
{
#if NRFX_CHECK(NRFX_EVENT_READBACK_ENABLED) && !defined(NRF51)
    (void)*((volatile uint32_t *)(p_event_reg));
#else
    (void)p_event_reg;
#endif
}

NRF_STATIC_INLINE void nrf_barrier_w(void)
{
#if defined(ISA_RISCV)
    RISCV_FENCE(ow, ow);
#endif
}

NRF_STATIC_INLINE void nrf_barrier_r(void)
{
#if defined(ISA_RISCV)
    RISCV_FENCE(ir, ir);
#endif
}

NRF_STATIC_INLINE void nrf_barrier_rw(void)
{
#if defined(ISA_RISCV)
    RISCV_FENCE(iorw, iorw);
#endif
}

#if defined(ADDRESS_DOMAIN_Msk)
NRF_STATIC_INLINE uint8_t nrf_address_domain_get(uint32_t addr)
{
    return (uint8_t)((addr & ADDRESS_DOMAIN_Msk) >> ADDRESS_DOMAIN_Pos);
}
#endif

#if defined(ADDRESS_REGION_Msk)
NRF_STATIC_INLINE nrf_region_t nrf_address_region_get(uint32_t addr)
{
    return (nrf_region_t)((addr & ADDRESS_REGION_Msk) >> ADDRESS_REGION_Pos);
}
#endif

#if defined(ADDRESS_SECURITY_Msk)
NRF_STATIC_INLINE bool nrf_address_security_get(uint32_t addr)
{
    return ((addr & ADDRESS_SECURITY_Msk) >> ADDRESS_SECURITY_Pos);
}
#endif

#if defined(ADDRESS_BUS_Msk)
NRF_STATIC_INLINE uint8_t nrf_address_bus_get(uint32_t addr, size_t size)
{
    return (uint8_t)((addr & ADDRESS_BUS_Msk & ~(size - 1)) >> ADDRESS_BUS_Pos);
}
#endif

#if defined(ADDRESS_BRIDGE_GROUP_Msk)
NRF_STATIC_INLINE uint8_t nrf_address_bridge_group_get(uint32_t addr)
{
    return (uint8_t)((addr & ADDRESS_BRIDGE_GROUP_Msk) >> ADDRESS_BRIDGE_GROUP_Pos);
}
#endif

#if defined(ADDRESS_DOMAIN_SPEED_Msk)
NRF_STATIC_INLINE nrf_domain_speed_t nrf_address_domain_speed_get(uint32_t addr)
{
    return (nrf_domain_speed_t)((addr & ADDRESS_DOMAIN_SPEED_Msk) >> ADDRESS_DOMAIN_SPEED_Pos);
}
#endif

#if defined(ADDRESS_SLAVE_Msk)
NRF_STATIC_INLINE uint8_t nrf_address_slave_get(uint32_t addr)
{
    return (uint8_t)((addr & ADDRESS_SLAVE_Msk) >> ADDRESS_SLAVE_Pos);
}
#endif

#if defined(ADDRESS_PERIPHID_Msk)
NRF_STATIC_INLINE uint16_t nrf_address_periphid_get(uint32_t addr)
{
    return (uint16_t)((addr & ADDRESS_PERIPHID_Msk) >> ADDRESS_PERIPHID_Pos);
}
#endif

NRF_STATIC_INLINE bool nrf_dma_accessible_check(void const * p_reg, void const * p_object)
{
#if defined(NRF_DMA_ACCESS_EXT)
    NRF_DMA_ACCESS_EXT
#else
    (void)p_reg;
    return ((((uint32_t)p_object) & 0xE0000000u) == 0x20000000u);
#endif
}

#endif // NRF_DECLARE_ONLY

#ifdef __cplusplus
}
#endif

#endif // NRF_COMMON_H__
