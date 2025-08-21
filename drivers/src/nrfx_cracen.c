/*
 * Copyright (c) 2025, Nordic Semiconductor ASA
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

#if NRFX_CHECK(NRFX_CRACEN_ENABLED)

#include <hal/nrf_cracen.h>
#include <hal/nrf_cracen_rng.h>
#if NRF_CRACEN_HAS_CRYPTOMASTER
#include <hal/nrf_cracen_cm.h>
#include <helpers/nrf_cracen_cm_dma.h>
#endif
#include <soc/nrfx_coredep.h>

/* TRNG HW chosen configuration options */
#if defined(NRF54L15_XXAA) || defined(NRF54L10_XXAA) || defined(NRF54L05_XXAA)
#define TRNG_CLK_DIV                0
#else
#define TRNG_CLK_DIV                1
#endif
#define TRNG_OFF_TIMER_VAL          0
#define TRNG_INIT_WAIT_VAL        512
#define TRNG_NUMBER_128BIT_BLOCKS   4

#define TRNG_CONDITIONING_KEY_SIZE 4 /* Size of the conditioning key: 4 words, 16 bytes */

#define AES_ECB_BLK_SZ 16U /* 128 bits */

#define CTR_DRBG_MAX_BYTES_PER_REQUEST  (1 << 16)                            /* NIST.SP.800-90Ar1:Table 3 */
#if NRF_CRACEN_HAS_CRYPTOMASTER
#define CTR_DRBG_KEY_SIZE               32U                                  /* 256 bits AES Key */
#define CTR_DRBG_RESEED_INTERVAL        ((uint64_t)1 << 48)                  /* 2^48 as per NIST spec */
#define CTR_DRBG_ENTROPY_SIZE           (CTR_DRBG_KEY_SIZE + AES_ECB_BLK_SZ) /* Seed equals key-len + 16 */
#endif

/* Return values common between these driver internal functions: */
typedef enum {
    OK                =  0,  /* The function or operation succeeded */
    ERROR             = -1,  /* Generic error */
    ERR_TOO_BIG       = -2,  /* Requested size too large */
    TRNG_RESET_NEEDED = -5,  /* TRNG needs to be reset due to a failure */
    HW_PROCESSING     = -10, /* Waiting for the hardware to produce data */
} cracen_ret_t;

/* Internal status of the CTR_DRBG RNG driver */
typedef struct {
#if NRF_CRACEN_HAS_CRYPTOMASTER
    uint8_t          key[CTR_DRBG_KEY_SIZE];
    uint8_t          value[AES_ECB_BLK_SZ];
    uint64_t         reseed_counter;
#endif
    nrfx_drv_state_t initialized;
    bool             trng_conditioning_key_set;
} nrfx_cracen_cb_t;

static nrfx_cracen_cb_t m_cb;

/*
 * Initialize the TRNG HW and the TRNG driver status.
 */
static void trng_init(void)
{
    m_cb.trng_conditioning_key_set = false;

    /* Disable and softreset the RNG */
    static const nrf_cracen_rng_control_t control_reset = {.soft_reset = true};

    nrf_cracen_rng_control_set(NRF_CRACENCORE, &control_reset);

    /* Change from configuration defaults to what we prefer: */
#if NRF_CRACEN_RNG_HAS_IDLE_TIMER
    nrf_cracen_rng_off_timer_set(NRF_CRACENCORE, TRNG_OFF_TIMER_VAL);
#endif
    nrf_cracen_rng_clk_div_set(NRF_CRACENCORE, TRNG_CLK_DIV);
    nrf_cracen_rng_init_wait_val_set(NRF_CRACENCORE, TRNG_INIT_WAIT_VAL);

    /* Configure the control register and enable */
    static const nrf_cracen_rng_control_t control_enable = {
            .enable = true,
            .number_128_blocks = TRNG_NUMBER_128BIT_BLOCKS,
#if NRF_CRACEN_RNG_HAS_BLENDING
            .blending_method = NRF_CRACEN_RNG_BLENDING_CONCATENATION,
#endif
    };

    nrf_cracen_rng_control_set(NRF_CRACENCORE, &control_enable);
}

/*
 * Set the TRNG HW conditioning key.
 *
 * If there is not yet enough data to do so, return HW_PROCESSING otherwise return OK.
 */
static cracen_ret_t trng_setup_conditioning_key(void)
{
    uint32_t level = nrf_cracen_rng_fifo_level_get(NRF_CRACENCORE);

    if (level < TRNG_CONDITIONING_KEY_SIZE)
    {
        return HW_PROCESSING;
    }

    for (uint8_t i = 0; i < TRNG_CONDITIONING_KEY_SIZE; i++)
    {
        uint32_t key;
        key = nrf_cracen_rng_fifo_get(NRF_CRACENCORE);
        nrf_cracen_rng_key_set(NRF_CRACENCORE, i, key);
    }

    m_cb.trng_conditioning_key_set = true;

    return OK;
}

/*
 * If the TRNG HW detected the entropy quality was not ok, return TRNG_RESET_NEEDED.
 * If the HW is still starting or there is not enough data, return HW_PROCESSING.
 * If the conditioning key is not yet setup, attempt to fill it or return HW_PROCESSING if
 * we don't have enough data to fill it yet.
 * If enough data is ready, fill the /p dst buffer with /p size bytes and return OK.
 */
static cracen_ret_t trng_get(uint8_t * dst, size_t size)
{
    /* Check that startup tests did not fail and we are ready to read data */
    switch (nrf_cracen_rng_fsm_state_get(NRF_CRACENCORE))
    {
        case NRF_CRACEN_RNG_FSM_STATE_ERROR:
            return TRNG_RESET_NEEDED;
        case NRF_CRACEN_RNG_FSM_STATE_RESET:
            return HW_PROCESSING;
        case NRF_CRACEN_RNG_FSM_STATE_STARTUP:
        default:
            break;
    }

    /* Program random key for the conditioning function */
    if (!m_cb.trng_conditioning_key_set)
    {
        cracen_ret_t status = trng_setup_conditioning_key();

        if (status != OK)
        {
            return status;
        }
    }

    uint32_t level = nrf_cracen_rng_fifo_level_get(NRF_CRACENCORE);

    if (size > level * 4) /* FIFO level in 4-byte words */
    {
        return HW_PROCESSING;
    }

    while (size)
    {
        uint32_t data = nrf_cracen_rng_fifo_get(NRF_CRACENCORE);

        for (int i = 0; i < 4 && size; i++, size--)
        {
            *dst = (uint8_t)(data & 0xFF);
            dst++;
            data >>= 8;
        }
    }

    return OK;
}

/*
 * @brief Function for filling a buffer with entropy from the CRACEN TRNG.
 *
 * When this function returns OK, \p size random bytes have been written to \p p_buf.
 *
 * Up to 64 bytes can be requested.
 * If more is requested the function will return ERR_TOO_BIG without copying any data.
 *
 * The entropy generated by this function is NIST800-90B and AIS31 compliant, and can be used to
 * seed FIPS 140-2 compliant pseudo random number generators.
 *
 * @note This function is blocking. It will take around a couple of tenths of microseconds to
 *       complete depending on the amount of bytes requested.
 *       (~40 microseconds for an nRF54L15 for the maximum 64 bytes)
 *
 * @note Note this is a quite power hungry operation.
 *
 * @note This function will enable and configure the CRACEN TRNG HW, wait until the entropy has been
 *       generated, copy it to the destination buffer and disable the HW.
 *       This function is meant as as internal utility of this driver but may be used by others with
 *       extra care, specially if some other component is using CRACEN.
 *
 * @param[out] p_buf Buffer into which to copy \p size bytes of entropy.
 * @param[in]  size  Number of bytes to copy.
 *
 * @return OK on success, ERR_TOO_BIG if the requested size is too big.
 */
static cracen_ret_t trng_entropy_get(uint8_t * p_buf, size_t size)
{
    /* Prevent sizes above the FIFO size to guarantee that the hardware will be able to provide the
     * requested bytes in one go. */
    if (size > NRF_CRACEN_RNG_FIFO_SIZE)
    {
        return ERR_TOO_BIG;
    }

    nrf_cracen_module_enable(NRF_CRACEN, NRF_CRACEN_MODULE_RNG_MASK);

    int ret = TRNG_RESET_NEEDED;

    while (true)
    {
        if (ret == TRNG_RESET_NEEDED)
        {
            trng_init();
        }
        ret = trng_get(p_buf, size);
        if (ret == OK)
        {
            break;
        }
#if defined(CONFIG_SOC_SERIES_BSIM_NRFXX)
        nrfx_coredep_delay_us(1);
#endif
    }

    nrf_cracen_module_disable(NRF_CRACEN, NRF_CRACEN_MODULE_RNG_MASK);

    return OK;
}

#if NRF_CRACEN_HAS_CRYPTOMASTER
/*
 * Check if the CryptoMaster is done.
 *
 * returns OK if done, HW_PROCESSING if not yet done, and ERROR on error.
 */
static cracen_ret_t cm_done_check(void)
{
    uint32_t ret;
    uint32_t busy;

    ret = nrf_cracen_cm_int_pending_get(NRF_CRACENCORE);

    if (ret & (NRF_CRACEN_CM_INT_FETCH_ERROR_MASK | NRF_CRACEN_CM_INT_PUSH_ERROR_MASK))
    {
        return ERROR;
    }

    busy = nrf_cracen_cm_status_get(NRF_CRACENCORE,
                                    (NRF_CRACEN_CM_STATUS_BUSY_FETCH_MASK |
                                     NRF_CRACEN_CM_STATUS_BUSY_PUSH_MASK  |
                                     NRF_CRACEN_CM_STATUS_PUSH_WAITING_MASK));

    if (busy)
    {
        return HW_PROCESSING;
    }

    return OK;
}

/*
 * Function for encrypting with AES-ECB the input data using the CRACEN CryptoMaster module.
 *
 * @note The key, input and output data are in big endian/cryptographic order. That is, input[0]
 *       corresponds to the highest byte of the 128bit input.
 *
 * @note The only failure one can normally expect are bus failures due to incorrect pointers.
 *
 * @note This function is meant to be used by the nrfx_random_ctr_drbg driver.
 *       If using it outside of this driver it must be used with care specially if any other
 *       component is using CRACEN.
 *
 * @note The key size needs to be supported by the CRACEN CryptoMaster AES engine.
 *
 * @param[in] p_key    Pointer to the key.
 * @param[in] key_size Size of the key in bytes (valid sizes 16, 24 or 32 => 128, 192 or 256 bits).
 * @param[in] p_input  Pointer to the input data (16 bytes/128 bits).
 * @param[in] p_output Pointer to the output data (16 bytes/128 bits).
 *
 * @return OK on success, ERROR on failure.
 */
static cracen_ret_t cm_aes_ecb(uint8_t * p_key, size_t key_size, uint8_t * p_input,
                               uint8_t * p_output)
{
    cracen_ret_t ret;

    static const uint32_t aes_config_value = NRF_CRACEN_CM_AES_CONFIG(
            NRF_CRACEN_CM_AES_CONFIG_MODE_ECB,
            NRF_CRACEN_CM_AES_CONFIG_KEY_SW_PROGRAMMED,
            false, false, false);

    struct nrf_cracen_cm_dma_desc in_descs[3];

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif /* __GNUC__ */
    in_descs[0].p_addr = (uint8_t *)&aes_config_value;
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif /* __GNUC__ */
    in_descs[0].length = sizeof(aes_config_value) | NRF_CRACEN_CM_DMA_DESC_LENGTH_REALIGN;
    in_descs[0].dmatag = NRF_CRACEN_CM_DMA_TAG_AES_CONFIG(NRF_CRACEN_CM_AES_REG_OFFSET_CONFIG);
    in_descs[0].p_next = &in_descs[1];

    in_descs[1].p_addr = p_key;
    in_descs[1].length = key_size | NRF_CRACEN_CM_DMA_DESC_LENGTH_REALIGN;
    in_descs[1].dmatag = NRF_CRACEN_CM_DMA_TAG_AES_CONFIG(NRF_CRACEN_CM_AES_REG_OFFSET_KEY);
    in_descs[1].p_next = &in_descs[2];

    in_descs[2].p_addr = p_input;
    in_descs[2].length = AES_ECB_BLK_SZ | NRF_CRACEN_CM_DMA_DESC_LENGTH_REALIGN;
    in_descs[2].dmatag = NRF_CRACEN_CM_DMA_TAG_LAST | NRF_CRACEN_CM_DMA_TAG_ENGINE_AES
                          | NRF_CRACEN_CM_DMA_TAG_DATATYPE_AES_PAYLOAD;
    in_descs[2].p_next = NRF_CRACEN_CM_DMA_DESC_STOP;

    struct nrf_cracen_cm_dma_desc out_desc;

    out_desc.p_addr = p_output;
    out_desc.length = AES_ECB_BLK_SZ | NRF_CRACEN_CM_DMA_DESC_LENGTH_REALIGN;
    out_desc.p_next = NRF_CRACEN_CM_DMA_DESC_STOP;
    out_desc.dmatag = NRF_CRACEN_CM_DMA_TAG_LAST;

    nrf_cracen_module_enable(NRF_CRACEN, NRF_CRACEN_MODULE_CRYPTOMASTER_MASK);

    nrf_cracen_cm_fetch_addr_set(NRF_CRACENCORE, (void *)in_descs);
    nrf_cracen_cm_push_addr_set(NRF_CRACENCORE, (void *)&out_desc);

    nrf_cracen_cm_config_indirect_set(NRF_CRACENCORE,(nrf_cracen_cm_config_indirect_mask_t)
                                                     (NRF_CRACEN_CM_CONFIG_INDIRECT_FETCH_MASK |
                                                      NRF_CRACEN_CM_CONFIG_INDIRECT_PUSH_MASK));

    /* Make sure the contents of in_descs and out_desc are updated before starting CryptoMaster. */
    __DMB();

    nrf_cracen_cm_start(NRF_CRACENCORE);

    do {
        /* The HW is so fast that it is better to "busy wait" here than program an
         * interrupt. This will normally already succeed in the first try
         */
#if defined(CONFIG_SOC_SERIES_BSIM_NRFXX)
        nrfx_coredep_delay_us(1);
#endif
        ret = cm_done_check();
    } while (ret == HW_PROCESSING);

    nrf_cracen_cm_softreset(NRF_CRACENCORE);
    nrf_cracen_module_disable(NRF_CRACEN, NRF_CRACEN_MODULE_CRYPTOMASTER_MASK);

    return ret;
}

/*
 * Increment by 1 a number stored in memory in big endian representation.
 * /p v is a pointer to the first byte storing the number.
 * /p size is the size of the number.
 */
static inline void be_incr(unsigned char * v, size_t size)
{
    unsigned int add = 1;

    do {
        size--;
        add += v[size];
        v[size] = add & 0xFF;
        add >>= 8;
    } while ((add != 0) && (size > 0));
}

/*
 * XOR two arrays of /p size bytes.
 * /p size must be a multiple of 4.
 */
static inline void xor_array(uint32_t * a, const uint32_t * b, size_t size)
{
    uint8_t * end = (uint8_t *)a + size;

    for (; (uint8_t *)a < end; a++, b++)
    {
        *a = *a ^ *b;
    }
}

/*
 * Implementation of the CTR_DRBG_Update process as described in NIST.SP.800-90Ar1 with ctr_len
 * equal to blocklen.
 *
 * Returns OK on success, ERROR on error.
 */
static cracen_ret_t ctr_drbg_update(uint8_t * data)
{
    int r = 0;
    uint8_t temp[CTR_DRBG_ENTROPY_SIZE];
    size_t temp_length = 0;

    while (temp_length < sizeof(temp))
    {
        be_incr(m_cb.value, AES_ECB_BLK_SZ);

        r = cm_aes_ecb(m_cb.key, sizeof(m_cb.key), m_cb.value, temp + temp_length);

        if (r != OK)
        {
            return ERROR;
        }
        temp_length += AES_ECB_BLK_SZ;
    }

    if (data)
    {
        xor_array((uint32_t *)temp, (uint32_t *)data, sizeof(temp));
    }

    memcpy(m_cb.key, temp, sizeof(m_cb.key));
    memcpy(m_cb.value, temp + sizeof(m_cb.key), sizeof(m_cb.value));

    return OK;
}

/*
 * Re-seed the CTR_DRBG.
 *
 * return OK on success, ERROR on error.
 */
static cracen_ret_t ctr_drbg_reseed(void)
{
    int r;
    uint8_t entropy[CTR_DRBG_ENTROPY_SIZE];

    /* Get the entropy used to seed the DRBG */
    r = trng_entropy_get(entropy, sizeof(entropy));
    if (r != OK)
    {
        return ERROR;
    }

    r = ctr_drbg_update(entropy);
    if (r != OK)
    {
        return ERROR;
    }

    m_cb.reseed_counter = 1;

    return OK;
}
#endif

nrfx_err_t nrfx_cracen_ctr_drbg_init(void)
{
    if (m_cb.initialized == NRFX_DRV_STATE_INITIALIZED)
    {
        return NRFX_ERROR_ALREADY;
    }

    memset(&m_cb, 0, sizeof(m_cb));

#if NRF_CRACEN_HAS_CRYPTOMASTER
    int r;

    r = ctr_drbg_reseed();
    if (r != OK)
    {
        return NRFX_ERROR_INTERNAL;
    }
#endif

    m_cb.initialized = NRFX_DRV_STATE_INITIALIZED;
    return NRFX_SUCCESS;
}

void nrfx_cracen_ctr_drbg_uninit(void)
{
    NRFX_ASSERT(m_cb.initialized != NRFX_DRV_STATE_UNINITIALIZED);

    m_cb.initialized = NRFX_DRV_STATE_UNINITIALIZED;
}

nrfx_err_t nrfx_cracen_ctr_drbg_random_get(uint8_t * p_buf, size_t size)
{
    NRFX_ASSERT(m_cb.initialized != NRFX_DRV_STATE_UNINITIALIZED);

    int r = 0;

    if (size > 0 && p_buf == NULL)
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

    if (size > CTR_DRBG_MAX_BYTES_PER_REQUEST )
    {
        return NRFX_ERROR_INVALID_PARAM;
    }

#if NRF_CRACEN_HAS_CRYPTOMASTER
    if (m_cb.reseed_counter >= CTR_DRBG_RESEED_INTERVAL)
    {
        r = ctr_drbg_reseed();
        if (r != OK)
        {
            return NRFX_ERROR_INTERNAL;
        }
    }

    while (size > 0)
    {
        uint8_t temp[AES_ECB_BLK_SZ];
        size_t cur_len = (size < AES_ECB_BLK_SZ) ? size : AES_ECB_BLK_SZ;

        be_incr(m_cb.value, AES_ECB_BLK_SZ);

        r = cm_aes_ecb(m_cb.key, sizeof(m_cb.key), m_cb.value, temp);
        if (r != OK)
        {
            return NRFX_ERROR_INTERNAL;
        }

        memcpy(p_buf, temp, cur_len);

        size -= cur_len;
        p_buf += cur_len;
    }

    r = ctr_drbg_update(NULL);
    if (r != OK)
    {
        return NRFX_ERROR_INTERNAL;
    }

    m_cb.reseed_counter += 1;
#else
    r = trng_entropy_get(p_buf, size);

    if (r != OK)
    {
        return NRFX_ERROR_INTERNAL;
    }
#endif
    return NRFX_SUCCESS;
}

#endif // NRFX_CHECK(NRFX_CRACEN_ENABLED)
