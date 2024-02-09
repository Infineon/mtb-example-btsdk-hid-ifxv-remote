/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 */
#include "app.h"
#include "ifxv_opus.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if CODEC_TRACE == 1
 #define     IFXV_CODEC_TRACE       WICED_BT_TRACE
 #define     IFXV_CODEC_TRACE2(...)
#elif CODEC_TRACE == 2
 #define     IFXV_CODEC_TRACE       WICED_BT_TRACE
 #define     IFXV_CODEC_TRACE2      WICED_BT_TRACE
#else
 #define     IFXV_CODEC_TRACE(...)
 #define     IFXV_CODEC_TRACE2(...)
#endif

static CELT_ENC_PARAMS celt_enc_params = {
    .sampling_rate  = OPUS_ENC_SAMPLING_RATE,
    .channels       = 1,
    .bitrate        = OPUS_ENC_BITRATE,
    .complexity     = 3,
    .use_vbr        = 0,                    // must be 0 (CBR only)
    .use_cvbr       = 0,                    // must be 0 (CBR only)
    .frame_size     = OPUS_ENC_FRAME_SIZE,  // frame_size default to AUDIO_SAMPLE_CNT (16000/50=320)
    .enc_handler = NULL,
};

#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
static CELT_DEC_PARAMS celt_dec_params = {
    .sampling_rate = OPUS_ENC_SAMPLING_RATE,        /* 8k, 16k, 24k or 48k */
    .channels = 1,                                  /* mono or streo */
    .pkt_len = OPUS_ENC_PACKET_SIZE,                /* Input packet length (bytes) */
    .frame_status = 0,                              /* Frame status: 0:GOOD, 1:BAD(lost)  */
    .frame_size = OPUS_ENC_FRAME_SIZE,              /* PCM samples per frame per channel, needed for PLC init*/
    .enable_plc = 0,                                /* Enable PLC: 1:enable, 0:disable */
};
#endif

static uint8_t opus_sequence;

void ifxv_opus_init()
{
    opus_sequence = 0;

    IFXV_CODEC_TRACE("Opus Init");

    CELT_Encoder_Init(&celt_enc_params);
#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
    CELT_Decoder_Init(&celt_dec_params);
#endif
}

#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
// return out data length
uint16_t ifxv_opus_decode(uint8_t * p_pkt, uint16_t sample_cnt, int16_t * p_pcm)
{
    if (sample_cnt != AUDIO_SAMPLE_CNT)
    {
        WICED_BT_TRACE("OPUS encode error! sample cnt %d is not %d", sample_cnt, OPUS_ENC_FRAME_SIZE);
        return 0;
    }
    celt_dec_params.packet = &p_pkt[OPUS_ENC_OFFSET];
    celt_dec_params.pcmBuffer = p_pcm;
    uint16_t frame_size = CELT_Decoder(&celt_dec_params);  // returns number of samples

    if (frame_size != AUDIO_SAMPLE_CNT)
    {
        WICED_BT_TRACE("OPUS encode error! sample cnt %d is not %d after decode", frame_size, OPUS_ENC_FRAME_SIZE);
        return 0;
    }
    return frame_size * 2;
}
#endif

// return pkt length
uint16_t ifxv_opus_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_pkt)
{
    uint16_t len;

    if (sample_cnt != AUDIO_SAMPLE_CNT)
    {
        WICED_BT_TRACE("OPUS encode error! sample cnt %d is not %d", sample_cnt, OPUS_ENC_FRAME_SIZE);
        return 0;
    }

    //first byte is sequence number
    p_pkt[0] = opus_sequence++;
    p_pkt[1] = 0; // Id

    //followed by CELT encoded audio data
    celt_enc_params.pcmBuffer = p_pcm;
    celt_enc_params.packet = &p_pkt[OPUS_ENC_OFFSET];
    len = CELT_Encoder(&celt_enc_params); //encode one frame

    if (len > OPUS_ENC_SIZE)
    {
        WICED_BT_TRACE("OPUS encode error! encoded data len (%d) is larger than buffer size %d", len, OPUS_ENC_SIZE);
        len = OPUS_ENC_SIZE;
    }

    return len+OPUS_ENC_OFFSET;
}
