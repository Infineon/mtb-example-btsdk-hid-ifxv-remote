/*
 * (c) 2016-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
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
