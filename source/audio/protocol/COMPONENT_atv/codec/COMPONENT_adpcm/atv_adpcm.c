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
#include "adpcm_codec.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

static CodecState adpcm_enc;

#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
static CodecState adpcm_dec;
#endif

static uint16_t adpcm_sequence;

void atv_adpcm_reset()
{
    adpcm_sequence = 0;
    memset(&adpcm_enc, 0, sizeof(CodecState));
#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
    memset(&adpcm_dec, 0, sizeof(CodecState));
#endif
}

#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
// return out data length
uint16_t atv_adpcm_decode(uint8_t * p_frame, uint16_t sample_cnt, int16_t * p_pcm)
{
    decode(&adpcm_dec, p_frame, sample_cnt, p_pcm);
    return sample_cnt * 2;
}
#endif

// return out data length
uint16_t atv_adpcm_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_frame)
{
    if (sample_cnt > AUDIO_SAMPLE_CNT)
    {
        WICED_BT_TRACE("ADPCM encode error! sample cnt %d is too large", sample_cnt);
        return 0;
    }

    //header is big endian. i.e. 0x0001 => "00 01". Used by Android TV
    p_frame[0] = (adpcm_sequence >> 8) & 0xFF;  //2 bytes seq number
    p_frame[1] = adpcm_sequence & 0xFF;
    p_frame[2] = 0; //Id. <reserved>
    p_frame[3] = (adpcm_enc.valprev >> 8) & 0xFF; //2 bytes Prev pred
    p_frame[4] = adpcm_enc.valprev & 0xFF;
    p_frame[5] = adpcm_enc.index & 0xFF;

    encode(&adpcm_enc, p_pcm, sample_cnt, &p_frame[ADPCM_HEADER_SIZE]);
    adpcm_sequence++;

    return (sample_cnt/2) + ADPCM_HEADER_SIZE;  //6 bytes header + 128 bytes ADPCM data
}
