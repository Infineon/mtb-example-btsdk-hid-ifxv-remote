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
#include "adpcm_codec.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

static CodecState adpcm_enc;

#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
static CodecState adpcm_dec;
#endif

static uint8_t adpcm_sequence;

void ifxv_adpcm_reset()
{
    adpcm_sequence = 0;
    memset(&adpcm_enc, 0, sizeof(CodecState));
#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
    memset(&adpcm_dec, 0, sizeof(CodecState));
#endif
}

#ifdef CODEC_DECODE_SUPPORT   // need to encode and then decode before sending to HCI host
// return out data length
uint16_t ifxv_adpcm_decode(uint8_t * p_frame, uint16_t sample_cnt, int16_t * p_pcm)
{
    decode(&adpcm_dec, &p_frame[ADPCM_HEADER_SIZE], sample_cnt, p_pcm);
    return sample_cnt * 2;
}
#endif

// return out data length
uint16_t ifxv_adpcm_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_frame)
{
    if (sample_cnt > AUDIO_SAMPLE_CNT)
    {
        WICED_BT_TRACE("ADPCM encode error! sample cnt %d is too large", sample_cnt);
        return 0;
    }

    //header is little endian. i.e. 0x0001 => "01 00"
    p_frame[0] = adpcm_sequence++;
    p_frame[1] = 0; // Id
    p_frame[2] = adpcm_enc.valprev & 0xFF;
    p_frame[3] = (adpcm_enc.valprev >> 8) & 0xFF; //2 bytes Prev pred -- little-endian
    p_frame[4] = adpcm_enc.index & 0xFF;
    p_frame[5] = 0; // set to 0, future use

    encode(&adpcm_enc, p_pcm, sample_cnt, &p_frame[ADPCM_HEADER_SIZE]);
    return (sample_cnt/2) + ADPCM_HEADER_SIZE;  //6 bytes header + 128 bytes ADPCM data
}
