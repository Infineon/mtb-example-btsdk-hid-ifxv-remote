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

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if CODEC_TRACE
 #define APP_PCM_TRACE     WICED_BT_TRACE
#else
 #define APP_PCM_TRACE(...)
#endif

static uint8_t pcm_sequence;

void ifxv_pcm_init()
{
    APP_PCM_TRACE("%s",__FUNCTION__);
    pcm_sequence = 0;
}

#ifdef CODEC_DECODE_SUPPORT
// return number of samples
uint16_t ifxv_pcm_decode(uint8_t * p_frame, uint16_t frame_size, int16_t * p_pcm)
{
    memcpy(p_pcm, p_frame[PCM_DATA_OFFSET], frame_size);
    return frame_size/2;
}
#endif

// return frame size
uint16_t ifxv_pcm_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_frame)
{
    p_frame[PCM_FRAME_SEQ] = pcm_sequence++;
    p_frame[PCM_FRAME_CH] = 0;
    memcpy(&p_frame[PCM_DATA_OFFSET], p_pcm, sample_cnt*2);
    APP_PCM_TRACE("return size =%d", sample_cnt * 2 + PCM_HEADER_SIZE);
    return sample_cnt * 2 + PCM_HEADER_SIZE;
}
