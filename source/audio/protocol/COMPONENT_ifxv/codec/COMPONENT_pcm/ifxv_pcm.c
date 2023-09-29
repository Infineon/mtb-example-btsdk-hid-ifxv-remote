/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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
