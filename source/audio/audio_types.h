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
 * Bluetooth LE Remote audio functions
 *
 * This file provides definitions and function prototypes for LE remote control
 * device
 *
 */
#ifndef AUDIO_TYPES_H__
#define AUDIO_TYPES_H__

#include "wiced_bt_types.h"

#ifdef SUPPORT_AUDIO
#define audio_is_active()  mic_audio_is_active()
#else
#define audio_is_active()  FALSE
#endif

enum hidd_audio_encoding_e
{
    AUDIO_ENC_TYPE_PCM  = 0,
    AUDIO_ENC_TYPE_MSBC = 1,
    AUDIO_ENC_TYPE_CELT = 2,
    AUDIO_ENC_TYPE_ADPCM = 3,
    AUDIO_ENC_TYPE_INVALID = 0xFF  // keep this at the end
};
typedef uint8_t hidd_audio_encoding_t;

enum sample_freq_e
{
    HIDD_CODEC_SAMP_FREQ_8K = 0,
    HIDD_CODEC_SAMP_FREQ_16K = 1,
};

enum hidd_audio_codec_param_e
{
    HIDD_CODEC_SR   = 0,
    HIDD_CODEC_PCM  = 1,
    HIDD_CODEC_PGA  = 2,
    HIDD_CODEC_BPS  = 3,
    HIDD_CODEC_HPF  = 4,
};

typedef int16_t pcm_data_t;
typedef void (*pcm_data_available_cb_t) (pcm_data_t * p_pcm, uint16_t cnt);

#define AUDIO_BYTE_CNT (AUDIO_SAMPLE_CNT * sizeof(pcm_data_t))

#endif // AUDIO_TYPES_H__
