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
 * mic.c
 *
 * This file contains MIC functions
 *
 */

#ifdef SUPPORT_AUDIO
#include "app.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if MIC_TRACE
 #define APP_MIC_TRACE WICED_BT_TRACE
#else
 #define APP_MIC_TRACE(...)
#endif

// Digital MIC port defines: PDM_CLK and PDM_DATA should defined in platform. If it is not defined, we will define it here
#ifndef PDM_CLK
#define PDM_CLK     WICED_P27
#endif
#ifndef PDM_DATA
#define PDM_DATA    WICED_P26
#endif

#define AUDIO_FIFO_CNT                  4
#define AUDIO_START_DELAY_IN_MS         100    // wait for 100 ms to start send audio data

/*****************************************************************************
 * Application Audio config
 ****************************************************************************/
// data defines
hidd_voice_report_t audioData[AUDIO_FIFO_CNT] = {0};
uint16_t            dataCount[AUDIO_FIFO_CNT] = {0};

static hidd_microphone_config_t audio_cfg = {
    .mic_codec = NULL,
    .audio_fifo = audioData,
    .data_count = dataCount,
    .audio_delay = AUDIO_START_DELAY_IN_MS,
    .fifo_count = AUDIO_FIFO_CNT,
    .audio_gain = AUDIO_ADC_GAIN_IN_DB,
    .codec_sampling_freq = HIDD_CODEC_SAMP_FREQ_16K,
    .enable = TRUE,
    .audio_boost = TRUE,               // not used
};

static hidd_microphone_enhanced_config_t enhanced_audio_cfg =
{
    //# audio enc type: 0=PCM, 1=mSBC, 2=OPUS CELT, 3=ADPCM
    .audioEncType = AUDIO_ENC_TYPE,
    .drcSettings = {
        //# DRC settings
        .enable = 1,            // 1 Enable DRC, 0 Disable DRC
        .waitTime = 0x02EE,     // Wait time in mSec, 0x2EE = 750 mSec.
        .knee1 = 70,            // Knee 1, 68.5dB,       2660, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
        .knee2 = 85,            // Knee 2, 75dB,         5623, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
        .knee3 = 95,            // Knee 3, 81dB,        11220, in 1/2 dB steps.  10^((RSSI_target/2 + 30)/20).
        .attackTime = 0x03E8,   // Attack time in mSec.  0x03E8 = 1000 mSec
        .decayTime = 0x001F,    // Decay time in mSec.  0x001F = 31 mSec.
        .saturationLevel = 0x6800, // Saturation Level, 0x6800 = 26624.  This will be the max output level.
                   // The DRC will behave like an AGC when the DRC curve exceeds this amount.
                   // This value will be used when the pga gain is set lower than 18dB by the DRC loop.
    },
    //# DRC custom gain boost. Default value = 1000
    .custom_gain_boost = 1496,
    //# End of DRC settings

#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
    //#Anti Alias Audio Filter Coefficients.  Set index 0 to 0x00 0x00 for default filter settings
    .audioFilterData = {
        .audio_aux_filter_coef = {
            0x6D00,  //#index 0
            0xB5FF,  //#index 1
            0x23FF,  //#index 2
            0xE4FF,  //#index 3
            0x1B01,  //#index 4
            0x4E00,  //#index 5
            0x4EFE,  //#index 6
            0x10FF,  //#index 7
            0x3E02,  //#index 8
            0xEC01,  //#index 9
            0x2BFD,  //#index 10
            0x70FC,  //#index 11
            0x5C03,  //#index 12
            0x6606,  //#index 13
            0x36FC,  //#index 14
            0x87F3,  //#index 15
            0x1204,  //#index 16
            0x5D28,  //#index 17
            0xD53B,  //#index 18
        },
        .biQuadEqFilterCoeffs = {},

    /*# EQ Filter 1, 116 Coefficients(int16_t)
    #        1       2          3        4         5          6         7         8        9         10
    #    LSB  MSB LSB  MSB   LSB  MSB LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB  LSB  MSB
    #   --------- --------- --------- --------- --------- --------- --------- --------- --------- ---------
    #   To disable EQ filter, set the first two bytes below to 0.
    #   Customer should fill in the actually EQ coeff's based on specific test setup and HW */
        .eqFilter = {},
    }
#endif
};

/////////////////////////////////////////////////////////////////////////////////
/// This function physically stops audio
/////////////////////////////////////////////////////////////////////////////////
void mic_stop(void)
{
    APP_MIC_TRACE("%s", __FUNCTION__);

    //stop audio codec
    if( audio_is_active() )
    {
        mic_audio_stop();
        APP_MIC_TRACE("overflow = %d", mic_audio_is_overflow());
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function physically starts audio
/////////////////////////////////////////////////////////////////////////////////
void mic_start(void)
{
    APP_MIC_TRACE("%s", __FUNCTION__);

    mic_audio_set_active(WICED_TRUE);
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// This function will be called from app_start() during start up.
/////////////////////////////////////////////////////////////////////////////////////////////
void mic_init(void (*activityDetectedPtr)())
{
    APP_MIC_TRACE("%s", __FUNCTION__);
#ifdef SUPPORT_DIGITAL_MIC
    APP_MIC_TRACE("Digital MIC clk=p%d, data=p%d", PDM_CLK, PDM_DATA);
    /* assign PDM pins */
    mic_assign_mic_pdm_pins(PDM_CLK, PDM_DATA); // pdm clk & data pin assignment
#endif

    /* assign Audio Enable gpio pin if it is used */
    //mic_assign_mic_en_pin(WICED_P37, 0);

    mic_audio_config(&audio_cfg);
    mic_audio_config_enhanced((uint8_t *)&enhanced_audio_cfg);
    mic_audio_init(activityDetectedPtr, NULL);
}

#endif // SUPPORT_AUDIO
