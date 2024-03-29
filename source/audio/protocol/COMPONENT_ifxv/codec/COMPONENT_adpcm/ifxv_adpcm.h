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
#ifndef IFXV_ADPCM_H__
#define IFXV_ADPCM_H__

#include "adpcm.h"

#define AUDIO_CODEC_CAPABILITY          IFX_AUDIO_ADPCM_CAP
#define AUDIO_DEVICE_CODEC              IFX_AUDIO_CFG_CODEC_ADPCM
#define AUDIO_ENC_TYPE                  AUDIO_ENC_TYPE_ADPCM
#define ADUIO_SAMPLE_RATE_CAPABILITY    (IFX_AUDIO_8KHZ | IFX_AUDIO_16KHZ)
#define HCI_CONTROL_HIDD_AUDIO          (HCI_CONTROL_HIDD_AUDIO_SUPPORT | HCI_CONTROL_HIDD_AUDIO_ADPCM | HCI_CONTROL_HIDD_AUDIO_8K | HCI_CONTROL_HIDD_AUDIO_16K)

// In ADPCM, we use 256 16-bit samples per transaction.
#define AUDIO_SAMPLE_CNT        320
// We have 6 bytes header
#define ADPCM_HEADER_SIZE       6
// The compressioin rate is 4:1. Therefore, the encoded buffer size should be (256 * 2) / 4 + 6 = 128 + 6 = 134
#define ENCODE_BUF_SIZE         (AUDIO_SAMPLE_CNT/2 + ADPCM_HEADER_SIZE)

// FRAME FORMAT
//--------------- 2-byte IFXV Header
// uint8_t seq
// uint8_t channel
//----------------            <---
// int16_t len                     little-endian
// uint8_t index
// uint8_t not used                reserved
//----------------            <--- HEADER SIZE 6, pass this offset to ADPCM encoder
// uint8_t adpcm_data[160]         Ratio 4:1, 640 byte -> 160 byte
//

void ifxv_adpcm_reset();
uint16_t ifxv_adpcm_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_frame);
uint16_t ifxv_adpcm_decode(uint8_t * p_frame, uint16_t sample_cnt, int16_t * p_pcm);

#endif // IFXV_ADPCM_H__
