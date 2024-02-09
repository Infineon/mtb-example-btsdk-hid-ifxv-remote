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
#ifndef IFXV_OPUS_H__
#define IFXV_OPUS_H__

#include "celt_encoder_api.h"
#ifdef CODEC_DECODE_SUPPORT
#include "celt_decoder_api.h"
#endif
#define AUDIO_CODEC_CAPABILITY          IFX_AUDIO_OPUS_CAP
#define AUDIO_DEVICE_CODEC              IFX_AUDIO_CFG_CODEC_OPUS
#define AUDIO_ENC_TYPE                  AUDIO_ENC_TYPE_CELT
#define ADUIO_SAMPLE_RATE_CAPABILITY    IFX_AUDIO_16KHZ
#define HCI_CONTROL_HIDD_AUDIO          (HCI_CONTROL_HIDD_AUDIO_SUPPORT | HCI_CONTROL_HIDD_AUDIO_OPUS | HCI_CONTROL_HIDD_AUDIO_16K)

#define AUDIO_SAMPLE_CNT        OPUS_ENC_FRAME_SIZE    // 320

#define OPUS_ENC_FRAME_SIZE     (OPUS_ENC_SAMPLING_RATE/50) // must be equal to (sampling_rate / N), (sampling_rate / 50) = 16000/50=320
                                                            // where N = 400, 200, 100, 50, 25, 50/3
#define OPUS_ENC_SAMPLING_RATE  16000                       // 16K samples per second means 1 frame of 320 samples for every 20 ms
#define OPUS_ENC_BITRATE        32000                       // 16000 samples per sec x 16 bit = 256k bit per second. 1:8 compression, 256k/8, 32k effective bitrate
#define OPUS_ENC_OFFSET         2
#define OPUS_ENC_HEADER_SIZE    8
#define OPUS_ENC_MS_PER_FRAME   20                           // AUDIO_SAMPLE_CNT=320, with 16kHz, we get 20 ms per frame
#define OPUS_ENC_DATA_SIZE      (OPUS_ENC_BITRATE * OPUS_ENC_MS_PER_FRAME / 1000 / 8) // 32000 * 20ms / 1000 / 8 = 80
#define OPUS_ENC_PACKET_SIZE    OPUS_ENC_DATA_SIZE
#define OPUS_ENC_SIZE           (OPUS_ENC_DATA_SIZE + OPUS_ENC_HEADER_SIZE)           // DATA + 8-byte header = 88
#define ENCODE_BUF_SIZE         (OPUS_ENC_OFFSET + OPUS_ENC_SIZE)                     // With IFXV header, the encoded buffer size is 90

// FRAME FORMAT
//--------------- 2-byte IFXV Header
// uint8_t seq
// uint8_t channel
//----------------            <--- pass this offset to CELT encoder
// uint32_t len                    big-endian
// uint32_t enc_final_range        big-endian
// ---------------            <--- HEADER SIZE 10
// uint8_t [OPUS_MAX_DATA_SIZE]    CELT data (max 80 variable length)
//

void     ifxv_opus_init();
uint16_t ifxv_opus_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_pkt);
uint16_t ifxv_opus_decode(uint8_t * p_pkt, uint16_t sample_cnt, int16_t * p_pcm);

#endif // IFXV_OPUS_H__
