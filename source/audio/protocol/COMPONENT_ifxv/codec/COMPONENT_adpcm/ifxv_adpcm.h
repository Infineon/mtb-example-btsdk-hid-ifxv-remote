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
