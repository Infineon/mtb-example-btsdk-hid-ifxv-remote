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
#ifndef IFXV_PCM_H__
#define IFXV_PCM_H__

#define AUDIO_CODEC_CAPABILITY          IFX_AUDIO_CFG_CODEC_NONE
#define AUDIO_DEVICE_CODEC              IFX_AUDIO_CFG_CODEC_NONE
#define AUDIO_ENC_TYPE                  AUDIO_ENC_TYPE_PCM
#define ADUIO_SAMPLE_RATE_CAPABILITY    IFX_AUDIO_16KHZ
#define HCI_CONTROL_HIDD_AUDIO          (HCI_CONTROL_HIDD_AUDIO_SUPPORT | HCI_CONTROL_HIDD_AUDIO_16K)

// In ADPCM, we use 256 16-bit samples per transaction.
#define AUDIO_SAMPLE_CNT      122
// We have 6 bytes header
#define PCM_FRAME_SEQ         0
#define PCM_FRAME_CH          1
#define PCM_DATA_OFFSET       2
#define PCM_HEADER_SIZE       PCM_DATA_OFFSET
// The compressioin rate is 4:1. Therefore, the encoded buffer size should be (256 * 2) / 4 + 6 = 128 + 6 = 134
#define ENCODE_BUF_SIZE         (AUDIO_SAMPLE_CNT*2 + PCM_HEADER_SIZE)

// FRAME FORMAT
//--------------- 2-byte IFXV Header
// uint8_t seq
// uint8_t channel
//----------------            <--- HEADER SIZE 2
// uint8_t pcm_data[]
//

void     ifxv_pcm_init();
uint16_t ifxv_pcm_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_frame);
uint16_t ifxv_pcm_decode(uint8_t * p_frame, uint16_t sample_cnt, int16_t * p_pcm);

#endif // IFXV_PCM_H__
