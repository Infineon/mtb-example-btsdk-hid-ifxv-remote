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
 * Bluetooth LE Remote audio functions
 *
 * This file provides definitions and function prototypes for LE remote control
 * device
 *
 */
#ifndef GOOGLE_VOICE_H__
#define GOOGLE_VOICE_H__

#define ATV_V1_00 1

#define ATV_VOICE_SERVICE_GET_CAPS_REQUEST          0x0A
#define ATV_VOICE_SERVICE_MIC_OPEN                  0x0C
#define ATV_VOICE_SERVICE_MIC_CLOSE                 0x0D

#define ATV_VOICE_SERVICE_AUDIO_STOP                0x0
#define ATV_VOICE_SERVICE_AUDIO_START               0x04
#define ATV_VOICE_SERVICE_START_SEARCH              0x08
#define ATV_VOICE_SERVICE_AUDIO_SYNC                0x0A
#define ATV_VOICE_SERVICE_GET_CAPS_RESP             0x0B
#define ATV_VOICE_SERVICE_MIC_OPEN_ERROR            0x0C

#define is_atv_handle(handle)  ((handle >= HDLS_ATVS) && (handle <= HDLD_ATVS_ATV_CONTROL_CHAR_CCCD))

void atv_init();
void atv_send_cmd(uint8_t cmd);
void atv_stop_request(void);
void atv_start_request(void);
void atv_send_data(uint8_t * ptr, uint16_t len);
wiced_bool_t atv_supports_ver(uint8_t major);
wiced_bt_gatt_status_t atv_gatts_req_write_callback( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );

#endif // GOOGLE_VOICE_H__
