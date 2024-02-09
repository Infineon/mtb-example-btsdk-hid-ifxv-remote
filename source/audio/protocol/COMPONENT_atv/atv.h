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
