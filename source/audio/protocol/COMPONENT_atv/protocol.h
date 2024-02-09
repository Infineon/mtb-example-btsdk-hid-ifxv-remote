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
#ifndef AUDIO_PROTOCOL_H__
#define AUDIO_PROTOCOL_H__

#include "atv.h"

#define AUDIO_COPY_SIZE                     app_atvs_atv_read_char_len
#define AUDIO_MTU_BASIC_SIZE                20

#define protocol_start()                    atv_send_cmd(ATV_VOICE_SERVICE_AUDIO_START)
#define protocol_stop()
#define protocol_stop_request()             atv_stop_request()
#define protocol_start_request()            atv_start_request()
#define protocol_send_data(p,l)             atv_send_data(p, l)
#define protocol_audio_data_max_size()      (atv_supports_ver(ATV_V1_00) ? AUDIO_COPY_SIZE : AUDIO_MTU_BASIC_SIZE)
#define is_protocol_handle(h)               is_atv_handle(h)
#define protocol_gatt_write_handler(i, d)   WICED_BT_GATT_NOT_FOUND
#define protocol_write_handler(i, d)        atv_gatts_req_write_callback(i, d)
#define protocol_init()                     atv_init()

#endif // AUDIO_PROTOCOL_H__
