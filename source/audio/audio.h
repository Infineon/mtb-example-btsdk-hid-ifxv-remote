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
#ifndef AUDIO_H__
#define AUDIO_H__

#include "protocol.h"
#include "codec.h"
#include "mic.h"
#include "audio_types.h"
#include "audio_v.h"

#define AUDIO_ADC_GAIN_IN_DB            30     // Audio ADC Gain in dB. Valid range 0..42. When DRC is disabled 21 dB is recommended
#define AUDIO_PDM_MULTIPLIER            20     // Audio PDM Gain multiplier. PDM does not have hardware amplifier. We need to amplify in software.

void audio_init();
void audio_shutdown();
void audio_queue_stop_event();
void audio_start();
void audio_stop();
void audio_pollActivityVoice(void);
void audio_start_request();
void audio_stop_request();
void audio_procEvent(uint8_t eventType);
void audio_voiceCtlSend(uint8_t eventType);
void audio_button(wiced_bool_t down);
uint8_t audio_capability();
void audio_set_routing_to_hci(wiced_bool_t en, wiced_bool_t encoded);
void audio_set_routing_from_hci(wiced_bool_t en, wiced_bool_t encoded);
void audio_process_input_data(int16_t * ptr, uint16_t sample_cnt);
void audio_amplify_data(int16_t * p_data, uint16_t cnt, uint32_t multiplier);
uint16_t audio_wdata_cnt();
wiced_bool_t audio_data_from_hci(void);

#define audio_START()                  audio_start()
#define audio_STOP_REQ()               audio_stop_request()
#define audio_send_data(p,l)           protocol_send_data(p,l)
#define is_audio_handle(h)             is_protocol_handle(h)
#define audio_gatt_read_handler(i,d)   protocol_gatt_write_handler(i, d)
#define audio_gatt_write_handler(i,d)  protocol_write_handler(i, d)

#endif // AUDIO_H__
