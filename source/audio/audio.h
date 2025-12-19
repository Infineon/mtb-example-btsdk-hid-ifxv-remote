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
