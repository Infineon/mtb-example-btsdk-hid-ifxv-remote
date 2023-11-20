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
 * Bluetooth LE Remote audio functions
 *
 * This file provides definitions and function prototypes for LE remote control
 * device
 *
 */
#ifndef IFX_VOICE_H__
#define IFX_VOICE_H__

// sample rate defines
#define IFX_AUDIO_CFG_SAMPLE_RATE_8K  HIDD_CODEC_SAMP_FREQ_8K
#define IFX_AUDIO_CFG_SAMPLE_RATE_16K HIDD_CODEC_SAMP_FREQ_16K

typedef uint8_t sample_freq_t; // See enum sample_freq_e

void ifxv_init(void);
void ifxv_stop_request(void);
void ifxv_start_request(void);
void ifxv_send_data(uint8_t * ptr, uint16_t len);
wiced_bt_gatt_status_t ifxv_gatts_req_write_callback(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data);
uint16_t ifxv_audio_data_max_size();
void ifxv_set_sampling_rate(sample_freq_t sampling_rate);

#endif // IFX_VOICE_H__
