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
 * audio.c
 *
 * This file contains audio functions
 *
 */
#include "app.h"
#include "wiced_bt_gatt.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if PROTOCOL_TRACE
 #define APP_ATV_TRACE        WICED_BT_TRACE
 #if PROTOCOL_TRACE>1
  #define APP_ATV_TRACE2      WICED_BT_TRACE
 #else
  #define APP_ATV_TRACE2(...)
 #endif
#else
 #define APP_ATV_TRACE(...)
 #define APP_ATV_TRACE2(...)
#endif

/******************************************************
 *                      Constants
 ******************************************************/
#define HANDLE_HOST_CMD     HDLC_ATVS_ATV_WRITE_CHAR_VALUE
#define HANDLE_DATA         HDLC_ATVS_ATV_READ_CHAR_VALUE
#define HANDLE_DATA_CCCD    HDLD_ATVS_ATV_READ_CHAR_CCCD
#define HANDLE_CTL          HDLC_ATVS_ATV_CONTROL_CHAR_VALUE
#define HANDLE_CTL_CCCD     HDLD_ATVS_ATV_CONTROL_CHAR_CCCD

#define ATV_VERSION_STR_LEN 4
#define ATV_VERSION_RESP_LEN 9
#define ATV_VERSION_MAJOR   version_str[0]

static uint8_t version_str[ATV_VERSION_STR_LEN]={0};

/////////////////////////////////////////////////////////////////////////////////
/// atv_send_data
/////////////////////////////////////////////////////////////////////////////////
static void ATV_send(uint16_t handle, uint16_t cccd, uint8_t * buf, uint16_t max_len, uint8_t * ptr, uint16_t len)
{
    if (len <= max_len)
    {
        ///Send notification if allowed for this handle
        if (cccd && GATT_CLIENT_CONFIG_NOTIFICATION)
        {
            memcpy(buf, ptr, len);
            wiced_bt_gatt_send_notification( link_conn_id(), handle, len, buf );
        }
        else
        {
            WICED_BT_TRACE("notification flag is not set for handle %04x", handle);
        }
    }
    else
    {
        WICED_BT_TRACE("Invalid Len: len=%d, max=%d", len, max_len);
    }
}

/*
 * Process GATT Write request
 */
static char * ATV_codec_supported_str(uint16_t code)
{
    switch (code) {
        case 1: return "ADPCM 8khz/16bit";
        case 3: return "ADPCM 8khz/16bit & 16khz/16bit";
        case 5: return "Opus&ADPCM 8khz/16bit";
        case 7: return "Opus&ADPCM 8khz/16bit & 16khz/16bit";
        default: return "Unknown";
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// atv_supports_ver
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t atv_supports_ver(uint8_t major)
{
    return ATV_VERSION_MAJOR >= major;
}

/*
 * Process GATT Write request
 */
wiced_bt_gatt_status_t atv_gatts_req_write_callback(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
    switch (p_write_data->handle) {
    case HANDLE_HOST_CMD:
    {
        uint8_t *p = p_write_data->p_val;
        uint16_t len = p_write_data->val_len - 1;
        uint8_t command = *p++;

        APP_ATV_TRACE("<----CHAR_TX:0x%02X", command);
        switch (command) {
        case ATV_VOICE_SERVICE_GET_CAPS_REQUEST:
        {
            uint8_t rsp_data_1_0[ATV_VERSION_RESP_LEN]={ATV_VOICE_SERVICE_GET_CAPS_RESP, 1,0, 3, 0, 0,0xa0, 1, 0}; //version 1.0; codec supported: ADPCM 8khz/16bit & 16khz/16bit; on-request mode; 160 bytes;DLE; not used; big endian
            uint8_t rsp_data_0_4[ATV_VERSION_RESP_LEN]={ATV_VOICE_SERVICE_GET_CAPS_RESP, 0, 4, 0, 1, 0, 0x86, 0, 0x14}; //version 0.4; codec supported: ADPCM 8khz/16bit; bytes/frames: 0x0086; bytes/characteristic: 0x0014 . From the TV data, it looks like it is using big endian.
            uint8_t * rsp_data_ptr = rsp_data_0_4;

            APP_ATV_TRACE("ATV CAPS_REQUEST: %A", p, len);
            if (len < ATV_VERSION_STR_LEN)
            {
                WICED_BT_TRACE(" -- len too short: %d", len);
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }

            // save the version string
            memcpy(version_str, p, ATV_VERSION_STR_LEN);
            // if host supports version 1.0, we reply with 1.0 info.
            if (atv_supports_ver(ATV_V1_00))
            {
                rsp_data_ptr = rsp_data_1_0;
            }

            APP_ATV_TRACE(", version: %d.%d ", p[0], p[1]);
            APP_ATV_TRACE(", codec supported: %s", ATV_codec_supported_str((p[2]<<8)|p[3]));
            APP_ATV_TRACE("Reply: %A", rsp_data_ptr, ATV_VERSION_RESP_LEN);
            ATV_send(HANDLE_CTL, *(uint16_t *)app_atvs_atv_control_char_cccd, app_atvs_atv_control_char, app_atvs_atv_control_char_len, rsp_data_ptr, ATV_VERSION_RESP_LEN);
            return WICED_BT_GATT_SUCCESS;
        }
        case ATV_VOICE_SERVICE_MIC_OPEN:
        {
            APP_ATV_TRACE("ATV MIC_OPEN %A", p, len);
            if (len < 2)
            {
                APP_ATV_TRACE(" -- len too short: %d", len);
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }

            uint16_t tv_codec = (p[0] << 8) | p[1];

            if (tv_codec == 1)
            {
                APP_ATV_TRACE(" Sampling rate: 8KHz", tv_codec);
                mic_set_codec_sampling_freq(HIDD_CODEC_SAMP_FREQ_8K);
            }
            else if (tv_codec == 2)
            {
                APP_ATV_TRACE(" Sampling rate: 16KHz", tv_codec);
                mic_set_codec_sampling_freq(HIDD_CODEC_SAMP_FREQ_16K);
            }
            else
            {
                WICED_BT_TRACE(" Not capable to comply for 0x%04x", tv_codec);
                // reply that we comply
                atv_send_cmd(ATV_VOICE_SERVICE_MIC_OPEN_ERROR);
                return WICED_BT_GATT_SUCCESS;
            }

            audio_start();
            return WICED_BT_GATT_SUCCESS;
        }
        case ATV_VOICE_SERVICE_MIC_CLOSE:
        {
            APP_ATV_TRACE("ATV MIC_CLOSE");
            atv_send_cmd(ATV_VOICE_SERVICE_AUDIO_STOP);
            audio_stop();
            return WICED_BT_GATT_SUCCESS;
        }
        default:
            APP_ATV_TRACE("ATV host unknown cmd %02x", command);
            break;
        }
        return WICED_BT_GATT_ILLEGAL_PARAMETER;
    }

    case HANDLE_DATA_CCCD:
        APP_ATV_TRACE2("HANDLE_DATA_CCCD");
        break;

    case HANDLE_CTL_CCCD:
        APP_ATV_TRACE2("HANDLE_CTL_CCCD");
        break;

    default:
        WICED_BT_TRACE("%s: ignored handle 0x%x", __FUNCTION__, p_write_data->handle);
    }
    return WICED_BT_GATT_ATTRIBUTE_NOT_FOUND;
}

/////////////////////////////////////////////////////////////////////////////////
/// atv_send_cmd
/////////////////////////////////////////////////////////////////////////////////
void atv_send_cmd(uint8_t cmd)
{
    APP_ATV_TRACE("ATV -----> CHAR_CTL: 0x%02X",cmd);
    ATV_send(HANDLE_CTL, *(uint16_t *)app_atvs_atv_control_char_cccd, app_atvs_atv_control_char, app_atvs_atv_control_char_len, &cmd, 1);
}

/////////////////////////////////////////////////////////////////////////////////
/// atv_send_data
/////////////////////////////////////////////////////////////////////////////////
void atv_send_data(uint8_t * ptr, uint16_t len)
{
    // Sending data is very intense. The debug output will affect throughput. Do not use use trace level 2 unless just to verify the logic.
    APP_ATV_TRACE2("ATV Send Data:%d->", len);
    ATV_send(HANDLE_DATA, *(uint16_t *)app_atvs_atv_read_char_cccd, app_atvs_atv_read_char, app_atvs_atv_read_char_len, ptr, len);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles start audio request
/////////////////////////////////////////////////////////////////////////////////
//extern void utilslib_delayUs(uint32_t delay);
void atv_start_request(void)
{
    APP_ATV_TRACE("ATV start request");

    // send Page 0x0c, consumer page, SEARCH key
    key_event(AUDIO_KEY_INDEX, TRUE);

    // send ATV command SEARCH
    atv_send_cmd(ATV_VOICE_SERVICE_START_SEARCH);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles stop audio request
///
/// if we send host to request stop, when issue stop message to host.
/// Otherwise, we just stop it.
/////////////////////////////////////////////////////////////////////////////////
void atv_stop_request(void)
{
    APP_ATV_TRACE("ATV stop request");
    atv_send_cmd(ATV_VOICE_SERVICE_AUDIO_STOP);
    audio_stop();
    // clear SEARCH key. Spec does not require to send SEARCH (0x0c, 0x221) key release, but we sent it anyway, in case of other system needs that.
    key_event(AUDIO_KEY_INDEX, FALSE);
}

/////////////////////////////////////////////////////////////////////////////////
/// atv_init
///
/// This function is called only once to initialize atv system opon reset or system start.
/// Noramlly, it is used for timer or some variable/memory initialization.
/// Since we don't have any to initialize, we don't do anything until the link is up.
/////////////////////////////////////////////////////////////////////////////////
void atv_init()
{
    APP_ATV_TRACE("ATV init");
}
