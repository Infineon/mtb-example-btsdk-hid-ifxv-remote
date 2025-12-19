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
 * ifx_voice.c
 *
 * This file contains IFX voice functions
 *
 */
#include "app.h"
#include "wiced_timer.h"
#include "wiced_bt_gatt.h"
#include "cycfg_gatt_db.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if PROTOCOL_TRACE
 #define APP_IFXV_TRACE                 WICED_BT_TRACE
 #if PROTOCOL_TRACE>1
  #define APP_IFXV_TRACE2               WICED_BT_TRACE
 #else
  #define APP_IFXV_TRACE2(...)
 #endif
#else
 #define APP_IFXV_TRACE(...)
 #define APP_IFXV_TRACE2(...)
#endif

/******************************************************
 *                      Constants
 ******************************************************/
#define VERSION_MAJOR       0
#define VERSION_MINOR       2

#define HANDLE_HOST_CMD     HDLC_IFXVS_IFXVS_HOST_VALUE
#define HANDLE_DATA         HDLC_IFXVS_IFXVS_DATA_VALUE
#define HANDLE_DATA_CCCD    HDLD_IFXVS_IFXVS_DATA_CCCD
#define HANDLE_CTL          HDLC_IFXVS_IFXVS_DEVICE_VALUE
#define HANDLE_CTL_CCCD     HDLD_IFXVS_IFXVS_DEVICE_CCCD

#define IFX_AUDIO_CAP           0x01
#define IFX_AUDIO_CAP_RESP      0x02
#define IFX_AUDIO_RESP          0x03
#define IFX_AUDIO_START_REQ     0x04
#define IFX_AUDIO_START         0x05
#define IFX_AUDIO_STOP_REQ      0x06
#define IFX_AUDIO_STOP          0x07

#define IFX_AUDIO_REASON_USER_REQUEST  0x00
#define IFX_AUDIO_REASON_NOT_CAPABLE   0x01
#define IFX_AUDIO_REASON_TIMEOUT       0x02
#define IFX_AUDIO_REASON_INVALID_CFG   0x03

// capability bit defines
#define MAX_CAPABILITY_STRING   3
#define IFX_AUDIO_MSBC_CAP      0x1
#define IFX_AUDIO_ADPCM_CAP     0x2
#define IFX_AUDIO_OPUS_CAP      0x4
// Data size capability
#define IFX_AUDIO_8_BIT         0x1
#define IFX_AUDIO_16_BIT        0x2
// Speed capability
#define IFX_AUDIO_8KHZ          0x1
#define IFX_AUDIO_16KHZ         0x2

// codec config defines
#define IFX_AUDIO_CFG_CODEC_NONE  0
#define IFX_AUDIO_CFG_CODEC_MSBC  1
#define IFX_AUDIO_CFG_CODEC_ADPCM 2
#define IFX_AUDIO_CFG_CODEC_OPUS  3

// data config defines
#define IFX_AUDIO_CFG_DATA_8_BIT  0
#define IFX_AUDIO_CFG_DATA_16_BIT 1

#pragma pack(1)
typedef struct
{
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t codec;
    uint8_t sample_data_depth;
    uint8_t sample_frequency;
    uint8_t streaming_limit_in_sec;

} ifxv_capability_t;

typedef struct
{
    uint8_t             msg;
    ifxv_capability_t   capa;

} ifxv_capability_msg_t;

typedef struct
{
    uint8_t  codec;
    uint8_t  sample_data_depth;
    uint8_t  sample_frequency;
    uint8_t  streaming_limit_in_sec;
    uint16_t frame_length;  // little endian

} ifxv_cfg_t;

typedef struct
{
    uint8_t    msg;
    ifxv_cfg_t cfg;

} ifxv_cfg_msg_t;

#pragma pack()

#define IFXV_CAPABILITY_LEN     sizeof(ifxv_capability_t)
#define IFXV_CAPABILITY_MSG_LEN sizeof(ifxv_capability_msg_t)
#define IFXV_CFG_LEN            sizeof(ifxv_cfg_t)
#define IFXV_CFG_MSG_LEN        sizeof(ifxv_cfg_msg_t)

typedef struct
{
    ifxv_cfg_msg_t      cfg;
    ifxv_capability_t   host_capability;

} ifxv_data_t;

static ifxv_data_t ifxv =
{
    .cfg =
    {
        .msg = IFX_AUDIO_START_REQ,
        .cfg =
        {
            .codec = AUDIO_DEVICE_CODEC,
            .sample_data_depth = IFX_AUDIO_CFG_DATA_16_BIT,
            .sample_frequency = IFX_AUDIO_CFG_SAMPLE_RATE_16K,
            .streaming_limit_in_sec = 0,
            .frame_length = ENCODE_BUF_SIZE
        }
    }
};

/////////////////////////////////////////////////////////////////////////////////
/// Initiate send notification to host based on the handle.
///
/// It copies the data to attribute butter and send notification to host to retrieve data
/////////////////////////////////////////////////////////////////////////////////
static void IFXV_send(uint16_t handle, uint16_t cccd, uint8_t * buf, uint16_t max_len, uint8_t * ptr, uint16_t len)
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

/////////////////////////////////////////////////////////////////////////////////
/// Sends control message to host.
/////////////////////////////////////////////////////////////////////////////////
static void IFXV_send_ctl(void * buf, uint16_t len)
{
    APP_IFXV_TRACE("IFXV CTL handle:%04x --> %A", HANDLE_CTL, buf, len);
    IFXV_send(HANDLE_CTL, *(uint16_t *)app_ifxvs_ifxvs_device_cccd, app_ifxvs_ifxvs_device, app_ifxvs_ifxvs_device_len, buf, len);
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles stop audio request
/////////////////////////////////////////////////////////////////////////////////
static void IFXV_stop_request(uint8_t reason)
{
    uint8_t cmd[2] = {IFX_AUDIO_STOP_REQ};
    cmd[1] = reason;

    APP_IFXV_TRACE("IFXV stop request");
    IFXV_send_ctl(cmd, 2);
    audio_stop();
}

/////////////////////////////////////////////////////////////////////////////////
/// FXV_verify_capability, print out capability
/////////////////////////////////////////////////////////////////////////////////
static wiced_bool_t IFXV_verify_capability(uint8_t info, char * msg, uint8_t cnt, char * str[MAX_CAPABILITY_STRING])
{
    uint8_t pr = 0;

    info &= (1 << cnt)-1;
    if (info)
    {
        APP_IFXV_TRACE(" %s:", msg);
        for (int i=0; info && (i<MAX_CAPABILITY_STRING); i++)
        {
            if (info & 1)
            {
                APP_IFXV_TRACE("%s%s",pr ? "/" : "", str[i]);
                pr++;
            }
            info>>=1;
        }
        if (!pr)
        {
            APP_IFXV_TRACE("NONE");
        }
        return TRUE;
    }
    return FALSE;
}

/******************************************************************************************
 *    Public Functions
 ******************************************************************************************/

/*
 * Process GATT Write request
 */
wiced_bt_gatt_status_t ifxv_gatts_req_write_callback(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_ATTRIBUTE_NOT_FOUND;

    switch (p_write_data->handle) {
    case HANDLE_HOST_CMD:
    {
        uint8_t *p = p_write_data->p_val;
        uint16_t len = p_write_data->val_len - 1;
        uint8_t command = *p++;

        APP_IFXV_TRACE("<--CHAR_TX:0x%02X", command);
        switch (command) {
        case IFX_AUDIO_CAP:
        {
            char * codec_str[MAX_CAPABILITY_STRING] = {"mSBC", "ADPCM", "OPUS"};
            char * data_str[MAX_CAPABILITY_STRING] = {"8-bit", "16-bit", ""};
            char * freq_str[MAX_CAPABILITY_STRING] = {"8k", "16K", ""};

            if (len < IFXV_CAPABILITY_LEN)
            {
                WICED_BT_TRACE("Parameter length=%d -- too short", len);
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }

            // copy the host capability info
            memcpy(&ifxv.host_capability, p, IFXV_CAPABILITY_LEN);

            APP_IFXV_TRACE("IFXV CAPA: %A, v%d.%d", p, len, ifxv.host_capability.version_major, ifxv.host_capability.version_minor);
            if (!IFXV_verify_capability(ifxv.host_capability.codec, "Codec", 3, codec_str) ||
                !IFXV_verify_capability(ifxv.host_capability.sample_data_depth, "Data", 2, data_str) ||
                !IFXV_verify_capability(ifxv.host_capability.sample_frequency, "SampleRate", 2, freq_str))
            {
                result = WICED_BT_GATT_INVALID_CFG;
                break;
            }

            if (ifxv.host_capability.streaming_limit_in_sec)
            {
                APP_IFXV_TRACE(", streaming time limit %d sec", ifxv.host_capability.streaming_limit_in_sec);
            }
            else
            {
                APP_IFXV_TRACE(", No streaming time limit");
            }

            ifxv_capability_msg_t rsp_data = {
                .msg = IFX_AUDIO_CAP_RESP,
                .capa = {
                    .version_major = VERSION_MAJOR,
                    .version_minor = VERSION_MINOR,
                    .codec = AUDIO_CODEC_CAPABILITY,
                    .sample_data_depth = IFX_AUDIO_16_BIT,
                    .sample_frequency = ADUIO_SAMPLE_RATE_CAPABILITY,
                    .streaming_limit_in_sec = 0
                }
            };
            IFXV_send_ctl(&rsp_data, IFXV_CAPABILITY_MSG_LEN);
            result = WICED_BT_GATT_SUCCESS;
            break;
        }
        case IFX_AUDIO_START:
        {
            APP_IFXV_TRACE("IFXV START %A", p, len);

            result = WICED_BT_GATT_INVALID_CFG;
            if (len >= IFXV_CFG_LEN)
            {
                ifxv_cfg_t * cfg = (ifxv_cfg_t *) p;
                if ((cfg->codec == AUDIO_DEVICE_CODEC) && (cfg->sample_data_depth == IFX_AUDIO_CFG_DATA_16_BIT))
                {
                    if (cfg->sample_frequency <= IFX_AUDIO_CFG_SAMPLE_RATE_16K)
                    {
                        char * codec = "";
                        UNUSED_VARIABLE(codec);

                        switch (cfg->codec) {
                            case IFX_AUDIO_CFG_CODEC_NONE:
                                codec = "Raw PCM";
                                break;
                            case IFX_AUDIO_CFG_CODEC_MSBC:
                                codec = "Codec: mSBC";
                                break;
                            case IFX_AUDIO_CFG_CODEC_OPUS:
                                codec = "Codec: OPUS";
                                break;
                            case IFX_AUDIO_CFG_CODEC_ADPCM:
                                codec = "Codec: ADPCM";
                                break;
                        }

                        if (cfg->sample_frequency == IFX_AUDIO_CFG_SAMPLE_RATE_8K)
                        {
                            APP_IFXV_TRACE("%s, Sampling rate: 8KHz", codec);
                            mic_set_codec_sampling_freq(HIDD_CODEC_SAMP_FREQ_8K);
                        }
                        else
                        {
                            APP_IFXV_TRACE("%s, Sampling rate: 16KHz", codec);
                            mic_set_codec_sampling_freq(HIDD_CODEC_SAMP_FREQ_16K);
                        }

                        // good config, start
                        audio_start();
                        result = WICED_BT_GATT_SUCCESS;
                        break;
                    }
                    else
                    {
                        WICED_BT_TRACE(" -- unsupported sample rate %d", cfg->codec);
                    }
                }
                else
                {
                    WICED_BT_TRACE(" -- unsupported codec %d or sample data is not 16-bit", cfg->codec);
                }
            }
            else
            {
                APP_IFXV_TRACE(" -- len too short: %d", len);
            }

            uint8_t rsp_data[2]={IFX_AUDIO_CAP_RESP, IFX_AUDIO_REASON_NOT_CAPABLE};
            IFXV_send_ctl(rsp_data, 2);
            break;
        }
        case IFX_AUDIO_STOP:
        {
            APP_IFXV_TRACE("IFXV STOP");
            audio_stop();
            result = WICED_BT_GATT_SUCCESS;
            break;
        }
        default:
            APP_IFXV_TRACE("IFXV unknown msg %02x", command);
            result = WICED_BT_GATT_ILLEGAL_PARAMETER;
        }
        break;
    }

    case HANDLE_DATA_CCCD:
        APP_IFXV_TRACE2("HANDLE_DATA_CCCD");
        break;

    case HANDLE_CTL_CCCD:
        APP_IFXV_TRACE2("HANDLE_CTL_CCCD");
        break;

    default:
        WICED_BT_TRACE("%s: ignored handle 0x%x", __FUNCTION__, p_write_data->handle);
        break;
    }
    return result;
}

/*
 * Send audio data to host
 */
void ifxv_send_data(uint8_t * ptr, uint16_t len)
{
    APP_IFXV_TRACE2("IFXV Data %d -->", len);
    IFXV_send(HANDLE_DATA, *(uint16_t *)app_ifxvs_ifxvs_data_cccd, app_ifxvs_ifxvs_data, app_ifxvs_ifxvs_data_len, ptr, len);
}

/*
 * Send start audio request to host
 */
void ifxv_start_request(void)
{
    APP_IFXV_TRACE("IFXV start request");

    // send IFXV command start request
    IFXV_send_ctl(&ifxv.cfg, IFXV_CFG_MSG_LEN);
}

/*
 * Set sampling rate
 *
 * The sampling_rate must be sample_freq_e; either HIDD_CODEC_SAMP_FREQ_16K or HIDD_CODEC_SAMP_FREQ_8K.
 */
void ifxv_set_sampling_rate(sample_freq_t sampling_rate)
{
    ifxv.cfg.cfg.sample_frequency = sampling_rate;
}

/*
 * This function handles stop audio request
 */
void ifxv_stop_request()
{
    IFXV_stop_request(IFX_AUDIO_REASON_USER_REQUEST);
}

/////////////////////////////////////////////////////////////////////////////////
/// ifxv_init
///
/// This function is called only once to initialize ifxv system opon reset or system start.
/// Noramlly, it is used for timer, some variable/memory initialization.
/// Since we don't have any to initialize, we don't do anything until the link is up.
/////////////////////////////////////////////////////////////////////////////////
void ifxv_init()
{
    APP_IFXV_TRACE("IFXV init");
}

/*
 * ifxv_audio_data_max_size
 *
 * This function returns mtu size so we know the maximum size can be sent for each notification.
 * When the audio frame size is larger than this limit, it sends in frangments.
 * Ideally, the mtu size should be equal or larger than the audio frame size to be more efficient.
 */
uint16_t ifxv_audio_data_max_size()
{
    uint16_t mtu_size = gatt_get_att_mtu_size();
    APP_IFXV_TRACE2("mtu=%d, data_len=%d", mtu_size, app_ifxvs_ifxvs_data_len);
    if (!mtu_size || mtu_size > app_ifxvs_ifxvs_data_len)
    {
        mtu_size = app_ifxvs_ifxvs_data_len;
    }

    if (mtu_size < app_ifxvs_ifxvs_data_len)
    {
        APP_IFXV_TRACE("Fragmentation size: %d", mtu_size);
    }
    return mtu_size;
}
