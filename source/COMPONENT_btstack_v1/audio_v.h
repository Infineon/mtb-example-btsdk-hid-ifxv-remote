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
/**  \addtogroup interfaces
*/
/*! @{ */
/**
* Defines the microphone interface for voice over HIDD/HOGP.
* The (default) microphone uses on-chip Audio ADC.
*/

#ifndef AUDIO_V1_H__
#define AUDIO_V1_H__

#include "adc.h"
#include "audio_types.h"
#include "wiced_audiocodec_interface.h"
#include "hidevent.h"

//#define AUDIO_QUEUE_EVENT            // don't use queue so it is more efficient and save code space
//#define SUPPORT_AUDIO_REG_RD_WR      // Support host codec register read/write, we disable it to save code space

//Fixed report ID for voice
#define HIDD_VOICE_REPORT_ID      0xF7
//Fixed report ID for voice ctrl
#define HIDD_VOICE_CTL_REPORT_ID  0xF8

enum
{
    HIDD_AUDIO_BUTTON_NONE     = 0x00,   // For cases when remote does not have a dedicated audio button
    HIDD_AUDIO_BUTTON_SEND_MSG = 0x01,   // For cases when remote requests host to send the correct command:
                                         //  RC-> Host:   HIDD_RC_MIC_START_REQ,
                                         //  Host->RC:    HIDD_MIC_START
    HIDD_AUDIO_BUTTON_SEND_PCM = 0x02,   // For cases when remote starts/stops audio streaming upon button
                                                // PCM data is sent for as long as the audio key is held down
};

enum
{
    HIDD_AUDIO_DATA             = 0x01,
    HIDD_MIC_START              = 0x02,
    HIDD_MIC_STOP               = 0x03,
    HIDD_SPK_START              = 0x04,
    HIDD_SPK_STOP               = 0x05,

    HIDD_RC_CODECSETTINGS_RD_REQ    = 0x06,
    HIDD_RC_CODECSETTINGS_RD_ACK    = 0x07,
    HIDD_RC_CODECSETTINGS_WT_REQ    = 0x08,
    HIDD_RC_CODECSETTINGS_WT_ACK    = 0x09,
    HIDD_RC_VOICEMODE_RD_REQ        = 0x0A,
    HIDD_RC_VOICEMODE_RD_ACK        = 0x0B,

    HIDD_RC_MIC_START_REQ       = 0x0C,
    HIDD_RC_MIC_STOP_REQ        = 0x0D,
    HIDD_RC_SPEAKER_START_REQ   = 0x0E,
    HIDD_RC_SPEAKER_STOP_REQ    = 0x0F,
    HIDD_PHONECALL_START        = 0x10,
    HIDD_PHONECALL_STOP         = 0x11,
    HIDD_RC_PHONECALL_START_REQ = 0x12,
    HIDD_RC_PHONECALL_END_REQ   = 0x13,

};


#pragma pack(1)
typedef struct
{
    uint8_t  reportId;                 // Audio data report ID 0xf7
    uint8_t  format;                   // BRCM_AUDIO_DATA 0x01
    uint16_t sqn;                      // sequence number
    uint16_t ts;                       // time stamp
    uint16_t buf_state;                // buffer state
    uint16_t rsvd;                     // reserved
    uint16_t dataCnt;                  // sizeof(array(databuffer[]))
    int16_t  dataBuffer[AUDIO_SAMPLE_CNT];  // Audo data
} hidd_voice_report_t;

typedef struct
{
    uint8_t  reportId;                 // Audio control report ID 0xf7
    uint8_t  format;                   // 0x2+
    uint8_t  channel;                  // 0/1
    uint8_t  rsvd;                     // reserved
    uint16_t dataCnt;                  // sizeof(array(databuffer[]))
    uint8_t  dataBuffer[6];
} hidd_voice_control_report_t;

#pragma pack()


typedef struct
{
    wiced_audio_codec_interface_func_tbl *mic_codec;
    hidd_voice_report_t *audio_fifo;
    uint16_t *data_count;
    uint16_t audio_delay;
    uint8_t fifo_count;
    uint8_t audio_gain;
    uint8_t codec_sampling_freq;

    uint8_t enable:1;
    uint8_t audio_boost:1;
}hidd_microphone_config_t;


#pragma pack(1)

typedef PACKED struct {
    hidd_audio_encoding_t      audioEncType;
    AdcAudioDrcSettings drcSettings;
    int32_t             custom_gain_boost;
#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
    AdcAudioFilterCfg_t audioFilterData;
#endif
} hidd_microphone_enhanced_config_t;
#pragma pack()

typedef struct MicAudio
{
    wiced_audio_codec_interface_func_tbl *codec;

    void (*userFn)(void*);
    void * master;

    hidd_voice_report_t *audioAdcData;
    uint16_t *dataCnt;


    uint32_t voicePktTime;
    uint16_t audioCounter;
    uint16_t underflowCounter;
    uint16_t overflowCounter;

    volatile uint8_t active;
    uint8_t pin_en_audio;
    uint8_t fifo_cnt;
    uint8_t fifoOutIndex;
    uint8_t fifoInIndex;
    uint8_t voiceStarted;
    uint8_t timerRunning;
    uint8_t audio_gain;
    uint8_t audio_boost;
    uint8_t codec_sampling_freq;
    uint16_t audio_delay;
}tMicAudio;

////////////////////////////////////////////////////////////////////////////////
/// Configure Audio ADC for microphone
///
/// \param config - configuration
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void mic_audio_config(hidd_microphone_config_t * config);

///////////////////////////////////////////////////////////////////////////////
/// Enhanced configure Audio ADC for microphone (DRC, eq filter etc)
/// This is needed for on-chip Audio ADC. i.e. not needed for external audio codec
///
/// \param pConfig - pointer to the configuration data
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_config_enhanced(uint8_t *pConfig);

////////////////////////////////////////////////////////////////////////////////
/// microphone init
///
/// \param callback - application callback function when audio data is generated and ready to send
/// \param context - pass back to the application as callback parameter as is passed in
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void  mic_audio_init(void (*callback)(void*), void* context);

/// poll microphone for audio activity
///
/// \param dataPtr - application callback function when audio data is generated and ready to send
/// \param context - pass back to the application as callback parameter as is passed in
///
/// \return 0 - no audio activity
///           otherwise - audio activity available
///////////////////////////////////////////////////////////////////////////////
uint8_t mic_audio_poll_activity(HidEventUserDefine * dataPtr);

///////////////////////////////////////////////////////////////////////////////
/// get audio output data to send out
///
/// \param audio_In - pointer to audio activity data
/// \param audio_outData - pointer to the audio output data
///
/// \return the length (in byte) of the audio output data. If 0, no output data
///////////////////////////////////////////////////////////////////////////////
uint16_t mic_audio_get_audio_out_data(hidd_voice_report_t *audio_In, uint8_t *audio_outData);

///////////////////////////////////////////////////////////////////////////////
/// set audio to active/inactive
///
/// \param active - active/inactive
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_set_active(wiced_bool_t active);
#define mic_audio_start() mic_audio_set_active(1)

///////////////////////////////////////////////////////////////////////////////
/// is audio active?
///
/// \param none
///
/// \return WICED_TRUE/WICED_FALSE
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t mic_audio_is_active(void);

///////////////////////////////////////////////////////////////////////////////
//  stop the audio and clear the buffer
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_stop(void);

///////////////////////////////////////////////////////////////////////////////
/// increase internal audio overflow counter
///
/// \param none
///
/// \return none
///////////////////////////////////////////////////////////////////////////////
void mic_audio_overflow(void);

///////////////////////////////////////////////////////////////////////////////
/// is audio overflow for the current session?
///
/// \param none
///
/// \return 0 - no overflow. otherwise indicates the number of lost audio packages
///////////////////////////////////////////////////////////////////////////////
UINT16 mic_audio_is_overflow(void);

///////////////////////////////////////////////////////////////////////////////
/// get the total audio FIFO number
///
/// \param none
///
/// \return the total audio FIFO number
///////////////////////////////////////////////////////////////////////////////
uint8_t mic_audio_FIFO_count(void);

///////////////////////////////////////////////////////////////////////////////
/// mic_assign_mic_en_pin
///
/// \param gpio
/// \param disabled_level (disabled logic)
///
/// \return WICED_TRUE/WICED_FALSE
void mic_assign_mic_en_pin(uint8_t gpio, wiced_bool_t disabled_level);

////////////////////////////////////////////////////////////////////////////////
/// Configure Audio pdm pins
///
/// \param clk, ch1, ch2 gpio pins
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void mic_assign_mic_pdm_pins(uint8_t clk, uint8_t data);

////////////////////////////////////////////////////////////////////////////////
/// Configure codec sampling frequency
///
/// \param freq -- see sample_freq_e
///
/// \return none
////////////////////////////////////////////////////////////////////////////////
void mic_set_codec_sampling_freq(uint8_t freq);

#endif // AUDIO_V1_H__
